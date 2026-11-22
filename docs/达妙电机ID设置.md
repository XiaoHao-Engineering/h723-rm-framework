# 达妙电机 ID 怎么设（和 DJI 电机的区别）

## 0. 当前工程的实际配置

**拨弹盘电机没有改过 ID，用出厂默认 `1`**，代码里 `LOADER_CAN_ID` 已设为 **1**，
所以**不需要上位机，也不需要改电机**。

## 1. 结论

- **达妙电机没有拨码开关、没有按键**，CAN ID 存在**电机内部 flash** 里，掉电保存；
- 设 ID 的方式只有两种：**达妙上位机**（USB-CAN 调试器），或者**用 CAN 写参数寄存器**；
- 我们代码里的 `LOADER_CAN_ID`（默认 **4**）必须和电机 flash 里存的 ID **完全一致**，
  否则电机不会响应，`DMMotorIsEnabled()` 永远是 0（日志里 `[dm_motor] Motor lost, id [4]`）。

> 如果电机是刚买的、没改过 ID，一般是**出厂 ID = 1**。这时要么用上位机改成 4，
> 要么把 `rm_config.h` 的 `LOADER_CAN_ID` 改成 1。

## 2. 和 DJI 电机的区别

| | DJI（M3508+C620 / GM6020） | 达妙（DM4310 / DM4340…） |
|---|---|---|
| ID 怎么设 | C620 电调**按键**设 1~8；GM6020 **拨码** | **电机 flash** 里存，靠上位机/参数帧写 |
| 控制帧 ID | 固定：0x200 / 0x1FF / 0x2FF | **随模式变**：MIT=`ID`，位置=`0x100+ID`，速度=`0x200+ID`，力位=`0x300+ID` |
| 同帧多电机 | 一帧 8 字节，槽位区分 4 台 | 一帧一台 |
| 反馈帧 ID | `0x200 + ID` | 电机自己的 `ID`（帧内也带 ID 字节） |
| 使能/失能 | 无（上电即受控） | 必须发 `0xFC`/`0xFD` 命令到对应模式的帧 ID |
| 参数 | 基本无（PID 在电调里） | 有参数表（RID 0x00~0x1D），可在线读写并保存 |

所以我们代码里这种写法是有原因的：

```c
DMMotorEnableAt(loader, 0x200u + LOADER_CAN_ID);  /* 速度模式: 使能帧 = 0x200 + ID */
DMMotorSetVelocity(loader, rad_s);                /* 控制帧 = 0x200 + ID */
```

## 3. 用上位机改 ID（推荐）

1. 用达妙官方 **USB-CAN 调试器**接电机，打开**达妙上位机**；
2. 扫描/连接后，进入**参数设置**；
3. 把 `CAN_ID`（接收帧 ID / `ESC_ID`）改成目标值，同时把 `MST_ID`（反馈帧 ID）改成同一个值；
4. 点**保存参数**（写入 flash），断电重启后生效；
5. 回读确认：ID = 4。

## 4. 用我们的固件改 ID（可选）

`dm_motor` 模块已经封装了参数读写：

```c
/* 把 ID 改成 4:  0x08 = ESC_ID(接收帧 ID), 0x07 = MST_ID(反馈帧 ID) */
DMMotorWriteParam(motor, DM_RID_ESC_ID, 4);
DMMotorWriteParam(motor, DM_RID_MST_ID, 4);
DMMotorSaveParams(motor);   /* 写 flash, 约 30ms, 只在失能时做 */
```

> 注意：改 ID 要在**电机失能**状态下做；改完之后**必须断电重启**，用新 ID 重新通信。
> 如果电机当前 ID 未知，需要先按“未知 ID”的方式广播（0x7FF）——这属于上位机更方便的场景。

## 5. 怎么确认 ID 对不对

看串口 / RTT 日志：

```
[dm_motor] motor id [4] mode [3] registered     ← 注册成功
[shoot] loader enabled, ready                   ← 电机真的在响应(状态字节 ERR=1)
```

如果一直卡在 `[shoot] loader enabled, ready` 之前、且反复出现：

```
[dm_motor] Motor lost, id [4]
```

说明电机没回帧：ID 不对 / 没供电 / CAN 线序反 / 波特率不对（我们固定 1 Mbps）。
