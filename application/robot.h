#ifndef ROBOT_H
#define ROBOT_H

/**
 * @file robot.h
 * @brief 应用层总入口: 初始化与周期任务
 *
 * 初始化顺序(由 main() 调用 RobotInit()):
 *   BSPInit() -> 各 app 初始化(注册 module) -> 创建 RTOS 任务
 * 周期任务(由 robot_task.c 中的 ROBOT 任务调用):
 *   RobotTask() -> 分发给各个 app 的 Task 函数
 */

/**
 * @brief 机器人初始化, 需要在实时系统启动前调用(会临时关闭中断)
 */
void RobotInit(void);

/**
 * @brief 机器人核心周期任务, 在 rtos 任务中循环调用
 */
void RobotTask(void);

#endif /* ROBOT_H */
