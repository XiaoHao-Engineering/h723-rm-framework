#ifndef BSP_INIT_H
#define BSP_INIT_H

/**
 * @file bsp_init.h
 * @brief bsp 层统一初始化入口
 *
 * @note 需要在实时系统启动**之前**调用(目前由 RobotInit() 调用).
 *       串口/CAN/SPI 等实例型外设不用在这里初始化, 它们在各自的 Register 函数中
 *       被"按需初始化", 不注册就不会占用资源.
 */
void BSPInit(void);

#endif /* BSP_INIT_H */
