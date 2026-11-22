#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

/* 注意该文件应只用于任务初始化,只能被 robot.c / robot_task.c 包含 */
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#include "robot.h"
#include "robot_def.h"
#include "ins_task.h"
#include "motor_task.h"
#include "daemon.h"
#include "buzzer.h"
#include "bsp_log.h"

extern osThreadId_t insTaskHandle;
extern osThreadId_t motorTaskHandle;
extern osThreadId_t daemonTaskHandle;
extern osThreadId_t robotTaskHandle;

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里创建
 *
 * @note 任务优先级: INS(AboveNormal, 1kHz) > MOTOR(Normal, 1kHz) > ROBOT/DAEMON(Normal)
 *       CMSIS-RTOS v1 的 osPriority 只有 7 档, 数值越大优先级越高
 */
void OSTaskInit(void);

#endif /* ROBOT_TASK_H */
