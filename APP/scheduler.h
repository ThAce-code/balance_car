/**
 * @file scheduler.h
 * @brief 任务调度器头文件 - 基于时间片轮转的多任务调度系统
 */
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "mydefine.h"

/**
 * @brief 调度器初始化函数
 */
void scheduler_init(void);

/**
 * @brief 调度器运行函数
 */
void scheduler_run(void);


#endif