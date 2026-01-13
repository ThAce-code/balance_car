/**
* @file scheduler.c
 * @brief 任务调度器实现 - 基于HAL_GetTick()的时间片轮转调度
 */
#include "scheduler.h"

uint8_t task_num; // 全局变量，用于存储任务数量

typedef struct {
    void (*task_func)(void); // 任务函数指针
    uint32_t rate_ms;        // 执行周期(毫秒)
    uint32_t last_run;       // 上次运行时间(毫秒)
} task_t;


// 静态任务数组，每个任务包含任务函数、执行周期（毫秒）和上次运行时间（毫秒）
// 错峰执行策略：避免同时执行多个重负载任务
// 时序安排：0ms-Gary, 10ms-Encoder, 15ms-IMU (20ms周期错峰)
static task_t scheduler_task[] =
{
    {led_task,500,0}
};

/**
 * @brief 调度器初始化函数
 * @retval 无
 */
void scheduler_init(void)
{
    task_num = sizeof(scheduler_task) / sizeof(task_t); // 计算任务数组的元素个数
}

/**
 * @brief 调度器运行函数
 * @retval 无
 */
void scheduler_run(void)
{
    for (uint8_t i = 0; i < task_num; i++) // 遍历任务数组中的所有任务
    {
        uint32_t now_time = HAL_GetTick(); // 获取当前的系统时间（毫秒）

        if (now_time >= scheduler_task[i].rate_ms + scheduler_task[i].last_run) // 检查当前时间是否达到任务的执行时间
        {
            scheduler_task[i].last_run = now_time; // 更新任务的上次运行时间为当前时间
            scheduler_task[i].task_func();         // 执行任务函数
        }
    }
}