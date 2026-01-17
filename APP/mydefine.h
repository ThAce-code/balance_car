/**
 * @file mydefine.h
 * @brief 全局公共定义与常用数据结构
 *
 * 用法建议：
 * - HAL/外设相关头文件放这里统一包含。
 * - 业务模块头文件尽量在对应 `.c` 内按需包含，避免循环依赖。
 */

#ifndef MYDEFINE_H
#define MYDEFINE_H

// STM32 HAL / 外设句柄
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

// 标准 C 库
#include "math.h"
#include "stdarg.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// =========================
// UART3 功能编译开关（P2 阶段二）
// =========================
// 默认：协议遥测（MCU->Host 发送二进制 STATUS 帧），RX 走 comm 协议解析。
// 可选：VOFA 遥测（MCU->Host 发送 JustFloat），用于 VOFA+ 画图。
// Echo 调试：PC 发什么 MCU 回什么（占用 RX/TX 链路，不与遥测并存）。

#ifndef APP_UART3_TELEM_PROTOCOL
#define APP_UART3_TELEM_PROTOCOL 1
#endif

#ifndef APP_UART3_TELEM_VOFA
#define APP_UART3_TELEM_VOFA 0
#endif

#ifndef APP_UART3_ECHO_ENABLE
#define APP_UART3_ECHO_ENABLE 0
#endif

#if (APP_UART3_ECHO_ENABLE != 0)
  #if ((APP_UART3_TELEM_PROTOCOL != 0) || (APP_UART3_TELEM_VOFA != 0))
    #error "Echo mode: set APP_UART3_TELEM_PROTOCOL=0 and APP_UART3_TELEM_VOFA=0"
  #endif
#else
  #if (APP_UART3_TELEM_PROTOCOL + APP_UART3_TELEM_VOFA) != 1
    #error "Choose exactly one of APP_UART3_TELEM_PROTOCOL / APP_UART3_TELEM_VOFA"
  #endif
#endif

// =========================
// 通信/控制数据结构
// =========================
// 说明：这些结构体会被用于 CMSIS-RTOS2 message queue（见 `Core/Src/freertos.c`），
// 因此类型需要在生成的 `freertos.c` 可见，放在公共头里最省事。

typedef struct {
    // 目标前进速度（m/s）
    float target_speed_mps;
    // 目标偏航角速度（deg/s），可按项目需要替换为转向量/角度等
    float target_yaw_rate_dps;
    // 0=STOP, 1=RUN, 2=ESTOP
    uint8_t mode;
} HostCommand_t;

typedef struct {
    // 时间戳：采样/计算时刻（ms）
    uint32_t timestamp_ms;
    // 姿态：互补滤波输出的 pitch（deg）
    float pitch_deg;
    // 姿态：未融合（加速度计推算）的 pitch（deg），用于对比/调参
    float pitch_acc_deg;
    // IMU：加速度（g）
    float ax_g;
    float ay_g;
    float az_g;
    // IMU：角速度（deg/s，已去零偏）
    float gx_dps;
    float gy_dps;
    float gz_dps;
    // 速度估计（m/s），后续由编码器补上
    float speed_mps;
    // Left/right wheel speed (m/s), to be filled by encoder module later
    float wheel_l_mps;
    float wheel_r_mps;
    // PWM 输出（示例占位，后续按你的电机驱动/映射定义）
    int16_t pwm_l;
    int16_t pwm_r;
    // 当前模式（同 HostCommand_t.mode）
    uint8_t mode;
    // 故障位：bit0=IMU timeout, bit1=IMU init failed, ...
    uint16_t fault_bits;
} StatusData_t;

#endif /* MYDEFINE_H */
