/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mydefine.h"
#include "imu.h"
#include "imu/imu_filter.h"
#include "uart_echo.h"
#include "oled_ui.h"
#include "status_store.h"
#include "vofa_telemetry.h"
#include "encoder.h"
#include "comm.h"

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MainControlTask */
osThreadId_t MainControlTaskHandle;
const osThreadAttr_t MainControlTask_attributes = {
  .name = "MainControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLEDDisplayTask */
osThreadId_t OLEDDisplayTaskHandle;
const osThreadAttr_t OLEDDisplayTask_attributes = {
  .name = "OLEDDisplayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for xHostCommandQueue */
osMessageQueueId_t xHostCommandQueueHandle;
const osMessageQueueAttr_t xHostCommandQueue_attributes = {
  .name = "xHostCommandQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LEDtask(void *argument);
void main_control_task(void *argument);
void comm_task(void *argument);
void OLED_display_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xHostCommandQueue */
  xHostCommandQueueHandle = osMessageQueueNew (1, sizeof(HostCommand_t), &xHostCommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(LEDtask, NULL, &LEDTask_attributes);

  /* creation of MainControlTask */
  MainControlTaskHandle = osThreadNew(main_control_task, NULL, &MainControlTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(comm_task, NULL, &CommTask_attributes);

  /* creation of OLEDDisplayTask */
  OLEDDisplayTaskHandle = osThreadNew(OLED_display_task, NULL, &OLEDDisplayTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LEDtask */
/**
  * @brief  Function implementing the LEDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LEDtask */
void LEDtask(void *argument)
{
  /* USER CODE BEGIN LEDtask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
    osDelay(500);
  }
  /* USER CODE END LEDtask */
}

/* USER CODE BEGIN Header_main_control_task */
/**
* @brief Function implementing the MainControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_main_control_task */
void main_control_task(void *argument)
{
  /* USER CODE BEGIN main_control_task */
  // 说明：本任务被设计为“事件驱动”——由 IMU 的 DRDY→SPI DMA 完成中断唤醒。
  // 这样控制计算尽量与 IMU 新数据对齐，避免固定 2ms tick 与真实采样相位漂移。
  static const float kDtS = 0.002f;
  static const float kGyroLsbPerDps = 16.4f;       // ±2000 dps
  static const float kAccelLsbPerG = 4096.0f;      // ±8 g

  // 初始化 IMU（配置寄存器 + WHO_AM_I 校验；DRDY→DMA 采样流水线在回调中运行）。
  imu_status_t imu_status = IMU_Init();

  imu_comp_filter_t pitch_filter;
  // 互补滤波权重：提高 alpha 可减少加速度噪声/振动引入的抖动，但会降低“加速度纠偏”速度。
  // 结合当前的线加速度门控（急加速/飞坡时会暂时忽略加速度角），这里取更偏向陀螺的权重。
  imu_comp_filter_cfg_t filter_cfg = {.alpha = 0.99f};
  imu_comp_filter_init(&pitch_filter, filter_cfg, 0.0f);

  // 线加速度门控（用于急加速/急刹/飞坡等场景）：
  // 当 |acc_norm - 1g| > threshold 时，在 hold_ms 内忽略加速度角，仅用陀螺积分更新。
  static const float kAccelGateThresholdG = 0.30f;
  static const uint32_t kAccelGateHoldMs = 80u;

  // 简单陀螺零偏标定：上电静止累计约 1s（500 点）求均值。
  // 建议：标定期禁止输出电机 PWM（避免车子乱动导致 bias 不准）。
  // 这里使用“在线均值”（running average）做零偏估计：标定期间也正常输出姿态用于上位机观察。
  float gx_bias = 0.0f;
  float gy_bias = 0.0f;
  float gz_bias = 0.0f;
  uint32_t calib_count = 0;
  const uint32_t calib_target = 500; // ~1s at 500Hz

  // Pitch zeroing: average the initial accel-derived pitch while the IMU is static/level,
  // then subtract it so "flat" reads ~0deg (e.g. fixes initial ~0.4deg offset).
  float pitch_zero_deg = 0.0f;
  float pitch_zero_sum = 0.0f;
  uint32_t pitch_zero_count = 0;
  const uint32_t pitch_zero_target = 200;      // ~0.4s at 500Hz
  const float pitch_zero_acc_tol_g = 0.05f;    // only accept samples when |acc_norm-1g| is small

  HostCommand_t cmd = {0};
  StatusData_t status = {0};

  // 初始化编码器模块（TIM3/TIM4 正交编码器）
  Encoder_Init(NULL);  // 使用默认配置：PPR=500, 轮径=65mm, 减速比=28
  encoder_reading_t enc_reading = {0};

  /* Infinite loop */
  for(;;)
  {
    // 等待 SPI DMA 完成回调置位的线程 flag（新一帧 IMU 数据就绪）。
    uint32_t flags = osThreadFlagsWait(IMU_THREAD_FLAG_NEW_SAMPLE, osFlagsWaitAny, 5);
    if ((flags & IMU_THREAD_FLAG_NEW_SAMPLE) == 0u) {
      // 超时：IMU 没有持续更新，进入安全状态（后续应在此处停 PWM）。
      status.fault_bits |= 0x0001u; // IMU timeout

      // 即使 IMU 超时也发布一次状态，便于上位机/VOFA+ 观测到“链路异常”。
      // pitch_deg 设为负值作为诊断：
      // - -1.0：IMU 超时但初始化成功（多半 DRDY/DMA 没跑起来）
      // - -2.0：IMU 初始化失败（多半 SPI/供电/CS/连线问题）
      status.timestamp_ms = HAL_GetTick();
      status.pitch_deg = (imu_status == IMU_OK) ? -1.0f : -2.0f;
      status.wheel_l_mps = 0.0f;
      status.wheel_r_mps = 0.0f;

      StatusStore_Write(&status);
      continue;
    }

    if (imu_status != IMU_OK) {
      status.fault_bits |= 0x0002u; // IMU init failed
      status.timestamp_ms = HAL_GetTick();
      status.pitch_deg = -2.0f;
      status.wheel_l_mps = 0.0f;
      status.wheel_r_mps = 0.0f;
      StatusStore_Write(&status);
      continue;
    }

    icm20602_raw_sample_t raw = {0};
    if (!IMU_TakeLatestRaw(&raw)) {
      // flag 可能合并（coalescing），出现“被唤醒但来不及取到新帧”的情况：直接跳过即可。
      continue;
    }

    // 原始值换算为物理单位（当前量程：±2000dps、±8g；若改了寄存器需同步修改比例）。
    float gx_dps = ((float)raw.gx) / kGyroLsbPerDps;
    float gy_dps = ((float)raw.gy) / kGyroLsbPerDps;
    float gz_dps = ((float)raw.gz) / kGyroLsbPerDps;

    float ax_g = ((float)raw.ax) / kAccelLsbPerG;
    float ay_g = ((float)raw.ay) / kAccelLsbPerG;
    float az_g = ((float)raw.az) / kAccelLsbPerG;
    float acc_norm_g = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

    // 陀螺零偏标定（要求小车静止）。
    // 标定期间不影响遥测输出；但控制输出（PWM）应保持关闭/安全态。
    if (calib_count < calib_target) {
      calib_count++;
      float inv_n = 1.0f / (float)calib_count;
      gx_bias = gx_bias + (gx_dps - gx_bias) * inv_n;
      gy_bias = gy_bias + (gy_dps - gy_bias) * inv_n;
      gz_bias = gz_bias + (gz_dps - gz_bias) * inv_n;
    }

    // 去零偏后的陀螺（标定未完成时使用“当前估计的 bias”）。
    gx_dps -= gx_bias;
    gy_dps -= gy_bias;
    gz_dps -= gz_bias;

    // 轴向约定（按车体坐标）：pitch 轴为 Y。
    // pitch_acc：绕 Y 轴转动时，重力在 X/Z 平面内变化，因此用 ax/az 计算角度；陀螺使用 gy。
    // 约定：车体“前倾”为正。根据当前安装方向，需要对 pitch 取负以匹配该约定。
    float pitch_acc_raw_deg = -atan2f(ax_g, az_g) * (180.0f / (float)M_PI);

    // Learn the "flat" offset from accel early on (best-effort; requires the car to be still).
    if (pitch_zero_count < pitch_zero_target) {
      if (fabsf(acc_norm_g - 1.0f) < pitch_zero_acc_tol_g) {
        pitch_zero_sum += pitch_acc_raw_deg;
        pitch_zero_count++;
        pitch_zero_deg = pitch_zero_sum / (float)pitch_zero_count;
      }
    }

    float pitch_acc_deg = pitch_acc_raw_deg - pitch_zero_deg;
    float pitch_deg = imu_comp_filter_update_pitch_deg_gated(&pitch_filter,
                                                             -gy_dps,
                                                             pitch_acc_deg,
                                                             kDtS,
                                                             acc_norm_g,
                                                             kAccelGateThresholdG,
                                                             kAccelGateHoldMs,
                                                             raw.timestamp_ms);

    // 读取最新的上位机命令（队列深度=1，只保留最新目标；取不到则沿用上一次命令）。
    while (osMessageQueueGet(xHostCommandQueueHandle, &cmd, NULL, 0) == osOK) {
      // keep newest
    }

    // 编码器读取 -> 速度估计
    Encoder_Update(kDtS, &enc_reading);

    // TODO: control law -> pwm output (with safety checks)

    // 发布状态给 Comm/OLED 任务（队列深度=1：如果队列满则丢弃旧的，始终保留最新一份）。
    status.timestamp_ms = raw.timestamp_ms;
    status.pitch_deg = pitch_deg;
    status.pitch_acc_deg = pitch_acc_deg;

    // 遥测保持 IMU 物理轴：ax/ay/az 与 gx/gy/gz 不做对调，便于后续标定与轴向确认。
    status.ax_g = ax_g;
    status.ay_g = ay_g;
    status.az_g = az_g;
    status.gx_dps = gx_dps;
    status.gy_dps = gy_dps;
    status.gz_dps = gz_dps;
    status.speed_mps = enc_reading.avg_speed_mps;
    status.wheel_l_mps = enc_reading.left.speed_filtered_mps;
    status.wheel_r_mps = enc_reading.right.speed_filtered_mps;
    status.pwm_l = 0;
    status.pwm_r = 0;
    status.mode = cmd.mode;

    StatusStore_Write(&status);
  }
  /* USER CODE END main_control_task */
}

/* USER CODE BEGIN Header_comm_task */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_comm_task */
void comm_task(void *argument)
{
  /* USER CODE BEGIN comm_task */

#if (APP_UART3_ECHO_ENABLE != 0)
  // Echo 模式：PC 发什么 MCU 回什么（用于接线/串口基本链路验证）
  (void)UART_Echo_StartRxDma(&huart3);

  for(;;)
  {
    uint32_t flags = osThreadFlagsWait(UART_ECHO_FLAG_RX | UART_ECHO_FLAG_TX, osFlagsWaitAny, 20);
    if ((int32_t)flags >= 0) {
      if ((flags & (UART_ECHO_FLAG_RX | UART_ECHO_FLAG_TX)) != 0u) {
        UART_Echo_TaskPump(&huart3);
      }
    }
  }
#else
  // 协议模式：RX=comm 解析命令；TX=协议遥测（默认）或 VOFA 遥测（可选）
  static uint8_t rx_dma_buf[256];
  comm_config_t cfg = {
    .huart = &huart3,
    .cmd_queue = xHostCommandQueueHandle,
    .rx_dma_buf = rx_dma_buf,
    .rx_dma_buf_size = (uint16_t)sizeof(rx_dma_buf),
  };
  Comm_Init(&cfg);
  Comm_StartRx();

  #if (APP_UART3_TELEM_VOFA != 0)
  // VOFA 遥测：保持与旧链路一致的 50Hz 发送节拍
  const uint32_t kVofaPeriodMs = 20u;
  uint32_t next_vofa_ms = HAL_GetTick() + kVofaPeriodMs;
  #endif

  for(;;)
  {
    Comm_Poll();

  #if (APP_UART3_TELEM_PROTOCOL != 0)
    Comm_TelemetryTick();
  #endif

  #if (APP_UART3_TELEM_VOFA != 0)
    uint32_t now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - next_vofa_ms) >= 0) {
      next_vofa_ms = now_ms + kVofaPeriodMs;
      StatusData_t s;
      if (StatusStore_Read(&s)) {
        (void)VOFA_SendPitch2Speed2_Dma(&huart3,
                                       s.pitch_deg,
                                       s.pitch_acc_deg,
                                       s.wheel_l_mps,
                                       s.wheel_r_mps);
      }
    }
  #endif

    osDelay(10);
  }
#endif
  /* USER CODE END comm_task */
}

/* USER CODE BEGIN Header_OLED_display_task */
/**
* @brief Function implementing the OLEDDisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLED_display_task */
void OLED_display_task(void *argument)
{
  /* USER CODE BEGIN OLED_display_task */
  OledUi_Init();

  StatusData_t last = {0};

  /* Infinite loop */
  for(;;)
  {
    (void)StatusStore_Read(&last);

    OledUi_Render(&last);
    osDelay(100);
  }
  /* USER CODE END OLED_display_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
