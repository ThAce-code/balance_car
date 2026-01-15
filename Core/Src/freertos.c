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
/* Definitions for xStatusQueue */
osMessageQueueId_t xStatusQueueHandle;
const osMessageQueueAttr_t xStatusQueue_attributes = {
  .name = "xStatusQueue"
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
  /* creation of xStatusQueue */
  xStatusQueueHandle = osMessageQueueNew (1, sizeof(StatusData_t), &xStatusQueue_attributes);

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
  imu_comp_filter_cfg_t filter_cfg = {.alpha = 0.98f};
  imu_comp_filter_init(&pitch_filter, filter_cfg, 0.0f);

  // 简单陀螺零偏标定：上电静止累计约 1s（500 点）求均值。
  // 建议：标定期禁止输出电机 PWM（避免车子乱动导致 bias 不准）。
  float gx_bias = 0.0f;
  float gy_bias = 0.0f;
  float gz_bias = 0.0f;
  uint32_t calib_count = 0;
  const uint32_t calib_target = 500; // ~1s at 500Hz

  HostCommand_t cmd = {0};
  StatusData_t status = {0};

  /* Infinite loop */
  for(;;)
  {
    // 等待 SPI DMA 完成回调置位的线程 flag（新一帧 IMU 数据就绪）。
    uint32_t flags = osThreadFlagsWait(IMU_THREAD_FLAG_NEW_SAMPLE, osFlagsWaitAny, 5);
    if ((flags & IMU_THREAD_FLAG_NEW_SAMPLE) == 0u) {
      // 超时：IMU 没有持续更新，进入安全状态（后续应在此处停 PWM）。
      status.fault_bits |= 0x0001u; // IMU timeout
      continue;
    }

    if (imu_status != IMU_OK) {
      status.fault_bits |= 0x0002u; // IMU init failed
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
    (void)ax_g; // reserved for future use (e.g., roll/tilt checks)
    float ay_g = ((float)raw.ay) / kAccelLsbPerG;
    float az_g = ((float)raw.az) / kAccelLsbPerG;

    // 陀螺零偏标定（要求小车静止）。
    if (calib_count < calib_target) {
      gx_bias += gx_dps;
      gy_bias += gy_dps;
      gz_bias += gz_dps;
      calib_count++;
      if (calib_count == calib_target) {
        gx_bias /= (float)calib_target;
        gy_bias /= (float)calib_target;
        gz_bias /= (float)calib_target;
      }
      continue;
    }

    gx_dps -= gx_bias;
    gy_dps -= gy_bias;
    gz_dps -= gz_bias;

    // 轴向约定（你给的约定）：X 轴为 pitch 轴（绕 X 旋转是俯仰）。
    // 加速度求 pitch：使用 ay/az；陀螺积分用 gx。
    float pitch_acc_deg = atan2f(ay_g, az_g) * (180.0f / (float)M_PI);
    float pitch_deg = imu_comp_filter_update_pitch_deg(&pitch_filter, gx_dps, pitch_acc_deg, kDtS);

    // 读取最新的上位机命令（队列深度=1，只保留最新目标；取不到则沿用上一次命令）。
    while (osMessageQueueGet(xHostCommandQueueHandle, &cmd, NULL, 0) == osOK) {
      // keep newest
    }

    // TODO: encoder read -> speed estimate
    // TODO: control law -> pwm output (with safety checks)

    // 发布状态给 Comm/OLED 任务（队列深度=1：如果队列满则丢弃旧的，始终保留最新一份）。
    status.timestamp_ms = raw.timestamp_ms;
    status.pitch_deg = pitch_deg;
    status.gyro_y_dps = gy_dps;
    status.speed_mps = 0.0f;
    status.pwm_l = 0;
    status.pwm_r = 0;
    status.mode = cmd.mode;

    osStatus_t qret = osMessageQueuePut(xStatusQueueHandle, &status, 0, 0);
    if (qret == osErrorResource) {
      StatusData_t junk;
      (void)osMessageQueueGet(xStatusQueueHandle, &junk, NULL, 0);
      (void)osMessageQueuePut(xStatusQueueHandle, &status, 0, 0);
    }
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OLED_display_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

