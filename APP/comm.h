/**
 * @file comm.h
 * @brief P2 阶段一：统一通信模块（协议解析 + 命令分发）接口
 *
 * 说明：
 * - 本阶段只“新增 comm 模块”，不替换现有 uart_echo/VOFA 链路，避免影响当前功能。
 * - 协议格式以 `doc/plan/plan_v2.md` 为准（SOF+LEN+CRC，支持快速重同步）。
 * - 接收路径建议：USART + DMA ReceiveToIdle；ISR 只搬运字节到 ringbuf；解析在任务上下文执行。
 */

#ifndef APP_COMM_H
#define APP_COMM_H

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * Protocol constants
 * ========================= */

// SOF is 0xAA 0x55 on wire (little-endian view: 0x55AA), we store as two bytes.
#define COMM_SOF0            (0xAAu)
#define COMM_SOF1            (0x55u)
#define COMM_VER             (0x01u)

#define COMM_FLAG_ACK_REQ    (0x01u)
#define COMM_FLAG_ACK        (0x02u)

typedef enum {
  /* Host -> MCU */
  COMM_MSG_SET_TARGET   = 0x01,
  COMM_MSG_SET_PID      = 0x02,
  COMM_MSG_ESTOP        = 0x03,
  COMM_MSG_TELEM_CONFIG = 0x04,

  /* MCU -> Host */
  COMM_MSG_STATUS       = 0x10,
  COMM_MSG_IMU_RAW      = 0x11,
  COMM_MSG_DEBUG        = 0x12,

  COMM_MSG_ACK          = 0x7F,
  COMM_MSG_NACK         = 0x80,
} comm_msg_id_t;

typedef struct {
  uint8_t ver;
  uint16_t len;     // payload length
  uint8_t msg_id;
  uint8_t flags;
  uint16_t seq;
} comm_header_t;

typedef struct {
  float target_speed_mps;
  float target_yaw_rate_dps;
  uint8_t mode; // 0=STOP, 1=RUN, 2=ESTOP
} __attribute__((packed)) comm_set_target_t;

typedef struct {
  uint16_t period_ms;
  uint16_t mask; // bit0=STATUS, bit1=IMU_RAW, bit2=DEBUG
} __attribute__((packed)) comm_telem_config_t;

typedef struct {
  uint16_t period_ms;
  uint16_t mask;
} comm_telem_state_t;

/* STATUS payload (MCU -> Host)
 * 为了便于串口助手快速验证，保留“滤波后 pitch + pitch_acc + 左右轮速”等关键量。
 *
 * 注意：该结构仅用于“字段定义说明”。协议实际发送使用按字节序列化（little-endian）；
 * 不建议把 RX buffer 直接强转成该结构体（可能存在未对齐访问风险）。
 */
typedef struct {
  uint32_t timestamp_ms;
  float pitch_deg;
  float pitch_acc_deg;
  float wheel_l_mps;
  float wheel_r_mps;
  int16_t pwm_l;
  int16_t pwm_r;
  uint8_t mode;
  uint16_t fault_bits;
} __attribute__((packed)) comm_status_t;

typedef struct {
  UART_HandleTypeDef *huart;     // e.g. &huart3
  osMessageQueueId_t cmd_queue;  // e.g. xHostCommandQueueHandle (depth=1)
  uint8_t *rx_dma_buf;           // DMA RX buffer (user-provided)
  uint16_t rx_dma_buf_size;      // e.g. 256
} comm_config_t;

typedef struct {
  uint32_t rx_overflow;
  uint32_t rx_frames_ok;
  uint32_t rx_crc_fail;
  uint32_t rx_len_reject;
  uint32_t rx_sof_resync;
} comm_stats_t;

/* =========================
 * Public API
 * ========================= */

void Comm_Init(const comm_config_t *cfg);
void Comm_StartRx(void);

/**
 * @brief Task context polling.
 * @note Pulls bytes from ringbuf and parses frames; dispatches commands to cmd_queue.
 */
void Comm_Poll(void);

/**
 * @brief 按当前遥测配置发送（阶段二：协议遥测）
 * @note 周期/开关由 TELEM_CONFIG 或默认值控制；内部使用 HAL_GetTick()。
 */
void Comm_TelemetryTick(void);

comm_telem_state_t Comm_GetTelemState(void);
comm_stats_t Comm_GetStats(void);

/* =========================
 * HAL callback routing (optional in later stages)
 * ========================= */

void Comm_OnRxEventIsr(UART_HandleTypeDef *huart, uint16_t size);
void Comm_OnTxCpltIsr(UART_HandleTypeDef *huart);
void Comm_OnErrorIsr(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMM_H */
