#ifndef APP_VOFA_TELEMETRY_H
#define APP_VOFA_TELEMETRY_H

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 发送 VOFA+ JustFloat：pitch(滤波) + pitch_acc(未融合) + 3轴acc + 3轴gyro（共 8 通道）
 *
 * 说明：
 * - 使用 USART TX DMA（非阻塞）。若上一帧仍在发送，则本次会丢帧（返回 false）。
 */
bool VOFA_SendImu8_Dma(UART_HandleTypeDef *huart,
                       float pitch_deg,
                       float pitch_acc_deg,
                       float ax_g,
                       float ay_g,
                       float az_g,
                       float gx_dps,
                       float gy_dps,
                       float gz_dps);

// 由 UART HAL 回调分发调用（中断上下文），用于清除 TX busy
void VOFA_OnUartTxCpltIsr(UART_HandleTypeDef *huart);
void VOFA_OnUartErrorIsr(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* APP_VOFA_TELEMETRY_H */
