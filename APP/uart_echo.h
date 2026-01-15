#ifndef APP_UART_ECHO_H
#define APP_UART_ECHO_H

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// CommTask 使用的线程 flag（由 ISR 置位）
#define UART_ECHO_FLAG_RX (1u << 4)
#define UART_ECHO_FLAG_TX (1u << 5)

// 初始化并启动 USART3 RX：ReceiveToIdle + DMA
bool UART_Echo_StartRxDma(UART_HandleTypeDef *huart);

// 任务侧调用：尽量把 RX 缓冲中的数据用 TX DMA 回显出去
void UART_Echo_TaskPump(UART_HandleTypeDef *huart);

// 由 HAL 回调分发调用（中断上下文）
void UART_Echo_OnRxEventIsr(UART_HandleTypeDef *huart, uint16_t size);
void UART_Echo_OnTxCpltIsr(UART_HandleTypeDef *huart);
void UART_Echo_OnErrorIsr(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* APP_UART_ECHO_H */

