#include "uart_echo.h"
#include "vofa_telemetry.h"

#include "usart.h"

// 统一接管 USART3 的 HAL 回调，避免多个模块重复实现同名 callback。
// 目前用于：
// - RX：DMA ReceiveToIdle（echo 测试）
// - TX：DMA 发送完成（echo/VOFA）

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3) {
        UART_Echo_OnRxEventIsr(huart, Size);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        UART_Echo_OnTxCpltIsr(huart);
        VOFA_OnUartTxCpltIsr(huart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        UART_Echo_OnErrorIsr(huart);
        VOFA_OnUartErrorIsr(huart);
    }
}

