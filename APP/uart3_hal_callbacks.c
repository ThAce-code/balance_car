#include "mydefine.h"

#if (APP_UART3_ECHO_ENABLE != 0)
#include "uart_echo.h"
#endif

#include "comm.h"
#include "vofa_telemetry.h"

#include "usart.h"

// 统一接管 USART3 的 HAL 回调，按编译开关选择工作模式：
// - 协议模式：RX=comm（DMA ReceiveToIdle + ringbuf + 解析），TX=comm（协议遥测）。
// - VOFA 模式：RX=comm，TX=VOFA（JustFloat）。
// - Echo 模式：RX/TX=echo（不与遥测并存）。

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3) {
#if (APP_UART3_ECHO_ENABLE != 0)
        UART_Echo_OnRxEventIsr(huart, Size);
#else
        Comm_OnRxEventIsr(huart, Size);
#endif
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
#if (APP_UART3_ECHO_ENABLE != 0)
        UART_Echo_OnTxCpltIsr(huart);
#else
  #if (APP_UART3_TELEM_PROTOCOL != 0)
        Comm_OnTxCpltIsr(huart);
  #endif
  #if (APP_UART3_TELEM_VOFA != 0)
        VOFA_OnUartTxCpltIsr(huart);
  #endif
#endif
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
#if (APP_UART3_ECHO_ENABLE != 0)
        UART_Echo_OnErrorIsr(huart);
#else
        Comm_OnErrorIsr(huart);
  #if (APP_UART3_TELEM_VOFA != 0)
        VOFA_OnUartErrorIsr(huart);
  #endif
#endif
    }
}
