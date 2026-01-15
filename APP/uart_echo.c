#include "uart_echo.h"

#include "comm/ringbuf.h"

#include "cmsis_os.h"
#include "usart.h"

#include <string.h>

// CubeMX 生成（Core/Src/freertos.c）
extern osThreadId_t CommTaskHandle;

static uint8_t s_uart3_rx_dma_buf[256];
static uint8_t s_uart3_rx_rb_storage[512];
static comm_ringbuf_t s_uart3_rx_rb;

static uint8_t s_uart3_tx_dma_buf[256];
static volatile uint8_t s_uart3_tx_in_flight = 0u;

static uint32_t s_uart3_rx_overflow = 0;

bool UART_Echo_StartRxDma(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return false;
    }

    comm_ringbuf_init(&s_uart3_rx_rb, s_uart3_rx_rb_storage, (uint16_t)sizeof(s_uart3_rx_rb_storage));

    // Receive-to-IDLE + DMA：收到空闲帧间隔时回调 HAL_UARTEx_RxEventCallback
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, s_uart3_rx_dma_buf, (uint16_t)sizeof(s_uart3_rx_dma_buf)) != HAL_OK) {
        return false;
    }

    // 禁用 RX DMA 半传输中断，降低中断频率（我们只关心 IDLE 事件）
    if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    return true;
}

void UART_Echo_OnRxEventIsr(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart3) {
        return;
    }

    if (size > 0u) {
        uint16_t written = comm_ringbuf_write(&s_uart3_rx_rb, s_uart3_rx_dma_buf, size);
        if (written != size) {
            s_uart3_rx_overflow++;
        }
    }

    // 立刻重启接收，避免 IDLE 后停收
    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, s_uart3_rx_dma_buf, (uint16_t)sizeof(s_uart3_rx_dma_buf));
    if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    if (CommTaskHandle != NULL) {
        (void)osThreadFlagsSet(CommTaskHandle, UART_ECHO_FLAG_RX);
    }
}

void UART_Echo_OnTxCpltIsr(UART_HandleTypeDef *huart)
{
    if (huart != &huart3) {
        return;
    }
    s_uart3_tx_in_flight = 0u;
    if (CommTaskHandle != NULL) {
        (void)osThreadFlagsSet(CommTaskHandle, UART_ECHO_FLAG_TX);
    }
}

void UART_Echo_OnErrorIsr(UART_HandleTypeDef *huart)
{
    if (huart != &huart3) {
        return;
    }
    s_uart3_tx_in_flight = 0u;

    // 尝试恢复 RX
    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, s_uart3_rx_dma_buf, (uint16_t)sizeof(s_uart3_rx_dma_buf));
    if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    if (CommTaskHandle != NULL) {
        (void)osThreadFlagsSet(CommTaskHandle, UART_ECHO_FLAG_RX | UART_ECHO_FLAG_TX);
    }
}

void UART_Echo_TaskPump(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return;
    }

    if (s_uart3_tx_in_flight) {
        return;
    }

    uint16_t avail = comm_ringbuf_available(&s_uart3_rx_rb);
    if (avail == 0u) {
        return;
    }

    uint16_t to_send = avail;
    if (to_send > (uint16_t)sizeof(s_uart3_tx_dma_buf)) {
        to_send = (uint16_t)sizeof(s_uart3_tx_dma_buf);
    }

    uint16_t got = comm_ringbuf_read(&s_uart3_rx_rb, s_uart3_tx_dma_buf, to_send);
    if (got == 0u) {
        return;
    }

    if (HAL_UART_Transmit_DMA(huart, s_uart3_tx_dma_buf, got) == HAL_OK) {
        s_uart3_tx_in_flight = 1u;
    }
}

