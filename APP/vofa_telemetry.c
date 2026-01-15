#include "vofa_telemetry.h"

#include "comm/vofa_justfloat.h"

#include <string.h>

static volatile uint8_t s_vofa_tx_in_flight = 0u;
static uint8_t s_vofa_frame[8 * 4 + 4];

bool VOFA_SendImu8_Dma(UART_HandleTypeDef *huart,
                       float pitch_deg,
                       float pitch_acc_deg,
                       float ax_g,
                       float ay_g,
                       float az_g,
                       float gx_dps,
                       float gy_dps,
                       float gz_dps)
{
    if (huart == NULL) {
        return false;
    }

    // 非阻塞：如果上一帧还没发完，为避免覆盖 DMA buffer，直接丢帧。
    if (s_vofa_tx_in_flight) {
        return false;
    }

    // 通道顺序（VOFA+ 通道 1..8）：
    // pitch_deg, pitch_acc_deg, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps
    float ch[8] = {pitch_deg, pitch_acc_deg, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps};
    if (vofa_justfloat_pack(s_vofa_frame, sizeof(s_vofa_frame), ch, 8) != sizeof(s_vofa_frame)) {
        return false;
    }

    // 36字节在 921600bps 下发送时间很短，使用 TX DMA 可避免任务阻塞。
    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(huart, s_vofa_frame, (uint16_t)sizeof(s_vofa_frame));
    if (ret != HAL_OK) {
        return false;
    }
    s_vofa_tx_in_flight = 1u;
    return true;
}

void VOFA_OnUartTxCpltIsr(UART_HandleTypeDef *huart)
{
    (void)huart;
    s_vofa_tx_in_flight = 0u;
}

void VOFA_OnUartErrorIsr(UART_HandleTypeDef *huart)
{
    (void)huart;
    s_vofa_tx_in_flight = 0u;
}
