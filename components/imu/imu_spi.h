#ifndef COMPONENTS_IMU_IMU_SPI_H
#define COMPONENTS_IMU_IMU_SPI_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ICM20602 原始采样（寄存器值直接解析）
 *
 * 说明：这里不做比例换算，比例换算在任务/上层完成。
 * burst 读取起始寄存器为 ACCEL_XOUT_H，连续 14B：
 * AX/AY/AZ(6) + TEMP(2) + GX/GY/GZ(6)
 */
typedef struct {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
    uint32_t timestamp_ms;
} icm20602_raw_sample_t;

/**
 * @brief SPI 驱动返回状态
 */
typedef enum {
    ICM20602_SPI_OK = 0,
    ICM20602_SPI_ERR_PARAM = -1,
    ICM20602_SPI_ERR_BUS = -2,
    ICM20602_SPI_ERR_TIMEOUT = -3,
    ICM20602_SPI_ERR_BUSY = -4,
    ICM20602_SPI_ERR_WHOAMI = -5,
} icm20602_spi_status_t;

/**
 * @brief ICM20602 SPI + DMA 运行时上下文（含双缓冲）
 *
 * 关键点：
 * - 该结构体会同时在“中断上下文”和“任务上下文”被访问。
 * - 采样采用 SPI DMA 双缓冲：DMA 写入 active_index 指向的 rx_buf，
 *   任务侧只读取 ready_index 对应的已完成缓冲，避免读写冲突。
 * - drdy_pending 用于记录 DMA 忙时来的 DRDY 边沿（追帧/统计用）。
 */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint8_t whoami_expected;

    volatile uint8_t dma_in_flight;
    volatile uint8_t ready;
    volatile uint8_t ready_index;
    uint8_t active_index;
    volatile uint32_t drdy_pending;

    uint32_t drdy_count;
    uint32_t sample_count;
    uint32_t overrun_count;
    uint32_t dma_error_count;

    // tx/rx 缓冲必须在 DMA 期间保持有效：
    // tx: [0]=起始寄存器|0x80(读)，[1..]=dummy
    // rx: [0]=dummy，rx[1..14] 对应 ACCEL_XOUT_H..GYRO_ZOUT_L
    uint8_t tx_buf[1 + 14];
    uint8_t rx_buf[2][1 + 14];
} icm20602_spi_t;

void icm20602_spi_bind(icm20602_spi_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
 * @brief 初始化 ICM20602：ODR=500Hz，开启 DATA_RDY 中断（DRDY），配置 DLPF，默认量程(±2g/±250dps)
 *
 * 注意：该函数只做寄存器配置与 WHO_AM_I 校验；DRDY→DMA 采样启动在中断回调中进行。
 */
icm20602_spi_status_t icm20602_spi_init_500hz_drdy(icm20602_spi_t *dev);
uint8_t icm20602_spi_read_whoami(icm20602_spi_t *dev);

icm20602_spi_status_t icm20602_spi_read_reg(icm20602_spi_t *dev, uint8_t reg, uint8_t *value);
icm20602_spi_status_t icm20602_spi_write_reg(icm20602_spi_t *dev, uint8_t reg, uint8_t value);

// 在 IMU 的 DRDY EXTI ISR（或等效中断）中调用：若 DMA 空闲则立即启动一次 burst DMA。
void icm20602_spi_on_drdy_isr(icm20602_spi_t *dev);

// 在 HAL SPI 回调（中断上下文）中调用：DMA 完成/错误处理。
void icm20602_spi_on_txrx_cplt_isr(icm20602_spi_t *dev, SPI_HandleTypeDef *hspi);
void icm20602_spi_on_error_isr(icm20602_spi_t *dev, SPI_HandleTypeDef *hspi);

// 任务上下文获取“最新已完成的一帧”原始数据；若没有新数据则返回 false。
bool icm20602_spi_take_latest(icm20602_spi_t *dev, icm20602_raw_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* COMPONENTS_IMU_IMU_SPI_H */
