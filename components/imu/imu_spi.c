#include "imu/imu_spi.h"

#include <string.h>

enum {
    ICM20602_REG_SMPLRT_DIV = 0x19,
    ICM20602_REG_CONFIG = 0x1A,
    ICM20602_REG_GYRO_CONFIG = 0x1B,
    ICM20602_REG_ACCEL_CONFIG = 0x1C,
    ICM20602_REG_ACCEL_CONFIG2 = 0x1D,
    ICM20602_REG_INT_PIN_CFG = 0x37,
    ICM20602_REG_INT_ENABLE = 0x38,
    ICM20602_REG_INT_STATUS = 0x3A,
    ICM20602_REG_ACCEL_XOUT_H = 0x3B,
    ICM20602_REG_PWR_MGMT_1 = 0x6B,
    ICM20602_REG_I2C_IF = 0x70,
    ICM20602_REG_WHO_AM_I = 0x75,
};

static const uint32_t kSpiTimeoutMs = 5;

// CS 片选控制（任务/中断上下文都会用到，保持极短执行时间）。
static inline void icm20602_select(const icm20602_spi_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void icm20602_deselect(const icm20602_spi_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

// 阻塞式 SPI 读写（bring-up/寄存器配置用；不用于 500Hz 采样路径）。
static icm20602_spi_status_t icm20602_spi_txrx_blocking(icm20602_spi_t *dev, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(dev->hspi, (uint8_t *)tx, rx, len, kSpiTimeoutMs);
    if (result == HAL_TIMEOUT) {
        return ICM20602_SPI_ERR_TIMEOUT;
    }
    return (result == HAL_OK) ? ICM20602_SPI_OK : ICM20602_SPI_ERR_BUS;
}

static icm20602_spi_status_t icm20602_spi_tx_blocking(icm20602_spi_t *dev, const uint8_t *tx, uint16_t len)
{
    HAL_StatusTypeDef result = HAL_SPI_Transmit(dev->hspi, (uint8_t *)tx, len, kSpiTimeoutMs);
    if (result == HAL_TIMEOUT) {
        return ICM20602_SPI_ERR_TIMEOUT;
    }
    return (result == HAL_OK) ? ICM20602_SPI_OK : ICM20602_SPI_ERR_BUS;
}

void icm20602_spi_bind(icm20602_spi_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (dev == NULL) {
        return;
    }
    memset(dev, 0, sizeof(*dev));
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->whoami_expected = 0x12u;
    dev->active_index = 0;
    dev->ready_index = 0;
    dev->ready = 0;
    dev->dma_in_flight = 0;
    dev->drdy_pending = 0;

    // 预先构造 burst 读取命令：从 ACCEL_XOUT_H 起始连续读取 14 字节。
    // SPI 读操作：寄存器地址 MSB 置 1。
    dev->tx_buf[0] = (uint8_t)(ICM20602_REG_ACCEL_XOUT_H | 0x80u);
    memset(&dev->tx_buf[1], 0xFF, 14);
}

uint8_t icm20602_spi_read_whoami(icm20602_spi_t *dev)
{
    uint8_t whoami = 0;
    (void)icm20602_spi_read_reg(dev, ICM20602_REG_WHO_AM_I, &whoami);
    return whoami;
}

icm20602_spi_status_t icm20602_spi_read_reg(icm20602_spi_t *dev, uint8_t reg, uint8_t *value)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL || value == NULL) {
        return ICM20602_SPI_ERR_PARAM;
    }

    uint8_t tx[2] = {(uint8_t)(reg | 0x80u), 0x00u};
    uint8_t rx[2] = {0};

    icm20602_select(dev);
    icm20602_spi_status_t status = icm20602_spi_txrx_blocking(dev, tx, rx, (uint16_t)sizeof(tx));
    icm20602_deselect(dev);

    if (status != ICM20602_SPI_OK) {
        return status;
    }
    *value = rx[1];
    return ICM20602_SPI_OK;
}

icm20602_spi_status_t icm20602_spi_write_reg(icm20602_spi_t *dev, uint8_t reg, uint8_t value)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL) {
        return ICM20602_SPI_ERR_PARAM;
    }
    uint8_t tx[2] = {(uint8_t)(reg & 0x7Fu), value};

    icm20602_select(dev);
    icm20602_spi_status_t status = icm20602_spi_tx_blocking(dev, tx, (uint16_t)sizeof(tx));
    icm20602_deselect(dev);

    return status;
}

// 在中断上下文启动一次 burst DMA（从 ACCEL_XOUT_H 开始连续读 14B）。
// - 使用双缓冲：本次 DMA 写入 rx_buf[next]，任务侧读取 ready_index 对应缓冲。
// - 仅做“启动 DMA + 计数/标志位”，禁止在 ISR 里做耗时计算。
static icm20602_spi_status_t icm20602_start_burst_dma_isr(icm20602_spi_t *dev)
{
    if (dev->dma_in_flight) {
        return ICM20602_SPI_ERR_BUSY;
    }

    uint8_t next = (uint8_t)(dev->active_index ^ 1u);
    dev->active_index = next;
    dev->dma_in_flight = 1u;

    icm20602_select(dev);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive_DMA(dev->hspi, dev->tx_buf, dev->rx_buf[next],
                                                          (uint16_t)sizeof(dev->tx_buf));
    if (result != HAL_OK) {
        icm20602_deselect(dev);
        dev->dma_in_flight = 0u;
        dev->dma_error_count++;
        return ICM20602_SPI_ERR_BUS;
    }

    return ICM20602_SPI_OK;
}

void icm20602_spi_on_drdy_isr(icm20602_spi_t *dev)
{
    if (dev == NULL) {
        return;
    }

    dev->drdy_count++;
    dev->drdy_pending++;

    // DRDY 来了：如果 DMA 空闲，立即启动一次 burst DMA。
    // 如果 DMA 正忙，则只记录 drdy_pending，在 DMA 完成回调中“追帧”启动下一次。
    if (!dev->dma_in_flight) {
        (void)icm20602_start_burst_dma_isr(dev);
        if (dev->drdy_pending > 0u) {
            dev->drdy_pending--;
        }
    } else {
        dev->overrun_count++;
    }
}

void icm20602_spi_on_txrx_cplt_isr(icm20602_spi_t *dev, SPI_HandleTypeDef *hspi)
{
    if (dev == NULL || hspi != dev->hspi) {
        return;
    }

    icm20602_deselect(dev);
    dev->dma_in_flight = 0u;
    dev->ready_index = dev->active_index;
    dev->ready = 1u;
    dev->sample_count++;

    // 若 DMA 忙时累计了 DRDY，则立刻再启动一次 DMA 追帧（尽量减少“漏采样”的时间窗口）。
    if (dev->drdy_pending > 0u) {
        dev->drdy_pending--;
        (void)icm20602_start_burst_dma_isr(dev);
    }
}

void icm20602_spi_on_error_isr(icm20602_spi_t *dev, SPI_HandleTypeDef *hspi)
{
    if (dev == NULL || hspi != dev->hspi) {
        return;
    }
    icm20602_deselect(dev);
    dev->dma_in_flight = 0u;
    dev->dma_error_count++;
}

static int16_t s16_be(uint8_t high, uint8_t low)
{
    return (int16_t)((int16_t)((uint16_t)high << 8) | (uint16_t)low);
}

bool icm20602_spi_take_latest(icm20602_spi_t *dev, icm20602_raw_sample_t *out)
{
    if (dev == NULL || out == NULL) {
        return false;
    }

    if (!dev->ready) {
        return false;
    }

    uint8_t idx = dev->ready_index;
    dev->ready = 0u;

    const uint8_t *b = dev->rx_buf[idx];
    // b[0] 为 dummy；b[1..14] 对应 ACCEL_XOUT_H..GYRO_ZOUT_L。
    out->ax = s16_be(b[1], b[2]);
    out->ay = s16_be(b[3], b[4]);
    out->az = s16_be(b[5], b[6]);
    out->temp = s16_be(b[7], b[8]);
    out->gx = s16_be(b[9], b[10]);
    out->gy = s16_be(b[11], b[12]);
    out->gz = s16_be(b[13], b[14]);
    out->timestamp_ms = HAL_GetTick();

    return true;
}

icm20602_spi_status_t icm20602_spi_init_500hz_drdy(icm20602_spi_t *dev)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL) {
        return ICM20602_SPI_ERR_PARAM;
    }

    // 复位：PWR_MGMT_1 DEVICE_RESET=1
    (void)icm20602_spi_write_reg(dev, ICM20602_REG_PWR_MGMT_1, 0x80u);
    HAL_Delay(50);

    // 唤醒 + 选择时钟源：CLKSEL=1（X gyro PLL）
    icm20602_spi_status_t status = icm20602_spi_write_reg(dev, ICM20602_REG_PWR_MGMT_1, 0x01u);
    if (status != ICM20602_SPI_OK) {
        return status;
    }
    HAL_Delay(10);

    // 仅使用 SPI：I2C_IF_DIS(bit6)=1，禁用 I2C 接口
    status = icm20602_spi_write_reg(dev, ICM20602_REG_I2C_IF, 0x40u);
    if (status != ICM20602_SPI_OK) {
        return status;
    }

    // 开启陀螺 DLPF：CONFIG DLPF_CFG=3（具体截止频率见手册）
    status = icm20602_spi_write_reg(dev, ICM20602_REG_CONFIG, 0x03u);
    if (status != ICM20602_SPI_OK) {
        return status;
    }

    // 开启加速度 DLPF：ACCEL_CONFIG2 A_DLPF_CFG=3
    status = icm20602_spi_write_reg(dev, ICM20602_REG_ACCEL_CONFIG2, 0x03u);
    if (status != ICM20602_SPI_OK) {
        return status;
    }

    // DLPF 开启时内部采样率为 1kHz；SMPLRT_DIV=1 => 输出 500Hz（即 2ms 一帧）
    status = icm20602_spi_write_reg(dev, ICM20602_REG_SMPLRT_DIV, 0x01u);
    if (status != ICM20602_SPI_OK) {
        return status;
    }

    // 量程配置：
    // - 陀螺 ±2000 dps：GYRO_CONFIG[4:3] FS_SEL=3 -> 0b11<<3 = 0x18
    // - 加速度 ±8 g：ACCEL_CONFIG[4:3] AFS_SEL=2 -> 0b10<<3 = 0x10
    // 注意：改量程后，上层的比例系数也必须同步更新（见 Core/Src/freertos.c）。
    status = icm20602_spi_write_reg(dev, ICM20602_REG_GYRO_CONFIG, 0x18u);
    if (status != ICM20602_SPI_OK) {
        return status;
    }
    status = icm20602_spi_write_reg(dev, ICM20602_REG_ACCEL_CONFIG, 0x10u);
    if (status != ICM20602_SPI_OK) {
        return status;
    }

    // INT_PIN_CFG：高电平有效、推挽输出、脉冲 50us（按手册默认/需求调整）
    status = icm20602_spi_write_reg(dev, ICM20602_REG_INT_PIN_CFG, 0x00u);
    if (status != ICM20602_SPI_OK) {
        return status;
    }

    // INT_ENABLE：开启 DATA_RDY_INT(bit0)
    status = icm20602_spi_write_reg(dev, ICM20602_REG_INT_ENABLE, 0x01u);
    if (status != ICM20602_SPI_OK) {
        return status;
    }

    // 读一次 INT_STATUS 清除中断状态（避免上电残留）
    uint8_t dummy = 0;
    (void)icm20602_spi_read_reg(dev, ICM20602_REG_INT_STATUS, &dummy);

    uint8_t whoami = icm20602_spi_read_whoami(dev);
    if (whoami != dev->whoami_expected) {
        return ICM20602_SPI_ERR_WHOAMI;
    }

    return ICM20602_SPI_OK;
}
