#include "imu.h"

#include "cmsis_os.h"
#include "main.h"
#include "spi.h"

// 组件层 SPI 驱动实例（包含 DMA 双缓冲与计数器）。
static icm20602_spi_t g_imu_spi;
// 使能标志：初始化成功后才允许在回调里启动采样。
static volatile uint8_t g_imu_enabled = 0u;

// Defined in Core/Src/freertos.c (CubeMX generated).
extern osThreadId_t MainControlTaskHandle;

imu_status_t IMU_Init(void)
{
    // 绑定 SPI 句柄与 CS 引脚（CS 由 CubeMX 生成的宏给出）。
    icm20602_spi_bind(&g_imu_spi, &hspi1, CS_GPIO_Port, CS_Pin);

    // 初始化 IMU：寄存器配置 + WHO_AM_I 严格校验。
    icm20602_spi_status_t status = icm20602_spi_init_500hz_drdy(&g_imu_spi);
    if (status == ICM20602_SPI_ERR_WHOAMI) {
        return IMU_ERR_WHOAMI;
    }
    if (status == ICM20602_SPI_OK) {
        g_imu_enabled = 1u;
    }
    return (status == ICM20602_SPI_OK) ? IMU_OK : IMU_ERR_INIT;
}

bool IMU_TakeLatestRaw(icm20602_raw_sample_t *out)
{
    return icm20602_spi_take_latest(&g_imu_spi, out);
}

const icm20602_spi_t *IMU_DebugGetSpiState(void)
{
    return &g_imu_spi;
}

// HAL 回调分发（中断上下文）：把 DRDY 与 SPI DMA 完成事件转交给 components/imu/imu_spi.c
// 后续如果 EXTI/SPI 还有其他用途，建议在这里做“分发器”，避免互相覆盖回调。

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // DRDY/INT：由 CubeMX 将 PC5 标记为 IMU_INT 后，会生成 IMU_INT_Pin 宏。
    if (g_imu_enabled && GPIO_Pin == IMU_INT_Pin) {
        icm20602_spi_on_drdy_isr(&g_imu_spi);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // SPI DMA 完成：标记“新帧就绪”，然后用线程 flag 唤醒 MainControlTask 去消费数据。
    icm20602_spi_on_txrx_cplt_isr(&g_imu_spi, hspi);
    if (g_imu_enabled && MainControlTaskHandle != NULL) {
        // CMSIS-RTOS2：允许在 ISR 中调用 osThreadFlagsSet（会走 *FromISR 路径）。
        (void)osThreadFlagsSet(MainControlTaskHandle, IMU_THREAD_FLAG_NEW_SAMPLE);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    // SPI 错误：取消片选、清 busy 标志并计数；后续可在任务里根据错误计数做复位恢复。
    icm20602_spi_on_error_isr(&g_imu_spi, hspi);
}
