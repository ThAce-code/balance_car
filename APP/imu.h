#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdbool.h>

#include "imu/imu_spi.h"

// IMU 新数据到达标志：由 SPI DMA 完成中断置位，MainControlTask 等待该标志后处理一帧数据。
#define IMU_THREAD_FLAG_NEW_SAMPLE (1u << 0)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IMU_OK = 0,
    IMU_ERR_INIT = -1,
    IMU_ERR_WHOAMI = -2,
} imu_status_t;

// 初始化 ICM20602：配置为 500Hz DRDY + SPI DMA burst 采样。
imu_status_t IMU_Init(void);

// 获取 DMA 捕获的最新原始数据；若当前没有新帧则返回 false。
bool IMU_TakeLatestRaw(icm20602_raw_sample_t *out);

// （可选）用于调试/遥测：读取计数器等状态。
const icm20602_spi_t *IMU_DebugGetSpiState(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_H */
