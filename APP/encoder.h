/**
 * @file encoder.h
 * @brief 编码器驱动模块 - 双轮速度测量
 *
 * 硬件配置：
 * - TIM4: 左轮编码器 (PD12=CH1, PD13=CH2)
 * - TIM3: 右轮编码器 (PA6=CH1, PA7=CH2)
 * - 模式: 正交编码器 4 倍频 (TIM_ENCODERMODE_TI12)
 *
 * 编码器参数：
 * - 类型: 磁巨阻 (GMR)，安装在电机尾部
 * - PPR: 500 (电机轴)
 * - 减速比: 1:28
 * - 轮径: 65mm
 */

#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================
 * 配置参数
 * ============================================ */

typedef struct {
    uint16_t ppr;              /**< 编码器 PPR (电机轴，四倍频前) */
    float wheel_diameter_m;    /**< 轮径 (米) */
    float gear_ratio;          /**< 减速比 (电机轴转数/输出轴转数) */
    float lpf_alpha;           /**< 低通滤波系数 (0-1，1.0=无滤波) */
    bool left_inverted;        /**< 左轮方向取反 */
    bool right_inverted;       /**< 右轮方向取反 */
} encoder_config_t;

/** 默认配置: PPR=500, 轮径=65mm, 减速比=28, 滤波系数=0.3 */
#define ENCODER_CONFIG_DEFAULT { \
    .ppr = 500,                  \
    .wheel_diameter_m = 0.065f,  \
    .gear_ratio = 28.0f,         \
    .lpf_alpha = 0.3f,           \
    .left_inverted = false,      \
    .right_inverted = false      \
}

/* ============================================
 * 数据结构
 * ============================================ */

/** 单轮编码器数据 */
typedef struct {
    int32_t delta_counts;      /**< 本周期计数差 (有符号，正=前进) */
    float speed_mps;           /**< 原始线速度 (m/s) */
    float speed_filtered_mps;  /**< 滤波后线速度 (m/s) */
} encoder_data_t;

/** 双轮编码器读数 */
typedef struct {
    encoder_data_t left;       /**< 左轮数据 */
    encoder_data_t right;      /**< 右轮数据 */
    float avg_speed_mps;       /**< 左右平均线速度 (滤波后) */
    uint32_t timestamp_ms;     /**< 时间戳 (ms) */
} encoder_reading_t;

/* ============================================
 * API 函数
 * ============================================ */

/**
 * @brief 初始化编码器模块
 * @param cfg 配置参数 (传 NULL 使用默认配置)
 * @note 内部会调用 HAL_TIM_Encoder_Start()
 */
void Encoder_Init(const encoder_config_t *cfg);

/**
 * @brief 读取编码器并计算速度 (在 MainControlTask 中周期调用)
 * @param dt_s 采样周期 (秒)，通常为 0.002f (2ms)
 * @param out 输出结构体
 * @return true=成功，false=参数错误或未初始化
 * @note 非阻塞，直接读 TIM->CNT 寄存器
 */
bool Encoder_Update(float dt_s, encoder_reading_t *out);

/**
 * @brief 获取上次更新的编码器数据 (只读快照)
 * @param out 输出结构体
 * @return true=有有效数据，false=尚未初始化
 */
bool Encoder_GetLatest(encoder_reading_t *out);

/**
 * @brief 重置编码器计数器和滤波器状态
 * @note 通常在系统初始化或故障恢复时调用
 */
void Encoder_Reset(void);

/**
 * @brief 运行时更新配置参数
 * @param cfg 新配置
 */
void Encoder_SetConfig(const encoder_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* APP_ENCODER_H */
