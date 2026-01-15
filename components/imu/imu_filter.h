#ifndef COMPONENTS_IMU_IMU_FILTER_H
#define COMPONENTS_IMU_IMU_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float alpha; // 0..1：越接近 1 越信任陀螺（响应快但漂移大），越接近 0 越信任加速度（抗漂移但噪声大）
} imu_comp_filter_cfg_t;

typedef struct {
    float alpha;
    float pitch_deg;
    uint32_t gate_until_ms;
} imu_comp_filter_t;

void imu_comp_filter_init(imu_comp_filter_t *f, imu_comp_filter_cfg_t cfg, float initial_pitch_deg);

/**
 * @brief 互补滤波更新 pitch（单位：deg）
 *
 * 输入：
 * - gyro_pitch_dps：俯仰角速度（deg/s），来自陀螺，先做零偏校准再传入
 * - pitch_acc_deg：由加速度计算的俯仰角（deg），静止时可信但有抖动
 * - dt_s：采样周期（秒）
 *
 * 输出：新的 pitch 角（deg）
 */
float imu_comp_filter_update_pitch_deg(imu_comp_filter_t *f, float gyro_pitch_dps, float pitch_acc_deg, float dt_s);

/**
 * @brief 带“线加速度门控”的 pitch 互补滤波（单位：deg）
 *
 * 场景：快速平移/急加速/急刹/飞坡时，加速度计读数会包含线加速度，导致基于加速度的角度抖动。
 * 策略：当 |acc_norm_g - 1| 超过阈值时，在一段 hold 时间内只用陀螺积分更新（忽略加速度角）。
 */
float imu_comp_filter_update_pitch_deg_gated(imu_comp_filter_t *f,
                                             float gyro_pitch_dps,
                                             float pitch_acc_deg,
                                             float dt_s,
                                             float acc_norm_g,
                                             float gate_threshold_g,
                                             uint32_t gate_hold_ms,
                                             uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* COMPONENTS_IMU_IMU_FILTER_H */
