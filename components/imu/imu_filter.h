#ifndef COMPONENTS_IMU_IMU_FILTER_H
#define COMPONENTS_IMU_IMU_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float alpha; // 0..1：越接近 1 越信任陀螺（响应快但漂移大），越接近 0 越信任加速度（抗漂移但噪声大）
} imu_comp_filter_cfg_t;

typedef struct {
    float alpha;
    float pitch_deg;
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

#ifdef __cplusplus
}
#endif

#endif /* COMPONENTS_IMU_IMU_FILTER_H */
