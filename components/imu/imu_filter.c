#include "imu/imu_filter.h"

#include <math.h>

void imu_comp_filter_init(imu_comp_filter_t *f, imu_comp_filter_cfg_t cfg, float initial_pitch_deg)
{
    if (f == 0) {
        return;
    }

    // alpha 合法性保护：避免传入 0/1 或异常导致数值不稳定
    f->alpha = (cfg.alpha > 0.0f && cfg.alpha < 1.0f) ? cfg.alpha : 0.98f;
    f->pitch_deg = initial_pitch_deg;
    f->gate_until_ms = 0;
}

float imu_comp_filter_update_pitch_deg(imu_comp_filter_t *f, float gyro_pitch_dps, float pitch_acc_deg, float dt_s)
{
    if (f == 0) {
        return pitch_acc_deg;
    }

    // dt 异常保护：dt 太大/为 0 通常意味着采样丢失或调度问题，此时直接回到加速度角
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        f->pitch_deg = pitch_acc_deg;
        return f->pitch_deg;
    }

    // 互补滤波：pitch = alpha * (pitch + gyro*dt) + (1-alpha) * pitch_acc
    float pitch_gyro = f->pitch_deg + gyro_pitch_dps * dt_s;
    f->pitch_deg = f->alpha * pitch_gyro + (1.0f - f->alpha) * pitch_acc_deg;
    return f->pitch_deg;
}

float imu_comp_filter_update_pitch_deg_gated(imu_comp_filter_t *f,
                                             float gyro_pitch_dps,
                                             float pitch_acc_deg,
                                             float dt_s,
                                             float acc_norm_g,
                                             float gate_threshold_g,
                                             uint32_t gate_hold_ms,
                                             uint32_t now_ms)
{
    if (f == 0) {
        return pitch_acc_deg;
    }

    // dt 异常保护：直接回到加速度角
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        f->pitch_deg = pitch_acc_deg;
        return f->pitch_deg;
    }

    // 门控参数保护
    if (gate_threshold_g < 0.0f) {
        gate_threshold_g = 0.0f;
    }

    float acc_err = fabsf(acc_norm_g - 1.0f);
    if (acc_err > gate_threshold_g) {
        f->gate_until_ms = now_ms + gate_hold_ms;
    }

    float pitch_gyro = f->pitch_deg + gyro_pitch_dps * dt_s;

    // 门控期间：只用陀螺积分（忽略加速度角，避免线加速度污染）
    if (now_ms < f->gate_until_ms) {
        f->pitch_deg = pitch_gyro;
        return f->pitch_deg;
    }

    // 正常互补融合
    f->pitch_deg = f->alpha * pitch_gyro + (1.0f - f->alpha) * pitch_acc_deg;
    return f->pitch_deg;
}
