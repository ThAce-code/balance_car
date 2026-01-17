/**
 * @file encoder.c
 * @brief 编码器驱动模块实现
 */

#include "encoder.h"
#include "tim.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ============================================
 * 内部状态
 * ============================================ */

typedef struct {
    uint16_t last_cnt;
    float speed_filtered_mps;
} encoder_state_t;

static struct {
    encoder_config_t cfg;
    encoder_state_t left;
    encoder_state_t right;
    encoder_reading_t latest;
    float counts_to_m;    /**< 预计算: counts -> 米 */
    bool initialized;
} g_enc;

/* ============================================
 * 初始化
 * ============================================ */

void Encoder_Init(const encoder_config_t *cfg)
{
    /* 使用默认或传入配置 */
    if (cfg != NULL) {
        g_enc.cfg = *cfg;
    } else {
        encoder_config_t def = ENCODER_CONFIG_DEFAULT;
        g_enc.cfg = def;
    }

    /* 预计算 counts -> 米的转换系数
     * counts_per_wheel_rev = PPR * 4 * gear_ratio
     * counts_to_m = wheel_circumference / counts_per_wheel_rev
     *             = PI * diameter / (PPR * 4 * gear_ratio)
     */
    float counts_per_wheel_rev = (float)(g_enc.cfg.ppr * 4) * g_enc.cfg.gear_ratio;
    g_enc.counts_to_m = M_PI * g_enc.cfg.wheel_diameter_m / counts_per_wheel_rev;

    /* 启动编码器定时器 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* 初始化状态：读取当前计数作为基准 */
    // Hardware mapping: Left wheel = TIM4, Right wheel = TIM3
    g_enc.left.last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    g_enc.right.last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    g_enc.left.speed_filtered_mps = 0.0f;
    g_enc.right.speed_filtered_mps = 0.0f;

    memset(&g_enc.latest, 0, sizeof(g_enc.latest));
    g_enc.initialized = true;
}

/* ============================================
 * 周期更新
 * ============================================ */

bool Encoder_Update(float dt_s, encoder_reading_t *out)
{
    if (!g_enc.initialized || out == NULL || dt_s <= 0.0f) {
        return false;
    }

    /* 1. 读取当前计数器（直接读寄存器，非阻塞） */
    uint16_t cnt_l = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    uint16_t cnt_r = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    /* 2. 计算有符号差分（自动处理 16 位溢出）
     * 利用有符号 16 位减法：
     * - 若 current=0x0002, previous=0xFFFE，则 delta = +4
     * - 若 current=0xFFFE, previous=0x0002，则 delta = -4
     */
    int16_t delta_l = (int16_t)(cnt_l - g_enc.left.last_cnt);
    int16_t delta_r = (int16_t)(cnt_r - g_enc.right.last_cnt);

    /* 3. 根据配置取反方向 */
    if (g_enc.cfg.left_inverted) {
        delta_l = -delta_l;
    }
    if (g_enc.cfg.right_inverted) {
        delta_r = -delta_r;
    }

    /* 4. 更新上次计数 */
    g_enc.left.last_cnt = cnt_l;
    g_enc.right.last_cnt = cnt_r;

    /* 5. 计算速度 (m/s)
     * speed = delta_counts * counts_to_m / dt_s
     */
    float inv_dt = 1.0f / dt_s;
    float speed_l = (float)delta_l * g_enc.counts_to_m * inv_dt;
    float speed_r = (float)delta_r * g_enc.counts_to_m * inv_dt;

    /* 6. 一阶低通滤波
     * y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
     */
    float alpha = g_enc.cfg.lpf_alpha;
    float one_minus_alpha = 1.0f - alpha;
    g_enc.left.speed_filtered_mps = alpha * speed_l + one_minus_alpha * g_enc.left.speed_filtered_mps;
    g_enc.right.speed_filtered_mps = alpha * speed_r + one_minus_alpha * g_enc.right.speed_filtered_mps;

    // 死区钳零：避免低通滤波指数衰减后落入 float 次正规区，导致“理论 0 但非 0”的尾巴。
    // 阈值设得很小，仅用于清理数值残留，不影响正常低速运动。
    const float kSpeedDeadzoneMps = 1e-6f;
    if (fabsf(g_enc.left.speed_filtered_mps) < kSpeedDeadzoneMps) {
        g_enc.left.speed_filtered_mps = 0.0f;
    }
    if (fabsf(g_enc.right.speed_filtered_mps) < kSpeedDeadzoneMps) {
        g_enc.right.speed_filtered_mps = 0.0f;
    }

    /* 7. 填充输出 */
    out->left.delta_counts = delta_l;
    out->left.speed_mps = speed_l;
    out->left.speed_filtered_mps = g_enc.left.speed_filtered_mps;

    out->right.delta_counts = delta_r;
    out->right.speed_mps = speed_r;
    out->right.speed_filtered_mps = g_enc.right.speed_filtered_mps;

    out->avg_speed_mps = (g_enc.left.speed_filtered_mps + g_enc.right.speed_filtered_mps) * 0.5f;
    out->timestamp_ms = HAL_GetTick();

    /* 8. 保存快照 */
    g_enc.latest = *out;

    return true;
}

/* ============================================
 * 获取快照
 * ============================================ */

bool Encoder_GetLatest(encoder_reading_t *out)
{
    if (!g_enc.initialized || out == NULL) {
        return false;
    }
    *out = g_enc.latest;
    return true;
}

/* ============================================
 * 重置
 * ============================================ */

void Encoder_Reset(void)
{
    if (!g_enc.initialized) {
        return;
    }

    /* 重新读取当前计数作为基准 */
    g_enc.left.last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    g_enc.right.last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    g_enc.left.speed_filtered_mps = 0.0f;
    g_enc.right.speed_filtered_mps = 0.0f;

    memset(&g_enc.latest, 0, sizeof(g_enc.latest));
}

/* ============================================
 * 运行时更新配置
 * ============================================ */

void Encoder_SetConfig(const encoder_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    g_enc.cfg = *cfg;

    /* 重新计算转换系数 */
    float counts_per_wheel_rev = (float)(g_enc.cfg.ppr * 4) * g_enc.cfg.gear_ratio;
    g_enc.counts_to_m = M_PI * g_enc.cfg.wheel_diameter_m / counts_per_wheel_rev;
}
