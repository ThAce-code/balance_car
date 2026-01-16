编码器模块开发计划

     概述

     为平衡车项目实现编码器驱动，用于测量轮速。

     硬件配置（已完成）：
     - TIM3: 左轮 (PA6/PA7)
     - TIM4: 右轮 (PD12/PD13)
     - 模式: 正交编码器 4 倍频

     参数：
     - 编码器: 磁巨阻（GMR），安装在电机尾部
     - 结构: encoder → motor → reducer → output
     - PPR: 500（电机轴，4 倍频后 2000 counts/rev）
     - 减速比: 1:28
     - 轮径: 65mm（周长 ≈ 0.204m）
     - 输出轴每转计数: 500 × 4 × 28 = 56000 counts/rev

     ---
     实现方案

     1. 创建 APP/encoder.h

     // 配置结构
     typedef struct {
         uint16_t ppr;              // 500 (电机轴)
         float wheel_diameter_m;    // 0.065
         float gear_ratio;          // 28.0 (减速比)
         float lpf_alpha;           // 0.3 (滤波系数)
     } encoder_config_t;

     // 数据结构
     typedef struct {
         int32_t delta_counts;      // 计数差
         float speed_mps;           // 原始速度
         float speed_filtered_mps;  // 滤波后速度
     } encoder_data_t;

     typedef struct {
         encoder_data_t left;
         encoder_data_t right;
         float avg_speed_mps;       // 平均速度
     } encoder_reading_t;

     // API
     void Encoder_Init(const encoder_config_t *cfg);
     bool Encoder_Update(float dt_s, encoder_reading_t *out);
     bool Encoder_GetLatest(encoder_reading_t *out);
     void Encoder_Reset(void);

     2. 创建 APP/encoder.c

     核心逻辑：

     // 16 位溢出处理（有符号差分）
     int16_t delta = (int16_t)(current_cnt - last_cnt);

     // 速度计算（编码器在电机轴，需除以减速比）
     // counts_per_wheel_rev = PPR * 4 * gear_ratio = 500 * 4 * 28 = 56000
     // counts_to_m = PI * 0.065 / 56000 ≈ 3.644e-6 m/count
     speed_mps = delta * counts_to_m / dt_s;

     // 一阶低通滤波
     speed_filtered = alpha * speed_raw + (1 - alpha) * speed_filtered_prev;

     3. 集成到 Core/Src/freertos.c

     在 main_control_task() 中：

     // 初始化
     Encoder_Init(NULL);

     // 主循环（替换 TODO 第 296 行）
     encoder_reading_t enc;
     Encoder_Update(0.002f, &enc);
     status.speed_mps = enc.avg_speed_mps;

     ---
     关键文件
     ┌──────┬────────────────────────────────┐
     │ 操作 │              文件              │
     ├──────┼────────────────────────────────┤
     │ 新建 │ APP/encoder.h                  │
     ├──────┼────────────────────────────────┤
     │ 新建 │ APP/encoder.c                  │
     ├──────┼────────────────────────────────┤
     │ 修改 │ Core/Src/freertos.c (集成调用) │
     └──────┴────────────────────────────────┘
     ---
     验收测试

     1. 静态测试：轮子不动，speed_mps ≈ 0
     2. 方向测试：向前推车 speed > 0，向后 speed < 0
     3. 精度测试：轮子转 N 圈，对比理论速度与遥测值
     4. 溢出测试：计数器跨 0xFFFF→0x0000 时速度仍正确
     5. VOFA+ 波形：观察速度曲线平滑度

     ---
     实现步骤

     1. 创建 APP/encoder.h — API 定义
     2. 创建 APP/encoder.c — 核心实现
     3. 修改 freertos.c — 集成到 MainControlTask
     4. 编译验证
     5. 动态测试（方向、精度）