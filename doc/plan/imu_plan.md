# IMU 开发任务计划书（v1）

> 目标：在 `MainControlTask@2ms(500Hz)` 架构下，完成 IMU 的可靠采集（SPI DMA + 可选 DRDY）、标定与互补滤波输出，为平衡控制与上位机遥测提供稳定状态量。

## 0. 前置条件与约束

- MCU：STM32F407ZGT6，系统时钟 168MHz。
- IMU 接口（来自 `doc/basic_info.md`）：
  - SPI1：`PA5(SCK) / PB4(MISO) / PB5(MOSI)`，`SPI_BAUDRATEPRESCALER_8`，约 `10.5Mbit/s`
  - CS：`PB12`（GPIO 输出，上电默认 High）
  - INT：`PC5`（EXTI5，上升沿，内部下拉；通常用作 DRDY）
  - DMA：SPI1_RX `DMA2_Stream0`，SPI1_TX `DMA2_Stream3`（Normal）
- 采样目标：IMU ODR=500Hz，与控制周期 2ms 对齐。
- RTOS：FreeRTOS（CMSIS-RTOS v2 wrapper），Tick 1kHz。
- 中断优先级：当前 SPI/DMA/EXTI 优先级均为 5，满足 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5` 的 FromISR 调用约束。

## 1. 交付物（Deliverables）

1. IMU 驱动层（SPI 读写、寄存器初始化、burst 读取、错误码/状态）。
2. IMU 采集流水线（DMA + 双缓冲，可选 DRDY 触发），输出时间戳对齐的 `acc/gyro`。
3. 标定与补偿（至少：陀螺零偏；可选：加速度零偏/比例）。
4. 互补滤波输出（至少：`pitch`；可选：`roll`），并完成轴向映射/符号约定。
5. 遥测支持（按 `doc/plan/plan_v2.md` 协议：`STATUS` 默认，`IMU_RAW` 可选）。
6. 验证脚本/用例说明（如何测频率、抖动、数据一致性、异常恢复）。

## 2. 设计决策（推荐实现路径）

### 2.1 采集触发方式

优先级从高到低：

- **A. DRDY(EXTI) 定拍（推荐）**：IMU 产生 500Hz DRDY → EXTI ISR 唤醒任务/置位标志 → 主循环取“上一帧已完成数据”计算，并启动下一帧 DMA。
- **B. 定时器定拍**：用 TIM 产生 2ms 中断/节拍通知，IMU 不接 DRDY 或 DRDY 不稳定时采用。
- **C. 轮询**：仅用于 bring-up 阶段（验证 SPI 通道和寄存器）。

### 2.2 主循环“不可阻塞”原则

- `MainControlTask@2ms` 内禁止等待 DMA 完成、禁止 `xQueueReceive` 阻塞。
- 使用“双缓冲 + 状态机”：
  - `dma_in_flight`：上一帧 DMA 是否完成
  - `buf_ready`：哪一帧数据已完成可用
  - 主循环永远使用“最新可用快照”，并启动下一次 DMA（若空闲）。

### 2.3 RTOS 同步方式

二选一（保持全工程一致即可）：

- **FreeRTOS task notification**：`xTaskNotifyFromISR()` / `vTaskNotifyGiveFromISR()`，任务侧 `ulTaskNotifyTake()` 或 `xTaskNotifyWait()`。
- **CMSIS-RTOS2 thread flags**：`osThreadFlagsSet()`（ISR）+ `osThreadFlagsWait()`（任务）。

建议：若你后续大多直接用 FreeRTOS 原生 API，就用 task notify。

## 3. 工作分解（WBS）与里程碑

### M0：硬件连通性确认（0.5 天）
- 确认 IMU 供电 3.3V、地线回流与去耦（模块 VCC/GND，SPI 线序，CS 默认 High）。
- 用逻辑分析仪/示波器确认 `CS/SCK/MOSI` 波形和频率；确认 `PC5` 是否有 DRDY 脉冲。

验收：SPI 能稳定读到 `WHO_AM_I`（或芯片 ID 寄存器），多次读取一致。

### M1：SPI 轮询读写 & 寄存器初始化（1 天）
- 实现：
  - `imu_spi_read(reg)` / `imu_spi_write(reg,val)`
  - `imu_burst_read(reg, buf, len)`
  - `imu_reset()` / `imu_init(odr=500Hz, fsr配置, LPF配置)`
- 约定寄存器读写格式（MSB=1 读等细节按具体 IMU 型号）。

验收：能读出连续的 `acc/gyro` 原始值；静止时 `gyro` 接近常数，倾斜时 `acc` 重力分量变化符合预期。

### M2：SPI DMA + 双缓冲（1–2 天）
- 用 `HAL_SPI_TransmitReceive_DMA()` 完成一次 burst。
- 设计双缓冲：`rx_buf[2]`、`tx_buf[2]`（或同一 tx 反复发送地址+dummy）。
- 在 `HAL_SPI_TxRxCpltCallback()` 中仅做：标记完成、记录时间戳（可选）、必要时通知任务（FromISR）。

验收：DMA 连续运行无卡死；500Hz 读数稳定；主循环无阻塞等待。

### M3：DRDY/定拍触发与节拍对齐（0.5–1 天）
- 若使用 DRDY：在 `HAL_GPIO_EXTI_Callback()` 中发通知/置位。
- 若使用 TIM：在定时器 ISR 中发通知/置位。
- 主循环对齐：每次节拍到来取最新样本；若样本未更新则计数并触发故障策略。

验收：统计 1 秒内样本数≈500；记录“丢样计数”为 0 或极低（可接受阈值需定义）。

### M4：标定与补偿（1 天）
- 陀螺零偏：上电静止 N=1000 点平均（2 秒）得到 bias。
- 可选：加速度计静态标定（六面法/单面近似）后续再做。
- 输出数据：
  - `gyro_dps = (raw - bias) * scale`
  - `acc_g = raw * scale`

验收：静止时 `gyro` 均值接近 0；温漂/零偏变化有可视化（遥测）。

### M5：互补滤波（0.5–1 天）
- 明确轴映射：你当前规划“X 轴为 pitch 轴”（即前后俯仰绕 X 旋转）。
- 实现互补：
  - `pitch_gyro += gyro_x * dt`
  - `pitch_acc = atan2(acc_?, acc_?)`（按坐标系选择）
  - `pitch = alpha * pitch_gyro + (1-alpha) * pitch_acc`
- `dt` 用 2ms 常数或用时间戳计算（建议先常数，后续再做精细）。

验收：手动前倾/后仰时 `pitch` 单调变化且符号正确；无明显发散。

### M6：与控制闭环集成（1–2 天）
- 把 `pitch/gyro` 接入控制器（PID/LQR 等）。
- 加入保护：
  - IMU 丢样超时（例如 > 6ms）→ 目标置零/输出限幅/进入 fault
  - 姿态超角（例如 |pitch| > 阈值）→ 进入安全停机

验收：在空载/支架上能稳定闭环；落地可短时间站立并可控。

## 4. 数据接口（建议结构）

- 原始：`acc_raw[3]`, `gyro_raw[3]`, `timestamp_us/ms`
- 物理量：`acc_g[3]`, `gyro_dps[3]`
- 姿态：`pitch_deg`, `roll_deg`（可选）
- 质量指标：`sample_counter`, `drop_counter`, `last_update_ts`, `fault_bits`

建议由 `MainControlTask` 产出“最新快照”，供 `CommTask/OLEDTask` 读取（避免队列积压）。

## 5. 测试与测量（必须做）

- **频率与抖动**：在 IMU DMA 完成回调或主循环入口翻转 GPIO，示波器看 500Hz/2ms 是否稳定。
- **丢样与重同步**：故意拔插 IMU（或拉高/拉低 CS/INT）验证 fault 策略是否生效。
- **噪声与振动**：电机空转与地面运行时对比 `gyro` 噪声，必要时调整 LPF/互补系数。

## 6. 风险清单与对策

- DRDY 线未接/电平不对：先用轮询/定时器定拍实现闭环，再回头接 DRDY。
- 线序/轴向不一致：用“轴交换/取反”软件层解决，避免反复改机械安装。
- DMA 回调中误用非 FromISR API：严格只做置位/通知，不做耗时计算。
- 电机噪声影响 IMU：布局隔离、供电去耦、必要时对 IMU 板做轻微减振。

## 7. Definition of Done（完成标准）

- IMU 以 500Hz 稳定输出（丢样率可量化并低于阈值）。
- `pitch` 与 `gyro_pitch` 符号/轴向正确，互补滤波输出稳定不发散。
- 主循环无阻塞等待，周期抖动满足控制需求。
- 串口能上报 `STATUS`，并可按需开启 `IMU_RAW` 用于调试。
- 断线/异常触发时进入安全状态（PWM 受控停机）。

---

## 8. 文件结构与位置

### 8.1 需要创建的文件

| 文件路径 | 位置 | 用途 |
|---------|------|------|
| `components/imu/imu.h` | components | IMU 驱动头文件（API 声明、数据结构） |
| `components/imu/imu.c` | components | IMU 驱动实现（SPI 读写、DMA 采集、标定） |
| `components/imu/imu_spi.h` | components | SPI 底层操作头文件 |
| `components/imu/imu_spi.c` | components | SPI 底层操作实现 |
| `components/imu/imu_filter.h` | components | 滤波与姿态解算头文件 |
| `components/imu/imu_filter.c` | components | 互补滤波、姿态角计算实现 |
| `APP/imu_app.h` | APP | 应用层接口（可选，简化调用） |
| `APP/imu_app.c` | APP | 应用层封装（任务集成） |

### 8.2 需修改的现有文件

| 文件 | 修改内容 |
|-----|---------|
| `Core/Inc/stm32f4xx_it.h` | 声明 `HAL_GPIO_EXTI_Callback` 或 IMU 中断处理 |
| `Core/Src/stm32f4xx_it.c` | 在 `EXTI9_5_IRQHandler` 中调用 IMU DRDY 处理 |
| `Core/Inc/spi.h` | 如需暴露 SPI 句柄给 IMU 驱动 |
| `Core/Inc/gpio.h` | 确认 `IMU_CS_Pin`、`IMU_INT_Pin` 宏定义 |
| `Core/Src/gpio.c` | 初始化 `IMU_CS` 为 High 输出 |
| `APP/mydefine.h` | 添加 `imu.h`、`imu_filter.h`、`imu_app.h` 包含 |
| `Core/Src/freertos.c` | 创建 MainControlTask 或在任务中调用 IMU API |

---

## 9. 数据结构定义（imu.h）

```c
/* ============================================
 * IMU 驱动数据结构定义
 * ============================================ */

// ==================== 故障与状态 ====================

// IMU 故障位定义
#define IMU_FAULT_SPI       (1 << 0)   // SPI 通信错误
#define IMU_FAULT_TIMEOUT   (1 << 1)   // 采样超时
#define IMU_FAULT_DRDY      (1 << 2)   // DRDY 信号异常
#define IMU_FAULT_INIT      (1 << 3)   // 初始化失败
#define IMU_FAULT_CALIB     (1 << 4)   // 标定失败

// 标定状态
#define IMU_CALIB_NONE      0
#define IMU_CALIB_DONE      1
#define IMU_CALIB_BUSY      2

// ==================== 传感器数据 ====================

// 原始数据（寄存器直接读取）
typedef struct {
    int16_t acc[3];      // X, Y, Z 加速度原始值 (16-bit signed)
    int16_t gyro[3];     // X, Y, Z 陀螺仪原始值 (16-bit signed)
    uint32_t timestamp;  // 采集时间戳 (微秒)
} imu_raw_data_t;

// 物理量转换后的数据
typedef struct {
    float acc_g[3];      // X, Y, Z 加速度 (g, float)
    float gyro_dps[3];   // X, Y, Z 角速度 (deg/s, float)
    uint32_t timestamp;  // 采集时间戳 (微秒)
} imu_sensor_data_t;

// IMU 姿态输出
typedef struct {
    float pitch_deg;     // 俯仰角 (deg) - 绕 X 轴旋转
    float roll_deg;      // 翻滚角 (deg) - 绕 Y 轴旋转（可选）
    float yaw_deg;       // 偏航角 (deg) - 绕 Z 轴旋转（可选）
    float pitch_rate;    // 俯仰角速度 (deg/s) - gyro_x
} imu_attitude_data_t;

// ==================== 配置与状态 ====================

// IMU 配置参数
typedef struct {
    uint16_t odr_hz;         // 输出数据率 (默认 500)
    uint16_t gyro_fsr_dps;   // 陀螺仪量程 (dps, 如 250/500/1000/2000)
    uint16_t acc_fsr_g;      // 加速度计量程 (g, 如 2/4/8/16)
    uint8_t  lpf_bandwidth;  // LPF 带宽等级 (按 IMU 型号)
    uint8_t  use_drdy;       // 0=定时器触发, 1=DRDY 触发
} imu_config_t;

// IMU 运行时状态
typedef struct {
    // 状态标志
    uint8_t initialized;     // 初始化完成标志
    uint8_t calib_state;     // 标定状态
    uint32_t fault_flags;    // 故障位 (OR of IMU_FAULT_*)

    // 计数统计
    uint32_t sample_count;   // 采样总数
    uint32_t drop_count;     // 丢样计数
    uint32_t error_count;    // 错误计数

    // 最新快照（主循环只读）
    imu_sensor_data_t sensor;
    imu_attitude_data_t attitude;

    // 标定数据
    float gyro_bias[3];      // 陀螺仪零偏 (dps)
    float acc_bias[3];       // 加速度零偏 (g, 可选)
    float acc_scale[3];      // 加速度比例 (可选)

    // DMA 状态
    uint8_t current_buf;     // 当前 DMA 缓冲区索引 (0/1)
    uint8_t data_ready;      // 新数据就绪标志
} imu_handle_t;

// 外部全局变量声明
extern imu_handle_t imu;
```

---

## 10. API 接口定义（imu.h）

```c
/* ============================================
 * IMU 驱动公共 API 声明
 * ============================================ */

// ==================== 初始化与配置 ====================

/**
 * @brief  IMU 初始化
 * @param  config: 配置参数指针，为 NULL 则使用默认配置
 * @retval 0=成功, 负值=错误码
 */
int8_t IMU_Init(const imu_config_t *config);

/**
 * @brief  IMU 反初始化
 * @retval 无
 */
void IMU_DeInit(void);

/**
 * @brief  设置 IMU 配置
 * @param  config: 配置参数指针
 * @retval 0=成功, 负值=错误码
 */
int8_t IMU_SetConfig(const imu_config_t *config);

/**
 * @brief  获取当前配置
 * @param  config: 输出配置参数
 * @retval 无
 */
void IMU_GetConfig(imu_config_t *config);

// ==================== 标定 ====================

/**
 * @brief  陀螺仪零偏标定
 * @param  samples: 采样点数（建议 1000，对应 2 秒@500Hz）
 * @retval 0=成功, 负值=错误码
 */
int8_t IMU_CalibrateGyro(uint16_t samples);

/**
 * @brief  加速度计标定（可选）
 * @param  samples: 采样点数
 * @retval 0=成功, 负值=错误码
 */
int8_t IMU_CalibrateAccel(uint16_t samples);

/**
 * @brief  加载标定数据
 * @param  gyro_bias: 陀螺仪零偏数组 [dps]
 * @retval 无
 */
void IMU_LoadCalibration(const float gyro_bias[3]);

/**
 * @brief  保存标定数据到 Flash（可选）
 * @retval 0=成功, 负值=错误码
 */
int8_t IMU_SaveCalibration(void);

// ==================== 数据采集（非阻塞） ====================

/**
 * @brief  启动一次采样（DMA）
 * @retval 0=成功启动, 1=上一次未完成, 负值=错误码
 */
int8_t IMU_StartSampling(void);

/**
 * @brief  检查新数据是否就绪
 * @retval 1=有新数据, 0=无新数据
 */
uint8_t IMU_IsDataReady(void);

/**
 * @brief  获取最新传感器数据（物理量）
 * @param  data: 输出数据结构指针
 * @retval 无
 */
void IMU_GetSensorData(imu_sensor_data_t *data);

// ==================== 姿态解算 ====================

/**
 * @brief  更新姿态角（互补滤波）
 * @retval 无
 */
void IMU_UpdateAttitude(void);

/**
 * @brief  获取当前姿态
 * @param  attitude: 输出姿态数据结构指针
 * @retval 无
 */
void IMU_GetAttitude(imu_attitude_data_t *attitude);

// ==================== 状态查询 ====================

/**
 * @brief  获取采样计数
 * @retval 采样总数
 */
uint32_t IMU_GetSampleCount(void);

/**
 * @brief  获取丢样计数
 * @retval 丢样总数
 */
uint32_t IMU_GetDropCount(void);

/**
 * @brief  获取故障标志
 * @retval 故障位 (OR of IMU_FAULT_*)
 */
uint32_t IMU_GetFaultFlags(void);

/**
 * @brief  检查 IMU 是否健康
 * @retval 1=健康, 0=故障
 */
uint8_t IMU_IsHealthy(void);

// ==================== 测试与调试 ====================

/**
 * @brief  读取单个寄存器
 * @param  reg: 寄存器地址
 * @param  data: 输出数据指针
 * @retval 0=成功, 负值=错误码
 */
int8_t IMU_ReadReg(uint8_t reg, uint8_t *data);

/**
 * @brief  写入单个寄存器
 * @param  reg: 寄存器地址
 * @param  data: 写入值
 * @retval 0=成功, 负值=错误码
 */
int8_t IMU_WriteReg(uint8_t reg, uint8_t data);

/**
 * @brief  自检
 * @retval 0=通过, 负值=失败
 */
int8_t IMU_SelfTest(void);
```

---

## 11. 内部函数与回调（仅 imu.c 使用）

```c
/* ============================================
 * 内部函数声明（不暴露给外部）
 * ============================================ */

// ==================== SPI 底层操作 ====================

/**
 * @brief  读取单个字节
 * @param  reg: 寄存器地址
 * @param  data: 输出数据指针
 * @retval 0=成功, 负值=错误码
 */
int8_t imu_spi_read_byte(uint8_t reg, uint8_t *data);

/**
 * @brief  写入单个字节
 * @param  reg: 寄存器地址
 * @param  data: 写入值
 * @retval 0=成功, 负值=错误码
 */
int8_t imu_spi_write_byte(uint8_t reg, uint8_t data);

/**
 * @brief  突发读取多个字节
 * @param  start_reg: 起始寄存器地址
 * @param  buf: 输出缓冲区
 * @param  len: 读取长度
 * @retval 0=成功, 负值=错误码
 */
int8_t imu_spi_burst_read(uint8_t start_reg, uint8_t *buf, uint16_t len);

// ==================== DMA 相关 ====================

/**
 * @brief  启动 DMA burst 传输
 * @retval 0=成功, 负值=错误码
 */
int8_t imu_dma_start_burst(void);

/**
 * @brief  切换双缓冲
 * @retval 无
 */
void imu_dma_switch_buffer(void);

// ==================== HAL 回调 ====================

/**
 * @brief  SPI DMA 传输完成回调
 * @param  hspi: SPI 句柄
 * @retval 无
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

/**
 * @brief  SPI 错误回调
 * @param  hspi: SPI 句柄
 * @retval 无
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

/**
 * @brief  GPIO EXTI 中断回调（DRDY）
 * @param  GPIO_Pin: 引脚号
 * @retval 无
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// ==================== 初始化内部函数 ====================

/**
 * @brief  检查 WHO_AM_I 寄存器
 * @retval 0=成功, 负值=错误码
 */
int8_t imu_check_who_am_i(void);

/**
 * @brief  软件复位
 * @retval 0=成功, 负值=错误码
 */
int8_t imu_soft_reset(void);

/**
 * @brief  配置 IMU 寄存器
 * @param  config: 配置参数
 * @retval 0=成功, 负值=错误码
 */
int8_t imu_configure(const imu_config_t *config);

// ==================== 数据处理 ====================

/**
 * @brief  原始数据转换为物理量
 * @param  raw: 原始数据
 * @param  phys: 输出物理量数据
 * @retval 无
 */
void imu_raw_to_physical(const imu_raw_data_t *raw, imu_sensor_data_t *phys);

/**
 * @brief  应用标定补偿
 * @param  data: 传感器数据（原地修改）
 * @retval 无
 */
void imu_apply_calibration(imu_sensor_data_t *data);
```

---

## 12. 滤波与姿态解算 API（imu_filter.h）

```c
/* ============================================
 * 互补滤波器 API 声明
 * ============================================ */

// ==================== 配置 ====================

/**
 * @brief  互补滤波器配置
 */
typedef struct {
    float alpha;           // 陀螺仪权重 (0.9~0.99)
                           //   alpha = tau / (tau + dt)
                           //   tau = 融合时间常数 (通常 0.5~2s)
} imu_compliment_config_t;

/**
 * @brief  初始化互补滤波器
 * @param  config: 配置参数，为 NULL 使用默认值
 * @retval 无
 */
void IMU_Filter_Init(const imu_compliment_config_t *config);

/**
 * @brief  重置滤波器状态
 * @retval 无
 */
void IMU_Filter_Reset(void);

// ==================== 更新 ====================

/**
 * @brief  更新滤波器状态
 * @param  sensor: 传感器数据（加速度、陀螺仪）
 * @param  dt: 时间步长（秒）
 * @retval 无
 */
void IMU_Filter_Update(const imu_sensor_data_t *sensor, float dt);

// ==================== 获取结果 ====================

/**
 * @brief  获取姿态角
 * @param  pitch: 输出俯仰角 (deg)
 * @param  roll: 输出翻滚角 (deg)
 * @retval 无
 */
void IMU_Filter_GetAngle(float *pitch, float *roll);

/**
 * @brief  获取估计的零偏
 * @param  bias: 输出零偏数组 [dps]
 * @retval 无
 */
void IMU_Filter_GetBias(float *bias);

// ==================== 调试 ====================

/**
 * @brief  获取当前 alpha 值
 * @retval alpha 值
 */
float IMU_Filter_GetAlpha(void);
```

---

## 13. DMA 双缓冲设计

### 13.1 缓冲区定义

```c
// ==================== 缓冲区大小配置 ====================

#define IMU_BURST_SIZE   14   // 典型值：
                             //   ACCEL_XOUT_H/L (2)
                             //   ACCEL_YOUT_H/L (2)
                             //   ACCEL_ZOUT_H/L (2)
                             //   TEMP_OUT_H/L   (2)  // 可选
                             //   GYRO_XOUT_H/L  (2)
                             //   GYRO_YOUT_H/L  (2)
                             //   GYRO_ZOUT_H/L  (2)
                             // 根据实际 IMU 型号调整

// ==================== DMA 缓冲区结构 ====================

typedef struct {
    uint8_t tx[2][IMU_BURST_SIZE];   // TX: 发送寄存器地址 + dummy bytes
    uint8_t rx[2][IMU_BURST_SIZE];   // RX: 接收传感器数据
    uint8_t active_index;            // 当前正在填充的缓冲区 (0 或 1)
    uint8_t ready_index;             // 已完成的缓冲区
} imu_dma_buffers_t;

// 外部声明
extern imu_dma_buffers_t imu_dma;
```

### 13.2 缓冲区操作流程

```
1. 主循环/DRDY 中断:
   - 选择当前缓冲: idx = imu.dma.current_buf
   - 填充 TX[idx][0] = 读命令 | 0x80
   - 启动 DMA 传输

2. HAL_SPI_TxRxCpltCallback():
   - 标记 idx 为 ready
   - 切换 current_buf = 1 - idx
   - 置位 data_ready 标志
   - (可选) 发送 Task Notification

3. 主循环:
   - 检查 data_ready
   - 解析 ready 缓冲区的 rx 数据
   - 转换原始值 → 物理量
   - 应用标定补偿
   - 启动下一次 DMA
```

---

## 14. 轴向映射约定

### 14.1 物理轴定义

| 轴 | 符号 | 正方向定义 | 旋转 | 用途 |
|---|-----|----------|------|------|
| X | acc_x / gyro_x | 车头抬起为正 | Pitch | **主要控制轴** |
| Y | acc_y / gyro_y | 车身左侧抬起为正 | Roll | 可选 |
| Z | acc_z / gyro_z | 车头左转为正 | Yaw | 可选 |

### 14.2 姿态角计算

```c
// 俯仰角 (Pitch) - 绕 X 轴旋转
// 使用加速度计计算:
pitch_acc = atan2(acc_y, acc_z);  // 取决于安装方向

// 翻滚角 (Roll) - 绕 Y 轴旋转 (可选)
roll_acc = atan2(-acc_x, acc_z);  // 取决于安装方向
```

### 14.3 符号规则

```
右手坐标系:
  +X: 右手拇指指向车头
  +Y: 右手食指指向车身左侧
  +Z: 右手中指垂直向上

绕轴旋转正方向 (右手定则):
  +Pitch: 绕 X 轴，车头抬起
  +Roll: 绕 Y 轴，左侧抬起
  +Yaw: 绕 Z 轴，车头左转
```

---

## 15. 头文件包含与依赖关系

### 15.1 包含链

```
main.c
  └── mydefine.h
        ├── imu.h
        │     ├── imu_filter.h
        │     ├── spi.h
        │     └── stm32f4xx_hal.h
        ├── imu_app.h
        └── 其他模块...

imu.c
  ├── imu.h
  ├── imu_filter.h
  ├── imu_spi.h
  ├── spi.h
  └── stm32f4xx_hal.h

imu_filter.c
  ├── imu_filter.h
  └── imu.h (可选，共享数据类型)
```

### 15.2 外部依赖

| 依赖 | 来源 | 用途 |
|-----|------|------|
| `SPI_HandleTypeDef hspi1` | `Core/Src/spi.c` | SPI 通信 |
| `DMA_HandleTypeDef hdma_spi1_rx/tx` | `Core/Src/spi.c` | DMA 传输 |
| `GPIO_TypeDef* IMU_CS_GPIO_Port` | `Core/Src/gpio.c` | 片选控制 |
| `uint16_t IMU_CS_Pin` | `Core/Src/gpio.c` | 片选引脚 |
| `GPIO_TypeDef* IMU_INT_GPIO_Port` | `Core/Src/gpio.c` | DRDY 中断 |
| `uint16_t IMU_INT_Pin` | `Core/Src/gpio.c` | DRDY 引脚 |

---

## 16. 编译与链接注意事项

1. **栈大小**：`MainControlTask` 栈空间建议 >= 512 bytes
2. **内存**：双缓冲约 28 bytes (2×14)，几乎可忽略
3. **优化级别**：`-O1` 或 `-O2`，避免 `-Os`（影响浮点精度）
4. **对齐**：`imu_sensor_data_t` 应按 4 字节对齐
5. **FPU**：确保启用 `-mfloat-abi=hard -mfpu=fpv4-sp-d16`

---

## 17. 错误码定义

```c
// 错误码常量
#define IMU_OK             0
#define IMU_ERR            -1
#define IMU_ERR_INIT       -2
#define IMU_ERR_SPI        -3
#define IMU_ERR_TIMEOUT    -4
#define IMU_ERR_DRDY       -5
#define IMU_ERR_CALIB      -6
#define IMU_ERR_PARAM      -7
#define IMU_ERR_NO_DATA    -8
```

---

## 18. 验证清单

### 18.1 M0 验证项
- [ ] 读取 WHO_AM_I 寄存器正确
- [ ] SPI 时钟波形正常 (~10.5MHz)
- [ ] DRDY 引脚有 500Hz 脉冲（若接 DRDY）

### 18.2 M1 验证项
- [ ] 轮询读取 acc/gyro 原始值稳定
- [ ] 静止时 gyro 接近 0 (±5 LSB)
- [ ] 倾斜时 acc 重力分量正确

### 18.3 M2 验证项
- [ ] DMA 连续运行无卡死
- [ ] 500Hz 采样无丢数
- [ ] 双缓冲正确切换

### 18.4 M3 验证项
- [ ] 1 秒内采样数 ≈ 500
- [ ] 丢样计数 < 总数的 0.1%
- [ ] 周期抖动 < 50us

### 18.5 M4 验证项
- [ ] 静止时 gyro 均值接近 0
- [ ] 标定后零偏稳定

### 18.6 M5 验证项
- [ ] pitch 符号正确（前倾为正/负待确认）
- [ ] 互补滤波无发散
- [ ] 动态响应无振荡

### 18.7 M6 验证项
- [ ] 平衡控制闭环稳定
- [ ] IMU 异常时 PWM 进入安全状态
