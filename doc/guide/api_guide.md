# API 使用指南（components & APP 层）

> 本文档面向初学者，讲解本工程中 `components/` 和 `APP/` 目录下的主要 API，帮助你快速理解如何使用这些模块。

## 目录

1. [架构概述](#1-架构概述)
2. [components 层（底层驱动/算法）](#2-components-层底层驱动算法)
   - [2.1 IMU SPI 驱动](#21-imu-spi-驱动-componentsimuimu_spih)
   - [2.2 IMU 互补滤波](#22-imu-互补滤波-componentsimuimu_filterh)
   - [2.3 环形缓冲](#23-环形缓冲-componentscommringbufh)
   - [2.4 VOFA JustFloat](#24-vofa-justfloat-componentscommvofa_justfloath)
   - [2.5 OLED SSD1306](#25-oled-ssd1306-componentsoled)
3. [APP 层（应用封装）](#3-app-层应用封装)
   - [3.1 IMU 应用接口](#31-imu-应用接口-appimuh)
   - [3.2 串口 Echo](#32-串口-echo-appuart_echoh)
   - [3.3 VOFA 遥测](#33-vofa-遥测-appvofa_telemetryh)
   - [3.4 OLED UI](#34-oled-ui-appoled_uih)
   - [3.5 状态存储](#35-状态存储-appstatus_storeh)
   - [3.6 全局数据结构](#36-全局数据结构-appmydefineh)
   - [3.7 编码器驱动](#37-编码器驱动-appencoderh)
4. [典型使用流程](#4-典型使用流程)

---

## 1. 架构概述

本工程采用**分层架构**：

```
┌─────────────────────────────────────────┐
│              任务层 (FreeRTOS)           │
│   MainControlTask / CommTask / OLEDTask │
└───────────────────┬─────────────────────┘
                    │ 调用
┌───────────────────▼─────────────────────┐
│              APP 层（应用封装）           │
│   imu.h / uart_echo.h / oled_ui.h ...  │
│   - 绑定 HAL 句柄与 components          │
│   - 路由 HAL 回调到 components          │
│   - 提供简化的上层 API                   │
└───────────────────┬─────────────────────┘
                    │ 调用
┌───────────────────▼─────────────────────┐
│          components 层（可复用组件）      │
│   imu_spi.h / imu_filter.h / ringbuf.h │
│   - 与具体外设解耦（通过参数传入句柄）     │
│   - 可移植到其他项目                     │
└─────────────────────────────────────────┘
```

**设计原则**：
- **components 层**：不依赖具体硬件配置（CubeMX 生成的 `hspi1`、`huart3` 等），通过参数/结构体传入。
- **APP 层**：绑定本项目的具体外设句柄，并把 HAL 回调分发到 components。

---

## 2. components 层（底层驱动/算法）

### 2.1 IMU SPI 驱动 (`components/imu/imu_spi.h`)

#### 功能

ICM20602 六轴 IMU 的 SPI + DMA 双缓冲驱动，支持 DRDY 中断触发采样。

#### 核心数据结构

```c
// 原始采样数据（未做比例换算）
typedef struct {
    int16_t ax, ay, az;    // 加速度原始值
    int16_t temp;          // 温度原始值
    int16_t gx, gy, gz;    // 陀螺仪原始值
    uint32_t timestamp_ms; // 采样时间戳
} icm20602_raw_sample_t;

// SPI 驱动上下文（含双缓冲）
typedef struct {
    SPI_HandleTypeDef *hspi;    // SPI 句柄
    GPIO_TypeDef *cs_port;      // 片选端口
    uint16_t cs_pin;            // 片选引脚
    // ... 内部状态（DMA 标志、缓冲区等）
} icm20602_spi_t;
```

#### 主要 API

| 函数 | 说明 | 调用上下文 |
|------|------|-----------|
| `icm20602_spi_bind(dev, hspi, cs_port, cs_pin)` | 绑定 SPI 句柄和片选引脚 | 初始化时 |
| `icm20602_spi_init_500hz_drdy(dev)` | 初始化为 500Hz ODR + DRDY 中断 | 初始化时 |
| `icm20602_spi_on_drdy_isr(dev)` | DRDY 中断处理：启动 DMA 采样 | **ISR 中** |
| `icm20602_spi_on_txrx_cplt_isr(dev, hspi)` | SPI DMA 完成回调 | **ISR 中** |
| `icm20602_spi_take_latest(dev, out)` | 获取最新一帧数据（非阻塞） | 任务中 |

#### 使用示例

```c
// 1. 定义驱动实例
static icm20602_spi_t s_imu;

// 2. 初始化
icm20602_spi_bind(&s_imu, &hspi1, CS_GPIO_Port, CS_Pin);
if (icm20602_spi_init_500hz_drdy(&s_imu) != ICM20602_SPI_OK) {
    // 初始化失败处理
}

// 3. 在 EXTI 回调中触发采样
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == IMU_INT_Pin) {
        icm20602_spi_on_drdy_isr(&s_imu);
    }
}

// 4. 在 SPI DMA 完成回调中处理
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    icm20602_spi_on_txrx_cplt_isr(&s_imu, hspi);
    // 唤醒任务...
}

// 5. 任务中读取数据
icm20602_raw_sample_t raw;
if (icm20602_spi_take_latest(&s_imu, &raw)) {
    // 有新数据，处理 raw.ax, raw.gx 等
}
```

#### 关键概念：DMA 双缓冲

```
时间线:  ──────────────────────────────────────>
         │ DRDY │        │ DRDY │
         ↓      ↓        ↓      ↓
DMA:     [写 buf[0]]     [写 buf[1]]
任务:           [读 buf[0]]      [读 buf[1]]
```

- DMA 写入 `buf[active]`，任务读取 `buf[ready]`，避免数据撕裂。

---

### 2.2 IMU 互补滤波 (`components/imu/imu_filter.h`)

#### 功能

将加速度计和陀螺仪数据融合，输出稳定的俯仰角（pitch）。

#### 核心概念

**为什么需要融合？**
- **加速度计**：可测量重力方向，得到绝对角度，但噪声大、受线性加速度影响。
- **陀螺仪**：测量角速度，积分得角度，短期平滑但长期漂移。
- **互补滤波**：高频信任陀螺，低频信任加速度，取两者优点。

```
pitch = alpha * (pitch + gyro * dt) + (1 - alpha) * pitch_acc
         └── 陀螺积分 ──┘             └── 加速度角度 ──┘
```

#### 主要 API

| 函数 | 说明 |
|------|------|
| `imu_comp_filter_init(f, cfg, initial_pitch)` | 初始化滤波器，设置 alpha 和初始角度 |
| `imu_comp_filter_update_pitch_deg(f, gyro_dps, pitch_acc_deg, dt_s)` | 基础互补滤波更新 |
| `imu_comp_filter_update_pitch_deg_gated(...)` | **带线加速度门控**的互补滤波 |

#### 使用示例

```c
// 初始化
imu_comp_filter_t filter;
imu_comp_filter_cfg_t cfg = { .alpha = 0.98f };  // 98% 信任陀螺
imu_comp_filter_init(&filter, cfg, 0.0f);

// 每帧更新（dt = 0.002s = 2ms）
float pitch = imu_comp_filter_update_pitch_deg(&filter,
                                                gyro_x_dps,   // 陀螺角速度
                                                pitch_from_acc, // 加速度计算的角度
                                                0.002f);
```

#### 线加速度门控

当小车急加速/急刹/飞坡时，加速度计读数不准（包含线性加速度）。此时应暂时只用陀螺：

```c
float pitch = imu_comp_filter_update_pitch_deg_gated(&filter,
    gyro_x_dps,
    pitch_from_acc,
    dt_s,
    acc_norm_g,        // sqrt(ax^2+ay^2+az^2)，正常应≈1.0
    0.1f,              // 阈值：|acc_norm - 1| > 0.1 则触发门控
    100,               // 门控持续时间 100ms
    HAL_GetTick());
```

---

### 2.3 环形缓冲 (`components/comm/ringbuf.h`)

#### 功能

单生产者/单消费者的无锁环形缓冲，适合 ISR 写、任务读的场景。

#### 主要 API

| 函数 | 说明 |
|------|------|
| `comm_ringbuf_init(rb, storage, size)` | 初始化，绑定存储区 |
| `comm_ringbuf_available(rb)` | 返回可读字节数 |
| `comm_ringbuf_free(rb)` | 返回剩余可写空间 |
| `comm_ringbuf_write(rb, data, len)` | 写入数据（满则丢弃） |
| `comm_ringbuf_read(rb, out, len)` | 读取数据 |

#### 使用示例

```c
// 定义存储区和缓冲结构
static uint8_t s_buf[256];
static comm_ringbuf_t s_rb;

// 初始化
comm_ringbuf_init(&s_rb, s_buf, sizeof(s_buf));

// ISR 中写入
void UART_RxCallback(uint8_t *data, uint16_t len) {
    comm_ringbuf_write(&s_rb, data, len);
}

// 任务中读取
uint8_t tmp[64];
uint16_t n = comm_ringbuf_read(&s_rb, tmp, sizeof(tmp));
if (n > 0) {
    // 处理 tmp[0..n-1]
}
```

---

### 2.4 VOFA JustFloat (`components/comm/vofa_justfloat.h`)

#### 功能

打包 VOFA+ 上位机的 JustFloat 协议帧，用于实时波形显示。

#### 协议格式

```
[float ch0][float ch1]...[float chN][0x00 0x00 0x80 0x7F]
                                    └── 4 字节帧尾 ──┘
```

#### 主要 API

```c
// 打包 N 通道数据到缓冲区
size_t vofa_justfloat_pack(uint8_t *out,       // 输出缓冲
                           size_t out_len,     // 缓冲长度
                           const float *ch,    // 通道数据
                           size_t ch_count);   // 通道数
// 返回：实际写入字节数 = ch_count * 4 + 4
```

#### 使用示例

```c
float channels[3] = { pitch, roll, yaw };
uint8_t buf[64];
size_t len = vofa_justfloat_pack(buf, sizeof(buf), channels, 3);
HAL_UART_Transmit(&huart3, buf, len, 10);  // 发送到 VOFA+
```

---

### 2.5 OLED SSD1306 (`components/OLED/`)

#### 功能

0.96 寸 SSD1306 OLED 屏驱动（支持 I2C/SPI）。

#### 主要 API

| 函数 | 说明 |
|------|------|
| `ssd1306_Init()` | 初始化屏幕 |
| `ssd1306_Fill(color)` | 填充整屏（Black/White） |
| `ssd1306_UpdateScreen()` | 刷新显示（将缓冲写入屏幕） |
| `ssd1306_SetCursor(x, y)` | 设置光标位置 |
| `ssd1306_WriteString(str, font, color)` | 写字符串 |
| `ssd1306_DrawPixel(x, y, color)` | 画点 |
| `ssd1306_Line(x1, y1, x2, y2, color)` | 画线 |
| `ssd1306_DrawRectangle(...)` | 画矩形 |
| `ssd1306_FillRectangle(...)` | 填充矩形 |

#### 使用示例

```c
ssd1306_Init();
ssd1306_Fill(Black);               // 清屏
ssd1306_SetCursor(0, 0);
ssd1306_WriteString("Hello!", Font_7x10, White);
ssd1306_UpdateScreen();            // 刷新到屏幕
```

#### 可用字体

需在 `ssd1306_conf_template.h` 中启用：
- `Font_6x8`、`Font_7x10`、`Font_11x18`、`Font_16x26` 等

---

## 3. APP 层（应用封装）

### 3.1 IMU 应用接口 (`APP/imu.h`)

#### 功能

封装 `components/imu/` 的驱动，绑定本项目的 SPI1 和引脚，并路由 HAL 回调。

#### 主要 API

| 函数 | 说明 |
|------|------|
| `IMU_Init()` | 初始化 IMU（内部调用 `icm20602_spi_*`） |
| `IMU_TakeLatestRaw(out)` | 获取最新原始数据 |
| `IMU_DebugGetSpiState()` | 获取驱动内部状态（调试用） |

#### 线程标志

```c
#define IMU_THREAD_FLAG_NEW_SAMPLE (1u << 0)
```

任务可用 `osThreadFlagsWait()` 等待此标志，由 SPI DMA 完成时置位。

#### 使用示例（MainControlTask）

```c
// 初始化
if (IMU_Init() != IMU_OK) {
    // 处理错误
}

// 主循环
while (1) {
    osThreadFlagsWait(IMU_THREAD_FLAG_NEW_SAMPLE, osFlagsWaitAny, 5);

    icm20602_raw_sample_t raw;
    if (IMU_TakeLatestRaw(&raw)) {
        // 处理 raw 数据...
    }
}
```

---

### 3.2 串口 Echo (`APP/uart_echo.h`)

#### 功能

USART3 DMA + IDLE 串口收发，用于验证链路（收什么发什么）。

#### 主要 API

| 函数 | 说明 | 调用上下文 |
|------|------|-----------|
| `UART_Echo_StartRxDma(huart)` | 启动 RX DMA 接收 | 任务初始化 |
| `UART_Echo_TaskPump(huart)` | 处理收发（将 RX 数据回显） | 任务循环 |
| `UART_Echo_OnRxEventIsr(huart, size)` | RX 事件回调 | **ISR 中** |
| `UART_Echo_OnTxCpltIsr(huart)` | TX 完成回调 | **ISR 中** |

#### 线程标志

```c
#define UART_ECHO_FLAG_RX (1u << 4)  // RX 有新数据
#define UART_ECHO_FLAG_TX (1u << 5)  // TX 完成
```

#### 使用示例（CommTask）

```c
// 启动 RX
UART_Echo_StartRxDma(&huart3);

while (1) {
    // 等待 RX 或 TX 事件
    osThreadFlagsWait(UART_ECHO_FLAG_RX | UART_ECHO_FLAG_TX,
                      osFlagsWaitAny, osWaitForever);
    // 处理收发
    UART_Echo_TaskPump(&huart3);
}
```

---

### 3.3 VOFA 遥测 (`APP/vofa_telemetry.h`)

#### 功能

通过 VOFA+ JustFloat 协议发送 8 通道 IMU 数据，用于调参/观察波形。

#### 主要 API

```c
// 发送 8 通道数据：pitch、pitch_acc、3轴acc、3轴gyro
bool VOFA_SendPitch2Speed2_Dma(UART_HandleTypeDef *huart,
                               float pitch_deg,
                               float pitch_acc_deg,
                               float wheel_l_mps,
                               float wheel_r_mps);
// 返回：true=发送成功，false=TX 忙（丢帧）
```

---

### 3.4 OLED UI (`APP/oled_ui.h`)

#### 功能

封装 OLED 显示逻辑，显示 pitch 角度和系统状态。

#### 主要 API

```c
void OledUi_Init(void);                    // 初始化 OLED
void OledUi_Render(const StatusData_t *s); // 渲染一帧状态
```

#### 显示布局

```
PITCH:+12.34deg
M:1  F:0003
```

- `M`：mode（0=STOP, 1=RUN, 2=ESTOP）
- `F`：fault_bits（故障码）

---

### 3.5 状态存储 (`APP/status_store.h`)

#### 功能

**单写者/多读者**的最新状态快照存储，无需互斥锁。

#### 原理

使用序列号（sequence counter）检测读写冲突：
```
写入: seq++; 写数据; seq++;  // seq 变化表示正在写
读取: do { s1=seq; 读数据; s2=seq; } while (s1!=s2 || s1&1);
```

#### 主要 API

```c
void StatusStore_Write(const StatusData_t *s);  // 写入最新状态
bool StatusStore_Read(StatusData_t *out);       // 读取最新状态
```

#### 使用场景

```
MainControlTask (写)           CommTask / OLEDTask (读)
      │                              │
      ├── 计算 pitch ──┐             │
      │                │             │
      └── StatusStore_Write() ◄─────┼── StatusStore_Read()
                                     │
```

---

### 3.6 全局数据结构 (`APP/mydefine.h`)

#### HostCommand_t（上位机指令）

```c
typedef struct {
    float target_speed_mps;      // 目标速度 (m/s)
    float target_yaw_rate_dps;   // 目标偏航角速度 (deg/s)
    uint8_t mode;                // 0=STOP, 1=RUN, 2=ESTOP
} HostCommand_t;
```

#### StatusData_t（系统状态）

```c
typedef struct {
    uint32_t timestamp_ms;   // 时间戳
    float pitch_deg;         // 融合后的 pitch
    float pitch_acc_deg;     // 未融合的 pitch（对比用）
    float ax_g, ay_g, az_g;  // 加速度 (g)
    float gx_dps, gy_dps, gz_dps; // 角速度 (deg/s)
    float speed_mps;         // 速度 (m/s)
    int16_t pwm_l, pwm_r;    // 左右电机 PWM
    uint8_t mode;            // 当前模式
    uint16_t fault_bits;     // 故障位
} StatusData_t;
```

#### fault_bits 定义

| Bit | 含义 |
|-----|------|
| bit0 | IMU 超时 |
| bit1 | IMU 初始化失败 |
| ... | 待扩展 |

---

### 3.7 编码器驱动 (`APP/encoder.h`)

#### 功能

双轮正交编码器驱动，使用 TIM3/TIM4 的编码器接口模式，测量轮速用于平衡控制和速度反馈。

#### 硬件配置

| 外设 | 引脚 | 说明 |
|------|------|------|
| TIM4 | PD12=CH1, PD13=CH2 | **左轮**编码器 |
| TIM3 | PA6=CH1, PA7=CH2 | **右轮**编码器 |
| 模式 | TIM_ENCODERMODE_TI12 | 4 倍频计数 |

#### 编码器参数

| 参数 | 值 | 说明 |
|------|-----|------|
| PPR | 500 | 编码器线数（电机轴） |
| 减速比 | 1:28 | 电机轴 → 输出轴 |
| 轮径 | 65mm | 轮子直径 |
| 输出轴每转计数 | 56000 | 500 × 4 × 28 |

#### 核心数据结构

```c
// 配置参数
typedef struct {
    uint16_t ppr;              // 编码器 PPR (电机轴)
    float wheel_diameter_m;    // 轮径 (米)
    float gear_ratio;          // 减速比
    float lpf_alpha;           // 低通滤波系数 (0-1)
    bool left_inverted;        // 左轮方向取反
    bool right_inverted;       // 右轮方向取反
} encoder_config_t;

// 单轮数据
typedef struct {
    int32_t delta_counts;       // 本周期计数差（有符号）
    float speed_mps;            // 原始速度 (m/s)
    float speed_filtered_mps;   // 滤波后速度 (m/s)
} encoder_data_t;

// 双轮读数
typedef struct {
    encoder_data_t left;        // 左轮 (TIM4)
    encoder_data_t right;       // 右轮 (TIM3)
    float avg_speed_mps;        // 左右平均速度
    uint32_t timestamp_ms;      // 时间戳
} encoder_reading_t;
```

#### 主要 API

| 函数 | 说明 |
|------|------|
| `Encoder_Init(cfg)` | 初始化编码器，启动 TIM3/TIM4 |
| `Encoder_Update(dt_s, out)` | 读取计数、计算速度、滤波 |
| `Encoder_GetLatest(out)` | 获取只读快照 |
| `Encoder_Reset()` | 重置计数器和滤波器 |
| `Encoder_SetConfig(cfg)` | 运行时更新配置 |

#### 关键算法

**16 位计数器溢出处理**：

利用有符号 16 位减法自动处理溢出：

```c
// 假设 previous=0xFFFE, current=0x0002
// 无符号差分: 0x0002 - 0xFFFE = 0x0004 (+4) ✓
// 假设 previous=0x0002, current=0xFFFE
// 无符号差分: 0xFFFE - 0x0002 = 0xFFFC (-4) ✓
int16_t delta = (int16_t)(current_cnt - last_cnt);
```

**速度计算**：

```
counts_per_wheel_rev = PPR × 4 × gear_ratio = 500 × 4 × 28 = 56000
counts_to_m = π × diameter / counts_per_wheel_rev ≈ 3.644e-6 m/count
speed_mps = delta_counts × counts_to_m / dt_s
```

**一阶低通滤波**：

```c
// y[n] = α × x[n] + (1-α) × y[n-1]
speed_filtered = lpf_alpha * speed_raw + (1 - lpf_alpha) * speed_filtered_prev;
```

#### 使用示例（MainControlTask）

```c
// 初始化
encoder_config_t enc_cfg = ENCODER_CONFIG_DEFAULT;
enc_cfg.lpf_alpha = 0.3f;  // 可调参
Encoder_Init(&enc_cfg);

encoder_reading_t enc_reading = {0};

// 主循环（每 2ms）
Encoder_Update(0.002f, &enc_reading);

// 更新状态用于遥测
status.speed_mps = enc_reading.avg_speed_mps;
status.wheel_l_mps = enc_reading.left.speed_filtered_mps;
status.wheel_r_mps = enc_reading.right.speed_filtered_mps;
```

#### 方向调整

如果推车时速度符号与预期相反（正应该前进却显示负），可以在初始化时取反：

```c
encoder_config_t cfg = ENCODER_CONFIG_DEFAULT;
cfg.left_inverted = true;   // 左轮反转
// cfg.right_inverted = true; // 右轮反转（根据实际情况）
Encoder_Init(&cfg);
```

#### 与 StatusData_t 的字段对应

```c
// StatusData_t 中新增的编码器相关字段
float speed_mps;       // avg_speed_mps (左右平均)
float wheel_l_mps;     // left.speed_filtered_mps
float wheel_r_mps;     // right.speed_filtered_mps
```

---

## 4. 典型使用流程

### 4.1 MainControlTask（500Hz 控制循环）

```c
void main_control_task(void *arg) {
    // 1. 初始化 IMU
    IMU_Init();

    // 2. 初始化滤波器
    imu_comp_filter_t filter;
    imu_comp_filter_init(&filter, (imu_comp_filter_cfg_t){.alpha=0.98f}, 0);

    // 3. 陀螺零偏标定（静止 500 帧取平均）
    float gyro_bias[3] = {0};
    for (int i = 0; i < 500; i++) {
        osThreadFlagsWait(IMU_THREAD_FLAG_NEW_SAMPLE, osFlagsWaitAny, 5);
        icm20602_raw_sample_t raw;
        if (IMU_TakeLatestRaw(&raw)) {
            gyro_bias[0] += raw.gx / 16.4f;
            gyro_bias[1] += raw.gy / 16.4f;
            gyro_bias[2] += raw.gz / 16.4f;
        }
    }
    gyro_bias[0] /= 500; gyro_bias[1] /= 500; gyro_bias[2] /= 500;

    // 4. 主循环
    while (1) {
        osThreadFlagsWait(IMU_THREAD_FLAG_NEW_SAMPLE, osFlagsWaitAny, 5);

        icm20602_raw_sample_t raw;
        if (!IMU_TakeLatestRaw(&raw)) continue;

        // 单位换算
        float ax = raw.ax / 4096.0f;  // ±8g 量程
        float gx = raw.gx / 16.4f - gyro_bias[0];  // ±2000dps 量程

        // 互补滤波
        float pitch_acc = atan2f(raw.ay, raw.az) * 57.3f;
        float pitch = imu_comp_filter_update_pitch_deg(&filter, gx, pitch_acc, 0.002f);

        // 发布状态
        StatusData_t status = { .pitch_deg = pitch, ... };
        StatusStore_Write(&status);
    }
}
```

### 4.2 OLEDDisplayTask（100ms 显示刷新）

```c
void oled_display_task(void *arg) {
    OledUi_Init();

    while (1) {
        StatusData_t s;
        if (StatusStore_Read(&s)) {
            OledUi_Render(&s);
        }
        osDelay(100);
    }
}
```

---

## 附录：量程与比例系数速查

| 传感器 | 量程 | 比例系数 | 公式 |
|--------|------|----------|------|
| 加速度计 | ±8g | 4096 LSB/g | `acc_g = raw / 4096.0f` |
| 陀螺仪 | ±2000 dps | 16.4 LSB/(deg/s) | `gyro_dps = raw / 16.4f` |

> 注意：量程可在 IMU 寄存器中配置，修改后需同步更新比例系数。
