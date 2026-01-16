# FreeRTOS 任务与通信规划方案 v2.0（基于讨论更新）

> 目标：降低控制链路延迟与抖动，把“采样→估计→控制→输出”收敛为单一高优先级闭环；通信/显示等异步任务降优先级，互不干扰。

## 一、任务架构设计（v2）

### 1.1 任务优先级分配

```
优先级 5 (最高)
└─ 传感器与控制主任务 (MainControlTask) - 周期: 2ms (500Hz)
   触发: TIM 定时 或 IMU DRDY 中断 task-notify
   内容: 读取IMU/编码器 → 姿态解算(互补滤波) → PID → PWM输出

优先级 3 (中)
└─ 通信任务 (CommTask)                 - 周期: 10ms
   内容: 解析上位机指令、发送遥测(telemetry)、调试输出

优先级 2 (低)
└─ OLED 显示任务 (OLEDDisplayTask)     - 周期: 100ms

优先级 1 (最低)
└─ 状态灯任务 (LEDTask)                - 周期: 500ms
```

### 1.2 关键实时性原则（MainControlTask）

- 主循环严格使用 `vTaskDelayUntil()` 或“中断通知节拍”驱动，保证固定周期。
- 主循环内禁止阻塞等待：禁止等待 SPI/DMA 完成、禁止等待队列、禁止长时间持锁。
- 采用“数据流水”思路：主循环使用“上一帧已完成的 IMU 数据”进行计算，同时启动下一帧 SPI DMA（建议双缓冲/指针交换）。
- 增加超时/异常保护：若连续 N 次未获得新 IMU 数据或检测到超周期，立即限幅/置零 PWM 并置故障位（防止飞车）。

## 二、传感器与外设配置约束

### 2.1 IMU 与 SPI

- IMU 输出频率（ODR）：500Hz（2ms 一帧）。
- SPI 时钟：10.5 Mbit/s。
- 传输量级（估算）：典型突发读取约 14–15 字节（6轴+温度+地址），纯传输时间约 12µs 量级；实际还包含 CS 翻转、DMA/中断开销，但相对 2ms 周期仍有较大裕量。

### 2.2 编码器

- 编码器计数读取：在主循环内直接读 TIM 寄存器（常数时间）。
- 速度估计：用固定采样周期 dt=2ms 或做简单一阶滤波；注意除零与溢出处理。

## 三、数据流与同步对象（v2）

v2 目标是避免“控制链路排队延迟”，因此不再把 IMU/Encoder 作为多深度队列在任务间传递。

### 3.1 推荐数据模型（最新值快照）

- `MainControlTask` 产生并维护全局最新状态（或深度=1 的 overwrite 队列）。
- `CommTask/OLEDTask` 仅读取状态快照，不反向影响主循环。

建议两种实现二选一：

1) **共享结构体 + 临界区/互斥**（更直接）
- `volatile`/结构体快照 + 轻量临界区（短到只拷贝结构体）。

2) **最新状态快照（单写者/多读者）**
- 主控任务每周期写入 `StatusStore`；通信/显示任务按需读取最新快照（不会互相“消费掉”数据）。

### 3.2 上位机命令输入

- `xHostCommandQueue` 深度 1（只保留最新目标）；通信任务解析后 overwrite；主任务在每次周期边界读取最新目标。

## 四、串口通信协议设计（v2 新增）

设计目标：二进制、小开销、易于 DMA 接收、丢包后可快速重同步。

### 4.1 字节序与数据类型

- 小端序（Little-Endian）。
- 浮点为 IEEE754 `float32`。
- 如用结构体打包，需注意 `packed`/对齐问题；建议按字节流序列化。

### 4.2 帧格式（无转义，SOF + LEN + CRC 重同步）

```
SOF      : 2B  固定 0xAA 0x55
VER      : 1B  协议版本（当前 0x01）
LEN      : 2B  Payload 长度（0..512，建议限制上限）
MSG_ID   : 1B  消息类型
FLAGS    : 1B  bit0=ACK请求, bit1=ACK帧, 其余保留
SEQ      : 2B  序号（主机或设备各自递增）
PAYLOAD  : LEN 字节
CRC16    : 2B  CRC-16/CCITT-FALSE(poly=0x1021, init=0xFFFF)
              计算范围：VER..PAYLOAD（不含 SOF，不含 CRC）
```

接收侧重同步策略：扫描 `0xAA55` → 读取头部 → 按 LEN 收满 → 校验 CRC；失败则丢弃 1 字节继续扫描。

### 4.3 消息定义（最小集合）

#### Host → MCU

- `0x01 SET_TARGET`
  - `float target_speed_mps`
  - `float target_yaw_rate_dps`（或转向量，根据项目选其一）
  - `uint8 mode`（可选：0=停, 1=运行, 2=急停）

- `0x02 SET_PID`
  - `uint8 loop_id`（0=角度环, 1=速度环, 2=偏航/方向环）
  - `float kp, ki, kd`

- `0x03 ESTOP`
  - 无 payload（立即置零/刹车、置 fault 位；需要人工复位或收到 RUN 命令解除）

- `0x04 TELEM_CONFIG`
  - `uint16 period_ms`（遥测发送周期，如 10ms）
  - `uint16 mask`
    - bit0: STATUS
    - bit1: IMU_RAW
    - bit2: DEBUG

#### MCU → Host

- `0x10 STATUS`（默认打开，建议 10ms/100Hz）
  - `uint32 timestamp_ms`
  - `float pitch_deg`
  - `float gyro_y_dps`（或主要控制轴角速度）
  - `float speed_mps`
  - `int16 pwm_l`（-1000..1000 或 -100..100，需统一约定）
  - `int16 pwm_r`
  - `uint8 mode`
  - `uint16 fault_bits`

- `0x11 IMU_RAW`（调试流，默认关闭；建议 ≤100Hz 或抽点发送）
  - `uint32 timestamp_ms`
  - `int16 ax, ay, az`
  - `int16 gx, gy, gz`
  - 备注：需在文档中固定单位/比例（例如 LSB→g、LSB→dps）。

- `0x12 DEBUG`（文本调试，默认关闭）
  - `uint32 timestamp_ms`
  - `uint8 level`
  - `uint8[] msg_bytes`（UTF-8 或 ASCII，长度=LEN-5）

- `0x7F ACK`
  - `uint8 acked_msg_id`
  - `uint16 acked_seq`
  - `uint8 result`（0=OK，其它为错误码）

### 4.4 上报数据策略（与调参相关）

- 默认上报：滤波/估计后的状态（STATUS），用于调参/监控，曲线稳定且贴近控制实际使用量。
- 原始数据：仅在标定/排障时开启（IMU_RAW），必要时降采样以节省带宽。

## 五、实现步骤规划（v2）

### Phase 1: 主循环闭环跑通
1. 建立 `MainControlTask`（2ms/500Hz），空转验证抖动与 CPU 裕量
2. 接入编码器寄存器读取与速度计算
3. 接入 IMU 读取（建议 SPI DMA）与数据流水/双缓冲
4. 互补滤波输出 `pitch/roll`（按项目需要选择轴）
5. PID 输出与 PWM 更新

### Phase 2: 串口协议与上位机联调
1. 实现帧解析（SOF/LEN/CRC 重同步）
2. 实现最小指令集：SET_TARGET / ESTOP / SET_PID
3. 实现遥测：STATUS（默认 10ms）+ 可选 IMU_RAW/DEBUG

### Phase 3: 保护与鲁棒性
1. 控制超周期检测（deadline miss）与故障位
2. IMU 数据超时/连续无新数据保护
3. 输出限幅、软启动/刹车策略
4. 看门狗（建议独立看门狗 IWDG）

## 六、验证计划（v2）

- 周期抖动：在 `MainControlTask` 入口/出口翻转 GPIO，用逻辑分析仪/示波器测 2ms 稳定性与 worst-case 执行时间。
- 数据链路：故意插拔/干扰串口，验证协议重同步与 CRC 报错恢复。
- 安全保护：触发 IMU 断开/卡死、触发 ESTOP，确认 PWM 及时进入安全状态。
- 性能余量：开启/关闭 IMU_RAW/DEBUG，确认主循环不受影响。
