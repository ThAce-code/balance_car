# FreeRTOS 任务队列规划方案 v1.0

> **已废弃**：本文档为早期规划，与实际实现存在较大差异。**请以 `doc/plan/plan_v2.md` 为准。**
>
> 主要变更：
> - v2 将"采样→估计→控制→输出"收敛为单一 `MainControlTask`（500Hz），取消了独立的 `IMUTask` 和 `EncoderTask`
> - 数据传递方式由深度队列改为"最新快照"（避免排队延迟）
> - 任务优先级与同步方式有调整

---

## 一、任务架构设计

### 1.1 任务优先级分配（优先级 1-5，5 最高）

```
优先级 5 (最高)
├─ 平衡控制任务 (BalanceControlTask)     - 周期: 1ms
│
优先级 4 (高)
├─ IMU 采集任务 (IMUTask)                - 周期: 20ms
└─ 编码器任务 (EncoderTask)              - 周期: 10ms
│
优先级 3 (中)
├─ 通信任务 (CommTask)                   - 周期: 10ms
│
优先级 2 (低)
├─ OLED 显示任务 (OLEDDisplayTask)       - 周期: 100ms
│
优先级 1 (最低)
└─ LED 任务 (LEDTask)                    - 周期: 500ms
```

### 1.2 任务详情表

| 任务名称 | 优先级 | 周期 | 栈大小 | 核心功能 | 依赖对象 |
|---------|--------|------|--------|---------|---------|
| BalanceControlTask | 5 (osPriorityHigh) | 1ms | 768 bytes | PID控制、PWM输出 | 数据互斥锁、IMU队列、编码器队列 |
| IMUTask | 4 (osPriorityAboveNormal) | 20ms | 384 bytes | SPI DMA读取、互补滤波 | IMU信号量、IMU数据队列 |
| EncoderTask | 4 (osPriorityAboveNormal) | 10ms | 384 bytes | 读取TIM3/4、速度计算 | 编码器数据队列 |
| CommTask | 3 (osPriorityNormal) | 10ms | 640 bytes | UART通信、协议解析 | UART信号量x2、命令队列、状态队列 |
| OLEDDisplayTask | 2 (osPriorityBelowNormal) | 100ms | 512 bytes | I2C显示、状态刷新 | 状态互斥锁 |
| LEDTask | 1 (osPriorityLow) | 500ms | 192 bytes | 状态指示灯闪烁 | 无 |

## 二、同步对象设计

### 2.1 队列（Queues - x5）

```c
// 1. IMU数据队列（IMUTask → BalanceControlTask）
// 数据结构：IMU 6轴原始数据 + 姿态角
typedef struct {
    float acc_x, acc_y, acc_z;      // 加速度 (g)
    float gyro_x, gyro_y, gyro_z;   // 角速度 (°/s)
    float pitch, roll;             // 互补滤波后的姿态角 (°)
    uint32_t timestamp;             // 时间戳 (ms)
} IMUData_t;
QueueHandle_t xIMUDataQueue;        // 队列深度: 10

// 2. 编码器数据队列（EncoderTask → BalanceControlTask）
typedef struct {
    int32_t left_count;             // 左轮编码器计数值
    int32_t right_count;            // 右轮编码器计数值
    float left_speed;              // 左轮速度 (m/s)
    float right_speed;             // 右轮速度 (m/s)
    uint32_t timestamp;             // 时间戳 (ms)
} EncoderData_t;
QueueHandle_t xEncoderDataQueue;    // 队列深度: 10

// 3. 上位机命令队列（CommTask → BalanceControlTask）
typedef struct {
    float target_pitch;            // 目标俯仰角 (°)
    float target_speed;            // 目标速度 (m/s)
    uint8_t command;              // 命令类型（0=停止, 1=运行, 2=急停）
} HostCommand_t;
QueueHandle_t xHostCommandQueue;   // 队列深度: 5

// 4. 状态数据队列（BalanceControlTask → CommTask）
typedef struct {
    float current_pitch;           // 当前俯仰角 (°)
    float current_speed;           // 当前速度 (m/s)
    float left_pwm;               // 左电机 PWM (%)
    float right_pwm;              // 右电机 PWM (%)
    uint32_t timestamp;           // 时间戳 (ms)
} StatusData_t;
QueueHandle_t xStatusQueue;       // 队列深度: 10

// 5. 调试数据队列（所有任务 → CommTask）
typedef struct {
    uint8_t task_id;              // 任务ID
    char message[32];             // 调试信息
    uint32_t timestamp;           // 时间戳 (ms)
} DebugData_t;
QueueHandle_t xDebugQueue;       // 队列深度: 5
```

### 2.2 信号量（Semaphores - x3）

```c
// 1. IMU DMA 完成信号量（中断 → IMUTask）
SemaphoreHandle_t xIMUDMASem;    // 二值信号量

// 2. UART TX DMA 完成信号量（中断 → CommTask）
SemaphoreHandle_t xUARTTxSem;     // 二值信号量

// 3. UART RX DMA 完成信号量（中断 → CommTask）
SemaphoreHandle_t xUARTRxSem;     // 二值信号量（或计数信号量）
```

### 2.3 互斥锁（Mutexes - x2）

```c
// 1. 全局状态保护互斥锁
SemaphoreHandle_t xStateMutex;    // 保护 balance_state 结构体

// 2. 输出保护互斥锁
SemaphoreHandle_t xOutputMutex;   // 保护 PWM 输出和命令发送
```

## 三、数据流架构

### 3.1 数据流向图

```
┌─────────────┐         ┌─────────────┐         ┌─────────────┐
│  IMUTask    │───────>│   Queue1    │<────────│   Balance   │
│  (20ms)     │ (IMU数据)│  (IMUData)  │  (命令)  │ ControlTask│
└─────────────┘         └─────────────┘         │   (1ms)     │
                                              └──────┬──────┘
┌─────────────┐         ┌─────────────┘              │
│ EncoderTask │───────>│   Queue2    │<─────────────┤
│  (10ms)     │ (编码器) │ (Encoder)   │   PWM输出   │
└─────────────┘         └─────────────┘              │
                                                       │
┌─────────────┐         ┌─────────────┘              ▼
│  CommTask   │         ┌─────────────┐      ┌─────────────┐
│  (10ms)     │<──────>│   Queue3/4  │      │   Motor     │
│   UART3     │ (命令/状态) │ (Host/Status│     │   Driver    │
└─────────────┘         └─────────────┘      └─────────────┘
        ▲
        │ DMA 信号
┌─────────────┐
│  DMA 中断   │
└─────────────┘
```

### 3.2 任务状态机

**BalanceControlTask 状态机：**
```
初始化 → 等待数据 → 数据有效性检查 → PID计算 → PWM输出 → 等待下一次触发
```

**CommTask 状态机：**
```
接收命令 → 解析协议 → 更新命令队列 → 读取状态队列 → 打包发送 → 等待DMA完成
```

## 四、FreeRTOSConfig.h 配置调整

```c
// 基础配置调整
#define configTOTAL_HEAP_SIZE              ((size_t)20480)    // 20KB
#define configENABLE_FPU                   1                  // 启用FPU
#define configCHECK_FOR_STACK_OVERFLOW      2                  // 栈溢出检测

// 栈溢出钩子函数（需实现）
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

// 内存分配失败钩子函数（需实现）
void vApplicationMallocFailedHook(void);

// 优先级配置（使用 CMSIS-RTOS V2 映射）
// osPriorityRealtime    = 56 (不使用)
// osPriorityHigh       = 48  → FreeRTOS: 48
// osPriorityAboveNormal = 40  → FreeRTOS: 40
// osPriorityNormal      = 32  → FreeRTOS: 32
// osPriorityBelowNormal= 24  → FreeRTOS: 24
// osPriorityLow        = 16  → FreeRTOS: 16
// osPriorityIdle       = 8   → FreeRTOS: 8

// 建议的实际优先级分配（FreeRTOS 原生值）
#define TASK_PRIORITY_BALANCE_CONTROL      (osPriorityHigh)        // 48
#define TASK_PRIORITY_IMU                  (osPriorityAboveNormal) // 40
#define TASK_PRIORITY_ENCODER              (osPriorityAboveNormal) // 40
#define TASK_PRIORITY_COMM                 (osPriorityNormal)       // 32
#define TASK_PRIORITY_OLED                 (osPriorityBelowNormal)  // 24
#define TASK_PRIORITY_LED                  (osPriorityLow)          // 16
```

## 五、文件结构规划

```
APP/
├── freertos/
│   ├── freertos_config.h           # FreeRTOS 配置调整（新增）
│   ├── task_balance.c/h            # 平衡控制任务
│   ├── task_imu.c/h               # IMU 采集任务
│   ├── task_encoder.c/h           # 编码器任务
│   ├── task_comm.c/h              # 通信任务
│   ├── task_oled.c/h             # OLED 显示任务
│   └── task_led.c/h              # LED 任务
├── drivers/
│   ├── icm20602.c/h              # IMU 驱动
│   ├── encoder.c/h               # 编码器驱动
│   ├── motor.c/h                 # 电机驱动
│   └── protocol.c/h              # 通信协议
└── mydefine.h                    # 全局头文件（更新）

Core/Src/
├── freertos.c                    # FreeRTOS 初始化（更新任务创建）
└── stm32f4xx_it.c               # 中断处理（添加DMA中断回调）
```

## 六、实现步骤规划

### Phase 1: 基础框架搭建
1. ✅ 调整 FreeRTOSConfig.h 配置
2. 创建 `APP/freertos/` 目录结构
3. 在 `freertos.c` 中创建所有任务句柄和属性
4. 创建队列、信号量、互斥锁
5. 实现任务空框架（无限循环 + vTaskDelay）

### Phase 2: 驱动层实现
1. 实现 `drivers/icm20602.c/h`（SPI DMA 读取、寄存器配置）
2. 实现 `drivers/encoder.c/h`（TIM3/TIM4 读取、速度计算）
3. 实现 `drivers/motor.c/h`（PWM 输出、方向控制）
4. 实现 `drivers/protocol.c/h`（通信协议定义、打包/解析）

### Phase 3: 任务功能实现
1. **IMUTask**: SPI DMA + 互补滤波
2. **EncoderTask**: 定时读取 + 速度计算
3. **BalanceControlTask**: PID 算法 + PWM 输出
4. **CommTask**: UART DMA + 协议解析
5. **OLEDDisplayTask**: I2C 显示
6. **LEDTask**: 状态指示

### Phase 4: 调试与优化
1. 添加栈溢出检测钩子
2. 添加内存分配失败钩子
3. 实现 `uxTaskGetStackHighWaterMark` 监控
4. 性能分析和任务周期验证

## 七、关键代码示例

### 7.1 freertos.c 任务创建代码框架

```c
/* Private variables */
osThreadId_t balanceControlTaskHandle;
osThreadId_t imuTaskHandle;
osThreadId_t encoderTaskHandle;
osThreadId_t commTaskHandle;
osThreadId_t oledTaskHandle;
osThreadId_t ledTaskHandle;

/* Queues */
QueueHandle_t xIMUDataQueue;
QueueHandle_t xEncoderDataQueue;
QueueHandle_t xHostCommandQueue;
QueueHandle_t xStatusQueue;
QueueHandle_t xDebugQueue;

/* Semaphores */
SemaphoreHandle_t xIMUDMASem;
SemaphoreHandle_t xUARTTxSem;
SemaphoreHandle_t xUARTRxSem;

/* Mutexes */
SemaphoreHandle_t xStateMutex;
SemaphoreHandle_t xOutputMutex;

void MX_FREERTOS_Init(void) {
    /* 创建队列 */
    xIMUDataQueue = xQueueCreate(10, sizeof(IMUData_t));
    xEncoderDataQueue = xQueueCreate(10, sizeof(EncoderData_t));
    xHostCommandQueue = xQueueCreate(5, sizeof(HostCommand_t));
    xStatusQueue = xQueueCreate(10, sizeof(StatusData_t));
    xDebugQueue = xQueueCreate(5, sizeof(DebugData_t));

    /* 创建信号量 */
    xIMUDMASem = xSemaphoreCreateBinary();
    xUARTTxSem = xSemaphoreCreateBinary();
    xUARTRxSem = xSemaphoreCreateBinary();

    /* 创建互斥锁 */
    xStateMutex = xSemaphoreCreateMutex();
    xOutputMutex = xSemaphoreCreateMutex();

    /* 任务属性定义 */
    const osThreadAttr_t balanceControl_attributes = {
        .name = "BalanceCtrl",
        .stack_size = 768,
        .priority = TASK_PRIORITY_BALANCE_CONTROL,
    };

    const osThreadAttr_t imu_attributes = {
        .name = "IMU",
        .stack_size = 384,
        .priority = TASK_PRIORITY_IMU,
    };

    /* 创建任务 */
    balanceControlTaskHandle = osThreadNew(BalanceControlTask, NULL, &balanceControl_attributes);
    imuTaskHandle = osThreadNew(IMUTask, NULL, &imu_attributes);
    // ... 其他任务创建
}
```

## 八、验证计划

1. **单元测试**: 每个任务独立测试
2. **集成测试**: 任务间通信验证
3. **压力测试**: 高负载下稳定性
4. **实时性测试**: 任务周期验证
5. **内存监控**: 栈使用率监控

## 九、资源需求总结

### 9.1 内存需求估算

| 类别 | 大小 (bytes) |
|------|--------------|
| 任务栈 | 2880 |
| 队列结构体 | 380 |
| 队列数据存储 | 1400 |
| 信号量 | 228 |
| 互斥锁 | 128 |
| 定时器任务 | 1104 |
| 内核开销 | 1184 |
| **总计** | **~7304** |
| 建议堆大小 | **20480 (20KB)** |

### 9.2 硬件资源利用

| 资源 | 用途 |
|------|------|
| SPI1 + DMA2 Stream0/3 | IMU 传感器读取 |
| TIM3/TIM4 | 编码器计数 |
| TIM1 | 电机 PWM 输出 |
| UART3 + DMA1 Stream1/3 | 上位机通信 |
| I2C1 | OLED 显示 |

## 十、注意事项

1. **中断优先级**: 确保 DMA 中断优先级 ≥ 5 (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
2. **FPU 配置**: 启用 FPU 支持以优化 IMU 浮点计算性能
3. **栈监控**: 使用 `uxTaskGetStackHighWaterMark` 定期检查栈使用情况
4. **DMA 双缓冲**: 建议为 IMU 和 UART 实现 DMA 双缓冲以提高吞吐量
5. **看门狗**: 建议添加独立看门狗防止系统死锁
6. **错误处理**: 所有关键路径都需要错误处理和恢复机制
