# Balance Car

两轮自平衡小车下位机固件，基于 STM32F407 + FreeRTOS，支持强化学习策略部署。

## 系统架构

```
                    ┌──────────────────────────────┐
                    │     上位机 (RDK X5)           │
                    │   ROS2 + ONNX Runtime        │
                    └──────────────┬───────────────┘
                                   │ UART 921600 bps
                    ┌──────────────▼───────────────┐
                    │     下位机 (STM32F407)        │
                    │                              │
                    │  ┌────────────────────────┐  │
                    │  │   MainControlTask      │  │
                    │  │   500Hz 控制循环        │  │
                    │  │                        │  │
                    │  │  IMU ──► 姿态估计      │  │
                    │  │    ▼                   │  │
                    │  │  滤波 ──► PID ──► PWM  │  │
                    │  └────────────────────────┘  │
                    │                              │
                    │  CommTask    OLEDTask        │
                    │  (遥测/指令)  (状态显示)       │
                    └──────────────────────────────┘
                                   │
              ┌────────────────────┼────────────────────┐
              │                    │                    │
         ┌────▼────┐         ┌─────▼─────┐        ┌─────▼─────┐
         │ ICM20602 │         │  Encoder  │        │  AT8236   │
         │   IMU    │         │  TIM3/4   │        │   Motor   │
         └──────────┘         └───────────┘        └───────────┘
```

## 硬件规格

| 组件 | 型号/规格 | 接口 |
|------|----------|------|
| MCU | STM32F407ZGT6 @ 168MHz | - |
| IMU | ICM20602 | SPI1 + DMA, 500Hz DRDY |
| 电机驱动 | AT8236 × 2 | TIM1 PWM @ 20kHz |
| 编码器 | 正交编码器 500PPR | TIM3/TIM4 |
| 显示 | SSD1306 OLED 128×64 | I2C1 @ 400kHz |
| 通信 | USART3 | 921600 8N1, DMA |

## 快速开始

### 环境要求

- ARM GCC Toolchain (`arm-none-eabi-gcc`)
- CMake 3.22+
- STM32CubeMX (可选，用于修改硬件配置)

### 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

### 烧录

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program build/Debug/balance_car.elf verify reset exit"
```

## 项目结构

```
balance_car/
├── APP/                    # 应用层模块
│   ├── imu.c/.h           # IMU 应用封装
│   ├── uart_echo.c/.h     # 串口收发
│   ├── oled_ui.c/.h       # OLED 显示
│   └── status_store.c/.h  # 状态存储
├── components/             # 可复用组件
│   ├── imu/               # IMU 驱动与滤波
│   ├── comm/              # 通信协议与缓冲
│   └── OLED/              # SSD1306 驱动
├── Core/                   # CubeMX 生成代码
│   └── Src/freertos.c     # FreeRTOS 任务定义
├── doc/                    # 设计文档
│   ├── plan/              # 开发计划
│   ├── impl/              # 实现说明
│   └── guide/             # 使用指南
└── balance_car.ioc        # CubeMX 工程文件
```

## 软件架构

### 任务调度

| 任务 | 优先级 | 周期 | 职责 |
|------|--------|------|------|
| MainControlTask | High | 2ms | IMU 采集 → 姿态估计 → 控制输出 |
| CommTask | Normal | 10ms | 协议解析、遥测发送 |
| OLEDDisplayTask | BelowNormal | 100ms | 状态显示 |
| LEDTask | Low | 500ms | 状态指示 |

### 关键设计

- **非阻塞控制环**：MainControlTask 使用 DMA 双缓冲，读取上一帧数据的同时采集当前帧
- **DRDY 驱动采样**：IMU 数据就绪中断触发 SPI DMA，保证 500Hz 精确同步
- **无锁状态共享**：StatusStore 使用序列号机制实现单写者/多读者无锁访问

### 通信协议

二进制帧格式，支持丢包重同步：

```
SOF(0xAA55) | VER | LEN | MSG_ID | FLAGS | SEQ | PAYLOAD | CRC16
```

主要消息：`SET_TARGET`, `ESTOP`, `STATUS`, `IMU_RAW`

详见 [doc/plan/plan_v2.md](doc/plan/plan_v2.md)

## 文档

| 文档 | 说明 |
|------|------|
| [doc/basic_info.md](doc/basic_info.md) | 硬件配置（引脚、时钟、外设） |
| [doc/plan/plan_v2.md](doc/plan/plan_v2.md) | 系统架构与协议规范 |
| [doc/impl/imu_dataflow.md](doc/impl/imu_dataflow.md) | IMU 数据流说明 |
| [doc/guide/api_guide.md](doc/guide/api_guide.md) | API 使用指南 |

## 开发状态

- [x] IMU 采集与姿态估计（DRDY + SPI DMA + 互补滤波）
- [x] 串口 DMA 收发链路
- [x] OLED 状态显示
- [ ] 电机闭环控制
- [ ] 上位机通信协议
- [ ] RL 策略部署

## 许可证

本项目基于 ST 许可证开源。
