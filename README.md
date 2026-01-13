# Balance Car - 基于强化学习的两轮自平衡小车

## 📋 项目概述

本项目是一个采用强化学习（Reinforcement Learning）训练的两轮自平衡小车下位机系统，采用分层架构设计，融合实时控制、传感器融合与高速通信功能。系统通过 CLion + STM32CubeMX + CMake 开发环境构建，实现高性能的嵌入式控制。

## 🏗️ 系统架构

### 控制系统架构
```
┌─────────────────────────────────────────┐
│         上位机推理层                       │
│   RDK X5 (ROS2 + ONNX Runtime)           │
└──────────────┬──────────────────────────┘
               │  UART 921600 bps
┌──────────────▼──────────────────────────┐
│         下位机控制层                       │
│   STM32F407ZGT6 (168MHz Cortex-M4)       │
└──────────────┬──────────────────────────┘
               │
       ┌───────┴────────┐
       │  FreeRTOS RTOS │
       └───────┬────────┘
               │
    ┌──────────┼──────────┐
    │          │          │
传感器采集   电机控制   显示/通信
```

### 软件模块
- **FreeRTOS**: 实时操作系统，支持多任务调度
- **硬件抽象层 (HAL)**: STM32 HAL 库 + DMA 加速
- **应用层 (APP/**): 用户代码模块，支持热插拔式开发

## 🔧 技术栈

### 开发环境
| 组件 | 版本/配置 |
|------|-----------|
| 开发平台 | Windows 11 + CLion |
| 构建系统 | CMake 3.22+ + Ninja |
| 工具链 | ARM GCC Toolchain (arm-none-eabi-gcc) |
| 代码生成 | STM32CubeMX |
| 版本控制 | Git |

### 软件栈
| 类别 | 组件 | 说明 |
|------|------|------|
| MCU | STM32F407ZGT6 | 168MHz Cortex-M4F, 1MB Flash, 192KB RAM |
| RTOS | FreeRTOS | 实时操作系统 |
| 驱动库 | STM32 HAL v1.x | 官方 HAL 库 + DMA 支持 |
| 构建工具 | CMake + Ninja | 跨平台构建 |

## ⚙️ 硬件配置

### 主控制器
- **MCU**: STM32F407ZGT6
- **时钟**: 168 MHz (HSE 8MHz → PLL 168MHz)
- **浮点单元**: FPU (FPv4-SP-D16)
- **定时器**: TIM1 (PWM), TIM3/TIM4 (编码器)

### 外设配置
| 外设 | 型号/规格 | 通信协议 | 功能说明 |
|------|-----------|----------|----------|
| IMU | ICM20602 | SPI (DMA) | 6轴姿态传感器，用于平衡控制 |
| 显示 | OLED SSD1306 | I2C | 实时状态显示 |
| 电机驱动 | AT8236 × 2 | PWM | 双电机驱动，支持编码器反馈 |
| 编码器 | 正交编码器 | A/B 相 | 500 PPR，用于速度/位置反馈 |
| 通信 | UART3 | 921600 bps | 高速串口通信，与上位机数据交互 |

### 引脚与资源分配
- **定时器**: TIM1 (PWM 电机控制), TIM3/TIM4 (编码器)
- **串口**: USART3 (与上位机通信)
- **SPI**: SPI1 (IMU传感器)
- **I2C**: I2C1 (OLED显示)
- **DMA**: SPI/I2C 数据传输加速

## 🎯 功能特性

### 实时控制
- 高速传感器数据采集（IMU 50Hz+, 编码器实时）
- 低延迟电机控制响应
- FreeRTOS 多任务并行调度支持

### FreeRTOS 任务调度
基于优先级抢占的多任务调度系统，支持任务间通信机制：
- **传感器采集任务**: 实时读取 IMU、编码器数据
- **电机控制任务**: 1ms 周期，最高优先级
- **通信任务**: 与上位机数据交换
- **显示任务**: OLED 状态刷新

### 通信协议
- 波特率: 921600 bps
- 支持与 ROS2 系统的高速数据交换

## 🤖 强化学习训练环境

### 上位机推理
- **平台**: RDK X5
- **框架**: ROS2 + ONNX Runtime
- **功能**: 部署训练好的 ONNX 模型，实时推理控制指令

### 训练环境
- **平台**: Ubuntu 22.04
- **仿真器**: Genesis v0.3.11
- **RL 算法库**: rsl_rl
- **训练方法**: 强化学习（RL）端到端训练

## 📁 项目结构

```
balance_car/
├── APP/                          # 应用层代码（用户自定义）
│   ├── CMakeLists.txt           # APP 子模块 CMake 配置
│   ├── led.c/h                  # LED 控制模块
│   ├── mydefine.h               # 全局头文件定义
│   └── scheduler.c/h            # 自定义调度器（已废弃）
├── Core/                         # STM32CubeMX 生成代码
│   ├── Inc/                     # 头文件
│   │   ├── FreeRTOSConfig.h    # FreeRTOS 配置
│   │   ├── stm32f4xx_hal_conf.h # HAL 库配置
│   │   └── ...
│   └── Src/                     # 源文件
│       ├── freertos.c           # FreeRTOS 任务定义
│       ├── stm32f4xx_it.c       # 中断处理
│       └── main.c               # 主程序入口
├── Drivers/                      # HAL 驱动库
│   ├── CMSIS/                   # CMSIS 核心
│   └── STM32F4xx_HAL_Driver/    # STM32 HAL 驱动
├── Middlewares/                  # 第三方中间件
│   └── Third_Party/
│       └── FreeRTOS/           # FreeRTOS 内核源码
├── cmake/                        # CMake 配置
│   └── stm32cubemx/             # CubeMX 生成的 CMake 配置
├── components/                   # 第三方组件（预留）
├── CMakeLists.txt               # 根 CMake 配置（用户可修改）
├── CMakePresets.json            # CMake 预设配置
├── STM32F407XX_FLASH.ld         # 链接脚本
├── balance_car.ioc              # STM32CubeMX 项目文件
├── startup_stm32f407xx.s        # 启动文件
└── .gitignore                   # Git 忽略配置
```

### CMake 构建说明
- **根目录 CMakeLists.txt**: 用户可自由修改（只生成一次）
- **cmake/stm32cubemx/CMakeLists.txt**: 由 CubeMX 自动生成，包含 HAL 驱动和 FreeRTOS 配置
- **APP/CMakeLists.txt**: 自动收集 APP 目录下所有 .c 文件

## 🚀 快速开始

### 环境准备
1. 安装 STM32CubeMX
2. 安装 ARM GCC 工具链
3. 配置 CLion 的 STM32 开发环境
4. 配置 CMake 工具链（参考 CMakePresets.json）

### 编译项目
```bash
# 使用 CMake 预设构建项目
cmake --preset Debug
cmake --build --preset Debug

# 或在 CLion 中点击 Build 按钮
```

### 烧录固件
使用 ST-Link Utility 或 OpenOCD 进行固件烧录：
```bash
# 使用 OpenOCD 烧录
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program balance_car.elf verify reset exit"
```

### CubeMX 修改硬件配置
1. 打开 `balance_car.ioc`
2. 修改引脚配置或启用外设
3. 生成代码（**注意**: 根目录 CMakeLists.txt 不会被覆盖）
4. 在 APP/ 目录添加/修改用户代码

## 📝 开发规范

### 添加新模块
在 `APP/` 目录添加 `.c/.h` 文件，无需修改 CMakeLists.txt（自动收集）

### FreeRTOS 任务开发
- 在 `Core/Src/freertos.c` 中定义任务
- 使用 FreeRTOS API 进行任务间通信（队列、信号量、互斥锁）
- 参考 `FreeRTOSConfig.h` 配置系统参数

### CubeMX 代码保护
- 在 Core/Src/ 和 Core/Inc/ 中使用 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 标记用户代码
- 避免在自动生成区域外添加代码

### CMake 缓存清理
如果遇到编译器错误，清理构建目录重新配置：
```bash
rm -rf cmake-build-debug-stm32
```

## 📊 性能参数

| 参数 | 数值 |
|------|------|
| MCU 频率 | 168 MHz |
| UART 波特率 | 921600 bps |
| 控制周期 | 1ms (电机控制), 10-20ms (传感器) |
| 编码器分辨率 | 500 PPR (4倍频 = 2000 counts/rev) |
| FreeRTOS 堆大小 | 可配置（默认 30KB） |

## 🗺️ 开发路线图

### 已完成 ✅
- [x] 硬件驱动配置（IMU、编码器、电机驱动、UART）
- [x] FreeRTOS 集成与基础任务框架
- [x] CMake 构建系统配置

### 进行中 🔄
- [ ] 传感器数据采集任务实现
- [ ] 电机控制 PID 算法
- [ ] 上位机通信协议

### 待开发 📋
- [ ] 平衡控制算法实现
- [ ] FreeRTOS 任务间通信优化
- [ ] OLED 显示界面
- [ ] 系统调试与性能优化
- [ ] 模型部署与测试

## 🤝 贡献指南

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

本项目基于 ST 许可证开源，详见 LICENSE 文件。
