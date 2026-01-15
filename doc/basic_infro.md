# 平台与硬件基础信息（basic_infro）

> 用途：记录“平台 / 工具链 / 时钟 / 外设与引脚分配 / 中断优先级”等基础信息，方便后续调试、联调与复现。

## 1. 平台与工程

- 工程名：`balance_car`
- CubeMX 工程：`balance_car.ioc`（MX 版本：6.15.0）
- 代码结构：CubeMX 生成代码在 `Core/`，用户模块在 `APP/`

## 2. MCU 与时钟

- MCU：STM32F407ZGT6（Cortex-M4F，LQFP144）
- 外部晶振：HSE 8MHz（`PH0/PH1`）
- 系统时钟：168MHz
- 总线分频：
  - AHB(HCLK)：168MHz
  - APB1(PCLK1)：42MHz（定时器时钟 84MHz）
  - APB2(PCLK2)：84MHz（定时器时钟 168MHz）
- PLL（来自 `Core/Src/main.c` / `balance_car.ioc`）：PLLM=4, PLLN=168, PLLP=2, PLLQ=4

## 3. RTOS

- RTOS：FreeRTOS（CMSIS-RTOS v2 wrapper）
- Tick：1kHz（`configTICK_RATE_HZ=1000`）
- Heap：`heap_4`，`configTOTAL_HEAP_SIZE=15360`
- 中断优先级约束：`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`

参考：`Core/Inc/FreeRTOSConfig.h`

## 4. 工具链与构建

- 构建系统：CMake + Ninja（见 `CMakePresets.json`）
- GCC 工具链：`arm-none-eabi-gcc`（见 `cmake/gcc-arm-none-eabi.cmake`）
- 关键编译/链接参数：
  - `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`
  - 链接脚本：`STM32F407XX_FLASH.ld`

## 5. 外设与引脚配置（来自 CubeMX）

### 5.1 IMU（SPI1 + DMA）

- SPI：SPI1（主机模式，2线全双工，8-bit，CPOL=0，CPHA=1edge）
- SPI 时钟：Prescaler=8 → 10.5 Mbit/s（APB2=84MHz）
- SPI 引脚：
  - `PA5` → SPI1_SCK
  - `PB4` → SPI1_MISO
  - `PB5` → SPI1_MOSI
- 片选：`PB12` → `CS`（GPIO 输出，默认上电为 High）
- 数据就绪（推测为 IMU DRDY）：`PC5` → EXTI5（上升沿触发，内部下拉）
- DMA：
  - SPI1_RX：DMA2_Stream0, Channel 3, Normal
  - SPI1_TX：DMA2_Stream3, Channel 3, Normal

### 5.2 编码器（正交）

- TIM3 Encoder Interface（TI12）：
  - `PA6` → TIM3_CH1
  - `PA7` → TIM3_CH2
  - Period=0xFFFF, IC Filter=6
- TIM4 Encoder Interface（TI12）：
  - `PD12` → TIM4_CH1
  - `PD13` → TIM4_CH2
  - Period=0xFFFF, IC Filter=6

### 5.3 电机 PWM

- TIM1 PWM 输出：
  - `PE9`  → TIM1_CH1
  - `PE11` → TIM1_CH2
  - `PE13` → TIM1_CH3
  - `PE14` → TIM1_CH4
- TIM1 配置：Prescaler=0, Period(ARR)=0x20CF(8399)
  - PWM 基准频率约：`168MHz / (8399+1) ≈ 20kHz`

### 5.4 OLED（I2C1）

- I2C：I2C1，400kHz（Fast mode）
- 引脚：
  - `PB6` → I2C1_SCL
  - `PB7` → I2C1_SDA

### 5.5 上位机通信（USART3 + DMA）

- UART：USART3，921600 8N1
- 引脚：
  - `PD8` → USART3_TX
  - `PD9` → USART3_RX
- DMA：
  - USART3_RX：DMA1_Stream1, Channel 4, Circular, Priority High
  - USART3_TX：DMA1_Stream3, Channel 4, Normal, Priority Medium

### 5.6 其他 GPIO

- `PB2` → `LED`（GPIO 输出）

## 6. NVIC / 中断优先级（CubeMX 当前配置）

- Priority Group：`NVIC_PRIORITYGROUP_4`
- 关键中断：
  - EXTI9_5_IRQn（PC5/EXTI5）：priority=5, sub=0
  - SPI1_IRQn：priority=5, sub=0
  - USART3_IRQn：priority=5, sub=0
  - DMA1_Stream1_IRQn（USART3_RX）：priority=5, sub=0
  - DMA1_Stream3_IRQn（USART3_TX）：priority=5, sub=0
  - DMA2_Stream0_IRQn（SPI1_RX）：priority=5, sub=0
  - DMA2_Stream3_IRQn（SPI1_TX）：priority=5, sub=0
  - PendSV：priority=15

注意：需要在这些 ISR 中调用 FreeRTOS `FromISR` API 时，必须保证 ISR 优先级数值 **≥ 5**（当前配置满足）。

## 7. 引脚速查表

| 功能 | 外设/信号 | 引脚                       |
|---|---|--------------------------|
| HSE 晶振 | OSC_IN / OSC_OUT | PH0 / PH1                |
| SWD 调试 | SWDIO / SWCLK | PA13 / PA14              |
| IMU SPI | SCK / MISO / MOSI | PA5 / PB4 / PB5          |
| IMU 片选 | CS | PB12                     |
| IMU 中断 | DRDY(EXTI5) | PC5                      |
| 编码器 L | TIM3_CH1 / CH2 | PA6 / PA7                |
| 编码器 R | TIM4_CH1 / CH2 | PD12 / PD13              |
| 电机 PWM | TIM1_CH1/2/3/4 | PE9 / PE11 / PE13 / PE14 |
| OLED I2C | SCL / SDA | PB6 / PB7                |
| 上位机串口 | USART3_TX / RX | PD8 / PD9                |
| 指示灯 | LED | PB2                      |

---

维护建议：
- 引脚改动以 `balance_car.ioc` 为准，并同步更新本文件。
