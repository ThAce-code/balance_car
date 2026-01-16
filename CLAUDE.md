# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

Prereq: ARM GCC toolchain (`arm-none-eabi-gcc`) on PATH.

```bash
# Configure (creates build/<preset>)
cmake --preset Debug    # or Release

# Build
cmake --build --preset Debug

# Clean: delete build/Debug directory
```

Flashing via ST-Link/OpenOCD (probe-dependent).

## Project Structure

- `balance_car.ioc`: STM32CubeMX project file — regenerate HAL/FreeRTOS code from here
- `Core/`: CubeMX-generated code. **Edit only inside `/* USER CODE BEGIN/END */` blocks**
- `APP/`: User application modules (preferred location for new features)
- `components/`: Reusable drivers/libraries (e.g., `imu/`, `comm/`, `OLED/`)
- `Drivers/`, `Middlewares/`: Vendor libraries (HAL, FreeRTOS) — do not modify
- `doc/`: Design documents — `doc/plan/plan_v2.md` is the primary reference

## Architecture Overview

### Hardware
- MCU: STM32F407ZGT6 @ 168MHz (Cortex-M4F)
- IMU: ICM20602 via SPI1 + DMA, DRDY interrupt on PC5
- Encoders: TIM3/TIM4 quadrature interface
- Motors: TIM1 PWM @ 20kHz
- UART: USART3 @ 921600 bps with DMA (host communication)
- OLED: SSD1306 via I2C1

### FreeRTOS Task Architecture (see `doc/plan/plan_v2.md`)

| Task | Priority | Period | Purpose |
|------|----------|--------|---------|
| MainControlTask | High | 2ms | IMU read → complementary filter → PID → PWM output |
| CommTask | Normal | 10ms | UART protocol parsing, telemetry |
| OLEDDisplayTask | BelowNormal | 100ms | Status display |
| LEDTask | Low | 500ms | Status LED |

**Key design principle**: MainControlTask is non-blocking. Uses "data pipeline" with DMA double-buffering — reads previous frame while DMA fills current buffer.

### Data Flow Patterns

1. **IMU Pipeline** (`doc/impl/imu_dataflow.md`):
   - DRDY interrupt → SPI DMA burst → `osThreadFlagsSet()` → MainControlTask processes previous frame
   - Components: `components/imu/imu_spi.c` (driver), `components/imu/imu_filter.c` (complementary filter), `APP/imu.c` (HAL callback routing)

2. **Status Sharing**:
   - `StatusStore` (single-writer/multi-reader snapshot) — MainControlTask writes, CommTask/OLEDTask read
   - No deep queues for sensor data (avoids latency accumulation)

3. **UART Communication** (`doc/impl/uart_dma_idle.md`):
   - RX: DMA circular + IDLE interrupt → ringbuf → protocol parser
   - TX: DMA normal mode, non-blocking

### Communication Protocol (`doc/plan/plan_v2.md` §4)
- Binary, little-endian, CRC-16/CCITT-FALSE
- Frame: `SOF(0xAA55) + VER + LEN + MSG_ID + FLAGS + SEQ + PAYLOAD + CRC16`
- Key messages: `SET_TARGET`, `ESTOP`, `STATUS`, `IMU_RAW`

## Code Conventions

- Language: C11
- New modules: add to `APP/` as `module_name.c/.h` (auto-collected by CMake)
- FreeRTOS: use CMSIS-RTOS v2 wrapper (`osThreadFlagsSet`, `osMessageQueue`, etc.)
- ISR constraint: priority ≥ 5 for FreeRTOS `FromISR` API calls
- Keep CubeMX-generated code formatting unchanged

## Key Documentation

- `doc/basic_info.md`: Pin assignments, clock config, peripheral details
- `doc/plan/plan_v2.md`: Task architecture, protocol spec (authoritative reference)
- `doc/impl/imu_dataflow.md`: IMU data pipeline explanation
- `doc/impl/uart_dma_idle.md`: UART DMA implementation details

When changing pins/peripherals, update both `balance_car.ioc` and `doc/basic_info.md`.
