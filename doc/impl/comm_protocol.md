# 串口二进制协议（comm 模块）

本文描述 `APP/comm.c/.h` 使用的 MCU↔Host 二进制协议，以及与 VOFA 遥测的切换方式。

## 1. 工作模式（编译期开关）

配置位置：`APP/mydefine.h`

- 协议遥测（默认）：`APP_UART3_TELEM_PROTOCOL=1`，`APP_UART3_TELEM_VOFA=0`，`APP_UART3_ECHO_ENABLE=0`
  - RX：USART3 DMA ReceiveToIdle → `comm` 解析命令
  - TX：`comm` 周期发送 `STATUS` 帧（二进制）
- VOFA 遥测：`APP_UART3_TELEM_PROTOCOL=0`，`APP_UART3_TELEM_VOFA=1`，`APP_UART3_ECHO_ENABLE=0`
  - RX：仍由 `comm` 解析命令
  - TX：使用 `APP/vofa_telemetry.c` 发送 JustFloat（便于 VOFA+ 画图）
- Echo 调试：`APP_UART3_ECHO_ENABLE=1`，并将两种遥测都设为 `0`
  - RX/TX：`APP/uart_echo.c` 原样回显

## 2. 帧格式

```
SOF(2B) | VER(1B) | LEN(2B) | MSG_ID(1B) | FLAGS(1B) | SEQ(2B) | PAYLOAD(LEN) | CRC16(2B)
```

- `SOF`：固定 `0xAA 0x55`
- `VER`：当前 `0x01`
- `LEN`：payload 字节数（小端）
- `SEQ`：序号（小端）
- `CRC16`：CRC-16/CCITT-FALSE（poly=0x1021, init=0xFFFF），计算范围为 `VER..PAYLOAD`（不含 SOF/CRC）

接收重同步策略：扫描 `0xAA55` → 读取头部 → 等待完整帧 → CRC 校验；失败则丢 1 字节继续扫描。

## 3. 消息（阶段二最小集合）

### 3.1 Host → MCU

- `0x01 SET_TARGET`：`comm_set_target_t`
  - `float target_speed_mps`
  - `float target_yaw_rate_dps`
  - `uint8 mode`（0=STOP, 1=RUN, 2=ESTOP）
- `0x03 ESTOP`：无 payload
- `0x04 TELEM_CONFIG`：`comm_telem_config_t`
  - `uint16 period_ms`
  - `uint16 mask`（bit0=STATUS）

### 3.2 MCU → Host

- `0x10 STATUS`：`comm_status_t`
  - `uint32 timestamp_ms`
  - `float pitch_deg`（互补滤波输出）
  - `float pitch_acc_deg`（仅加速度推算）
  - `float wheel_l_mps`
  - `float wheel_r_mps`
  - `int16 pwm_l`
  - `int16 pwm_r`
  - `uint8 mode`
  - `uint16 fault_bits`

说明：若后续需要上报更多量（例如 gyro/acc 原始数据），建议新增消息（如 `IMU_RAW`）或扩展 `STATUS` 并同步更新文档。

