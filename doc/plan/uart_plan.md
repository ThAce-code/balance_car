# 串口通信开发计划（USART3 + DMA + 协议v2）

> 目标：实现稳定、可重同步、低开销的上位机通信链路；让 `CommTask` 可靠接收控制指令并下发遥测（`STATUS`/`IMU_RAW`），且不影响 `MainControlTask` 实时性。

## 0. 现状与约束（来自工程配置）

- UART：USART3 `921600 8N1`（见 `doc/basic_info.md`）
- DMA：
  - RX：`DMA1_Stream1` Circular + High priority（持续接收，不丢字节）
  - TX：`DMA1_Stream3` Normal（按帧发送）
- RTOS：CMSIS-RTOS2
  - 输入队列：`xHostCommandQueue` 深度=1（只保留最新目标）
  - 状态输出：`StatusStore`（只保留最新状态快照，供通信任务按需读取）
- 中断优先级：相关 IRQ 均为 priority=5（满足 FromISR 约束）

## 1. 协议（引用 v2 规范）

协议帧格式/MSG_ID/CRC 等以 `doc/plan/plan_v2.md` 为准（避免多处维护）。本计划仅定义落地实现方式：

- 帧：`SOF(0xAA55) + VER + LEN + MSG_ID + FLAGS + SEQ + PAYLOAD + CRC16(CCITT-FALSE)`
- RX 端：允许丢包、允许任意字节插入/丢失；通过扫描 SOF + LEN + CRC 完成快速重同步
- 默认遥测：`STATUS@10ms`
- 调试遥测：`IMU_RAW` 默认关闭（按 `TELEM_CONFIG` 打开，并建议降采样）

## 2. 模块设计（推荐分层）

### 2.1 components 层（可复用、与 HAL 解耦）

建议新增目录：`components/comm/`

- `proto_v2.c/.h`
  - 打包/解包：header 解析、payload 访问、CRC16 计算
  - RX 解析器：字节流→帧（状态机：SCAN_SOF → READ_HDR → READ_PAYLOAD → READ_CRC）
- `ringbuf.c/.h`
  - 环形缓冲（用于 DMA circular 的“已接收字节”抽取）

### 2.2 APP 层（与 HAL/RTOS 绑定）

建议新增目录：`APP/comm/`

- `uart_dma.c/.h`
  - USART3 RX DMA circular 启动
  - IDLE line 中断/回调：把 DMA 写指针推进到 ringbuf（“生产者”）
  - TX DMA：非阻塞发送（忙则丢弃/排队，按你策略选）
- `comm_task.c/.h`
  - 周期 10ms：解析 RX、处理命令、发送遥测

说明：如果你想少建文件，也可以先把 `uart_dma.* + proto_v2.*` 合并到 `APP/comm.c`，等稳定后再拆分。

## 3. 数据流（端到端）

1) USART3 RX DMA circular 持续写入 DMA buffer
2) `USART3_IRQHandler` 触发 IDLE（或周期轮询）→ 把 DMA 新增字节搬运/映射到 ringbuf
3) `CommTask` 每 10ms 从 ringbuf 取字节喂给 `proto_v2_rx_feed()`，解析出 0..N 帧
4) 对命令帧：
   - `SET_TARGET`/`ESTOP`/`SET_PID`：更新 `HostCommand_t` 并写入 `xHostCommandQueue`（深度=1 语义）
   - 若 FLAGS 请求 ACK：回 `ACK` 帧（带 SEQ）
5) 对遥测：
   - 从 `StatusStore` 取最新 `StatusData_t`（取不到就沿用上次快照）
   - 按配置（period/mask）发送 `STATUS` / `IMU_RAW`

## 4. 里程碑与工作分解（WBS）

### M0：串口基础连通（0.5 天）
- 确认 USART3 波特率与 PC 端一致
- 先用阻塞发送/接收验证链路（仅 bring-up）

验收：上位机能收到 MCU 发出的固定字符串/固定二进制帧。

### M1：RX DMA circular + IDLE 抽取（1 天）
- 启动 RX DMA circular
- 在 IDLE 中断回调中计算“DMA 写入到哪了”，把新增字节推进 ringbuf
- 提供统计计数：`rx_bytes`, `idle_hits`, `overflow` 等

验收：持续发送随机数据 1~2 分钟不丢字节、不死机。

### M2：协议解析器（1 天）
- 实现 SOF 扫描 + LEN + CRC 校验 + 重同步
- 输出 “完整帧事件”，支持多帧粘包/拆包

验收：故意插入噪声/丢字节，解析器能在有限字节内重同步；CRC 错帧被丢弃。

### M3：命令处理（0.5~1 天）
- 支持最小集合：`SET_TARGET`、`ESTOP`、`TELEM_CONFIG`
- 写入 `xHostCommandQueue`（深度=1，只保留最新）

验收：上位机能实时改变目标速度/转向；ESTOP 立即生效（后续联动 PWM 保护）。

### M4：遥测发送（0.5~1 天）
- `STATUS@10ms` 默认发送
- `IMU_RAW` 可配置开启（建议 ≤100Hz）
- TX 使用 DMA normal，避免阻塞 `CommTask`

验收：上位机能稳定画曲线；长时间运行无堆积/卡死。

### M5：鲁棒性与安全策略（0.5 天）
- 串口断开/无命令超时：设置 `cmd_timeout_ms`（例如 300ms）→ 自动 `STOP`
- `SEQ/ACK` 处理（可选）：用于上位机确认关键命令到达
- 限速/限幅与 mode 状态机（STOP/RUN/ESTOP）

验收：拔掉上位机/无线链路后，小车进入安全状态；恢复连接后可重新 RUN。

## 5. 与实时控制的边界（必须遵守）

- `MainControlTask` 不阻塞等待串口；只在每帧取一次 `HostCommand_t` 最新值（已实现深度=1 drain）
- `CommTask` 不做重计算：只做解析/打包/队列写入；大计算放低优先级或拆分
- ISR 中只做“搬运指针/推 ringbuf/置标志”，禁止解析整帧

## 6. 验证与工具

- PC 端脚本（建议 Python）：
  - 发送 `SET_TARGET`、`TELEM_CONFIG`
  - 解析 `STATUS` 并打印/画图
- 压测：
  - 连续发送小帧（例如 200Hz）+ MCU 回传 `STATUS@100Hz`
  - 验证 ringbuf 不溢出、CRC 错误率为 0（或可解释）

## 7. 交付清单（Definition of Done）

- RX：DMA circular + 重同步解析稳定
- 命令：`SET_TARGET/ESTOP/TELEM_CONFIG` 可用，能驱动 `xHostCommandQueue`
- 遥测：默认 `STATUS@10ms`，可配置 `IMU_RAW`
- 不提交图片资产：`/images` 不纳入版本库（按当前约定）
