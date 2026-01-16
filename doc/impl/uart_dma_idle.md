# USART3 DMA + IDLE 实现说明

> 目的：说明当前工程的 USART3 数据"如何接收、如何发送"，以及如何从当前的 echo 测试平滑升级到二进制协议（`doc/plan/plan_v2.md`）。

## 1. 总体思路

当前串口采用 **DMA + IDLE**：

- RX：DMA 负责把串口字节搬到内存缓冲（无需 CPU 逐字节中断）
- UART IDLE 中断用来判断“一段数据帧间隔结束”，触发回调把本段数据交给上层处理
- TX：使用 DMA 非阻塞发送，避免任务被 `HAL_UART_Transmit()` 阻塞

这样做的优点：
- 中断频率低、CPU 占用小
- 能自然处理粘包/拆包：上层拿到的是“字节流片段”，再按协议自行组帧
- 发送不阻塞：任务可把 CPU 留给控制环/滤波等实时逻辑

## 2. 关键文件与职责

- HAL 回调分发（统一入口）：`APP/uart3_hal_callbacks.c`
  - `HAL_UARTEx_RxEventCallback()`：收到 IDLE（或 DMA 缓冲满）事件时调用
  - `HAL_UART_TxCpltCallback()`：TX DMA 完成
  - `HAL_UART_ErrorCallback()`：错误恢复
- Echo 业务（RX→缓存→TX）：`APP/uart_echo.c/.h`
  - 启动 RX：`UART_Echo_StartRxDma(&huart3)`
  - ISR 接收事件：`UART_Echo_OnRxEventIsr()`
  - 任务侧回显：`UART_Echo_TaskPump()`
- 环形缓冲（ISR 写、任务读）：`components/comm/ringbuf.c/.h`

## 3. RX：DMA + IDLE 的数据流

1) `CommTask` 启动 RX：
   - 调用 `HAL_UARTEx_ReceiveToIdle_DMA(huart3, s_uart3_rx_dma_buf, 256)`
2) USART3 收到数据：
   - DMA 将字节写入 `s_uart3_rx_dma_buf[]`
3) 触发 RxEvent：
   - 当线路出现“空闲间隔”（IDLE）或 DMA 缓冲写满时，HAL 触发 `HAL_UARTEx_RxEventCallback(huart3, Size)`
4) ISR 中处理片段：
   - `UART_Echo_OnRxEventIsr()` 把 `s_uart3_rx_dma_buf[0..Size)` 写入 ringbuf
   - 为避免停止接收，立刻再次调用 `HAL_UARTEx_ReceiveToIdle_DMA()` 重新开始下一段接收
   - 关闭半传输中断：`__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT)`（减少无用中断）
   - `osThreadFlagsSet(CommTaskHandle, UART_ECHO_FLAG_RX)` 唤醒任务

注意：
- `Size` 是本次事件的“本段有效字节数”，不是累计值
- 如果对端持续不停发且没有间隔，IDLE 不会触发；此时会依赖“DMA 缓冲写满”触发事件

## 4. TX：DMA 发送的数据流（Echo 当前实现）

`CommTask` 不再周期性发送 IMU 数据（避免刷屏），而是：

- 等待 `UART_ECHO_FLAG_RX | UART_ECHO_FLAG_TX`
- 调用 `UART_Echo_TaskPump()`：
  - 如果当前 TX 空闲且 ringbuf 有数据
  - 从 ringbuf 取一段（最多 256B）拷贝到 `s_uart3_tx_dma_buf`
  - 调用 `HAL_UART_Transmit_DMA()` 发出
- 发送完成后 `HAL_UART_TxCpltCallback()` → `UART_Echo_OnTxCpltIsr()` 清 busy 并唤醒任务继续 pump

丢包策略：
- RX ringbuf 满时会丢弃多余字节（`s_uart3_rx_overflow++`）
- TX 忙时不会启动新的 TX（避免覆盖 DMA buffer；DMA 期间发送缓冲必须保持不变）

## 5. TX：从 Echo 升级到“协议发送”的建议做法

Echo 的 TX 逻辑只有一个目的：验证“你发什么我回什么”。
当你开始发 `STATUS/ACK/DEBUG` 等协议帧时，建议把 TX 抽成一个统一的“发送器”，避免多个模块抢同一个 `huart3`：

推荐最小 TX 模块能力：
- `uart_tx_try_send_dma(buf,len)`：若空闲则启动 `HAL_UART_Transmit_DMA()`，否则返回忙（可选择丢帧或排队）
- `HAL_UART_TxCpltCallback()`：清 busy，并通知任务继续发送下一帧（若你做了队列）
- 发送缓冲策略：
  - 发送固定小帧：用静态 buffer（像 echo/VOFA 那样）最简单
  - 发送多来源多帧：用“TX 队列/环形缓冲 + copy”更稳（代价是多一次 memcpy）

## 6. 如何从 Echo 升级到协议解析（v2）

Echo 只是验证链路与“RX 字节流抽取”正确。接入协议时建议：

1) 保留 RX DMA + ringbuf 的基础设施不变
2) 在 `CommTask` 内用“解析器状态机”消费 ringbuf：
   - `while (ringbuf_available) { read 1..N bytes; feed(proto_rx, bytes); }`
3) 解析器输出完整帧后，按 `doc/plan/plan_v2.md` 的 `MSG_ID` 分发：
   - `SET_TARGET` → 写 `xHostCommandQueue`（深度=1，只保留最新）
   - `TELEM_CONFIG` → 更新遥测开关/周期
   - `ESTOP` → 置模式并触发保护
4) 需要发送响应/遥测时，通过统一 TX 模块发送，避免与其他发送路径冲突

## 7. VOFA+/串口助手测试方法

- 目的：确认“发什么回什么”
- 工具：VOFA+ 或任意串口助手
- 配置：`921600 8N1`
- 测试：
  - 发送 ASCII：`hello\\r\\n` → 应回显相同文本
  - 发送 HEX：`01 02 03 04` → 应回显同样 4 字节

提示：如果一次性连续发送很长数据且无停顿，IDLE 事件不一定触发；建议测试时分段发送或带换行/间隔。
