# IMU 数据流转与处理说明（ICM20602 + DRDY + SPI DMA）

> 本文档描述当前工程里 IMU 数据从“硬件采样”到“任务处理/滤波/发布”的完整链路，便于你后续把编码器、控制器、上位机遥测接进来时不走弯路。

## 1. 角色与文件位置

- 组件层（底层驱动）：`components/imu/imu_spi.c/.h`
  - 负责 ICM20602 寄存器初始化、DRDY 触发后的 SPI DMA burst 读取、DMA 双缓冲、就绪标志与统计计数。
- 组件层（滤波算法）：`components/imu/imu_filter.c/.h`
  - 当前只提供 pitch 的互补滤波更新（gyro 积分 + acc 角度融合）。
- 应用层封装：`APP/imu.c/.h`
  - 负责把 CubeMX 的 HAL 回调分发到组件层，并用 `osThreadFlagsSet()` 唤醒 `MainControlTask`。
- 控制任务（消费与发布）：`Core/Src/freertos.c` 的 `main_control_task()`
  - 等待“新样本事件”，取最新 raw → 标定 → 单位换算 → pitch 估计 → 发布 `StatusData_t`。

## 2. 硬件与配置要点（当前约定）

- IMU：ICM20602
- ODR：500Hz（2ms/帧）
- DRDY/INT：`PC5`（EXTI 上升沿）
- SPI：SPI1，`PA5(SCK) / PB4(MISO) / PB5(MOSI)`，约 `10.5Mbit/s`
- CS：`PB12`
- WHO_AM_I：严格校验 `0x12`

## 3. 数据流转（从中断到任务）

### 3.1 事件链路（推荐理解为“DRDY 驱动的流水线”）

1) ICM20602 产生 **DRDY** 脉冲（500Hz）
2) `HAL_GPIO_EXTI_Callback()`（中断上下文，`APP/imu.c`）
   - 判断 `GPIO_Pin == IMU_INT_Pin` 后调用 `icm20602_spi_on_drdy_isr()`
3) `icm20602_spi_on_drdy_isr()`（中断上下文，`components/imu/imu_spi.c`）
   - 若 DMA 空闲：立即启动一次 `HAL_SPI_TransmitReceive_DMA()` burst 读取（14B）
   - 若 DMA 正忙：只累计 `drdy_pending`（用于追帧与统计）
4) `HAL_SPI_TxRxCpltCallback()`（中断上下文，`APP/imu.c`）
   - `icm20602_spi_on_txrx_cplt_isr()`：标记 ready 缓冲、更新计数
   - `osThreadFlagsSet(MainControlTaskHandle, IMU_THREAD_FLAG_NEW_SAMPLE)` 唤醒主控任务
5) `MainControlTask`（任务上下文，`Core/Src/freertos.c`）
   - `osThreadFlagsWait(IMU_THREAD_FLAG_NEW_SAMPLE, ..., timeout=5ms)` 等新样本事件
   - `IMU_TakeLatestRaw()` 取“最新已完成的一帧 raw 数据”

### 3.2 为什么用 DMA 双缓冲

- DMA 在写入 `rx_buf[active]` 时，任务侧只读取 `rx_buf[ready]`，避免读写同一块内存导致数据撕裂。
- “双缓冲 + ready 标志”也避免了在任务里等待 DMA 完成（任务不阻塞，实时性更好）。

## 4. 任务侧处理（标定、单位换算、互补滤波）

### 4.1 陀螺零偏标定（启动阶段）

- `MainControlTask` 前 500 帧（约 1 秒）对 `gx/gy/gz` 做在线均值估计作为 bias（running average）。
- 标定期间建议不输出电机 PWM，保证小车静止；遥测仍可正常发送用于观察姿态/噪声。

### 4.2 单位换算（当前默认量程）

当前寄存器配置量程为：
- Gyro：±2000 dps ⇒ `gyro_dps = raw_g / 16.4`
- Accel：±8 g ⇒ `acc_g = raw_a / 4096.0`

如果你后续改了 FSR 寄存器（例如 ±500dps/±4g），必须同步更新比例系数与代码注释。

### 4.3 pitch 轴向与计算

你当前约定：“X 轴为 pitch 轴（绕 X 旋转是俯仰）”，因此：
- 加速度推 pitch：`pitch_acc = atan2(ay, az)`
- 陀螺积分用 `gx`
- 互补滤波输出 `pitch_deg`

提示：轴向/正负号与安装方向强相关，后续调试时可先做“静止倾斜实验”验证符号是否一致。

## 5. 状态发布与队列策略

- `MainControlTask` 每帧产生 `StatusData_t`，写入 `StatusStore`（单写者、多读者最新快照），供 `CommTask/OLEDTask` 随时读取。
- 深度=1 的含义：不追求“全量历史”，只保留“最新状态快照”，队列满时丢弃旧的，避免排队延迟累积。

## 6. 异常与保护（当前实现/建议扩展）

已做：
- `WHO_AM_I` 严格校验：初始化失败直接返回
- `MainControlTask` 等待新样本超时：置 `fault_bits`（后续应在此处强制停 PWM）

建议后续补强：
- SPI 错误恢复：`HAL_SPI_ErrorCallback()` 计数后，任务检测阈值并复位 IMU/SPI
- DRDY 过载统计：关注 `overrun_count/drdy_pending`，评估是否需要降低 ODR 或优化 ISR
- dt 自适应：目前 `kDtS=0.002` 常量；可改用 `raw.timestamp_ms` 或 DWT 周期计算真实 dt
- 线加速度门控：当 `|sqrt(ax^2+ay^2+az^2) - 1g|` 较大（急加速/急刹/飞坡）时，短时间忽略加速度角，仅用陀螺积分更新，减少姿态抖动
