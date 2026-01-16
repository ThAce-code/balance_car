# OLED 显示（SSD1306 0.96"）开发计划

## 目标（MVP）

- OLED 仅用于本机调试：**稳定显示 Pitch 角**，无需曲线、无需菜单交互。
- 显示布局：**一行 Pitch + 角落 mode/fault 状态**（避免信息过多导致刷新慢/字体太小）。

建议布局示例（128x64）：

```
PITCH:+12.34deg
L:+0.00 R:+0.00
M:1  F:0003
```

## 硬件与配置假设

- 屏幕：0.96" SSD1306，分辨率 128x64。
- 总线：I2C1（PB6=SCL，PB7=SDA），设备地址 0x3C（7-bit）。
- 工程侧配置：CubeMX 里 I2C1 速率建议 400kHz；屏幕模块自带上拉电阻（已确认）。

## 软件架构与数据流

- `MainControlTask@500Hz`：计算姿态并写入“最新状态快照”`StatusStore`（单写者、多读者）。
- `OLEDDisplayTask@100ms`：从 `StatusStore` 读取最新 `StatusData_t` 快照并刷新显示。
- 底层驱动：复用 `components/OLED/ssd1306.*`（阻塞式 HAL I2C 写，刷新需在低优先级任务中进行）。

## 文件结构（新增/复用）

- 复用：`components/OLED/ssd1306.c/.h`、`components/OLED/ssd1306_fonts.c/.h`
- 新增：`APP/oled_ui.c/.h`（仅做 UI 封装与格式化，避免业务逻辑散落在任务里）
- 可选：在 `doc/basic_info.md` 追加 OLED/I2C1 引脚与速率说明（如后续需要）

## API 设计（APP 层）

- `void OledUi_Init(void);`
  - 初始化 SSD1306、清屏、写静态标题（可选）。
- `void OledUi_Render(const StatusData_t *s);`
  - 根据 `s->pitch_deg / s->mode / s->fault_bits` 格式化字符串并刷新屏幕。
  - 约束：只在 OLEDDisplayTask 中调用，保证线程安全。

## 任务集成要点

- OLEDDisplayTask 循环周期：100ms（可按观感调整到 50~200ms）。
- 取数策略：`StatusStore_Read()` 读取“最新快照”（内部用序号保证不读到写一半的数据）。
- 刷新策略：默认全屏刷新（库的 `ssd1306_UpdateScreen()`）；如观感/性能不够再考虑“局部刷新/减少字体”。

## 验收标准

- 上电后 OLED 能稳定显示 `PITCH:+x.xxdeg`、`L/R` 轮速，并显示 `mode` 与 `fault_bits`。
- 车体缓慢前后倾斜时，显示数值方向与 VOFA+ 上的 pitch 一致（正负号一致）。
- OLED 刷新不影响控制环：`MainControlTask` 无明显超时/丢帧（以线程 flag / fault_bits 观测）。
