# APP 层模块简化重构计划

## 背景

当前 `APP/` 目录包含多个独立模块，代码结构清晰但存在冗余 API 和临时实现。随着项目演进，部分模块需要简化以降低维护负担。

## 当前状态分析

### 文件清单与评估

| 文件 | 职责 | 代码行数 | 评价 |
|------|------|----------|------|
| `imu.h/.c` | IMU 封装、路由 HAL 回调 | ~130 | ✓ API 清晰，与 components 解耦 |
| `encoder.h/.c` | 编码器驱动、速度计算 | ~170 | △ API 过冗，实际只用 2 个 |
| `status_store.h/.c` | 无锁状态快照 | ~50 | ✓ 简洁优秀 |
| `vofa_telemetry.h/.c` | VOFA+ 遥测发送 | ~80 | △ 通道固定，灵活性低 |
| `oled_ui.h/.c` | OLED 显示界面 | ~100 | ✓ 简洁 |
| `uart_echo.h/.c` | 串口 DMA 回显测试 | ~120 | △ 临时性质，后续会被替代 |
| `uart3_hal_callbacks.c` | HAL 回调分发 | ~60 | ✓ 必要 |
| `mydefine.h` | 全局数据结构 | ~70 | ✓ 必要 |

### 主要问题

1. **`encoder.h` API 过冗**：5 个函数中仅 2 个常用
2. **`vofa_telemetry` 灵活性差**：固定 8 通道，难以扩展
3. **`uart_echo` 是临时实现**：后续会被协议模块替代
4. **`uart3_hal_callbacks.c` 是间接层**：后续可简化

---

## 简化方案

### P0：简化 encoder.h API

#### 当前 API（5 个函数）

```c
void Encoder_Init(const encoder_config_t *cfg);
bool Encoder_Update(float dt_s, encoder_reading_t *out);
bool Encoder_GetLatest(encoder_reading_t *out);
void Encoder_Reset(void);
void Encoder_SetConfig(const encoder_config_t *cfg);
```

#### 简化后 API（2 个函数）

```c
void Encoder_Init(const encoder_config_t *cfg);
bool Encoder_Update(float dt_s, encoder_reading_t *out);
```

#### 变更说明

| 删除的函数 | 原因 |
|------------|------|
| `Encoder_GetLatest()` | `StatusStore` 已实现快照，`Encoder_Update` 直接填充数据 |
| `Encoder_Reset()` | 极少使用，可合并到 `Encoder_Init` 或按需添加 |
| `Encoder_SetConfig()` | 运行时调参场景少，可重新初始化 |

#### 预期效果

- 头文件从 ~110 行减少到 ~60 行
- API 更聚焦，使用更简单

---

### P1：vofa_telemetry 通用化

#### 当前实现

```c
// 固定 8 通道
bool VOFA_SendPitch2Speed2_Dma(UART_HandleTypeDef *huart,
                               float pitch_deg,
                               float pitch_acc_deg,
                               float wheel_l_mps,
                               float wheel_r_mps);
```

#### 简化后实现

```c
// 通用 8 通道帧
typedef struct {
    float ch[8];  // 通道 0~7
} vofa_frame_t;

bool VOFA_SendFrame_Dma(UART_HandleTypeDef *huart, const vofa_frame_t *frame);

// 便捷宏（可选）
#define VOFA_MAKE_FRAME(pitch, pitch_acc, ax, ay, az, gx, gy, gz) \
    ((vofa_frame_t){{pitch, pitch_acc, ax, ay, az, gx, gy, gz}})
```

#### 使用示例

```c
// 当前
VOFA_SendPitch2Speed2_Dma(&huart3, pitch, pitch_acc, ax, ay, az, gx, gy, gz);

// 简化后
vofa_frame_t frame = {
    .ch[0] = pitch,
    .ch[1] = pitch_acc,
    .ch[2] = ax, .ch[3] = ay, .ch[4] = az,
    .ch[5] = gx, .ch[6] = gy, .ch[7] = gz
};
VOFA_SendFrame_Dma(&huart3, &frame);

// 或使用便捷宏
VOFA_SendFrame_Dma(&huart3, &VOFA_MAKE_FRAME(pitch, pitch_acc, ax, ay, az, gx, gy, gz));
```

#### 预期效果

- 可灵活添加速度、姿态等通道
- 兼容现有 8 通道用法
- 代码量略增但灵活性大幅提升

---

### P2：通信模块统一（长期）

#### 当前架构

```
HAL 回调
    │
    ▼
uart3_hal_callbacks.c （路由）
    │
    ├──► uart_echo.c （回显测试）
    │
    └──► future: comm.c （协议解析）[待实现]
```

#### 目标架构

```
HAL 回调
    │
    ▼
comm.c （统一处理）
    │
    ├──► 协议解析 （SET_TARGET, ESTOP...）
    ├──► 遥测发送 （STATUS, IMU_RAW...）
    └──► 调试模式 （可选：回显开关）
```

#### 演进路径

| 阶段 | 内容 |
|------|------|
| 阶段 1 | 创建 `comm.c`，实现协议解析，保持 `uart_echo.c` 独立 |
| 阶段 2 | `comm.c` 处理协议，echo 作为调试可选功能 |
| 阶段 3 | 移除 `uart_echo.c`、`uart3_hal_callbacks.c` |

#### 交付物

- `APP/comm.h/.c`：统一通信模块
- 复用 `ringbuf.c` 处理 RX 数据流
- 复用 `vofa_justfloat.c` 打包遥测帧

---

## 文件变更清单

| 操作 | 文件 | 说明 |
|------|------|------|
| 修改 | `APP/encoder.h` | 简化 API |
| 修改 | `APP/encoder.c` | 移除冗余函数 |
| 修改 | `APP/vofa_telemetry.h` | 改为通用接口 |
| 修改 | `APP/vofa_telemetry.c` | 实现通用接口 |
| 新建 | `APP/comm.h/.c` | 统一通信模块（阶段 1） |
| 删除 | `APP/uart_echo.h/.c` | 阶段 3 完成后删除 |
| 删除 | `APP/uart3_hal_callbacks.c` | 阶段 3 完成后删除 |

---

## 验收标准

### P0：encoder.h 简化

- [ ] `Encoder_GetLatest()` 调用点全部替换为 `StatusStore_Read()`
- [ ] `Encoder_Init()` 支持 `NULL` 配置（使用默认值）
- [ ] 编译通过，无链接错误
- [ ] 功能测试：编码器读取正常

### P1：vofa_telemetry 通用化

- [ ] 新 API `VOFA_SendFrame_Dma()` 可正常发送
- [ ] 现有调用点迁移完成
- [ ] 编译通过
- [ ] VOFA+ 能正常接收 8 通道数据

### P2：comm 模块（长期）

- [ ] 协议解析功能正常（SET_TARGET, ESTOP）
- [ ] 遥测发送正常
- [ ] 回显调试模式可选
- [ ] 移除 `uart_echo` 后功能完整

---

## 优先级与工作量

| 优先级 | 任务 | 工作量 | 风险 |
|--------|------|--------|------|
| P0 | encoder.h 简化 | 0.5 人天 | 低 |
| P1 | vofa_telemetry 通用化 | 0.5 人天 | 低 |
| P2 | comm 模块统一 | 2~3 人天 | 中（涉及协议设计） |

---

## 附录：当前 encoder.h 完整 API（待简化）

```c
// 配置参数
typedef struct {
    uint16_t ppr;              // 编码器 PPR (电机轴)
    float wheel_diameter_m;    // 轮径 (米)
    float gear_ratio;          // 减速比
    float lpf_alpha;           // 低通滤波系数
    bool left_inverted;        // 左轮方向取反
    bool right_inverted;       // 右轮方向取反
} encoder_config_t;

// 数据结构
typedef struct {
    int32_t delta_counts;
    float speed_mps;
    float speed_filtered_mps;
} encoder_data_t;

typedef struct {
    encoder_data_t left;
    encoder_data_t right;
    float avg_speed_mps;
    uint32_t timestamp_ms;
} encoder_reading_t;

// API（计划保留前 2 个）
void Encoder_Init(const encoder_config_t *cfg);
bool Encoder_Update(float dt_s, encoder_reading_t *out);
// 以下计划删除
bool Encoder_GetLatest(encoder_reading_t *out);
void Encoder_Reset(void);
void Encoder_SetConfig(const encoder_config_t *cfg);
```
