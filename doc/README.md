# 文档索引

本目录包含项目的设计文档、开发计划和使用指南。

## 目录结构

```
doc/
├── basic_info.md          # 平台与硬件基础信息（引脚、时钟、外设）
├── plan/                  # 开发计划
│   ├── plan_v2.md         # 【主要参考】任务架构与通信协议规划
│   ├── imu_plan.md        # IMU 开发计划
│   ├── uart_plan.md       # 串口通信开发计划
│   ├── oled_plan.md       # OLED 显示开发计划
│   └── plan_v1.md         # (已废弃) 早期任务架构规划
├── impl/                  # 实现说明
│   ├── imu_dataflow.md    # IMU 数据流转与处理说明
│   └── uart_dma_idle.md   # USART3 DMA + IDLE 实现说明
├── guide/                 # 教程与指南
│   └── api_guide.md       # API 使用指南（components & APP 层）
└── reference/             # 参考资料
    └── 规格书-手册/        # 芯片数据手册
```

---

## 快速导航

### 新手入门

1. **[basic_info.md](basic_info.md)** — 了解硬件配置（引脚、时钟、外设）
2. **[guide/api_guide.md](guide/api_guide.md)** — 学习如何使用各模块 API

### 开发参考

| 文档 | 说明 | 适用场景 |
|------|------|----------|
| [plan/plan_v2.md](plan/plan_v2.md) | 任务架构、通信协议规范 | 理解系统设计、添加新功能 |
| [impl/imu_dataflow.md](impl/imu_dataflow.md) | IMU 数据流转说明 | 调试 IMU、修改采样逻辑 |
| [impl/uart_dma_idle.md](impl/uart_dma_idle.md) | 串口 DMA 实现说明 | 调试串口、实现协议解析 |

### 模块开发计划

| 文档 | 状态 | 说明 |
|------|------|------|
| [plan/imu_plan.md](plan/imu_plan.md) | 已完成 | IMU 采集、标定、滤波 |
| [plan/uart_plan.md](plan/uart_plan.md) | 进行中 | 串口通信协议 |
| [plan/oled_plan.md](plan/oled_plan.md) | 进行中 | OLED 显示 |

---

## 文档维护说明

- **引脚/外设变更**：同步更新 `balance_car.ioc` 和 `basic_info.md`
- **任务/协议变更**：同步更新 `plan/plan_v2.md`
- **新增模块**：在 `plan/` 添加开发计划，在 `impl/` 添加实现说明
