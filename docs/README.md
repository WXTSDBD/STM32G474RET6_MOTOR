# 项目文档索引

本目录存放与本仓库相关的 **设计/部署** 说明，便于后续维护与交接。

## 构建与工程配置（必读）

| 文档 | 内容 |
|------|------|
| [Keil_AC6_FreeRTOS_CubeMX配置说明.md](./Keil_AC6_FreeRTOS_CubeMX配置说明.md) | CubeMX + Keil AC6 + FreeRTOS 为主要 `tools/` 脚本、日常 Generate、Rebuild 流程、寄存器级工程迁移 |

脚本目录 [`../tools/README.md`](../tools/README.md)

## 系统与硬件设计

| 文档 | 内容 |
|------|------|
| [SPI编码器架构实现说明.md](./SPI编码器架构实现说明.md) | **M1 v1.0 已冻结**；五层架构、API、性能；**§14 故意遗留的技术债务** |
| [FOC控制环架构设计说明.md](./FOC控制环架构设计说明.md) | FOC 电流环、PLL @ 2 kHz、C/C++ 边界、cycle 预算、迁移阶段 |
| [Bringup_串口遥测与AS5047_实际部署说明.md](./Bringup_串口遥测与AS5047_实际部署说明.md) | LPUART DMA 遥测 + AS5047 Watch（部分路径以 SPI 架构文档为准） |
| [串口DMA遥测系统设计文档.md](./串口DMA遥测系统设计文档.md) | UART DMA 遥测（JustFloat、双缓冲、RTOS 线程等） |
| [遥测BSP分层实施计划.md](./遥测BSP分层实施计划.md) | Bringup 阶段遥测 BSP 分层（**实施时必读**） |
| [编码器驱动与功能、驱动架构设计文档.md](./编码器驱动与功能、驱动架构设计文档.md) | 早期编码器架构设计 |
| [电机驱动软件框架——完整架构设计文档.md](./电机驱动软件框架——完整架构设计文档.md) | 通信域 + 控制域长期愿景 |
| [Foundation v1.0.0 硬件设计文档.md](./Foundation%20v1.0.0%20硬件设计文档.md) | 硬件设计 |

## 推荐阅读顺序（建议）

1. **Keil AC6 FreeRTOS 配置说明** → Generate、Rebuild  
2. **SPI 编码器架构实现说明** → M1 已冻结，FOC 只调用 API，不改骨架  
3. **FOC 控制环架构设计说明** → 接电流环 / PLL  
4. 遥测 bringup → **Bringup 串口遥测与 AS5047 实际部署说明**  
5. 其余按需查阅  
