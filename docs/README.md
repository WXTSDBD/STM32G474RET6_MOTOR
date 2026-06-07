# 项目文档索引

本目录存放与本仓库相关的设计与 **工具链/工程** 说明，便于后续维护与交接。

## 工具链与工程配置（必读）

| 文档 | 内容 |
|------|------|
| [Keil_AC6_FreeRTOS_CubeMX适配说明.md](./Keil_AC6_FreeRTOS_CubeMX适配说明.md) | CubeMX + Keil AC6 + FreeRTOS 为何需要 `tools/` 脚本、日常 Generate→Rebuild 流程、排错、新工程迁移 |

快捷入口：脚本目录 [`../tools/README.md`](../tools/README.md)

## 系统与硬件设计

| 文档 | 内容 |
|------|------|
| [串口DMA遥测系统设计文档.md](./串口DMA遥测系统设计文档.md) | UART DMA 遥测（JustFloat、双缓冲、RTOS 线程等） |
| [电机驱动软件框架——完整架构设计文档.md](./电机驱动软件框架——完整架构设计文档.md) | 电机驱动软件架构 |
| [Foundation v1.0.0 硬件设计文档.md](./Foundation%20v1.0.0%20硬件设计文档.md) | 硬件设计 |

## 新人上手顺序（建议）

1. 读 **Keil AC6 FreeRTOS 适配说明** → 会 Generate、会排 RVDS/BOM 类错误  
2. 读 **电机驱动软件框架** → 理解代码结构  
3. 按需读 **串口 DMA 遥测** 等专题文档  
