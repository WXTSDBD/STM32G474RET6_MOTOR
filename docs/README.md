# 项目文档索引

本目录存放与本仓库相关的 **设计/部署** 说明，便于后续维护与交接。

## 构建与工程配置（必读）

| 文档 | 内容 |
|------|------|
| [Keil_AC6_FreeRTOS_CubeMX配置说明.md](./Keil_AC6_FreeRTOS_CubeMX配置说明.md) | CubeMX + Keil AC6 + FreeRTOS 为主要 `tools/` 脚本、日常 Generate、Rebuild 流程、寄存器级工程迁移 |

脚本目录 [`../tools/README.md`](../tools/README.md)

## FOC 与 bringup 联调

| 文档 | 内容 |
|------|------|
| [**FOC后续实施计划.md**](./FOC后续实施计划.md) | **当前基线 + Step1～4 路线图**（TIM8/ADC2/SPI1，下一步电流环） |
| [**ADC采样与config层部署计划_2026-06-09.md**](./ADC采样与config层部署计划_2026-06-09.md) | **Step 1.1～1.2 部署**：adc_sample + bsp_axes + bridge，零偏标定 |
| [**出厂校准与编码器零偏_实施计划_2026-06-09.md**](./出厂校准与编码器零偏_实施计划_2026-06-09.md) | **编码器 add 锁转子标定 + 可插拔出厂整定 + 片内 NVM**（MVP→P1→P3） |
| [有感FOC电流环_死区补偿_参数辨识_分步实施计划.md](./有感FOC电流环_死区补偿_参数辨识_分步实施计划.md) | Step 3/4 方法详述 |
| [电流环部署计划_Step1.md](./电流环部署计划_Step1.md) | Step 1 子步骤 |
| [联调日报_编码器DMA与SVPWM合并_2026-06-09.md](./联调日报_编码器DMA与SVPWM合并_2026-06-09.md) | cycle 基线、encoder+SVPWM 合并记录 |

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
3. **FOC 后续实施计划** → 当前进度与 Step1～4 顺序  
4. **ADC 采样与 config 层部署计划** → Step 1.1 落地 adc_sample + bsp_axes  
5. **出厂校准与编码器零偏实施计划** → 锁转子测 add、出厂整定、片内 NVM  
6. **FOC 控制环架构设计说明** → 接电流环 / PLL  
7. 遥测 bringup → **Bringup 串口遥测与 AS5047 实际部署说明**  
8. 其余按需查阅  
