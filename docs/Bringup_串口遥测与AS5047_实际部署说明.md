# Bringup：串口 DMA 遥测与 AS5047 实际部署说明

本文档描述 **当前仓库已烧录运行** 的实现，与设计文档 / 分步计划对照，便于联调、Watch 与后续 FOC 接入。

**MCU**：STM32G474 @ 160 MHz  
**电机 TIM**：TIM1，中心对齐 PWM，**20 kHz**（`Period=3999`，`Prescaler=0`，`CENTERALIGNED1`）  
**最后更新**：2026-06（LL SPI1 DMA 链 + TIM1 只读缓存）

---

## 1. 总览

```text
                    ┌─────────────────────────────────────────┐
  TIM1 20kHz ISR    │ GetRaw() → as5047_spi1.raw              │
  (NVIC prio 1)     │ telem_bringup_tick() → 写 4KB 双缓冲     │
                    │ DWT → isr_delta / cyccnt_end            │
                    └─────────────────────────────────────────┘
                                        │
                    ┌───────────────────┴───────────────────┐
                    │                                       │
            后台（prio 2）                            RTOS 任务
                    │                                       │
  DMA1 Ch2 ISR      │ LL SPI1 双 CS 链                       │ UART_DMA_DEBUG_TASK
  TIM1 kick 读 AS5047│ → s_dma_raw 单槽缓存                   │ telem_bringup_try_send()
                    │                                       │ → HAL UART DMA 发 LPUART1
                    └───────────────────────────────────────┘
```

| 子系统 | 实现文件 | 要点 |
|--------|----------|------|
| 串口遥测 | `bringup/app_uart_dma_debug.c/.h` | ISR 写缓冲，任务 DMA 发送，JustFloat |
| AS5047 SPI1 | `bringup/as5047.c/.h` | 双 CS 协议 + TIM1 kick @20 kHz |
| SPI1 LL 底层 | `bringup/bsp_as5047_spi1_ll.c/.h` | 绕过 HAL DMA 热路径（默认开启） |
| TIM1 入口 | `Core/Src/main.c` | `HAL_TIM_PeriodElapsedCallback` |
| DMA 中断 | `Core/Src/stm32g4xx_it.c` | Ch2=编码器 LL；Ch1=LPUART TX |

**调试总结构**：`g_telem_dbg`（Keil Watch 直接添加）

---

## 2. 启动与调用顺序

`main()` 中与本文相关的初始化顺序（`Core/Src/main.c`）：

1. Cube 外设：`MX_DMA_Init()` → `MX_SPI1_Init()` / `MX_SPI3_Init()` → `MX_LPUART1_UART_Init()` → `MX_TIM1_Init()` 等
2. `AS5047_Init(&AS5047_spi1_PORT, &hspi1, GPIOA, GPIO_PIN_4)`
3. **`AS5047_DmaInit(&AS5047_spi1_PORT)`** — 阻塞 priming 读一次 + 启动 DMA 链（**此时尚未 `DWT_Init`**，见 §3.6）
4. `AS5047_Init(&AS5047_spi3_PORT, &hspi3, GPIOA, GPIO_PIN_15)` — SPI3 仍阻塞 HAL，未走 DMA
5. `angle_init()` — FOC 角度结构体初始化（`FOC_CAL`）
6. `telem_bringup_init()` — **`DWT_Init(160)`**、`g_telem_dbg` 清零
7. `HAL_TIM_PWM_Start`（TIM1 六路 PWM + CH4）→ **`HAL_TIM_Base_Start_IT(&htim1)`** — 20 kHz 周期中断（**在 RTOS 启动之前**）
8. `osKernelInitialize()` → `MX_FREERTOS_Init()` → `osKernelStart()`
9. **`UART_DMA_DEBUG_TASK`**（`osPriorityHigh`，`osDelay(1)`）周期调用 `telem_bringup_try_send()`

> **FreeRTOS 任务链接**：`app_freertos.c` 里 `UART_DMA_DEBUG_TASK` 为 `__weak` 空壳；**真实实现**在 `bringup/app_uart_dma_debug.c`，链接时由强符号覆盖。若遥测从不发送，检查是否链到了 bringup 里的实现。

### 2.1 编译开关 `BRINGUP_ADC_TEST`

`main.c` 中（撰写时默认 **`1`**）：

```c
#define BRINGUP_ADC_TEST 1
```

| 值 | 影响 |
|----|------|
| **1**（当前） | TIM8 PWM/IT、`setPhaseVoltage` 等 FOC 路径关闭；**TIM1 的 GetRaw + telem 仍运行** |
| 0 | 双电机 TIM8 与 FOC 相关代码参与编译；接 FOC 前需改 0 并重新验收 `isr_delta` |

本文 §1–§7 描述的是 **TIM1 bringup 路径**（编码器 LL 链 + 串口遥测），与 `BRINGUP_ADC_TEST` 无冲突。

### 2.2 Keil 工程依赖

除源码外，确认：

- `bringup/bsp_as5047_spi1_ll.c` 已加入 `MDK-ARM/*.uvprojx`（LL 模式）
- `Core/Src/stm32g4xx_it.c` USER CODE：`#include "bsp_as5047_spi1_ll.h"`，Ch2 仅 LL ISR

---

## 3. AS5047（SPI1）实际部署

### 3.1 硬件与 Cube 配置

| 项目 | 配置 |
|------|------|
| 编码器 | AS5047，SPI1，**16 bit**，CPOL=0 CPHA=1 |
| SCK 分频 | `Prescaler = 16` → SCK ≈ 10 MHz @ 160 MHz |
| CS | **PA4**，软件 CS（`AS5047_CS_L/H` → `HAL_GPIO_WritePin`） |
| DMA | **DMA1 Channel2** = SPI1_RX，**Channel3** = SPI1_TX |
| SPI3 | 另一路编码器，**阻塞 `AS5047_read`**；CS **PA15**，Prescaler **8**（快于 SPI1），无 DMA |

### 3.2 协议：双 CS 帧（不是 DMA 双缓冲）

AS5047 读 `ANGLEUNC` 必须 **两次 CS 事务**，每次 1 word：

```text
FRAME1: CS↓ → DMA 发 READ cmd  → CS↑   （丢弃本帧 rx，命令生效于下一帧）
FRAME2: CS↓ → DMA 发 NOP       → CS↑   （rx & 0x3FFF = 14 bit 角度）
```

状态机 `phase`：`IDLE(0) → FRAME1(1) → FRAME2(2) → IDLE`（下一拍由 TIM1 `DmaKick` 再启 FRAME1）

### 3.3 数据路径与延迟

- **单槽缓存**：`static volatile uint16_t s_dma_raw`，无 ping-pong。
- **TIM1 每拍**：`AS5047_DmaKick()`（若 IDLE）→ `as5047_spi1.raw = GetRaw()`。
- **后台链**：FRAME2 完成 → 写 `s_dma_raw` → **进入 IDLE**，不在回调里自续。
- **延迟与更新率**：
  - TIM1 **kick + 读缓存** 均为 **20 kHz**。
  - 一轮 wall ~8 µs/轮，稳态 **`enc_cplt_cnt / tick_total ≈ 1`**。
  - FOC 视角：角度最多 **1 个 PWM 周期** 旧。

### 3.4 SPI1 LL 热路径（唯一路径）

| 组件 | 说明 |
|------|------|
| 热路径 | `bsp_as5047_spi1_ll_*`：`start_word` / `restart_word` 轻量重启 |
| DMA1 Ch2 中断 | `bsp_as5047_spi1_ll_dma1_ch2_isr()` |
| Ch3 | NVIC 关闭，仅 Ch2 TC 驱动状态机 |
| 启动 | `DmaInit` 仅 priming + `ll_init`；**首拍 kick 由 TIM1** |

**Priming**：`AS5047_DmaInit()` 内仍用 **一次阻塞 `AS5047_read()`（HAL）** 灌初值，之后 SPI1 角度读仅走 LL 链。

### 3.5 关键 API

| 函数 | 说明 |
|------|------|
| `AS5047_DmaInit()` | 建 cmd/nop、priming、LL init、启动第一条链 |
| `AS5047_GetRaw()` | 读 `s_dma_raw`（TIM1 用） |
| `AS5047_DmaKick()` | 调试用手动 kick；**TIM1 不调用** |
| `AS5047_read()` / `GetAngle()` | 阻塞路径；SPI1 bringup 运行期勿与 LL 链并发 |

### 3.6 实测性能（LL，稳态快照参考）

| 指标 | 典型值 @ 160 MHz | 含义 |
|------|------------------|------|
| `enc_dma_kick_delta` | ~243 | TIM1 内 `DmaKick` → FRAME1 的 CPU cycle |
| `enc_dma_f1_cb_delta` | ~280 | FRAME1 完成 → 启 NOP 帧 |
| `enc_dma_f2_cb_delta` | ~36 | FRAME2 完成 → 存 raw |
| **kick+f1+f2 合计** | **~560 / 次完成读** | 相对 HAL 链 ~1782 明显下降 |
| `enc_dma_seq_delta` | ~1200–1300 | kick→FRAME2 完成 **wall cycle**（含硬件 SPI） |
| `enc_cplt_cnt / tick_total` | **≈ 1** | 链完成率与 TIM1 同频（~20 kHz） |
| `isr_delta` | ~350（轻拍）/ ~850（1% 写帧） | TIM1：DmaKick + GetRaw + telem |

**CPU 粗算（读角相关）**：~20 kHz 完成 × ~560 cycle ≈ **11 M cycle/s ≈ 7%**；TIM1 轻拍 ~20 kHz × ~350 cycle ≈ **7 M cycle/s ≈ 4.4%**（kick 在 TIM1，与 SPI 回调有重叠，非简单相加）；**整机编码器路径约 8–9%**（不含 FOC、不含 UART 发送任务 CPU）。若曾用链式自续（`cplt/tick≈2`），总 CPU 约 **15%**。

> ⚠️ **`enc_dma_seq_delta_max` 野值**：`DWT_Init()` 在 **`AS5047_DmaInit()` 之后** 才调用（`telem_bringup_init`），上电头几轮 DMA 可能在 CYCCNT 未使能/刚清零时更新 max，Watch 里偶见 ~4.29e9。**稳态只看 `enc_dma_seq_delta`（~1200–1300）**；若需干净 max，可在 `DWT_Init` 后手动清零或把 DWT 提前到 `DmaInit` 之前。

**LL 稳态实测 Watch 快照**（2026-06-10 01:08）：

| 字段 | 值 |
|------|-----|
| `isr_delta` | 99 |
| `enc_dma_kick_delta` | 243 |
| `enc_dma_f1_cb_delta` | 282 |
| `enc_dma_f2_cb_delta` | 36 |
| `enc_dma_cpu_delta` | 319 |
| `enc_total_delta` | 429 |
| `enc_cplt_cnt / tick_total` | ≈ 1.0 |
| `enc_err_cnt` | 0 |
| `enc_raw` | 4763 |

### 3.7 尚未接入 FOC

`main.c` 中 `setPhaseVoltage`、`AS5047_GetAngle` 仍注释。下一步：`raw → get（unwrap）→ setPhaseVoltage`。

---

## 4. 串口 DMA 遥测实际部署

### 4.1 硬件

| 项目 | 配置 |
|------|------|
| 外设 | **LPUART1**，PC0=RX，PC1=TX |
| 波特率 | **6 000 000** |
| TX DMA | **DMA1 Channel1** → `hdma_lpuart1_tx` |
| 协议 | **VOFA+ JustFloat**（帧尾 `00 00 80 7F`） |

### 4.2 双缓冲（4 KB × 2）

```text
状态: UNLOCKED → LOCKED（ISR 写入中）→ READY（满或 seal）→ SENDING（DMA）→ UNLOCKED
```

- 单缓冲 **4096 B**，双缓冲共 **8 KB SRAM**（`TELEM_BUF_BYTES = 4096`）。
- ISR **只 memcpy 小帧**，不在 TIM1 里调 `HAL_UART_Transmit_DMA`。
- **`UART_DMA_DEBUG_TASK`**（`osPriorityHigh`，`osDelay(1)`）调用 `telem_bringup_try_send()`（实现见 §2）。
- 发送完成：`HAL_UART_TxCpltCallback` → 缓冲 `UNLOCKED`。

### 4.3 TIM1 内：`telem_bringup_tick()`

- **每 TIM1 周期** `tick_total++`。
- ** decimation**：`TELEM_BRINGUP_DECIMATION = 100` → 99/100 拍 `tick_decim_skip++` 直接 return。
- **1/100 拍**写一个小帧（**200 Hz** 写帧率 @ 20 kHz TIM1）：
  - 小帧 **20 B**（4×float + 4 B tail，无 seq）
  - 满包约 **204 帧 ≈ 4 KB ≈ 1.0 s** 封包 READY

**小帧通道（JustFloat 4 通道）**：

| 通道 | 内容 | 备注 |
|------|------|------|
| ch0 | `(float)cnt` | `main.c` 里 `volatile uint8_t cnt`，TIM1 每拍 `++` |
| ch1 | 上一拍 `cyccnt_end` | **uint32 小端写入**，VOFA 勿当 float |
| ch2 | 上一拍 `isr_delta` | 同上 |
| ch3 | `as5047_spi1.get` | 当前多为 0（FOC 未接） |

写帧后 **`memcpy` 覆盖 ch1/ch2** 为真实 `cyccnt_end` / `isr_delta`（`TELEM_CH1/CH2_BYTE_OFF`）。

### 4.4 TIM1 ISR 负载与 `isr_delta`

`HAL_TIM_PeriodElapsedCallback`（TIM1）顺序：

1. DWT `isr_t0`
2. `AS5047_GetRaw()` → `as5047_spi1.raw`
3. `telem_bringup_tick()`
4. DWT `isr_t1` → `isr_delta = isr_t1 - isr_t0`

| 拍类型 | 典型 `isr_delta` | 原因 |
|--------|------------------|------|
| decim skip（99%） | **~99–180** | GetRaw + 计数 + DWT |
| 写帧（1%） | **~700–850** | 额外 `memcpy` 小帧到 4 KB 缓冲 |

**`isr_delta` 与 AS5047 DMA 无关**（SPI 在 prio-2 后台链）；SPI 耗时看 `enc_dma_*`。

### 4.5 RTOS 发送路径

`telem_bringup_try_send()`：

1. UART 非 `BUSY_TX`
2. 存在 `READY` 且 `used_bytes > 0` 的缓冲
3. `HAL_UART_Transmit_DMA(&hlpuart1, …)`
4. 失败/跳过计数：`dma_busy_skip`、`no_ready_skip`、`dma_start_fail`

### 4.6 边界情况与辅助变量

| 字段 / 变量 | 含义 |
|-------------|------|
| `acquire_fail` | 写帧时双缓冲均非 UNLOCKED，本拍丢弃写缓冲 |
| `dma_busy_skip` | UART 仍 `BUSY_TX`，任务未启动新 DMA |
| `no_ready_skip` | 无 READY 缓冲或 `used_bytes==0` |
| `time_cnt` | 等于最近一次写入小帧的 `isr_delta`（与 `ch2_wire` 同步） |
| `buf0_used` / `buf1_used` | 当前缓冲占用字节（**不是 CPU cycle**） |

本 bringup **不使用 LPUART RX** 传遥测；`USART1`（`huart1`）与 LPUART 遥测无关。

---

## 5. 中断优先级（实际）

| 中断 | NVIC prio | 用途 |
|------|-----------|------|
| TIM1 UP | **1** | 电机周期、`telem_bringup_tick`、GetRaw |
| DMA1 Ch2（SPI1 RX） | **2** | AS5047 LL 状态机 |
| DMA1 Ch1（LPUART TX） | **4** | 串口发送 DMA |
| LPUART1 | **4** | UART HAL |
| DMA1 Ch3（SPI1 TX） | 2（**LL 下 NVIC 关闭**） | 不与 Ch2 双入口 |

TIM1 可抢占 DMA 回调 → DWT 的 `enc_dma_f1_cb_delta` 偶发偏高；以 **kick+f1+f2 稳态 ~560** 为准。

---

## 6. `g_telem_dbg` Watch 字段速查

### 6.1 TIM1 / 遥测

| 字段 | 含义 |
|------|------|
| `isr_delta` | 本拍 TIM1 回调 CPU cycle |
| `cyccnt_end` | 本拍结束 DWT CYCCNT |
| `ch1_wire` / `ch2_wire` | 写入串口帧的 ch1/ch2 快照 |
| `tick_total` | `telem_bringup_tick` 调用次数 |
| `tick_decim_skip` | decim 提前 return 次数 |
| `tick_frame_ok` | 实际写小帧次数 |
| `buf0/1_state` | 0=UNLOCKED 1=LOCKED 2=READY 3=SENDING |
| `buf0/1_used` | 缓冲已写字节 |
| `dma_start_ok` / `tx_cplt_cnt` | UART DMA 发送统计 |
| `acquire_fail` | ISR 抢不到 UNLOCKED 缓冲 |
| `time_cnt` | 最近写入帧的 `isr_delta` 镜像 |

### 6.2 AS5047（`enc_*`）

| 字段 | 含义 |
|------|------|
| `enc_raw` | 最新 14 bit 角度（与 `GetRaw()` 一致） |
| `enc_dma_phase` | 0=IDLE 1=FRAME1 2=FRAME2 |
| `enc_cplt_cnt` | 完成读次数 |
| `enc_chain_kick_cnt` | 链式 kick 次数（≈ `enc_cplt_cnt` 量级） |
| `enc_kick_cnt` | 所有 FRAME1 启动次数（链式 + 手动 `DmaKick`） |
| `enc_kick_skip_busy` | 手动 `DmaKick` 时 phase≠IDLE；**TIM1 不 kick 后应不再增长** |
| `enc_kick_skip_spi` | kick 时 SPI 非 READY（LL 下少见） |
| `enc_err_cnt` | SPI/DMA 错误（应为 0） |
| `enc_dma_kick_delta` | 链式 kick CPU |
| `enc_dma_f1_cb_delta` / `enc_dma_f2_cb_delta` | 两帧回调 CPU |
| `enc_dma_cpu_delta` | f1+f2（**不含 kick**） |
| `enc_dma_seq_delta` | kick→FRAME2 完成 wall cycle |
| `enc_total_delta` | `isr_delta + enc_dma_cpu_delta`（**不含 kick**，且两个值可能来自不同时刻；**不应用此字段判断系统总负载，容易误判**） |

> ⚠️ **常见误判**：Watch 里看到 `enc_total_delta ≈ 800` 就以为"还有 800 cycle"。实际上这个值不含 kick，且 isr_delta 和 cpu_delta 是不同拍的。**正确的一轮 SPI CPU 看 `enc_dma_kick_delta + enc_dma_f1_cb_delta + enc_dma_f2_cb_delta`（同一快照时相加 ≈ 560）。**

---

## 7. 联调步骤

1. **Keil Watch** 添加 `g_telem_dbg` 或关键 `enc_*`、`isr_delta`。
2. 旋转电机轴：`enc_raw` / `as5047_spi1.raw` 应变化；`enc_err_cnt` 保持 0。
3. VOFA+：6M 波特率，JustFloat，通道 2 看 `isr_delta`（注意 ch1/ch2 为 uint32 小端）。
4. 对比 LL：`AS5047_SPI1_LL=1` vs `0` 看 `enc_dma_kick_delta` 与总和 cycle。
5. FOC 前：确认 `isr_delta` 轻拍 ~100、SPI 总和 ~560，再开 `setPhaseVoltage`。

---

## 8. 相关文档

| 文档 | 关系 |
|------|------|
| [遥测BSP分步实施计划.md](./遥测BSP分步实施计划.md) | 分步计划；**当前为 Step 6a 已落地** |
| [串口DMA遥测系统设计文档.md](./串口DMA遥测系统设计文档.md) | 设计级描述 |
| [完整联调回顾_串口DMA遥测与电机_2025-06-07.md](./完整联调回顾_串口DMA遥测与电机_2025-06-07.md) | 历史联调记录 |

---

## 9. 源码索引

```
bringup/
  app_uart_dma_debug.c/h    串口双缓冲遥测 + g_telem_dbg（强符号 UART_DMA_DEBUG_TASK）
  as5047.c/h                AS5047 驱动 + DMA 状态机
  bsp_as5047_spi1_ll.c/h    SPI1 LL DMA（AS5047_SPI1_LL=1）
  bsp_dwt.c/h               DWT_Init，供 isr_delta / enc_* 打点
Core/Src/
  main.c                    TIM1 回调、初始化顺序、BRINGUP_ADC_TEST
  stm32g4xx_it.c            DMA1 Ch2/Ch3 分发
  spi.c / usart.c / dma.c   Cube 外设与 DMA 映射
  app_freertos.c            UART_DMA_DEBUG 任务 weak 桩
```

---

## 10. 变更摘要（相对早期 HAL 阻塞读）

| 阶段 | TIM1 内 SPI | 读角 CPU/轮 | TIM1 读 / 后台更新 | 备注 |
|------|-------------|-------------|-------------------|------|
| 阻塞 `AS5047_read` | ~2000 | ~2000 | 20 kHz / 20 kHz | 全在 TIM1，RTOS 饿死 |
| HAL DMA + TIM1 kick | ~658（成功拍） | ~1380 | 20 kHz / ~10 kHz 链 | 隔拍 skip ~50%，TIM1 重 |
| HAL 链式自续 | 0 | ~1782 | 20 kHz / ~17.5 kHz 链 | kick 在 prio-2，总负载平移 |
| **LL + TIM1 kick（当前）** | **~250（kick）** | **~560** | **20 kHz kick / 20 kHz 更新** | **`cplt/tick≈1`；总 CPU ~8–9%** |

当前 bringup **编码器路径可冻结**；后续工作重点是 **FOC 接角度** 与按需优化 telem / CS BSRR。

---

## 11. 实际调试过程：（公司 AI 说"50 cycle 就够"，实测差了一个数量级）

以下按时间线记录 6 月 10 日晚的实际调试过程——每一步都有 DWT 实测数据。

### 11.1 起点的预期

SPI DMA 方案讨论阶段，公司 AI 给出的估计：

> "DMA 启动一发约 50 cycle，TIM1 内只读缓存约 3 cycle，总计约 50 cycle 搞定。"

按这个预期，SPI DMA 应该几乎免费。

### 11.2 第一轮实测：HAL DMA + TIM1 DmaKick

**实现**：CubeMX 配 SPI1 DMA，TIM1 ISR 末尾调 `HAL_SPI_TransmitReceive_DMA` 启动下一帧。

**Watch 实测**：

| 字段 | 值 |
|------|-----|
| `enc_dma_kick_delta`（TIM1 内 HAL 启动） | **~658 cycle** |
| `enc_dma_f1_cb_delta`（FRAME1 回调） | ~676 |
| `enc_dma_f2_cb_delta`（FRAME2 回调） | ~45 |
| `enc_dma_cpu_delta`（f1+f2） | ~722 |
| `enc_kick_skip_busy` / `enc_kick_cnt` | 约 50% / 50%（隔拍 skip） |
| `isr_delta` | ~181 / ~850 交替 |

**发现**：HAL 的 `HAL_SPI_TransmitReceive_DMA` 启动一次 **不是 50 cycle，是 658 cycle**。这是 HAL 入口检查、状态机、锁、DMA 寄存器配置、外设使能的真实成本。此外还有 ~50% 的拍因为 DMA 未 IDLE 而直接 skip——有效采样只有 ~10kHz。

**结论**：不是在 TIM1 里启动 DMA 就完事了。HAL 的重量远超预期。

### 11.3 第二轮改进：链式自续（kick 搬到 DMA 回调）

**实现**：FRAME2 完成回调末尾直接启动下一轮 FRAME1，TIM1 不再调任何 SPI 函数。

**Watch 实测**：

| 字段 | 改前（TIM1 kick） | 改后（链式自续） |
|------|-------------------|-------------------|
| `isr_delta` | ~181 / ~850 交替 | **~104 稳定** |
| TIM1 内 HAL SPI | 隔拍 ~658 | **0** |
| `enc_dma_kick_delta` | 658（HAL，TIM1 内） | **~1061**（HAL，prio-2 回调内） |
| `enc_dma_f1_cb_delta` | ~676 | ~676 |
| `enc_dma_f2_cb_delta` | ~45 | ~45 |
| 每轮总 CPU（kick+f1+f2） | ~1380 | **~1782** |
| 有效采样率 | ~10kHz | **~17.5kHz** |
| `enc_kick_skip_busy` | ~tick/2 | **不再增长** |

**发现**：
- TIM1 确实轻了（850→104），达到了设计目标——电机 ISR 变干净。
- 但总 CPU **不降反增**（1380→1782）。因为：（1）kick 挪到回调后仍然贵（HAL 启动 ~1061），（2）链完成率从 ~10 kHz 提到 ~17.5 kHz，每秒总 CPU 反而更高。
- **负载没有消失——是搬家。** 从 TIM1（prio 1）搬到了 DMA 回调（prio 2）。

### 11.4 第三轮：LL 替换 HAL（当前状态）

**实现**：新建 `bsp_as5047_spi1_ll.c`，绕过 HAL DMA 的 `HAL_SPI_TransmitReceive_DMA`，用 LL 寄存器直写。

**Watch 实测（稳态快照 01:08）**：

| 字段 | HAL 链 | LL 链 |
|------|--------|-------|
| `enc_dma_kick_delta` | ~1061 | **~243** |
| `enc_dma_f1_cb_delta` | ~676 | **~282** |
| `enc_dma_f2_cb_delta` | ~45 | **~36** |
| **每轮总 CPU** | **~1782** | **~561** |
| `isr_delta` | ~104 | **~99** |
| `enc_cplt_cnt/tick_total` | ~0.88 | **≈ 2.0** |
| `enc_err_cnt` | 0 | **0** |

**LL 省了 68%**（1782→561）。这才是"接近可接受"的量级——但仍然不是 50 cycle。

### 11.5 "50 cycle"和现实的差距

公司 AI 说的 50 cycle 指的是 LL 下 DMA 启动的**寄存器写操作本身**。但一轮完整的 AS5047 读需要：

```
kick (CS↓ + LL start cmd)        ~243
f1_cb (CS↑ CS↓ + LL start NOP)   ~282  
f2_cb (CS↑ + 存 raw + DWT 记账)  ~36
─────────────────────────────────────
合计                              ~561
```

多出来的开销来源：
- **CS 操作**：HAL_GPIO_WritePin × 6 次/轮（每帧 CS↓ + CS↑），每次 ~30-50 cycle
- **DWT 打点与 debug 字段写入**：~40 cycle/轮
- **DMA 寄存器操作**：Disable → Clear flag → Set CNDTR → Set CMAR → Enable，每轮 3 次 × ~15 个寄存器写
- **状态机分支与函数调用**：~10-20 cycle

**50 cycle 是"启动一次 DMA"的理论下限。561 cycle 是"真正完成一轮 AS5047 角度读"的全部 CPU 账单。**

### 11.6 核心教训

1. **不要根据理论估算做架构决策。** HAL 启动 DMA 的真实成本（658 cycle）是理论值（50 cycle）的 13 倍。
2. **DWT 实测是唯一可信的依据。** 每一步改进都有 `g_telem_dbg.enc_*` 的精确 cycle 计数支撑。
3. **"负载搬家"不是"负载消失"。** 链式自续让 TIM1 变轻，但总 CPU 平移甚至更高。只有 LL 替换 HAL 才真正减少了总开销。
4. **HAL 适合冷路径。** 在 20kHz 热路径上，`HAL_SPI_TransmitReceive_DMA` 的入口/出口/锁/状态机开销是不可接受的。
