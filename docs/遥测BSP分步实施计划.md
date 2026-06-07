# 遥测 BSP 分步实施计划（Bringup 阶段）

**版本**: v1.0  
**日期**: 2026-06-07  
**适用工程**: `STM32G474RET6_MOTOR`（当前为 bringup，无完整分层架构）  
**关联文档**: [串口DMA遥测系统设计文档.md](./串口DMA遥测系统设计文档.md)（协议与终版架构）  
**上位机**: VOFA+（JustFloat，6Mbps）

---

## 一、背景与目标

当前工程以 `bringup/` 为主，尚未落地架构文档中的 `service/debug`、`driver/comm` 目录。本计划在 **不改变整体架构的前提下**：

1. 在 `bringup/` 完成 **BSP + 遥测最小模块**（纯 C，不用 C++）
2. 分步实现、**每步可独立验证**（编译 / VOFA+ / 调试器 / 电机）
3. 为后期 CLI 预留 **运行时可变** 接口（decimation、k、通道指针、启停），不用 `const` 配置锁死
4. 终版热路径：`TIM1` 20kHz ISR 写缓冲；冷路径：RTOS 任务或（可选实验）ISR 试发 DMA

**Step 0 已完成**：Keil 编译通过；LPUART1 6Mbps + `HAL_UART_Transmit_DMA` 收发已验证。

---

## 二、为什么用 C，不用 C++

| 原因 | 说明 |
|------|------|
| 热路径在 ISR | TIM1 20kHz 回调，C 开销可预期 |
| 工程现状 | `bringup/`、`FOC_CAL`、`HAL` 回调均为 C |
| 复杂度 | 双缓冲 + 状态机 + `memcpy`，struct 足够 |
| C++ 风险 | ISR 中避免构造顺序、异常；与 HAL 混编需 `extern "C"` |

**约定**：

- **Bringup / MVP**：`bsp_telem_tx`、`bsp_telemetry` 全部 **纯 C**
- **以后**：CLI、注册表可用 C++，仅通过 `extern "C"` 调用 `telem_set_decimation()` 等冷路径 API；**ISR 仍只调 C 的 `telem_tick()`**

---

## 三、目录与分层（Bringup 期）

```text
bringup/
  bsp_telem_tx.h / bsp_telem_tx.c    # BSP：LPUART1 DMA 发、忙判断
  bsp_telemetry.h / bsp_telemetry.c  # 协议、双缓冲、运行时配置

Core/Src/main.c                      # 集成：init / TIM1 回调 / TxCplt
Core/Src/app_freertos.c              # Step 4 起：UART_DMA_DEBUG_TASK 发 DMA
```

```text
┌─────────────────────────────────────────┐
│  main.c / app_freertos.c（集成，4 根线）   │
└──────────────────┬──────────────────────┘
                   │ telem_init / tick / try_send / on_dma_done
┌──────────────────▼──────────────────────┐
│  bsp_telemetry.c   无 HAL include       │
│  JustFloat + 双缓冲 + g_telem_rt        │
└──────────────────┬──────────────────────┘
                   │ bsp_telem_tx_dma / busy
┌──────────────────▼──────────────────────┐
│  bsp_telem_tx.c    HAL hlpuart1 + DMA   │
└─────────────────────────────────────────┘
```

CubeMX 生成物 **一般不改**：`usart.c`（6Mbps）、`dma.c`（DMA1_Ch1）、`stm32g4xx_it.c`。

长期保留、与遥测并行维护：`tools/`、`docs/`、`bringup/` 内电机相关源文件。

---

## 四、宏 vs 运行时配置

### 适合编译期宏（CLI 不改）

| 宏 | 默认 | 含义 |
|----|------|------|
| `TELEMETRY_ENABLE` | 1 | 0 = 整模块不参与编译 |
| `TELEMETRY_BUFFER_BYTES` | 4096 | 单缓冲 SRAM 大小，静态分配 |

协议帧尾 `{0x00,0x00,0x80,0x7f}` 为 JustFloat 常量，写死在 `bsp_telemetry.c`。

### 必须运行时变量 + setter（留给 CLI）

```c
typedef struct {
    uint8_t  running;         /* telem on/off */
    uint8_t  decimation;      /* D，默认 1 */
    uint8_t  channel_count;   /* k，默认 4 */
    float   *channels[16];    /* 注册表指针 */
} telem_runtime_t;

extern telem_runtime_t g_telem_rt;
```

| API（冷路径） | 用途 |
|---------------|------|
| `telem_init_defaults(void)` | 默认通道绑定 |
| `telem_start()` / `telem_stop()` | 启停发送 |
| `telem_set_decimation(uint8_t d)` | 改降采样 |
| `telem_set_channel_count(uint8_t k)` | 改通道数 |
| `telem_set_channel(uint8_t idx, float *ptr)` | 改某路数据源 |

**不要用 `const` 包住整个配置结构体**；通道名目录可静态只读，**当前选用哪些 `float*` 必须可变**。

### ISR 热路径 API

| 函数 | 调用者 |
|------|--------|
| `telem_tick(void)` | TIM1 20kHz 回调 |
| `telem_try_send(void)` | Step 4：RTOS 任务；Step 5（可选）：ISR |
| `telem_on_dma_done(void)` | `HAL_UART_TxCpltCallback`（`hlpuart1`） |

---

## 五、集成契约（4 根线）

| 位置 | 调用 |
|------|------|
| `main` USER CODE 2 | `telem_init_defaults()`；**不得**再保留与遥测冲突的测试 `HAL_UART_Transmit_DMA` |
| TIM1 `HAL_TIM_PeriodElapsedCallback` Callback 1 | FOC 逻辑后 `telem_tick()`；（Step 5 可选）`telem_try_send()` |
| `UART_DMA_DEBUG_TASK` | Step 4 起 `telem_try_send()` + `osDelay(1)` |
| `HAL_UART_TxCpltCallback` | `huart == &hlpuart1` → `telem_on_dma_done()` |

---

## 六、分步实施与验收标准

### 总览

```text
Step 0  环境 / 去冲突          ? 已完成（6Mbps HAL TX 已验证）
Step 1  BSP 裸发               ? 实质完成（可并入 Step 2 正式封装）
Step 2  单帧 JustFloat（慢发）  ? 当前下一步
Step 3  ISR 只写缓冲（低频）
Step 4  双缓冲 + 任务发 DMA
Step 5  （可选）ISR 试发 DMA
Step 6  decimation=1 满速 20kHz
Step 7  电机联调（真实通道）
Step 8  运行时 API（CLI 预留）
```

---

### Step 0：环境与冲突清理 ?

**内容**：

- Keil Rebuild 0 Error
- LPUART1 6Mbps、DMA1_Channel1 已由 CubeMX 配置
- 确认不与遥测抢 DMA（`main.c` 中一次性测试 `HAL_UART_Transmit_DMA` 应在 Step 2 前删除或注释）

**验收**：编译通过；6Mbps 收发已实测可用。

---

### Step 1：BSP 裸发 ?（可正式封装）

**内容**：

- `bsp_telem_tx.c/h`：`bsp_telem_tx_busy()`、`bsp_telem_tx_dma(buf, len)`
- 内部使用 `hlpuart1`、`HAL_UART_Transmit_DMA`

**验收**：与 Step 0 等价的 DMA 完成行为（可与 Step 2 一并提交）。

---

### Step 2：单帧 JustFloat（main 慢发）? 当前

**内容**：

- 新建 `bsp_telemetry.c/h`（最小版）：组一帧 = `seq(4B) + k×float + 帧尾`
- 在 `main` 或 `StartDefaultTask` 每 **100ms** 发一帧测试值（如 1.0, 2.0, 3.0, 4.0）
- 实现 `HAL_UART_TxCpltCallback` → 标记完成（后续改为 `telem_on_dma_done`）

**不做**：ISR、`g_telem_rt` 全套、双缓冲、decimation。

**VOFA+ 设置**：

| 项 | 值 |
|----|-----|
| 波特率 | 6000000 |
| 协议 | JustFloat |
| CH_COUNT | **5**（k=4 时，含 seq） |
| 通道 0 | 隐藏 |

**通过标准**：四路测试波形稳定；VOFA+ 不胀缓冲、固件不卡死。

**失败排查**：

- 有数据但乱 → float 小端 / 帧尾四字节
- 无数据 → 回查 BSP / RS485 / 波特率
- VOFA+ 卡死 → 帧尾不是 `0x7F800000` 小端

**Keil**：bringup 组加入 `bsp_telem_tx.c`、`bsp_telemetry.c`（`../bringup` 已在 Include Path）。

---

### Step 3：ISR 只写缓冲（不发 DMA）

**内容**：

- `telem_tick()` 写入缓冲（可先单缓冲，再双缓冲）
- 引入 `g_telem_rt`、`telem_init_defaults()`，默认绑 `&as5047_spi1.get`、`&uq` 等 **全局/静态** 地址
- TIM1 Callback 1 末尾调用 `telem_tick()`
- **`decimation = 200`**（约 100Hz），降低调试压力
- **不在 ISR 启动 DMA**；用调试器观察 `seq`、`write_idx` 递增

**通过标准**：电机行为不变；`seq` 持续递增；CPU 负载可接受。

---

### Step 4：双缓冲 + RTOS 任务发 DMA（推荐终版发送路径）

**内容**：

- 状态机：`UNLOCKED` → `LOCKED` → `READY` → `SENDING` → `UNLOCKED`
- `telem_tick()` 写满 → `READY`
- **`UART_DMA_DEBUG_TASK`** 循环调用 `telem_try_send()`，`osDelay(1)`
- `HAL_UART_TxCpltCallback` → `telem_on_dma_done()`
- DMA `Size` 传 **`used_bytes`**，不传 4096

**通过标准**：VOFA+ 连续波形（decim=200 时约 100Hz）；长时间不卡死；双缓冲状态轮换正常。

**失败排查**：

- 只出一包 → `on_dma_done` 未接线或 `used_bytes` 错误
- 断档 → 任务优先级 / `osDelay` 过大

---

### Step 5：（可选）ISR 内 `telem_try_send`

**内容**：

- TIM1 回调：`telem_tick()` 后调用 `telem_try_send()`
- 任务内 `telem_try_send()` **关闭**，避免双入口抢 DMA
- 仍保持 `decimation=200` 先试

**通过标准**：与 Step 4 相当的 VOFA+ 表现；无 HardFault、电机不异常。

**若不通过**：**回退到 Step 4**（ISR 只写、任务发 DMA）即为正式方案，非失败。

---

### Step 6：满速（decimation = 1）

**内容**：

- `g_telem_rt.decimation = 1`，k=4
- 设计文档目标：约 80% 带宽利用率，seq 连续

**通过标准**：四路波形流畅；seq 偶发跳号可接受（丢采样）；电机控制无明显恶化。

---

### Step 7：电机联调

**内容**：

- `BRINGUP_ADC_TEST = 0`，PWM + `setPhaseVoltage` 运行
- 通道绑真实量：角度、Uq、电流、`cnt` 等

**通过标准**：电机转 + VOFA+ 物理量合理；改 `uq` 波形跟随变化。

---

### Step 8：运行时 API（CLI 预留，先调试器验证）

**内容**：

- 实现 `telem_start/stop`、`telem_set_decimation`、`telem_set_channel`、`telem_set_channel_count`
- 暂不接 CLI；调试器或临时代码改 `g_telem_rt` 验证

**通过标准**：

- `telem_stop()` 后 VOFA+ 停更
- `telem_set_decimation(10)` 后频率约为原来的 1/10
- 换通道指针后对应曲线变化

---

## 七、VOFA+ 检查清单（Step 2 起每步复用）

1. 波特率 **6000000**
2. 协议 **JustFloat**（非 Raw / FireWater）
3. **CH_COUNT = k + 1**（k=4 → 5）
4. 通道 0 不显示（seq）
5. USB 转 RS485 需支持 6M；若不支持，先用低波特率验证接线后再升 6M

---

## 八、最短路径（时间紧时）

```text
Step 0 ? → Step 2（VOFA+ 单帧）→ Step 4（任务 + 双缓冲）→ Step 6（满速）→ Step 7（电机）
```

Step 5（ISR 发 DMA）为 **可选实验**；不稳定则固定使用 Step 4。

---

## 九、与终版架构的迁移

| Bringup 现状 | 以后 |
|--------------|------|
| `bringup/bsp_telemetry` | 可迁至 `service/debug/telemetry_stream` |
| `bringup/bsp_telem_tx` | 可迁至 `driver/comm/lpuart_telemetry_tx` |
| `g_telem_rt` + setter | 直接供 CLI 使用 |
| ISR 内 `try_send`（若做过） | 改为仅 RTOS 任务调用同一函数 |

热路径函数名（`telem_tick` / `telem_try_send` / `telem_on_dma_done`）建议保持不变，减少 `main.c` 改动。

换 HAL → LL / 裸寄存器：**只替换 `bsp_telem_tx.c`**，`bsp_telemetry.c` 不动。

---

## 十、相关文件索引

| 类型 | 路径 |
|------|------|
| 协议与设计 | `docs/串口DMA遥测系统设计文档.md` |
| 工具链 | `docs/Keil_AC6_FreeRTOS_CubeMX适配说明.md`、`tools/` |
| LPUART / DMA | `Core/Src/usart.c`、`Core/Src/dma.c` |
| TIM1 FOC ISR | `Core/Src/main.c` Callback 1 |
| RTOS 任务 | `Core/Src/app_freertos.c`（`UART_DMA_DEBUG_TASK`） |
| 待新建 | `bringup/bsp_telem_tx.c/h`、`bringup/bsp_telemetry.c/h` |
| Keil 工程 | `MDK-ARM/STM32G474RET6_MOTOR.uvprojx`（bringup 组加源文件） |

---

## 十一、日常工程流程

```text
关 Keil → CubeMX Generate（After 脚本自动）→ Keil Rebuild
```

`bringup/` 下自研文件不会被 CubeMX 覆盖；Generate 后确认 Keil 工程中 telemetry 源文件仍在。

---

## 十二、修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-06-07 | 初版：Bringup 分步计划、C-only、运行时 API 约定；Step 0/1 已完成 |
