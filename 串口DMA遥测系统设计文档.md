# 串口 DMA 遥测系统设计文档（终版）

**版本**: v1.0
**作者**: 吴俊毅
**日期**: 2026-06-07
**适用平台**: STM32G474RBT6 + FreeRTOS
**上位机**: VOFA+（原生兼容）+ Python 分析脚本

---

## 一、总览

| 项目 | 参数 |
|------|------|
| 物理接口 | LPUART1 + RS485 半双工差分总线，硬件自动 DE/RE |
| 波特率 | 6Mbps（实际吞吐 600KB/s） |
| 协议 | 带 uint32 序号的 JustFloat 流 |
| 上位机 | VOFA+ 直读（隐藏通道一）+ Python 分析 |
| SRAM | 8KB（两个 4KB buffer） |
| ISR 开销 | k=4 时 8-9 cycle（0.11% @ 8000 cycle/tick @ 160MHz） |
| 编译控制 | `TELEMETRY_ENABLE` 宏 |

---

## 二、协议基础：VOFA+ JustFloat

### 协议原文

```cpp
#define CH_COUNT 通道数量
struct Frame {
    float fdata[CH_COUNT];
    unsigned char tail[4]{0x00, 0x00, 0x80, 0x7f};
};
```

帧尾固定为 `{0x00, 0x00, 0x80, 0x7f}`，即 IEEE 754 正无穷（`0x7F800000`）。这是 JustFloat 协议的**唯一帧边界标志**。

### 为什么必须用这个帧尾

JustFloat 是纯字节流协议——没有魔术字、没有长度前缀、不计数字节数。协议引擎唯一能做的就是在收到的字节流中扫描 `{0x00, 0x00, 0x80, 0x7f}`。出现了这四个字节，就认为一帧结束，前面收到的字节就是 CH_COUNT 个 float。

如果帧尾不是这四个字节，协议引擎永远找不到帧尾，缓冲区无限膨胀直到内存耗尽，软件卡死。

---

## 三、帧结构

### 一个小帧（一个采样）

| 字段 | 大小 | 类型 | 说明 |
|------|------|------|------|
| seq | 4B | uint32 小端 | 小帧序号，每采样自增 1 |
| data | 4k B | float[k] | k 个 IEEE 754 小端 float |
| tail | 4B | `{0x00,0x00,0x80,0x7f}` | JustFloat 固定帧尾 |
| **合计** | **4k+8 B** | | |

`CH_COUNT = k + 1`。VOFA+ 将 seq 显示为通道 0（隐藏即可），通道 1~k 为实际数据。

### 一个 buffer（4KB）

N 个小帧连续存放，无帧头、无 CRC、无大包序号。buffer 末尾不足一帧的零头不发送。

```
┌──────────────────────────────────────┐
│ 小帧[0]: seq + float[0..k-1] + tail  │
│ 小帧[1]: seq + float[0..k-1] + tail  │
│ ...                                  │
│ 小帧[N-1]: seq + float[0..k-1] + tail│
│ (padding: 不足一帧的剩余字节)          │
└──────────────────────────────────────┘
```

---

## 四、容量与带宽

```
B_uart = 600,000 B/s
每 tick 可用 = 600,000 / 20,000 = 30 B/tick
可持续条件: 4k + 8 ≤ 30  →  k ≤ 5.5
```

| k | 小帧大小 | 每 buffer 帧数 | 时间跨度 | 写入速率 | 占带宽 | 持续？ |
|---|---------|--------------|---------|---------|--------|--------|
| 4 | 24 B | 170 | 8.5 ms | 480 KB/s | 80% | ✅ |
| 5 | 28 B | 146 | 7.3 ms | 560 KB/s | 93% | ✅ |
| 6 | 32 B | 128 | 6.4 ms | 640 KB/s | 107% | ⚠️ 短时可 |

---

## 五、降采样

降采样因子 D，运行时可变。ISR 每 tick 计数器递增：

```c
if (counter < D) return;   // 1 cycle
counter = 0;
```

```
f_sample = 20,000 / D
每采样可用 = 30 × D 字节
k_max = min((30 × D - 8) / 4, REGISTRY_MAX)
```

| D | f_sample | 每采样可用 | k_max | Nyquist |
|---|----------|----------|-------|---------|
| 1 | 20 kHz | 30 B | 5 | 10 kHz |
| 2 | 10 kHz | 60 B | 13 | 5 kHz |
| 3 | 6.67 kHz | 90 B | 16 | 3.3 kHz |
| 4 | 5 kHz | 120 B | 16 | 2.5 kHz |
| 5 | 4 kHz | 150 B | 16 | 2 kHz |

---

## 六、双缓冲状态机

### 状态定义

| 状态 | 含义 | 持有者 | 谁能改 |
|------|------|--------|--------|
| `UNLOCKED` | 空闲 | 无 | ISR 接管时 → LOCKED；DMA 回调 → 来自 SENDING |
| `LOCKED` | ISR 正在写入 | ISR | ISR 写满时 → READY |
| `READY` | 写满，等待发送 | 无（过渡） | RTOS 线程 → SENDING |
| `SENDING` | DMA 发送中 | DMA 硬件 | DMA 回调 → UNLOCKED |

### 三路行为

| 上下文 | 触发 | 行为 | 状态变更 |
|--------|------|------|---------|
| **20kHz ISR** | 每 tick | ① D 降采样判断; ② `current_buf` 非空 → 写 seq+float+帧尾; ③ 空且有 `UNLOCKED` → 接管写入; ④ 空且无 `UNLOCKED` → `return`，seq 不自增 | ③ `UNLOCKED→LOCKED`; 写满 `LOCKED→READY` |
| **RTOS 线程** | 约 1ms 周期 | 有 `READY` 且 DMA 空闲 → 启动 DMA | `READY→SENDING` |
| **DMA TC 回调** | 硬件中断 | 有 `SENDING` → `UNLOCKED` | `SENDING→UNLOCKED` |

### 可达状态组合

| buf[0] | buf[1] | 可达 | ISR 行为 |
|--------|--------|------|---------|
| UNLOCKED | UNLOCKED | ✅ 初始 | 拿 buf[0] |
| LOCKED | UNLOCKED | ✅ 正常 | 写 buf[0] |
| READY | UNLOCKED | ✅ | 拿 buf[1] |
| SENDING | UNLOCKED | ✅ | 拿 buf[1] |
| LOCKED | READY/SENDING | ✅ | 写 buf[0]，写满后丢采样 |
| READY | READY/SENDING | ✅ 极少 | 丢采样 |
| SENDING | SENDING | ❌ | 单通道 DMA |
| LOCKED | LOCKED | ❌ | ISR 只有一个 `current_buf` |

---

## 七、seq 字段

### 丢采样检测

ISR 丢采样时 seq 不自增。上位机看相邻小帧 seq 差值：

```
差 N → 丢了 N-1 个采样 → 时间缝隙 = (N-1) × 50μs × D
```

### seq 碰撞风险

seq 的值可能碰巧等于 `0x7F800000`（IEEE 754 正无穷，即帧尾）。JustFloat 协议引擎在字节流中扫描帧尾，遇到此值会误判——当 seq 恰好等于该值时，其小端字节序列与真实帧尾完全一致，协议引擎误判此处为帧尾。

| 项目 | 值 |
|------|-----|
| 碰撞值 | 0x7F800000 = 2,139,095,040 |
| 20kHz 下触发间隔 | ≈ 29.7 小时 |
| 后果 | 丢一个小帧（50μs × D），下一帧尾自动恢复 |
| 处理策略 | **不规避**——协议固有风险，影响可忽略 |

---

## 八、丢 tick 行为

| k | ISR 填满 | DMA 发送 | 关系 | 丢 tick？ | 丢多少 |
|---|---------|---------|------|----------|--------|
| 4 | 8.5 ms | 6.8 ms | DMA 快 | 不丢 | 0 |
| 8 | 5.5 ms | 6.8 ms | ISR 快 | 丢 | ~27 tick（1.35ms，burst 式集中在 buffer 末尾） |

---

## 九、错误处理

| 错误类型 | 处理 |
|---------|------|
| ISR 拿不到 UNLOCKED buffer | return，seq 不自增，本 tick 丢弃 |
| DMA 字节错位 | 上位机帧尾不匹配 → 丢当前小帧 → 从下一个有效帧尾恢复 |
| DMA 传输失败 | HAL 错误回调 → 标记 UNLOCKED → 回收 |
| 整 buffer 丢失 | 上位机超时 → 从下一个 buffer 第一个有效帧尾重同步 |
| seq 碰撞帧尾 | 丢一个小帧，自动恢复 |

---

## 十、接口清单

| 组件 | 方法 | 调用者 | 功能 |
|------|------|--------|------|
| `TelemetryStream` | `tick()` | 20kHz ISR | 写 seq + float + 帧尾 |
| | `checkAndSend()` | RTOS 线程 | 检查 READY → 启动 DMA |
| | `onDMADone()` | DMA TC 回调 | SENDING → UNLOCKED |
| | `setRegistry(idx, name)` | CLI / RS485 | 按名称查目录 → 改注册表 |
| | `getRegistry(idx)` | CLI | 查询当前注册表 |
| | `setDecimation(D)` | CLI/RS485 | 运行时改降采样，触发 Flush |
| | `start()` / `stop()` | CLI | 暂停/恢复（切换注册表时用） |
| | `parseRxLine(line)` | LPUART RX 回调 | RS485 同链路接收指令 |

---

## 十一、注册表

| 组件 | 说明 |
|------|------|
| 变量目录 | 编译期静态表 `{name, float* ptr}` |
| 注册表 | 运行时可变 `float*[16]`，ISR 遍历 |
| CLI/RS485 | `telem set <idx> <name>` |
| 预设 | `current`(id,iq,omega,theta) / `sweep`(iq_ref,iq,id,omega) / `observer`(e_alpha,e_beta,smo_theta,theta) |

---

## 十二、编译开关

| 宏 | 默认值 | 含义 |
|----|--------|------|
| `TELEMETRY_ENABLE` | 1 | 0 = 全部遥测代码不参与编译 |
| `TELEMETRY_BUFFER_KB` | 4 | 单个 buffer 大小 |
| `TELEMETRY_BAUDRATE` | 6000000 | LPUART 波特率 |
| `TELEMETRY_REGISTRY_MAX` | 16 | 注册表最大条目数 |

---

## 十三、分层

| 层 | 模块 | 职责 |
|----|------|------|
| 热路径 | `TelemetryStream::tick()` | 遍历注册表 → 写 seq + float + 帧尾 |
| 冷路径 | RTOS 线程 + DMA 回调 | 状态检查 → 启动 DMA → 解锁 |
| 配置 | 变量目录 + 注册表 + `build_config.h` | 定义可观测变量、运行时切换 |
| 物理 | LPUART1 + SP3485 + RS485 双绞线 | 6Mbps 差分信号 |
| 上位机 | VOFA+ / Python | 实时波形 / seq 丢采样分析 |

---

## 十四、实现注意事项

1. **buffer 末尾零头**：DMA 发送时 `Size` 参数传实际使用量（`used_bytes`），不传 `4096`
2. **`setDecimation(D)` 跨边界**：实现时触发 Flush，强制封当前 buffer 发出，新 buffer 从新 D 值开始
3. **注册表指针安全**：目录条目只注册全局或静态变量地址，不注册栈上局部变量
4. **seq 不自增**：丢采样时 seq 不自增，上位机靠跳号检测丢失
5. **DMA IRQ 优先级**：DMA1_Channel1 优先级建议 prio 4，不打断 FOC 热路径（prio 0）

---

## 十五、设计原则

> ISR 每 tick 写 seq + float + 帧尾，遍历注册表取值。RTOS 只启动 DMA。上位机从 seq 跳号精确检测丢采样。双缓冲 LOCKED/UNLOCKED 所有权互斥。没有帧头、没有 CRC、没有时间戳——削到最简。

---

## 十六、ISR 时序测量（DWT 插桩法）

遥测系统 50μs 的颗粒度看不到 FOC ISR **内部**各环节的时序细节。利用 MCU 内置 DWT CYCCNT 计数器（160MHz，6.25ns 分辨率），在 ISR 关键节点前后打点，差值作为遥测通道导出——遥测系统测自己。

### 插桩方式

```c
// FOC ISR (20kHz, prio 0)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim != &htim1) return;
    uint32_t t0, t1, t2, t3, t4;

    t0 = DWT->CYCCNT;                    // ISR 入口

    // ADC 读数
    adc_read[0] = (int16_t)hadc1.Instance->JDR1;
    adc_read[1] = (int16_t)hadc3.Instance->JDR1;
    adc_read[2] = (int16_t)hadc5.Instance->JDR1;
    t1 = DWT->CYCCNT;                    // ADC 读完

    // 编码器
    as5047_spi1.get = AS5047_GetAngle(&AS5047_spi1_PORT) * 7;
    t2 = DWT->CYCCNT;                    // 编码器读完

    // FOC 本体
    setPhaseVoltage(&htim1, uq, 0, as5047_spi1.get);
    t3 = DWT->CYCCNT;                    // SVPWM 完

    // 遥测写入
    Telemetry_WriteBatch(batch, 4);
    t4 = DWT->CYCCNT;                    // 遥测写完

    // 时序差值作为遥测通道导出（单位：cycle）
    Telemetry_Write(0x50, (float)(t1 - t0));   // ADC 耗时
    Telemetry_Write(0x51, (float)(t2 - t1));   // 编码器耗时
    Telemetry_Write(0x52, (float)(t3 - t2));   // FOC/SVPWM 耗时
    Telemetry_Write(0x53, (float)(t4 - t3));   // 遥测自身耗时
    Telemetry_Write(0x54, (float)(t4 - t0));   // ISR 总耗时
}
```

### 变量 ID 分配

| 范围 | 用途 |
|------|------|
| `0x01-0x0F` | 电机控制数据（id, iq, omega, theta 等） |
| `0x50-0x5F` | ISR 时序测量（各环节 cycle 数） |

### 开销分析

| 操作 | cycle |
|------|-------|
| `t = DWT->CYCCNT`（1 条 LDR） | ~3 |
| 差值计算 + 写入遥测（1 条 SUB + 1 条 STR + 休止符） | ~10 |
| 4 个打点总计 | ~40 |

打点开销落在 `t4 - t3`（遥测自身）区间内，不污染 ADC/ENC/FOC 的测量结果。FOC 本身 600+ cycle，测量误差 ~3 cycle（0.5%），可忽略。

### 可选的降采样

时序数据通常不需要 20kHz 全量。可在插桩代码中加入降采样：

```c
static uint16_t prof_cnt = 0;
if (++prof_cnt >= 20) {   // 每 1ms 发一次（1kHz）
    prof_cnt = 0;
    Telem_Write(0x50, (float)(t1 - t0));
    // ...
}
```

### 验证方法

用 GPIO 翻转 + 示波器交叉验证 DWT 读数：

```c
GPIOB->BSRR = GPIO_PIN_0;               // 拉高
setPhaseVoltage(&htim1, uq, 0, theta);
GPIOB->BSRR = (GPIO_PIN_0 << 16);       // 拉低
// 示波器量高电平宽度 ≈ DWT CYCCNT 差值
```

### VOFA+ 观察

VOFA+ 中直接显示各环节 cycle 数波形。160MHz 下 1 cycle = 6.25ns，除以 160 即微秒。看三样东西：

| 观察指标 | 含义 |
|---------|------|
| 均值 | 每环节正常开销（如 FOC ~680 cycle = 4.25μs） |
| 抖动 | 同样环节 cycle 数波动 → Cache miss / 中断冲突 |
| 尖峰 | 偶然的异常高值 → 记录当时业务上下文（θ, iq, Vbus）辅助定位 |

### 最差值捕获

长测跑数小时，只记录历史最差情况：

```c
static uint32_t g_worst_foc = 0;
uint32_t elapsed = t3 - t2;
if (elapsed > g_worst_foc) {
    g_worst_foc = elapsed;
    // 同时记录当时的 θ, iq, Vbus 等上下文
}
```

CLI 敲 `telem stats` 即可查看各环节历史最差值，不需要跑完数小时后回放全部波形。
