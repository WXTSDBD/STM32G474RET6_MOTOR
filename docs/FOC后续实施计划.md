# FOC 后续实施计划

> **更新日期：** 2026-06  
> **当前阶段：** TIM8 开环 bringup 已通过 → 进入 **Step 1 电流环**  
> **平台：** STM32G474RET6 @ 160 MHz，Keil AC6 -O2，FreeRTOS + VOFA 遥测  

---

## 一、当前基线（已完成）

| 项目 | 状态 |
|------|------|
| **电机 PWM** | **TIM8**（PC6/7/8 + N），20 kHz 中心对齐，CH4=1998，DeadTime=120 |
| **电流采样** | **ADC2** 三 rank 注入，`T8_CC4` 触发 → `adc_read[3/4/5]` **已确认就绪** |
| **编码器** | **SPI1 / `enc_m1`** DMA 双帧，`encoder_kick` @ TIM8 UP，**不改动 encoder v1.0 骨架** |
| **开环** | Ud=0，`uq=2`，`setPhaseVoltage(&htim8, ...)` + CORDIC |
| **极对数** | `M1_POLE_PAIRS = 7` |
| **性能** | `isr_delta` ≈ **1200**（TIM8 UP）；`motor_path_cpu` ≈ **1515**（含 SPI ~581） |
| **优化** | 已去冗余 `_normalizeAngle`（`add` 并入 `encoder_get_theta_el`） |

**数据流（当前）：**

```text
TIM8 UP (20 kHz):
  encoder_kick → get_raw / get_theta_el → setPhaseVoltage(htim8) → telem → DWT

ADC2 JEOC:
  更新 adc_read[3/4/5]（dbg 用，尚未参与控制）

SPI1 DMA (prio 2):
  kick + F1/F2 回调（~581 cycle/拍 CPU）
```

---

## 二、总路线图（四步，严格顺序）

```text
Step 1 ──→ Step 2 ──→ Step 3 ──→ Step 4
电流环      粗 L/Rs     死区 LUT    精标参数
(现在)      (调 PI)     (线性化)    (论文法)
```

| Step | 目标 | 前置 | 详细说明 |
|------|------|------|----------|
| **1** | **Id/Iq 电流闭环** | ADC2 就绪 ✓ | 本文 §三；[电流环部署计划_Step1.md](./电流环部署计划_Step1.md) |
| **2** | Rs / Ld / Lq 粗值 + 换 Kp/Ki | Step 1 稳 | [有感FOC…分步实施计划.md §Step2](./有感FOC电流环_死区补偿_参数辨识_分步实施计划.md) |
| **3** | 死区 + 小电流非线性 LUT | Step 2 | 同上 §Step3（Id 锁轴 + 扫表） |
| **4** | Rs / Ld / Lq / ψf 精标 | Step 3 | 同上 §Step4（斜坡 LS + VASI + 反电动势） |

**本阶段不做：**

- 速度环 / PLL @ 2 kHz（Step 1 稳后再做，见 [FOC控制环架构设计说明.md](./FOC控制环架构设计说明.md)）
- 死区补偿、参数辨识（Step 3/4）
- sector Q15 LUT / 全链路定点（cycle 仍有余量）
- 修改 encoder DMA 架构（已冻结）

---

## 三、Step 1：电流环（下一步，重点）

### 3.1 目标架构

FOC **主流程进 ADC2 回调**；TIM8 UP **只负责时序与编码器**，不跑 SVPWM。

```text
TIM8 UP (prio 1, 20 kHz):
  encoder_kick(&enc_m1)
  telem_bringup_tick()          // FOC 联调期可用 TELEM_PROFILE 关 DWT/telem
  return                        // 不写 CCR、不跑 Clarke/Park/PI

ADC2 JEOC (hadc2, prio 1 建议):
  motor_current_tick():
    1. 读 JDR1/2/3 → 减偏置 → 乘标度 → ia, ib, ic
    2. encoder_get_raw + get_theta_el(..., add)
    3. motor_trig_sincos ×1 → sin/cos 缓存
    4. Clarke / Park → id, iq
    5. foc_pi_step(id_ref, id) → ud
    6. foc_pi_step(iq_ref, iq) → uq
    7. (feedforward NULL) (deadband NULL)
    8. InvPark + SVPWM → 写 htim8 CCR1/2/3
```

**角度语义不变：** kick 在本拍 TIM8 UP；`get_raw()` 仍为 **上一拍完成** 的角（1 拍延迟 ~50 µs）。

### 3.2 子任务清单

| # | 任务 | 说明 |
|---|------|------|
| 1.1 | **偏移标定** | PWM 未开，采 100 拍 `adc_read[3/4/5]` 平均 → offset |
| 1.2 | **标度系数** | 10 mΩ + 外部运放增益 + 3.3V/12bit → `ia_scale` 等；验收 Ia+Ib+Ic≈0 |
| 1.3 | **`MotorContext`** | `motor/motor_context.h`：ia/ib/ic/id/iq/ud/uq/θ_el、ref、offset |
| 1.4 | **`foc_current_sense`** | 读 ADC、标定应用 |
| 1.5 | **`foc_transform`** | Clarke/Park + InvPark；与 SVPWM **共用 1 次 sincos** |
| 1.6 | **`foc_pi`** | 并联 PI + 抗饱和；输出限幅 `Vbus/√3` |
| 1.7 | **`svpwm`** | 从 `bringup/trans.c` 提炼，或过渡期 `setPhaseVoltage(htim8, uq, ud, θ)` |
| 1.8 | **拆 ISR** | TIM8 UP 去掉 `setPhaseVoltage`；ADC2 只跑 **一次** `motor_current_tick`（仅 `hadc==&hadc2`） |
| 1.9 | **替换开环** | 去掉固定 `uq=2`；`id_ref=0`，`iq_ref` 小阶跃测试 |
| 1.10 | **PI 占位** | Kp=0.5, Ki=50（Step 2 再按 L/Rs 换） |
| 1.11 | **Keil** | `uvprojx` 追加源文件与 include 路径 |
| 1.12 | **WCET** | 可选 DWT：`adc_isr_delta`；目标 TIM8 UP + ADC FOC 总 CPU 留 RTOS 窗口 |

### 3.3 Step 1 验收

- [ ] PWM 使能、`id_ref=iq_ref=0` 时 Id/Iq 接近 0（偏置+标度正确）
- [ ] Id≈0 时 Ud≈0（PI 未乱出力）
- [ ] `iq_ref` 阶跃（如 0→0.5A→1A），Iq 能跟踪
- [ ] Ia+Ib+Ic ≈ 0
- [ ] 电机不失控、不过流；`enc_kick_skip_busy` ≈ 0
- [ ] `uart_task_loops` 仍增长（RTOS 未饿死）
- [ ] VOFA 可看 Id/Iq/θ（扩展 telem 通道或 Watch）

---

## 四、Step 2：初始 L / Rs（粗测）

**目的：** 把 Step 1 占位 PI 换成有物理意义的增益。

| 方法 | 内容 |
|------|------|
| **Rs** | LCR 线电阻÷2，或 Id 锁轴：`Rs ≈ Ud/Id` |
| **Ld/Lq** | 不同转子位置测三相电感 → Park 关系 |
| **Kp/Ki** | `Kp = L × ωc`，`Ki = Rs × ωc × Ts`（ωc 如 1000 rad/s，Ts=50 µs） |

**验收：** 三个粗值拿到（~30% 误差可接受）；阶跃响应明显改善。

---

## 五、Step 3：死区补偿 LUT

**前提：** Step 1 PI 能 Id 锁轴；Step 2 有 Rs 粗值。

**方法摘要：**

1. Id_ref 锁轴，转子不动  
2. 扫 Id_ref（小电流区密、大电流区疏）  
3. `Ud_error = Ud_pi − Id×Rs`  
4. 生成 32 点 LUT，运行时 `deadband_apply()` 查表  
5. 大电流区（\|i\|>2A）可简化为常数补偿  

**验收：** 补偿前后 VOFA 看过零点相电流畸变；Id 锁轴稳定。

详见 [有感FOC…分步实施计划.md §Step3](./有感FOC电流环_死区补偿_参数辨识_分步实施计划.md)。

---

## 六、Step 4：参数精标

**前提：** Step 3 死区已补偿（小信号方波不被吞）。

| 参数 | 方法 |
|------|------|
| **Rs** | d 轴斜坡 Id + 最小二乘（只用 \|Id\|>2A 段） |
| **Ld/Lq** | VASI 对称方波注入 + 磁链积分 + 变幅值 |
| **ψf** | 开环拖转反电动势，或 Kt 反推 |

**验收：** Rs/Ld/Lq/ψf 三次重复接近；写入 `MotorConfig` 供 PI/MTPA 使用。

详见 [有感FOC…分步实施计划.md §Step4](./有感FOC电流环_死区补偿_参数辨识_分步实施计划.md)。

---

## 七、Step 1 之后（可选扩展，不阻塞主链）

| 优先级 | 内容 | 说明 |
|--------|------|------|
| P1 | PLL + 速度环 @ 2 kHz | [FOC控制环架构设计说明.md](./FOC控制环架构设计说明.md) §5、§9 |
| P2 | `#if TELEM_PROFILE` 关 DWT | FOC 联调减 cycle |
| P3 | 文档刷新 | TIM8+ADC2 基线写入联调日报 |
| P4 | sector Q15 LUT | 仅当 ADC FOC WCET 顶满时再 diet |

---

## 八、Cycle 预算提醒

| 指标 | 当前约值 | Step 1 注意 |
|------|----------|-------------|
| TIM8 UP `isr_delta` | ~1200 | 迁走 SVPWM 后应变轻（~400～500 + kick + telem） |
| SPI `enc_spi_cpu_total` | ~581 | **固定**，勿动 encoder 骨架 |
| FOC 新增（ADC ISR） | 估 ~700～1200 | 单独 DWT；与 TIM8 UP **加总** 看 RTOS |
| TIM8 UP 硬经验 | ≤ **2700** | 历史红线；ADC 路径另计但同核占用 |

规划时以 **motor_path_cpu ~1515 + FOC(ADC)** 为起点，FOC 联调监控 `uart_task_loops`。

---

## 九、涉及文件（Step 1 预计新建/改）

| 路径 | 操作 |
|------|------|
| `motor/motor_context.h` | 新建 |
| `drivers/foc/foc_current_sense.c/.h` | 新建 |
| `drivers/foc/foc_transform.c/.h` | 新建 |
| `drivers/foc/foc_pi.c/.h` | 新建 |
| `drivers/foc/svpwm.c/.h` | 新建（或暂用 `trans.c`） |
| `motor/motor_current.c/.h` | 新建 `motor_current_tick()` |
| `Core/Src/main.c` | 拆 TIM8 UP / ADC2 回调；去开环 uq |
| `MDK-ARM/*.uvprojx` | 加源文件与 include |
| `bringup/trans.c` | 暂保留；逐步提炼到 `svpwm.c` |

**冻结不改：** `platform/encoder_spi_bus*`、`drivers/as5047*`、`board/board_encoder_m1*`、DMA Ch2 LL ISR。

---

## 十、相关文档

| 文档 | 用途 |
|------|------|
| [有感FOC电流环_死区补偿_参数辨识_分步实施计划.md](./有感FOC电流环_死区补偿_参数辨识_分步实施计划.md) | Step 1～4 方法细节（死区/辨识详述） |
| [电流环部署计划_Step1.md](./电流环部署计划_Step1.md) | Step 1 子步骤与验收 |
| [FOC控制环架构设计说明.md](./FOC控制环架构设计说明.md) | 长期架构、PLL、目录规划 |
| [SPI编码器架构实现说明.md](./SPI编码器架构实现说明.md) | encoder v1.0 冻结、API |
| [联调日报_编码器DMA与SVPWM合并_2026-06-09.md](./联调日报_编码器DMA与SVPWM合并_2026-06-09.md) | cycle 基线、P0 优化记录 |

---

## 十一、建议执行顺序（Checklist）

```text
□ Step 1.1  偏移标定（100 拍，PWM off）
□ Step 1.2  标度系数 + Ia+Ib+Ic≈0 验证
□ Step 1.3  建 motor/ + drivers/foc/ 骨架
□ Step 1.4  ADC2 回调 motor_current_tick（PI 占位）
□ Step 1.5  TIM8 UP 去掉 setPhaseVoltage
□ Step 1.6  id=0 / iq 阶跃验收
□ Step 1.7  记 WCET，确认 RTOS/VOFA 正常
□ Step 2    L/Rs 粗测 → 换 Kp/Ki
□ Step 3    死区 LUT
□ Step 4    参数精标
□ 后续      PLL / 速度环 / 外环策略
```

---

*下一步行动：**Step 1.1 偏移标定 + Step 1.3 建模块 + Step 1.8 拆 ISR**。*
