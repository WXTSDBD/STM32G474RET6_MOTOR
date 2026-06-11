# FOC 控制环架构设计说明

> **版本**: v1.0（设计态，待 Phase A 落地）  
> **平台**: STM32G474 @ 160 MHz，TIM1 20 kHz 电流环  
> **关联文档**: [SPI编码器架构实现说明.md](./SPI编码器架构实现说明.md)、[电机驱动软件框架——完整架构设计文档.md](./电机驱动软件框架——完整架构设计文档.md)

本文定义 **有感 FOC 电流环 + PLL 速度估计 + 低频外环策略** 的软件架构、接口边界与 cycle 预算。编码器 DMA 路径已于 **v1.0 冻结**（见 [SPI 架构说明 §14 技术债务](./SPI编码器架构实现说明.md#14-故意遗留的技术债务以后再说)）；FOC 通过 `encoder_get_raw` / `encoder_get_theta_el`（20 kHz）/ `encoder_get_angle`（2 kHz）接入，**不再改** encoder 骨架。

---

## 1. 设计目标

| 目标 | 做法 |
|------|------|
| 20 kHz 电流环 WCET 可控 | 热路径 **C 单函数** `motor_current_tick()`，避免 ISR 内多层 virtual |
| 速度平滑、低噪声 | **PLL 跟踪编码器角**，禁止 20 kHz 裸差分算 ω |
| 角度源可替换 | 编码器 / SMO / HFI 统一 `motor_observer` 出口：`theta_el`、`omega_el` |
| 策略可扩展 | **C++ Service 层** 1–2 kHz 外环；共享 `MotorContext` 写 `id_ref/iq_ref` |
| trig 可替换 | CORDIC LL（主）+ LUT（备），**一次输入、同时输出 sin/cos** |
| 与 bringup 共存 | 遥测 profile 可 unbind；FOC sign-off 在 **Release -O2** |

---

## 2. 多速率架构

```text
┌────────────────────────────────────────────────────────────────────────┐
│ 20 kHz — 电流环（ADC JEOC 或 TIM 中心点触发，裸 ISR / 最高优先级）      │
│   encoder_kick()          ← 尽早 kick，与 [SPI 架构说明] 一致           │
│   adc_sample → ia, ib, vbus                                            │
│   observer.get_theta_el() ← 读缓存/PLL 内插角，不在此 unwrap             │
│   trig.sincos(theta_el)   ← 一次，Park + InvPark 共用                    │
│   clarke → park → id/iq PI → [feedforward] → [deadband]                │
│   inv_park → svpwm → write CCR                                           │
└────────────────────────────────────────────────────────────────────────┘
                                    ▲
                          id_ref, iq_ref（低频写）
                                    │
┌────────────────────────────────────────────────────────────────────────┐
│ 2 kHz — 速度环 + PLL 更新（TIM 分频 / RTOS 定时，不进 20 kHz ISR）       │
│   theta_mech = encoder_get_angle(&enc_m1, raw)  // unwrap 在此频率      │
│   motor_pll_update(&pll, theta_mech, dt)        // ★ 速度唯一来源       │
│   omega_mech = pll.omega_mech                                            │
│   speed_pi → iq_ref                                                      │
│   [observer 慢更新：SMO/ESO 参数]                                        │
└────────────────────────────────────────────────────────────────────────┘
                                    ▲
                          speed_ref / torque_ref / pos_ref
                                    │
┌────────────────────────────────────────────────────────────────────────┐
│ 1 kHz — 外环策略（C++ IMotorCommandPolicy）                              │
│   力矩 / 速度 / 位置 PI / MIT 阻抗 / VF 开环 / HFI 标定 …               │
│   写 MotorContext 指令字段，不调用 HAL                                   │
└────────────────────────────────────────────────────────────────────────┘
```

**纪律**：八个外环策略 **不得** 挂进 20 kHz；PLL **不得** 用 `(θ[k]-θ[k-1])/dt` 在 20 kHz 裸算速度。

---

## 3. 分层与语言边界

```text
┌─────────────────────────────────────────────────────────┐
│ C++  service/motor/     模式管理、外环策略、CAN 映射     │
├─────────────────────────────────────────────────────────┤
│ C     motor/motor_context.h   共享状态（C struct）       │
│ C     motor/motor_current.c   20 kHz 热路径              │
│ C     motor/motor_pll.c       2 kHz PLL                 │
│ C     motor/motor_slow.c      2 kHz 速度环 dispatch      │
├─────────────────────────────────────────────────────────┤
│ C     drivers/foc/          clarke/park/pi/svpwm        │
│ C     platform/trig/        cordic_ll / lut256          │
│ C     drivers/encoder/      已有 encoder API            │
├─────────────────────────────────────────────────────────┤
│ HAL / LL / CubeMX                                         │
└─────────────────────────────────────────────────────────┘
```

| 层次 | 语言 | ISR |
|------|------|-----|
| 电流环 + SVPWM + CCR | C | 是 |
| PLL + 速度 PI | C | 否（2 kHz） |
| 外环策略（8 模式） | C++ | 否（1 kHz） |

C/C++ 边界：`extern "C" { #include "motor_context.h" }`；C++ 只调用 `motor_set_mode()`、`motor_slow_tick()` 等 C API。

---

## 4. 共享状态 `MotorContext`

```c
typedef struct {
    /* --- 20 kHz 读，低频写（双缓冲或 volatile + 单写者）--- */
    volatile float id_ref;
    volatile float iq_ref;

    /* --- 20 kHz 采样 --- */
    float ia, ib, ic;
    float v_bus;
    float id, iq;

    /* --- 角度/速度（观测器出口）--- */
    float theta_el;       /* 20 kHz 电角度 [rad]，来自 PLL 内插或编码器直通 */
    float omega_el;       /* 电角速度 [rad/s]，2 kHz PLL 输出经 hold 或内插 */
    float theta_mech;     /* 机械角 [rad]，2 kHz unwrap 更新 */
    float omega_mech;     /* 机械角速度 [rad/s]，★ PLL 输出，供速度环 */

    int pole_pairs;       /* 极对数，θ_el = θ_mech * pole_pairs + offset */

    /* --- 执行器 --- */
    float ud, uq;
    uint16_t pwm_a, pwm_b, pwm_c;

    /* --- 模式 --- */
    uint8_t enabled;
    svpwm_mode_t svpwm_mode;
} MotorContext;
```

---

## 5. 速度估计：PLL（强制）

### 5.1 为何不用差分

| 方法 | 问题 |
|------|------|
| `(θ[k]-θ[k-1])/dt` @ 20 kHz | 量化噪声大（14 bit 编码器）、对噪声敏感、ω 抖动直接进速度 PI |
| 滑动平均后再差分 | 延迟大，仍怕阶跃 |
| **PLL** | 工业伺服常规做法；ω 平滑、可滤波、与 θ 一致 |

### 5.2 PLL 结构（机械角域）

编码器在 **2 kHz** 提供测量角 `theta_meas`（`encoder_get_angle` unwrap 后）：

```text
        theta_meas ──→ [+]──→ Kp ──→ [+]──→ integrator ──→ omega_mech
                        ↑-              ↑-        │
                        │               Ki        │
                        └── pll_theta ←─┘         │
                              ↑                    │
                              └────── ∫ omega dt ──┘
```

**离散实现（每 2 kHz 一步，`dt = 0.0005 s`）：**

```c
typedef struct {
    float theta;          /* PLL 内部角 [rad] mech，wrap 到 [0, 2π) */
    float omega;          /* 估计机械角速度 [rad/s] */
    float integrator;     /* PI 积分项 */
    float kp;             /* 典型 50~500，待整定 */
    float ki;             /* 典型 1000~50000，待整定 */
    float omega_limit;    /* 限速 [rad/s] */
} motor_pll_t;

static float wrap_2pi(float x);

void motor_pll_reset(motor_pll_t *pll, float theta0);

void motor_pll_update(motor_pll_t *pll, float theta_meas, float dt)
{
    float err = wrap_pi(theta_meas - pll->theta);   /* 相位误差 [-π, π] */

    pll->integrator += pll->ki * err * dt;
    /* 可选：integrator 限幅 */

    pll->omega = pll->integrator + pll->kp * err;
    if (pll->omega >  pll->omega_limit) pll->omega =  pll->omega_limit;
    if (pll->omega < -pll->omega_limit) pll->omega = -pll->omega_limit;

    pll->theta = wrap_2pi(pll->theta + pll->omega * dt);
}
```

**输出：**

- `omega_mech = pll->omega` → **速度环反馈唯一来源**
- `theta_mech` 可用 `pll->theta` 或编码器 meas（电流环推荐见 5.4）
- `omega_el = omega_mech * pole_pairs`
- `theta_el` 见 5.4

### 5.3 初始化与失步

- 上电 / 使能：`motor_pll_reset(pll, encoder_get_angle(...))`，`integrator = 0`
- 失步检测：`|wrap_pi(theta_meas - pll->theta)| > threshold` 持续 N 次 → 故障或强制 reset
- 方向反：调整 `kp` 符号或 `pole_pairs`/编码器安装 offset

### 5.4 20 kHz 电流环用哪个角

| 方案 | 说明 | 推荐 |
|------|------|------|
| A. 编码器 raw → 电角，每拍换算 | 延迟最小，θ 无 PLL 平滑 | 高速可选 |
| B. PLL θ + 2 kHz 间线性内插 | `theta_el = (pll_theta + omega*dt_frac) * pole_pairs` | **默认推荐** |
| C. 20 kHz 也跑 PLL | cycle 增加 ~30–60，通常不必 | 备选 |

**unwrap**（跨 2π 累计）仅在 **2 kHz** `encoder_get_angle` 调用；20 kHz 只用 `encoder_get_raw` + 电角换算或 PLL 内插。

### 5.5 与旧工程关系

G431 霍尔工程 [`HALL_READ.c`](../cyka_mobile_G431RBT6_as5047%20_initial_board_for_version_two/application/HALL_READ.c) 已有 `standard_pll_update` 思路（相位误差 → PI → ω → 积分 θ）。本方案在 **机械角域** 对磁编码器做同样结构，输入改为 `encoder_get_angle` 而非霍尔扇区角。

**禁止**：`AS5047_Calculate_Speed` 式纯差分 + 滑动平均作为正式速度环反馈（bringup 可保留对比）。

---

## 6. 观测器接口（角度源）

```c
typedef struct {
    void (*init)(void *ctx);
    void (*reset)(void *ctx, float theta_mech0);
    /* 2 kHz：更新 PLL 用的 theta_meas；SMO/HFI 时在此做慢更新 */
    void (*update_slow)(void *ctx, MotorContext *m, float dt);
    /* 20 kHz：写 m->theta_el（及可选内插） */
    void (*get_theta_el)(void *ctx, MotorContext *m);
    float (*get_omega_mech)(void *ctx);   /* 来自 PLL，观测器内嵌 pll 指针 */
} motor_observer_ops_t;
```

| 实现 | `update_slow` | `get_theta_el` @ 20 kHz |
|------|---------------|-------------------------|
| `observer_encoder` | 读 angle + `motor_pll_update` | PLL 内插 × pole_pairs |
| `observer_smo` | SMO 更新 + 可选 PLL 跟踪 SMO 角 | SMO θ 或 PLL 内插 |
| `observer_hfi` | HFI 注入/解调（独立模式） | HFI 角 |

有感 MVP 仅实现 **`observer_encoder`**。

---

## 7. 20 kHz 热路径 `motor_current_tick()`

```c
void motor_current_tick(MotorContext *m, encoder_t *enc, TIM_HandleTypeDef *htim);
```

**固定顺序（monolithic，不逐步 ops）：**

1. `encoder_kick(enc)`
2. `current_sense_sample(m)` — 读 ADC JDR，减偏置，标度
3. `observer->get_theta_el(ctx, m)`
4. `trig_sincos(m->theta_el, &m->sin_el, &m->cos_el)` — **一次**
5. `foc_clarke_park(...)`
6. 读 `m->id_ref`, `m->iq_ref`
7. `foc_pi_id_iq(...)`
8. `feedforward_apply(...)` — NULL 跳过
9. `deadband_apply(...)` — NULL 跳过
10. `foc_inv_park_svpwm(...)` — 复用 sin/cos
11. `pwm_write_ccr(htim, ta, tb, tc)` — LL 写 `CCR1..3`

**Trig 接口（编译期或 init 绑定一次）：**

```c
typedef void (*trig_sincos_fn)(float theta, float *s, float *c);
extern trig_sincos_fn g_trig_sincos;   /* &trig_cordic_ll_sincos 或 &trig_lut256_sincos */
```

CORDIC：G474 LL 写 `CORDIC->DR`，读 `RDATA` 得 cos/sin（Cosine + Phase 模式）。LUT：256 点 + 线性插值，用于仿真/HFI/无 CORDIC 平台。

---

## 8. SVPWM

```c
typedef enum {
    SVPWM_7SEG_CENTER = 0,   /* MVP */
    SVPWM_5SEG_CENTER,
    SVPWM_ZSV_INJECTION,
} svpwm_mode_t;

int svpwm_execute(float u_alpha, float u_beta, float v_bus,
                  svpwm_mode_t mode, uint16_t period,
                  uint16_t *pwm_a, uint16_t *pwm_b, uint16_t *pwm_c);
```

- MVP：**七段、αβ 判扇区、无 atan2**、线性区 `|V| ≤ Vbus/√3`
- `period` 取自 `htim->Instance->ARR`，禁止魔数 4000/5300
- 五段 / 零序注入：后续按 mode 分支，不拆 ISR 多层 ops

---

## 9. 2 kHz 慢路径 `motor_slow_tick()`

```c
void motor_slow_tick(MotorContext *m, encoder_t *enc, motor_pll_t *pll);
```

1. `raw = encoder_get_raw(enc)`
2. `theta_meas = encoder_get_angle(enc, raw)` — unwrap
3. `motor_pll_update(pll, theta_meas, dt)` — **ω 更新**
4. `m->theta_mech = pll->theta`（或 meas）
5. `m->omega_mech = pll->omega`
6. `m->omega_el = pll->omega * m->pole_pairs`
7. `speed_pi`: `iq_ref = pi(omega_ref - omega_mech)`（`omega_ref` 由外环写）
8. `id_ref = 0`（MVP）或 MTPA/弱磁策略输出

---

## 10. 外环策略（C++，1 kHz）

```cpp
class IMotorCommandPolicy {
public:
    virtual void tick(MotorContext& m, float dt) = 0;
};

// TorquePolicy → iq_ref = T_cmd / Kt
// SpeedPolicy  → omega_ref
// PositionPolicy → omega_ref = pos_pi(...)
// MitPolicy    → τ = Kp*(p*-p) + Kd*(v*-v) + τ_ff → iq_ref
// ...
```

模式切换仅在 **非 ISR** 进行；20 kHz 只读 `id_ref/iq_ref/omega_ref`。

---

## 11. Cycle 预算（160 MHz，-O2）

**单拍上限 8000 cycle（50 µs）。目标：电流相关 ISR 合计 ≤ 30%（2400 cycle）留裕量。**

| 块 | 频率 | cycle（估） | 备注 |
|----|------|-------------|------|
| encoder kick | 20 kHz | ~295 | 实测 |
| DMA f1+f2 | 20 kHz async | ~330 | prio 2，与 TIM 部分重叠 |
| TIM1 遥测+DWT | 20 kHz | ~300 | 可 unbind |
| ADC 采样 | 20 kHz | 30–80 | |
| get_theta_el（内插） | 20 kHz | 15–40 | 不用 unwrap |
| sincos ×1 | 20 kHz | 15–40 | CORDIC LL |
| clarke+park+PI+inv+svpwm+ccr | 20 kHz | 350–700 | MVP |
| **20 kHz TIM 小计** | | **~700–1200** | 不含 DMA |
| **+ DMA 编码器** | | **~900–1500** | 保守合计 |
| **PLL update** | **2 kHz** | ~40–80 / 次 | 均摊 ~2–4/拍 |
| **speed PI** | **2 kHz** | ~30–60 / 次 | 均摊 ~2–3/拍 |
| C++ 外环 | 1 kHz | 可忽略均摊 | |

**结论**：有感 MVP + PLL @ 2 kHz **周期上可放下**；瓶颈在编码器 DMA CPU，不在 PLL。HFI/SMO 全量进 20 kHz 需单独立项 WCET。

---

## 12. 目录规划（待建）

```text
motor/
  motor_context.h
  motor_current.c      /* 20 kHz */
  motor_pll.c / .h     /* ★ PLL */
  motor_slow.c         /* 2 kHz */
drivers/foc/
  foc_transform.c      /* clarke, park, inv_park */
  foc_pi.c
  svpwm.c
  feedforward.c        /* 可选 */
  deadband.c           /* 可选 */
platform/trig/
  trig_cordic_ll.c
  trig_lut256.c
  trig.h
service/motor/         /* C++ */
  motor_mode_manager.cpp
  policies/            /* torque, speed, position, mit, ... */
```

现有 `bringup/trans.c` Phase A 修正 SVPWM 后逐步迁入 `drivers/foc/svpwm.c`；`FOC_CAL.c` 中差分速度逻辑 **不迁入** 正式路径。

---

## 13. 迁移阶段

| 阶段 | 内容 | 验收 |
|------|------|------|
| **A** | 修 SVPWM + ADC 采样 + 开环 `setPhaseVoltage` + encoder 角 | 空载平稳拖转 |
| **B** | `motor_pll.c` @ 2 kHz + 遥测 ω/θ | PLL ω 比差分平滑 |
| **C** | `motor_current_tick` 电流 PI，id=0 | 空载电流环稳定 |
| **D** | CORDIC LL trig + -O2 WCET 测量 | isr_delta 达标 |
| **E** | 2 kHz speed PI | 速度环稳定 |
| **F** | C++ 外环策略 1–2 种 | 力矩/速度模式 |
| **G** | 前馈、死区、SMO/HFI、SVPWM 高级模式 | 按产品需求 |

---

## 14. 与设计文档 v2 的关系

| v2 文档 | 本文 |
|---------|------|
| C++ `MotorCore::tick()` | 拆为 C `motor_current_tick` + C++ 外环 |
| `IObserver::getOmega()` | **PLL `omega_mech`**，2 kHz 更新 |
| `ITrig` | C `g_trig_sincos` 函数指针 |
| 四扩展点注入 | 热路径 NULL 跳过；feedforward/deadband 后加 |

编码器模块保持 [SPI编码器架构实现说明.md](./SPI编码器架构实现说明.md) 不变；FOC 仅调用 L1 API。

---

## 15. 相关文档

| 文档 | 用途 |
|------|------|
| [SPI编码器架构实现说明.md](./SPI编码器架构实现说明.md) | **v1.0 已冻结**；DMA kick、get_raw / get_theta_el / get_angle；§14 技术债务 |
| [Bringup_串口遥测与AS5047_实际部署说明.md](./Bringup_串口遥测与AS5047_实际部署说明.md) | Watch / cycle 实测 |
| [电机驱动软件框架——完整架构设计文档.md](./电机驱动软件框架——完整架构设计文档.md) | 通信域、CANopen、长期愿景 |

---

## 16. 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-06 | 初版：多速率、PLL 速度、C/C++ 边界、cycle 预算 |
