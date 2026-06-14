# 电流环部署计划（Step 1）

> **日期**: 2026-06-14（周六）
> **状态**: 待实施
> **基线**: isr_delta=1315, 编码器框架已冻结, SVPWM+CORDIC 已运行, 电机已转

---

## 一、目标

在 20kHz TIM1 ISR 内部署有感 FOC 电流环。电机当前是 Ud=0 开环拖动（开环 Uq=2）。改成 **Id=0 闭环 + Iq 闭环 PI**，让电流跟踪指令值。

---

## 二、不做什么

- 不做死区补偿（Step 3 的事，留 NULL 跳过去）
- 不做速度环（2kHz PLL 是后续的事）
- 不做参数辨识（Step 4 的事）
- Kp/Ki 先用占位值（0.5/50），Step 2 测到 L/Rs 后再换

---

## 三、子步骤

### 1.1 电流偏移标定

**目标**：电机未使能时，Id/Iq 读数为零（不是 ADC 零漂对应的偏置）。

**方法**：PWM 未启动、电机自然静止。前 20 帧丢弃，后 100 帧在 JEOC 内对 `adc_read[3]/[4]/[5]`（经 `adc_sample` 的 raw）做 **递推增量均值** → 写入 `offset`（与批处理 `sum/100` 等价，见 [ADC采样与config层部署计划_2026-06-09.md §3.1](./ADC采样与config层部署计划_2026-06-09.md)）。

**存放位置**：放入 `MotorContext.ia_offset/ib_offset/ic_offset`。

**验收**：使能 PWM 但 id_ref=iq_ref=0，VOFA+ 上 Id/Iq 接近零（残余 < 0.1A）。

---

### 1.2 电流采样 + 标度

**目标**：把 ADC 原始值变成实际安培值。

**方法**：
```c
ia = (adc_read[3] - ia_offset) * ia_scale;
ib = (adc_read[4] - ib_offset) * ib_scale;
ic = (adc_read[5] - ic_offset) * ic_scale;
v_bus = (adc_read[总线通道] - vbus_offset) * vbus_scale;
```

**标度系数**：取决于采样电阻、运放增益、ADC 参考电压。OPAMP PGA ×8 或其他增益设置对应不同的 scale。**当前硬件配置见 CubeMX OPAMP 设置和采样电阻值。**

**验收**：Ia+Ib+Ic ≈ 0（三相之和接近零，误差 < 5% 相电流量程）。

---

### 1.3 Clarke + Park

**目标**：三相电流 → dq 轴电流。

**方法**：复用一次 `g_trig_sincos(theta_el)` 同时获得 sin 和 cos（当前 CORDIC 已跑）。

```c
// Clarke
i_alpha = ia;
i_beta  = (ib - ic) * 0.577350269f;  // 1/√3

// Park
id =  i_alpha * cos_el + i_beta * sin_el;
iq = -i_alpha * sin_el + i_beta * cos_el;
```

**关键**：sin/cos 这组值在同一个 ISR 拍内既用于 Park、也用于后续反 Park + SVPWM。只调一次 `g_trig_sincos`，存下来复用。

**验收**：电机被开环拖动时（当前 Uq=2），VOFA+ 看 Iq 应该接近正值（对应输出力矩），Id 接近零。

---

### 1.4 PI 控制器（Id + Iq 各一）

**目标**：Id 跟踪 Id_ref（默认 0），Iq 跟踪 Iq_ref（由上层或测试注入）。

**方法**：并联 PI，带抗积分饱和（back-calculation）。

```c
typedef struct {
    float kp, ki;
    float integrator;
    float out_max, out_min;
} foc_pi_t;

float foc_pi_step(foc_pi_t *pi, float ref, float fb) {
    float err = ref - fb;
    pi->integrator += pi->ki * err;
    float out = pi->kp * err + pi->integrator;
    if (out > pi->out_max) {
        out = pi->out_max;
        pi->integrator = pi->out_max - pi->kp * err;
    } else if (out < pi->out_min) {
        out = pi->out_min;
        pi->integrator = pi->out_min - pi->kp * err;
    }
    return out;
}
```

**PI 输出限幅**：
```c
Ud_max = Vbus * 0.577f;  // Vbus/√3，SVPWM 线性区上限
Uq_max = Vbus * 0.577f;
```

**初值**：先用 LCR 电桥粗测的 L 和 Rs 算。带宽先设 1000 rad/s：
```
Kp = L × 1000        （如 0.5mH × 1000 = 0.5）
Ki = Rs × 1000 × Ts  （如 0.8Ω × 1000 × 0.00005 = 0.04）
```
若 LCR 还没测，用占位值 Kp=0.5, Ki=50→可跑但响应慢/可能振荡。

**验收**：Id_ref=0 时 Ud 输出接近零。Iq_ref 从 0 阶跃到 1A→Iq 跟踪，VOFA+ 看上升曲线。

---

### 1.5 反 Park + SVPWM + 写 CCR

**目标**：dq 电压 → αβ → 三相占空比 → 硬件 CCR。

**方法**：复用 1.3 里的 sin/cos。调现有 `svpwm_execute`（从 `trans.c` 的 `setPhaseVoltage` 提炼，Ud=0 时已跑稳）。

```c
u_alpha = ud * cos_el - uq * sin_el;
u_beta  = ud * sin_el + uq * cos_el;
svpwm_execute(u_alpha, u_beta, v_bus, mode, period, &ta, &tb, &tc);
htim->CCR1 = ta; htim->CCR2 = tb; htim->CCR3 = tc;
```

**验收**：和 1.4 同步验——电机不失控、isr_delta 不变差。

---

### 1.6 TIM1 回调重组

**目标**：把 1.1-1.5 全部串进 20kHz ISR，变成 `motor_current_tick()`。

**固定顺序**：
```c
encoder_kick(&enc_m1);
current_sense_sample(&motor);               // 1.1+1.2
observer->get_theta_el(ctx, &motor);        // 编码器+PLL内插
g_trig_sincos(motor.theta_el, &sin, &cos);  // 一次
foc_clarke_park(&motor);                    // 1.3
motor.ud = foc_pi_step(&pi_id, id_ref, id); // 1.4 Id
motor.uq = foc_pi_step(&pi_iq, iq_ref, iq); // 1.4 Iq
// feedforward_apply  → NULL (Step 7)
// deadband_apply     → NULL (Step 3)
foc_inv_park_svpwm(&motor, &htim1);         // 1.5
telem_bringup_tick();
// DWT isr_delta
```

**验收**：整合后 isr_delta ≤ 2000（比当前 1315 多 ~600-700，留够 Step 3/4 余量）。

---

## 四、新建/修改文件清单

| 文件 | 操作 | 内容 |
|------|------|------|
| `motor/motor_context.h` | 新建 | `MotorContext` C 结构体（ia/ib/ic/id/iq/ud/uq/theta_el/offset 等） |
| `drivers/foc/foc_transform.c` | 新建 | `foc_clarke_park()` + `foc_inv_park_svpwm()` |
| `drivers/foc/foc_pi.c` | 新建 | `foc_pi_t` + `foc_pi_step()` |
| `drivers/foc/foc_current_sense.c` | 新建 | `current_sense_sample()` + 偏移标定 |
| `drivers/foc/svpwm.c` | 新建 | 从 `bringup/trans.c` 提炼 `svpwm_execute()` |
| `Core/Src/main.c` | 修改 | TIM1 回调重组为 `motor_current_tick()` 顺序 |

---

## 五、验收总清单

- [ ] `isr_delta` 稳定（不超 2000，留 Step 3+4 余量）
- [ ] Id≈0 时 Ud≈0（PI 未在无效出力）
- [ ] Iq_ref 阶跃→Iq 跟踪（上升时间 ~1-2ms，过冲 <10%）
- [ ] Ia+Ib+Ic ≈ 0（标度正确）
- [ ] 电机不失控、不啸叫、不过流
- [ ] VOFA+ 四通道可看 Id/Iq/Iq_ref/theta_el
