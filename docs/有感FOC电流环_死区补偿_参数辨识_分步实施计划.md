# 有感 FOC 电流环 + 死区补偿 + 参数辨识 分步实施计划

> **平台**: STM32G474 @ 160 MHz
> **当前基线**: isr_delta=1315, 编码器框架已冻结, SVPWM+CORDIC 已运行, 电机已转
> **给 Cursor 的实施提示词** — 重点详述 Step 3 死区 LUT 和 Step 4 参数辨识的方法细节

---

## 总览：四步依赖链

```
Step 1 ──→ Step 2 ──→ Step 3 ──→ Step 4
电流环      初始L/Rs    死区LUT     精标参数
(地基)      (粗测)      (修正)      (辨识)
```

---

## 关键架构变更：主流程在 ADC 回调里执行

**当前**：TIM1 ISR 做所有事（kick + 读角 + SVPWM + telem）。

**目标**：FOC 主流程搬到 `HAL_ADCEx_InjectedConvCpltCallback`。TIM1 只做 kick + telem。

**数据流**：

```
TIM1 ISR (prio 1, 20kHz):
  → encoder_kick(&enc_m1)           // 尽早启动 SPI DMA
  → telem_bringup_tick()            // 遥测
  → return                          // 不碰 ADC、不碰 SVPWM

ADC JEOC ISR (prio 1, TIM1 CH4 触发):
  → 读 JDR → 减偏移 → 乘标度       // ia/ib/ic/vbus 已就绪
  → get_theta_el                    // 编码器+PLL内插
  → g_trig_sincos(theta_el)         // 一次
  → Clarke/Park
  → Id PI / Iq PI
  → (前馈 NULL) / (死区 见 Step 3)
  → InvPark / SVPWM / 写 CCR
  → return
```

**优先级不变**：TIM1 和 ADC JEOC 同优先级（prio 1），互不抢占。TIM1 先 kick 编码器，ADC 在数据到齐后跑 FOC 数学——两段自然衔接。

---

## Step 1：部署电流环

### 你要做的事情（概要）

1. ADC 偏移标定：PWM 启动前采 100 拍取平均，存入 `MotorContext`
2. ADC 回调里实现：读 JDR → 减偏移 → 乘标度 → 写入 `MotorContext.ia/ib/ic/v_bus`
3. 新建 `drivers/foc/foc_transform.c`：`foc_clarke_park()` + `foc_inv_park_svpwm()`，复用 CORDIC 一次调用
4. 新建 `drivers/foc/foc_pi.c`：并联 PI，抗积分饱和，输出限幅 Vbus/√3
5. 新建 `drivers/foc/svpwm.c`：从 `bringup/trans.c` 提炼，扇区判断不用 atan2
6. ADC 回调重组为 `motor_current_tick()`：固定 11 步顺序，前馈和死区留 NULL
7. Kp/Ki 先占位值（0.5/50），Step 2 后替换

### 验收
- isr_delta ≤ 2000
- Id≈0 时 Ud≈0
- Iq 能跟踪 Iq_ref 阶跃

---

## Step 2：初始 L 和 Rs

### 你要做的事情（概要）

1. 用 LCR 电桥测线电阻→除以2→Rs。或直流注入法：Id_ref=固定值→Ud 稳定→Rs=Ud/Id
2. 测转子不同位置的三相电感→通过 Park 关系算出 Ld/Lq
3. 替换 Kp/Ki：Kp=L×1000, Ki=Rs×1000×50e-6

### 验收
- Rs/Ld/Lq 三个粗值拿到（误差 ~30% 可接受）
- Kp/Ki 更新后阶跃响应改善

---

## Step 3：死区补偿 LUT（Id 锁轴 + 占空比扫描）⭐ 详述

### 3.1 背景：为什么要做死区补偿

逆变器输出到电机的实际电压 ≠ 你写的电压指令。三个误差源：

| 误差源 | 特性 |
|--------|------|
| **死区时间** | FD6288 固定 200ns。PWM 上下管切换插入的死区，电流流出方向不同导致电压损失方向也不同 |
| **IGBT/二极管导通压降** | MOSFET 和体二极管的 Vds/Vf，大电流时恒定，小电流时非线性 |
| **寄生电容效应** | 开关过程中 MOSFET Coss 充放电，**只在小电流区间显著** |

**关键**：小电流区（|i|<2A）误差电压是非线性的——不能用一个常数减掉。大电流区（|i|>2A）误差接近恒定，用固定值补偿即可。

### 3.2 方法概述：符号函数法 + 小电流区 LUT

bringup 阶段用**两段式方法**：
- 大电流区：固定补偿电压（按电流极性 ±ΔU_comp）
- 小电流区：离线扫描生成 LUT → 运行时查表 + 线性插值

### 3.3 电学原理：误差电压怎么算

电机静止、Id 闭环锁轴时，d 轴电压方程简化为：

```
Ud_actual = Id_actual × Rs + Ud_error
```

等号右边第一项是电阻压降（你写指令想让电机出来的），第二项是逆变器吞掉的。所以：

```
Ud_error = Ud_actual − Id_actual × Rs
```

`Ud_actual` 是 PI 输出的 Ud（为了让 Id 跟踪 Id_ref，PI 自动补偿了逆变器损失的电压）。`Id_actual × Rs` 是"如果逆变器是理想的话 Ud 应该是多少"。差值就是逆变器误差。

### 3.4 离线扫描流程

**前提**：Step 1 和 Step 2 已完成。PI 能锁轴。Rs 有粗值。

**步骤**：

1. **Id 锁轴**：设 Id_ref = 0（或一个小非零值），转子被永磁体 d 轴吸住不动。确认编码器 raw 不变。

2. **逐点扫描**：从 Id_ref = Id_min（约 0.1A）逐步增到 Id_max（约额定电流 × 1.2）。对每个 Id_ref：
   - PI 稳定后（等 ~50ms），记录该点的 `(Id_actual, Ud_output)`
   - 算 `Ud_error = Ud_output − Id_actual × Rs`
   - 存进数组 `(Id_actual, Ud_error)`

3. **采样密度**：
   - |Id| < 2A（小电流区）：步长 0.1~0.2A，密采样
   - |Id| > 2A（大电流区）：步长 0.5~1A，粗采样

4. **生成 LUT**：把离散点通过线性插值或分段线性拟合成一张一维表。表长 32 点，覆盖 [0, Id_max]。每个表项 = (电流幅值, 误差电压)。

5. **存储**：LUT 是 `float deadband_lut[32][2]` 或两个数组 `float dlut_amps[32]` + `float dlut_vals[32]`。放在 `MotorContext` 或全局，运行时只读。

### 3.5 运行时查表方式

每个 PWM 周期，在 `deadband_apply()` 里：

```c
// 三相分别处理
float comp_a = deadband_lookup(deadband_lut, fabsf(ia));
if (ia > 0) ua_comp = ua - comp_a;
else        ua_comp = ua + comp_a;
// ib, ic 同理
```

查表函数对 `fabsf(ia)` 在 LUT 里找两个相邻点→线性插值→返回误差电压。

### 3.6 极性判断与过零区

电流过零点附近（|i| < 某阈值，如 0.05A）极性判断可能出错——电流纹波导致 sign() 抖动。处理方式：过零阈值以下**不补偿**（或取平均补偿），避免误补。

### 3.7 大电流区简化

如果扫描发现大电流区（|i|>2A 段）的 Ud_error 趋于常数（平坦），那段可以直接用常数——流程和符号函数法一样——不用查表。查表只覆盖小电流非线性区。这样表更短，插值更快。

### 3.8 验收

- 逐点扫描脚本跑完，Ud_error vs Id 曲线连续合理
- LUT 表生成，数据可看（32 点）
- 死区补偿使能/关闭对比 VOFA+ 看相电流过零点畸变（补偿后畸变变小）
- Id 锁轴稳定（不抖、不转、不啸叫）

---

## Step 4：参数精标 ⭐ 详述

### 4.1 前置条件

Step 3 完成。死区已补偿→逆变器非线性误差被消掉→实际作用在电机上的电压 ≈ 你写的指令电压。

### 4.2 精标 Rs（论文 2.5 节：斜坡电流注入 + 最小二乘）

**物理原理**：

电机静止时（ω=0），d 轴电压方程：

```
Ud = Rs × Id + Ld × dId/dt
```

如果 Id 变化极慢（斜坡斜率极小），`Ld × dId/dt` 项可忽略（电感压降远小于电阻压降）。此时 Ud 与 Id 近似成正比，斜率 = Rs。

**步骤**：

1. d 轴 PI 跟踪指令：Id_ref 从 0 斜坡上升至 1.2×额定电流（如 2.5A → 3A）。斜坡斜率极小——每秒上升 0.5~1A——保证 `dId/dt` 很小。
2. 每个控制周期记录 `(Id_actual, Ud_output)`。
3. **仅保留大电流数据**：|Id| > 2A 的区间。小电流区间即使有死区补偿，残余非线性也偏大——剔除。
4. 对大电流区间数据做**最小二乘线性回归**：拟合 `Ud = Rs × Id + b`。斜率即 Rs。截距 b 是残余恒定误差（IGBT 导通压降等），不用管。
5. 补偿 IGBT 导通压降：如果知道 V_igbt 大概值（从器件手册），额外修正 Rs。对于 MOSFET 驱动的低压系统（24-48V bus），导通压降占比小，可先跳过此修正。

**公式（最小二乘）**：

```
Rs = (m × Σ(Id×Ud) − Σ(Id) × Σ(Ud)) / (m × Σ(Id²) − Σ(Id)²)
```

m = 大电流区间采样点数。

**为什么先做 Rs**：精标 Ld/Lq 时磁链积分需要用到 Rs（见 4.3）。Rs 不准→磁链不准→电感不准。

### 4.3 精标 Ld/Lq（论文 2.3 节：VASI 高频方波注入）

**物理原理**：

电机静止、ω=0。dq 轴电压方程退化为：

```
Ud = Rs×Id + Ld×dId/dt + Ldq×dIq/dt
Uq = Rs×Iq + Lq×dIq/dt + Lqd×dId/dt
```

注入高频方波电压→电流响应含高频分量→对电压方程积分得到磁链→磁链对电流的偏导数 = 电感。

**为什么用对称正反方波**：正半周注入 +Ud，负半周注入 −Ud，幅值相等、时长相等。产生的转矩脉冲大小相等方向相反→合力矩为零→电机不转。不需要机械锁轴装置。

**步骤**：

1. 选择注入参数：方波频率 ~500 Hz-2 kHz（每个半周持续 250-1000 µs）。电压幅值从 Vbus×10% 开始，逐组增加到 Vbus×60%——这叫"变幅值"。
2. 对每组注入电压，采样 dq 轴稳态电流(dId/dIq)和电压(Ud/Uq)。
3. 计算磁链：`ψd = ∫(Ud − Rs×Id)dt`，`ψq = ∫(Uq − Rs×Iq)dt`。
4. 对每组电流做数值偏导：`Ld = ∂ψd/∂Id`，`Lq = ∂ψq/∂Iq`。离散做法：相邻两个电流点的 Δψ/ΔI。
5. 变幅值注入 9-16 组后，得到一系列 `(Id, Ld)` 和 `(Iq, Lq)` 离散点。
6. 对这些离散点做**二次多项式拟合**：`Ld(Id,Iq) = a0 + a1×Id + a2×Iq + a3×Id² + a4×Iq² + a5×Id×Iq`。得到连续的电感曲面。Lq 同理。
7. 拟合系数通过递推最小二乘法(RLS)求解。

**论文数据参考**：1.6kW SPMSM 电感误差 < 5.9%，25kW IPMSM 误差 < 4.1%，总辨识时间 < 20 秒。

**注意事项**：

- 注入电压幅值要落入电压极限圆内，保证转子转角 < 8°（论文推导了这个约束）
- 方波频率取 500 Hz-2 kHz 之间：频率太低→电流纹波大、转矩>摩擦力转子会转。频率太高→di/dt 分辨率不够、电感计算噪声大
- 先注入低频（500 Hz）粗扫，再提高频率精扫——分段做
- 逆变器死区必须已补偿（Step 3），否则注入的低幅值方波会被死区吞掉——拿到的电流不是真正的电流

### 4.4 永磁磁链 ψf（反电动势法）

**方法**：开环拖动电机至稳定转速（如 500-1000 rpm），在 Uq 未加载时将 Uq 设为零→三相电压采反电动势波形→任意两相线电压幅值除以电角频率 = ψf。

```
ψf = E_peak / ω_el
```

E_peak 是反电动势正弦波的幅值。ω_el = ω_mech × pole_pairs。

**替代方法**：从扭矩常数 Kt 反推。`ψf = Kt`（SPMSM 成立；IPMSM 含磁阻转矩分量需分离）。

### 4.5 验收

- 四个参数：Rs(<5%), Ld(<6%), Lq(<6%), ψf 拿到最终值
- 辨识过程电机静止或准静止（VASI 最大转角 <8°）
- 连续跑三次，参数接近（Rs 相差 < 0.1Ω, Ld/Lq 相差 < 0.05mH）
- 四个数存入 `MotorConfig`，供 PI 公式整定和后续 MTPA 使用

---

## 依赖关系表

| Step | 前置条件 | 产出 | 下一步依赖 |
|------|---------|------|-----------|
| 1 电流环 | 编码器框架+SVPWM | PI 能力, Id锁轴 | Step 2/3/4 |
| 2 初始 L/Rs | Step 1 | Rs/Ld/Lq 粗值 | Kp/Ki 初值, Step 3 误差计算 |
| 3 死区 LUT | Step 2 | 死区补偿表, 逆变器线性化 | Step 4 |
| 4 精标参数 | Step 3 | Rs/Ld/Lq/ψf 精值 | 论文, PI/速度环 |

---

## 全局约束（Cursor 注意）

1. **ADC 回调是主流程**：`HAL_ADCEx_InjectedConvCpltCallback` 里跑完整的 FOC 数学。TIM1 只做 encoder_kick + telem
2. **20kHz 热路径单体 C 函数**：不做逐步 ops 表。扩展点（前馈/死区）NULL 跳过
3. **CORDIC 每拍调一次**：Park + InvPark + SVPWM 共用同一组 sin/cos
4. **Kp/Ki 可运行时改**：放 `foc_pi_t` 结构体里，不写死常量
5. **偏移标定用软件**：上电采 100 拍平均，不碰硬件 offset 寄存器
6. **新代码放 `drivers/foc/` 和 `motor/`**：不污染 bringup/
