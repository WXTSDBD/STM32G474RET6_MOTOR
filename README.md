# STM32G474RET6_MOTOR

基于 **STM32G474RET6** 的永磁同步电机 FOC 固件：有感三环（电流 / 速度 / 位置+MIT）、无感（HFI→SMO，GATE 141）、参数辨识与台架联调。

工程入口：`MDK-ARM/STM32G474RET6_MOTOR.uvprojx`（Keil AC6 + FreeRTOS + CubeMX）。

---

## 这个项目在做什么

一句话：**做一套「能换电机、能换板、能换传感器」的 PMSM 控制平台** —— 而不只是"让电机能转"。

它由四条线组成：

| 线 | 目标 | 现状 |
|---|---|---|
| **① 控制能力** | 电流 → 速度 → 位置/MIT 三环闭合；能用编码器（有感），也能不用（无感） | 大部分签收，见下一节 |
| **② 平台化** | 换硬件只改**数据**（板级描述 / 铭牌 / 实验表），**算法层零改动** | `motor/` 已零 HAL；四张 Port 契约；铭牌 `motor_cfg_t` |
| **③ 产品化** | 交出去**不会烧硬件**、能远程配置与升级 | ❌ 未做（安全层 / 总线 / OTA） |
| **④ 建模台** | 数据 → 模型 → 预测：参数单一源、质量门、传递函数/状态空间、真 C SIL | 🟡 可用骨架，见「统一建模台」 |

**最终交付**：一套**带实测签收证据**的代码 + 一套**可复用的过程方法**（交付包 / 金样 / 判据 / 宏账）。

### 术语速查（给不熟悉电机控制的人）

| 词 | 意思 |
|---|---|
| **FOC** | 磁场定向控制：把三相电流变换到转子坐标系（dq）分别控制，转矩 ≈ `Kt·iq` |
| **有感 / 无感** | 有编码器（知道转子角）/ 没编码器（**靠算法估**转子角） |
| **HFI** | 高频注入：静止或极低速时注入高频信号**估算**角度（"零速无感"） |
| **SMO / EMF-PLL** | 两种中高速无感观测器（滑模观测 / 反电势锁相环） |
| **I/F 起动** | 开环强拖起动，转速上来后把角度交接给观测器 |
| **辨识** | 自动测电机参数（`Rs` 相电阻 / `Ld·Lq` 电感 / `ψf` 磁链），代替手填 |
| **死区** | 逆变器上下管切换的必需空隙；它使实际电压偏离指令，**需要补偿** |
| **MIT 模式** | 位置环的"弹簧—阻尼"参数化形式（`Kp/Kd` + 前馈力矩） |
| **签收 / 金样** | 一次实验结论被正式接受；"金样"是那次的高质量录波，用于后续回归对比 |

---

## 硬件与软件

| 项 | 内容 |
|---|---|
| MCU | STM32G474RET6（Cortex-M4F @160 MHz / 512 KB Flash / 128 KB RAM） |
| 电机 | N5065 外转子，极对数 **7** |
| 编码器 | AS5047 磁编（SPI1 + DMA）；零偏已标定；**编码器可选**（拔掉可跑无感） |
| 功率级 | 三相全桥 + 采样电阻 + 片内 OPAMP |
| 工具链 | CubeMX（生成 `Core/`）+ Keil MDK **AC6（armclang）** + FreeRTOS |
| 节拍 | 电流环 **20 kHz**（ADC 注入 JEOC 链）/ 外环 **2 kHz** |
| 遥测 | LPUART + DMA → VOFA+ JustFloat，12 通道（可选带头 `layout_id`） |

---

## 能力现状（做了什么）

| 能力 | 状态 | 证据 / 备注 |
|---|---|---|
| **电流环** | ✅ **签收** | Id/Iq 扫频 Bode；带宽上限方法论（平台等效延迟 `Td ≈ 1.6~1.7 Ts`） |
| **速度环（有感）** | 🟡 有条件签收 | 轻载 + 带载阶跃 |
| **速度环（无感，空载）** | ✅ **GATE 141 冻结** | 编码器插着 / 拔着均过；**带载尚未做** |
| **位置环 / MIT（有感）** | 🟡 有条件签收 | P3 短表 9 段；日常短回归 `tools/regress_sensed_short.py` |
| **位置环（无感）** | ⛔ **结档** | θ̂ 多圈解包动态丢量（算法级） |
| **HFI 静止起动** | ✅ GATE 141 冻结 | 踢相 → HFI 速度阶梯 → 交接 |
| **中高速无感观测** | 🟡 有条件 | SMO 提供反电势 → **EMF-PLL 发布角**；软切地板 ~800 rpm |
| **参数辨识** | ✅ `Rs` / `Ld·Lq` 签收；🟡 `ψf` 未签收 | VASI 法；结果目前**需人工回填宏**（NVM 无电机参数段） |
| **编码器标定** | ✅ | AS5047 零偏（SPI1 + DMA） |
| **死区补偿** | 🟡 代码齐全（`motor/deadband/` 17 文件 ~4.6k 行），**签收档默认 OFF** | 烤表 + 标定链可用；**只有 Bode 档开几何法** → 收尾见 [`docs/规划_死区补偿收尾_2026-10-08.md`](docs/规划_死区补偿收尾_2026-10-08.md) |
| **前馈 / ESO / MTPA / 弱磁** | ❌ 未做 | 解耦前馈实现已在但宏默认关；运行时位 `atrb` 已备好 |
| **安全层**（故障状态机 / 错误码） | ❌ 未做 | BKIN / IWDG **当前不做**、保留重评条件（见「延后项与触发条件」）；错误码 / 故障状态机见 M3 |
| **通信 / OTA**（CANopen / CLI / 升级） | ❌ 未做 | 后排 |
| **工程设施** | ✅ | 交付包（hex + `sha256` + `git.txt`）/ 宏账 / 遥测表驱动 / 实验框架 E0 / 板级描述 / NVIC 集中 |
| **统一建模台** | 🟡 可用骨架 | 数据质量门 + 参数单一源 + 传递函数/状态空间 + 预测/SIL；`arch` 54 模块、`regress` 30/30 |
| **真 C SIL** | 🟡 原语级 | `siltick`（Clarke/Park/PI/SVPWM）、`silpll`（PLL scheme 0/1）；**不是完整 ISR** |

> 状态明细与切换方式以 [`docs/联调实例_INDEX.md`](docs/联调实例_INDEX.md) 为准。

---

## 统一建模台（2026-10-17）

一句话：把散落在录波、报告、固件宏里的“事实”收成**可追溯、可失败、可预测**的模型，
用 CLI + JSON/Markdown 输出；**不做 GUI，不做 RL**。它是分析工具，不替代台架签收。

| 层 | 现在有什么 | 证据 / 边界 |
|---|---|---|
| **数据** | 质量门（元数据/秩/条件数/残差/EMF 信息量/回放）、全段聚合、A/B/C 锚点 | `out/current_batch_20261017_225507.json`：回放 13 PASS/13 WARN/12 REJECT；质量 0 PASS/26 WARN/12 REJECT；新门重选锚点 A=0/B=13。**这不等于数据已清洗干净** |
| **对象** | `machine.py` 物理对象；`plant.py` 回放/拟合；参数单一源 | `params.py:113-133`：`Rs=0.123859 Ω`、`Ld=59.94 µH`、`Lq=87 µH`、`J=8.0e-5`（工作先验）、`τc=0.022 N·m`、`B=0`（B 不可辨识）；`Lq/Td` 仍有推断项 |
| **包络** | 稳态 dq 电压/电流约束、基速、每转速最大 iq | `envelope.py --firmware`：空载基速 3150 rpm、11 A 基速 2809 rpm；3000 rpm 仅 5.07 A；>3150 rpm 标电压阻断（无弱磁） |
| **控制器** | 唯一 PI 基线；LQR/极点配置/观测器/RLS | `electrical.py:210 pi_loop_baseline()`；LQR 延迟扫描 `out/lqr_current_3000_delay.json`：`nd=2` 时 ρ>1 |
| **预测** | P0 回放 / P1 插值 / P2 交叉 / P3 外推 | `predict.py`、`horizon.py`；真实外推仍未签收 |
| **SIL** | 真 C `foc_pi.c`、FOC 核心 tick、`motor_pll.c` | `siltick` 800 拍 PASS；`silpll` 200×64 PASS（`theta=3.26e-7`、`omega=2.95e-4`，tol `5e-4`） |
| **判据** | A1–A6 架构判据 + 单入口回归 | `arch.py` 54 模块全过；`regress.py` 30/30，日志 `out/regress_20261017_234253.log` |
| **换电机** | `plantview`/`predict`/`bench` 的 `--params-json` | 用 `PmsmParams.from_json()` 替换 Rs/Ld/Lq/ψ/J/B/τc，模型结构不变；见 `tools/model/README.md` §8 |

> 5065 商品页只作先验：J 6.9e-5~1.93e-4、B 3~9e-5；Kt 仍以实测 0.063 为准（照片 KV 270–400 与实测不自洽）。
> 编码器延迟已按 1 拍 = 50 µs 建模；10–500 rpm 仅 0.02–1.05° 电角。详见 [`docs/分析_5065铭牌先验与编码器延迟_2026-10-17.md`](docs/分析_5065铭牌先验与编码器延迟_2026-10-17.md)。
> **控制器配置不再手抄**：`python tools/simtool/fwcfg.py` 直接读当前 active profile；`python tools/simtool/firmwareview.py --rpm 10 --t-end 3` 用同一份配置做预测。
> **工作包络**：`python tools/simtool/envelope.py --firmware` 给出 24 V/11 A 下的基速与每转速最大 iq；当前空载 3150 rpm、满载 2809 rpm，>3150 rpm 无弱磁即失效。见 [`docs/分析_工作包络与基速_2026-10-17.md`](docs/分析_工作包络与基速_2026-10-17.md)。
> **当前固件模型卡**：`python tools/simtool/plantview.py --firmware --export`，再跑 `modelcard.py --export`。
> 2026-10-17 实测：`M1_USE_SPEED_LOW_SWEEP_PROFILE=1`，PLL bw=1000、β=0.4、齿槽前馈=1、I 限幅=11 A；
> 不要拿 `foccfg.py` 的 400/1.0 默认值当当前固件。详见 [`docs/分析_当前固件控制主体配置_2026-10-17.md`](docs/分析_当前固件控制主体配置_2026-10-17.md)。
> 想直接看当前配置的预测：`python tools\simtool\bench.py --firmware --t-end 3.0`。
> 录波包用 `tools/pkg_snapshot.py` 冻结：`config/` + 全部 profile + `params.py` + `fwcfg.json` + hex hash。

常用命令：

```powershell
python -B tools\simtool\arch.py
python -B tools\simtool\arch.py --selftest
python -B tools\simtool\regress.py
python -B tools\simtool\siltick.py --selftest
python -B tools\simtool\silpll.py --selftest
python -B tools\simtool\frictionrun.py --selftest
```

边界（必须先说清）：

- `siltick`/`silpll` 是**真 C 原语/核心链**，不是完整 `motor_current.c` ISR；ADC 重构、SMO/HFI、死区 LUT/GEO、启动状态机仍未拉回 host。编码器延迟已在 `topologies`/`plant` 建模（1 拍），但真 C 整机 ISR 尚未逐拍对账。
- 数据契约 D011 仍是“部分”：`tools/sim/sig.py:76` 的 `delay=0`，`layout_id/timestamp/drop_count` 未进动态拟合，阻塞在固件帧头/排期。
- LQR 未进固件（D021 `proposed`）；模型结论不等于硬件结论。
- `tools/sim/loader.py` 815 行是已登记的豁免项（`BCK-001`）。

入口文档：[`docs/纲领_统一建模台_使命与边界_2026-10-17.md`](docs/纲领_统一建模台_使命与边界_2026-10-17.md)、
[`docs/台账_建模台决策记录_2026-10-17.md`](docs/台账_建模台决策记录_2026-10-17.md)、
[`docs/收口_2026-10-17_1515.md`](docs/收口_2026-10-17_1515.md)、
[`docs/回应_外部评审_2026-10-17.md`](docs/回应_外部评审_2026-10-17.md)、
[`tools/simtool/README.md`](tools/simtool/README.md)。

---

## 接下来要做什么（路线）

**四轨并行**（无强制先后，按硬件到货与可支配时间插空）：

| 轨 | 下一步 | 前置 / 说明 |
|---|---|---|
| **① 功能** | **带载无感速度环** → 解耦前馈 A/B → ESO；死区 runtime ON；ψf 辨识签收；MIT 正式报告 | 前馈/ESO 的 A/B 需要"**段表带配置**"（实验框架的下一小步）；其余现在就能开 |
| **② 平台化** | 换一款**新编码器**（16-bit SPI）→ 换 **CBT6 板**（同家族不同型号）→ 每次记录「迁移成本 4 个数」 | 硬件到货；**保留一份 AS5047 接线**做已知好参考 |
| **③ 工程化** | 错误码 / 故障状态机 → 出厂整定（辨识→校验→写回 NVM）→ CANopen / OTA | BKIN / IWDG **当前不做**（理由见「延后项与触发条件」）；NVM 与 OTA 的 Flash 布局**同批做** |
| **④ 建模台** | 数据契约（`layout_id/timestamp/drop_count`）→ ADC 重构/编码器量化 → 死区 FIXED SIL → SMO/HFI | 先补证据再谈预测；GUI/RL 不做；每刀必须能失败、可复现 |

**贯穿**：每完成一项就出**对比图 + 签收**（论文素材），并把结论写回 `docs/`。

**两条硬承诺**：
1. **一次只让一样东西是新的**（新编码器 / 新板子 / 新负载 / 新功能，不同时上，否则出问题无法归因）；
2. **改一行 → 要么 hex 逐字相同（不必烧），要么必须烧 + 跑主回归门**。

---

**默认联调档**：`config/bringup_active.h` → 有感 `m1_sensed_pos_mit`。  
进度 / 欠账：[`docs/施工_总纲_Agent_2026-10-07.md`](docs/施工_总纲_Agent_2026-10-07.md)、[`docs/backlog_远古规划对照_2026-10-07.md`](docs/backlog_远古规划对照_2026-10-07.md)。

---

## 阶段计划（里程碑 M1–M6）

> 里程碑按"**能验证的成果**"划分，不按时间。每个里程碑的"完成" = **有签收物 / 判据的东西**。

| 里程碑 | 包含 | 完成定义 | 依赖 |
|---|---|---|---|
| **M1 · 功能闭环** | 带载无感速度环；解耦前馈 A/B；ESO；死区补偿 runtime ON；ψf 辨识签收；MIT 正式报告 | 无感速度环**带载签收**；前馈 / ESO 有**开·关对照** | 磁粉制动器（已有）、`atrb` 位（已有）；前馈/ESO 需"**段表带配置**" |
| **M2 · 可移植性实测** | ① 换编码器（同板）② 换 CBT6（同编码器）③ 两者同时；板级数据补齐 | 三次迁移各产出「**迁移成本 4 个数**」；**③ 的改动量 ≈ ①+② 并集** | 新编码器（16-bit SPI）、CBT6 板；**保留一份 AS5047 接线** |
| **M3 · 产品态最小集** | 错误码体系；故障分级 / 停止策略；故障记录持久化；（BKIN/IWDG 见"延后项"） | **故障能被记录并上报**，上位机能查到原因 | 无前置；**不依赖 BKIN** |
| **M4 · 参数与 NVM（离线可配置）** | 真实 Flash 分区表；**NVM 电机参数段**；出厂整定流水线（辨识→**校验判决门**→写回）；参数运行时写回 | **换一台电机 = 跑一遍整定，重编译 0 次** | 与 M5 的 OTA **共用同一次 Flash 布局改造** |
| **M5 · 总线与升级** | CANopen（NMT/HB/SDO/PDO/CiA402/EMCY）；CLI；OTA（Boot+App 双 Target、App 槽重划） | 上位机能**远程选实验 / 读结果 / 改参数 / 升级固件** | M3（有故障可报）、M4（有参数可写）；**受 Flash 预算约束** |
| **M6 · 结档与答辩收口**（贯穿） | 无感位置环结档说明；对比图汇总（死区 THD / 无感 θ 误差 / 软切冲击 / 辨识 vs LCR / 带宽重复性 / 动态 L）；口径统一 | 每个"有条件签收"要么升正式、要么**明确结档并写明原因**；论文里每个数字都能指到 CSV / 包 | M1 / M2 的产出 |

**并行与耦合**：
- **M1 与 M2 可并行**（一个动控制、一个动接线，互不干扰）；
- **M3 无前置**，可随时插；
- **M4 与 M5 的 OTA 必须同一批做**（Flash 布局只改一次，否则返工）；
- **M6 贯穿**：每做完一项就收一张图，**不要攒到最后**。

---

## 目录

| 路径 | 职责 |
|---|---|
| `motor/` | 控制算法与外环（电流环编排、FOC、观测器、实验框架） |
| `config/` | 联调选择、`motor_cfg`、profile、中断优先级 |
| `platform/` / `board/` | 板级与寄存器口（PWM / SPI / ADC 等） |
| `debug/` | 串口 JustFloat 遥测、`telem_table`、打点号 |
| `cal/` | 编码器 / 相位 / NVM 等标定 |
| `bringup/` | 轴回调、DWT 等联调支撑 |
| `Core/` | CubeMX 生成代码（尽量少改业务） |
| `docs/` | 设计、签收、施工与 RUNBOOK（**文档索引**见下） |
| `tools/` | 可复用分析 / 宏账 / 短回归脚本 |
| `tools/model/` | 建模数学：参数单一源、对象、PI/裕度、回放、PLL 参考 |
| `tools/simtool/` | CLI 入口、架构判据、质量门、LQR/观测器、真 C SIL |
| `tools/sim/` | 数据装载与信号契约 |
| `VOFA+CSV/<日期>/` | 录波与对应 `_analyze_*`（勿堆仓库根） |
| `MDK-ARM/_pkg/` | 本地可烧录包快照（一般不进 git） |

---

## 编译与烧录

1. 用 Keil 打开 `MDK-ARM/STM32G474RET6_MOTOR.uvprojx`
2. Rebuild，验收 **0 Error(s) / 0 Warning(s)**
3. 烧录生成的 hex；正式交付包习惯落在 `MDK-ARM/_pkg/<日期>_<实验名>/`

命令行（本机路径按环境改）：

```powershell
Set-Location "D:\stm32\STM32G474RET6_MOTOR\MDK-ARM"
Start-Process -FilePath "D:\keil5\UV4\UV4.exe" `
  -ArgumentList '-r','D:\stm32\STM32G474RET6_MOTOR\MDK-ARM\STM32G474RET6_MOTOR.uvprojx','-o','rebuild_log.txt' `
  -Wait -NoNewWindow
Get-Content .\rebuild_log.txt -Tail 5
```

CubeMX / AC6 / FreeRTOS 细则：[`docs/Keil_AC6_FreeRTOS_CubeMX适配说明.md`](docs/Keil_AC6_FreeRTOS_CubeMX适配说明.md)。

---

## 交付包规范（`_pkg/`）

每个包 = **一次可回放的实验**，建议含：

| 文件 | 作用 |
|---|---|
| `*.hex` | 可烧录固件 |
| `hex.sha256` | 指纹（用于「档 1：四数 + hex 逐字相同」） |
| `rebuild_log.txt` | 末尾必须 `0 Error(s) 0 Warning(s)` |
| `size.txt` | `Code / RO-data / RW-data / ZI-data` 四数 |
| `git.txt` | 提交号 / 分支 / dirty 清单 —— **让包能重建源码** |
| `bringup_active.h` + `*.profile.h` | 当时的实验与参数快照 |
| `README.md` | 改了什么 / 怎么验 / 台架结果 |

**档位判定**：四数 + hex **逐字相同 → 档 1，不必烧**；有差异 → **档 2，必须烧 + 跑主回归门**。
**一刀一包**：一次只改 1~2 处；动了 12ch 通道映射或金样 → **必须重录**。

---

## 切换联调实例

平时**只改** [`config/bringup_active.h`](config/bringup_active.h)：选哪份 `config/profiles/*.profile.h`，以及 `M1_HFI_GATE` 等。

不要在热路径里为「再试一个实验」堆宏。新实验流程：登记 `docs/联调实例_INDEX.md` → RUNBOOK → 签收报告。

有感签收短回归示例：

```powershell
python tools/regress_sensed_short.py --latest
```

---

## 分层与纪律（关键约束）

```text
Core/(CubeMX) → config/hal_bridge.h ─┬→ platform/（Port 契约 + backend）→ board/（实例）
                                     └→ motor/（算法与编排，零 HAL）
```

| 约束 | 说明 |
|---|---|
| **`motor/` 零 HAL** | 不得出现 HAL 类型 / 裸寄存器 / `main.h` |
| **热路径 20 kHz** | 允许寄存器直写；**禁止 LL 驱动**；LL 仅用于 init / 标定 / RTOS |
| **HAL 句柄** | 只从 `config/hal_bridge.h` 进 `board/`、`platform/`，**不要从 `main.h` 拿** |
| **CubeMX 生成文件** | 业务逻辑不写进 `USER CODE` 之外；**NVIC 优先级只在 `config/irq_priority.h` 改**（`.ioc` 里改不生效） |
| **实验开关** | 只在 `config/bringup_active.h` 选 profile；台架总线开关在 `config/bringup_bench.h` |
| **调试遥测** | 走 `debug/telem_table.*`（表驱动）；`dbg` 仅作探索类残留观测 |

---

## 遥测（VOFA）

- 默认 **JustFloat 12 通道**；签收布局由 `telem_table` 驱动（帧头 `layout_id` 默认关）
- CSV：`VOFA+CSV/<YYYYMMDD>/vofa+<YYYYMMDDHHMM>.csv`
- 一次性分析脚本与产出放**同日期目录**；跨实验工具放 `tools/`
- 分析脚本依赖：Python 3 + `numpy`（`pip install numpy`）

**有感签收 12ch（`M1_VOFA_SIGNOFF_CH=1`）**

| ch | 量 | ch | 量 |
|---|---|---|---|
| 0–2 | Ia Ib Ic | 6–7 | θ_ref / θ_err |
| 3 | Iq | 8–9 | ω_pll / ω_ref |
| 4–5 | θ_fb / θ_mech | 10–11 | Iq_ref / mark（`open_seq`） |

说明：[`docs/方案_遥测表驱动_2026-10-07.md`](docs/方案_遥测表驱动_2026-10-07.md)、[`tools/README.md`](tools/README.md)。

---

## 文档从哪读

完整索引：[`docs/README.md`](docs/README.md)

| 想了解 | 先看 |
|---|---|
| 统一建模台（北极星/边界） | [`docs/纲领_统一建模台_使命与边界_2026-10-17.md`](docs/纲领_统一建模台_使命与边界_2026-10-17.md) |
| 建模台决策台账 | [`docs/台账_建模台决策记录_2026-10-17.md`](docs/台账_建模台决策记录_2026-10-17.md) |
| 本轮收口 / 外部评审回应 | [`docs/收口_2026-10-17_1515.md`](docs/收口_2026-10-17_1515.md)、[`docs/回应_外部评审_2026-10-17.md`](docs/回应_外部评审_2026-10-17.md) |
| 建模命令与 SIL | [`tools/simtool/README.md`](tools/simtool/README.md) |
| Agent / 当前主线停在哪 | [`docs/施工_总纲_Agent_2026-10-07.md`](docs/施工_总纲_Agent_2026-10-07.md) |
| 架构接缝与触发条件 | [`docs/规划_架构改进_2026-10-07.md`](docs/规划_架构改进_2026-10-07.md) |
| 代码现状与问题 | [`docs/总结_代码现状与问题分析_2026-10-07.md`](docs/总结_代码现状与问题分析_2026-10-07.md) |
| 无感 GATE 141 | [`docs/签收_GATE141_无感速度反馈_2026-10-06.md`](docs/签收_GATE141_无感速度反馈_2026-10-06.md) |
| HFI 规格 | [`docs/方案_HFI具体实现_2026-10-07.md`](docs/方案_HFI具体实现_2026-10-07.md) |
| 欠账总表 | [`docs/backlog_远古规划对照_2026-10-07.md`](docs/backlog_远古规划对照_2026-10-07.md) |

---

## 延后项与触发条件（**不是"不做"**）

> ⚠️ **除"无感位置环"外，本节各项都要做，只是排在后面**，且各有明确的**触发条件**。
> 写成"触发条件"而不是"不做"，是为了**不让它们变成永远挂着的饼**，也**不让后人误以为已经否决**。

| 项 | 什么时候做 | 触发条件 / 说明 |
|---|---|---|
| **无感位置环** | ⛔ **已结档**（唯一真正关闭的项） | θ̂ 多圈解包动态丢量，算法级；不再投入 |
| **BKIN / IWDG** | 当前**不做**，**保留重新评估条件** | 硬件无 BKIN 源；20 kHz 硬实时不宜开狗。**若**后续加比较器/换板，或改为"低优先级任务喂狗 + 校验控制心跳"，可重新评估 |
| **死区补偿收尾**（产品档打开 + **THD 量化**） | **现在就可以做**（不需新硬件） | 判据 = 主回归门 141 + P3 金样 + **同一次上电 A/B 的 THD / 5·7 次谐波对比**；详见 [`docs/规划_死区补偿收尾_2026-10-08.md`](docs/规划_死区补偿收尾_2026-10-08.md) |
| **MTPA / 弱磁 / 在线自适应（铜阻温漂）** | M1 之后有余力时 | 凸极机有收益（`Lq/Ld ≈ 1.47`），但台架成本高 |
| **关节阻抗 Z / 多轴 / 关节电机移植** | 有对应硬件时 | 需第二轴或关节电机 |
| **跨 MCU 厂商适配（GD32 / 国产）** | **第二块非 ST 板到手时** | 换 `platform/` backend + 决定是否换 HAL 框架；**不自己发明"通用跨厂商抽象层"** |
| **生成器式配置（YAML → 代码）** | CANopen 阶段再评估 | 当 OD 表要求"协议 / 结构体 / NVM 布局**三者一致**"时 |
| **纯为可读性拆大文件**（`motor_current.c` 1960 行等） | **挂到功能目标上做** | 触发：需要"给角源 / 策略运行时可切"，或实验成批增加时 |
| **单元测试** | 等触发 | 触发：换后端 / 换平台 / **第二次修改同一个算法文件** |
| **HRTIM（换外设类型）** | 板子回来后 | 接口已留（`pwm_port_ops` + 板级描述）；届时**新写一个 backend** |
| **HFI ↔ SMO 全速域无缝切换** | 低速与中高速各自稳定之后 | 现在是"地板 ~800 rpm"的有条件签收 |

---

## 给 AI Agent 的读序

```text
1) 本 README
2) docs/README.md                      （文档总索引）
3) docs/联调实例_INDEX.md               （实验与签收状态）
4) docs/施工_总纲_Agent_2026-10-07.md   （当前主线 / 停止条件 / 红线）
5) .clinerules/*.md                    （纪律，优先级最高）
```

改 `.c/.h` → 立刻 `tools/quickcheck.ps1`；交付前 Keil **0E/0W**；**一刀一包一判据**；
**git 操作（`add`/`commit`/`reset`/`checkout`/`clean`）必须先经人工批准**。

---

## 仓库约定（短）

- **不要**把录波 CSV、一次性 `_analyze_*.py`、Keil `.o/.axf` 堆在仓库根或 `MDK-ARM/` 根
- git：不自动 commit / 不改历史；提交范围说清楚再动
- **文档命名**：`<前缀>_<主题>_<YYYY-MM-DD>.md`；前缀 `规划_` `方案_` `施工_` `验证方案_` `分析报告_` `验收报告_` `总结_` `签收_` `联调日报_` `结档_` `backlog_`
- **结论标注**：【事实】= 直接读到的 /【推断】= 未验证 /【待验证】= 需台架确认（**不许把推断写成事实**）
- **一件事一处定义**：同一事实不要在两处维护（宏、地址、中断优先级、参数……）
- 热路径（约 20 kHz JEOC）少加 `volatile` 镜像写；实验走外环 / `motor/experiment`

更完整的卫生规则：`.cursor/skills/repo-hygiene/SKILL.md`、`.clinerules/`。

---

## 旁支（未并入主交付）

- `G4_bootloader/`、`STM32G474CBT6_OTA_*`：OTA / CANopen 相关，合并方案见 `docs/OTA_CANopen_工程合并方案_2026-08-20.md`
- 主固件日常联调以本仓库 `MDK-ARM` 工程为准

---

*最后更新：2026-10-17*
