# 联调日报 · VESC 旋转 HF+DFT 测 Ld/Lq（散度与用法）

| 项 | 内容 |
|---|---|
| 日期 | 2026-10-27 |
| 方法 | `motor/ident/l_hfi_ident.c`：旋转 HF、双极性半拍、`inv_L=f·\|Δi\|/V`、DFT bin0/bin2、×0.9（对齐 VESC `measure_inductance`） |
| 相对旧路 | 取代/优于此前 VASI 开环阶跃估 L（散度大、难复现） |
| 固件包 | `MDK-ARM/_pkg/20261027_calib_rs_hfi_vesc_fft_r9/`（Rs 在前 + 10 轮 L + mark55 shout） |
| 主录波 | `VOFA+CSV/20261026/vofa+202610272257.csv` |
| 分析 | 同目录 `_analyze_vofa202610272257_rs_l.*` |
| LCR 对照 | Rs≈0.115 Ω；Ld≈59 / Lq≈87 / Lavg≈73 µH（0.6 V / 1 kHz，min/max） |

## 结论（本轮签收口径）

1. **重复性很好，方法可用**：同电 10 轮 MCU shout，轮间散度极低，远好于旧 VASI。
2. **绝对值相对 LCR 仍偏高**（Lavg≈+33%），突显差 ΔL 与 LCR 接近；**电流环 Kp 仍用 LCR 铭牌**，勿把偏高的 HFI L 直接灌进 `Kp=L·ωc`。
3. 偏高主因不在 TIM 死区（M1 控制在 TIM8，DT≈百 ns 级）或分流标尺（Rs 拟合合理）；更像指令 V / 有效 `|Δi|` 尺与 LCR 工况差。端电压非必测；可用 CSV + LCR 做 `V_eff` 对照，母线用万用表即可。

## 2026-10-28 收口

- **VASI（`ld_lq_ident`）已从工程移除**；参数辨识 L 路径仅 `l_hfi_ident` + 实验表 `EXP_ID_CALIB_CHAIN`。
- 宏门改为 `M1_L_HFI_IDENT_ENABLE`；旧 `M1_LD_LQ_IDENT_*` 宏山删除。源码归档 `_scratch/archive_vasi/`。

## 定量（mark55，单位 µH）

| | Ld | Lq | Lavg | ΔL=Lq−Ld |
|---|---:|---:|---:|---:|
| 10 轮均值 | 83.62 | 111.09 | 97.36 | 27.47 |
| 轮间峰峰值（约） | ~0.14 | ~0.38 | ~0.19 | — |
| LCR | 59 | 87 | 73 | 28 |
| MCU/LCR | ~1.42× | ~1.28× | **~1.33×** | ≈1.0× |

Rs（mark54，`id` vs `ud_pi`）：拟合 **0.126 Ω**（≈LCR×1.10），截距 V0≈0.052 V ≪ 软死区补偿标称。

## 时序 / 电压相对 VESC

- 半拍采样再翻电压、`inv_L`、DFT、×0.9：**同构**。
- 差：VESC 测 L 时强制 `V0` + `LONGEST_ZERO` 采流，且 **Vbus 实测**；我们仍日常 JEOC 采样 + **`M1_VBUS_V` 宏**。公式形态 `(2/3)·√3/2` 与 SVPWM `1.5/Vbus` 一致。
- 激励量级：seek `|Δi|≥1.5 A`，指令幅值约数伏；堵转时母线平均电流远小于相电流，**母线 2 A 限流一般不是瓶颈**。

## 用法建议

| 用途 | 用哪套 L |
|---|---|
| 电流环 `M1_PI_KP_ID/IQ`、带宽整定 | **LCR / 铭牌**（现 `59/87 µH`） |
| 出厂/联调「L 是否稳、突显是否在」 | **本 HFI 链**（看轮间散度与 ΔL） |
| 观测器 / 保守裕度 | 可继续带 ×0.9；勿再人为抬 Kp 追 HFI 绝对值 |

## 开放项（不挡「方法可用」）

- [ ] L 公式 / SVPWM 改实测 Vbus（对齐 VESC 母线条）
- [ ] 半拍末采样或靠近零矢量窗口，看 Lavg 是否向 LCR 靠拢
- [ ] CSV：`V_eff=L_LCR·f·|Δi|` vs 指令 `√(ud²+uq²)`，量化尺偏（无需差分探头）

## 相关代码

- `motor/ident/l_hfi_ident.c` / `.h`
- `motor/motor_current_exp_hooks.c`（先 `on_sample` 再取 `u_d/u_q`）
- `config/motor_params_m1.h`：`M1_L_HFI_REPEAT_N=10`，`INTER_ROUND_S=2`
