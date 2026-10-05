/**
 * @file bringup_active.h
 * @brief 联调实例选择 — 平时只改本文件。
 *
 * Bode / SPEED / FLUX / OBS_VEQ / IF：见各 profile。
 * HFI：config/profiles/m1_hfi_standstill.profile.h（须 SPEED_IDENT）
 *   切实验只改 M1_HFI_GATE（1…8 冻结；9+=捕获对照）
 */
#ifndef CONFIG_BRINGUP_ACTIVE_H
#define CONFIG_BRINGUP_ACTIVE_H

#undef M1_BRINGUP_MODE
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_SPEED_IDENT

#undef M1_USE_HFI_STANDSTILL_PROFILE
#define M1_USE_HFI_STANDSTILL_PROFILE   1
#undef M1_USE_IF_100_PROFILE
#define M1_USE_IF_100_PROFILE           0
#undef M1_USE_SPEED_1000_PROFILE
#define M1_USE_SPEED_1000_PROFILE       0
#undef M1_USE_FLUX_ID_PROFILE
#define M1_USE_FLUX_ID_PROFILE          0
#undef M1_USE_OBS_VEQ_PROFILE
#define M1_USE_OBS_VEQ_PROFILE          0

/*
 * HFI 关卡。只改这一行回退。
 *   1 = S1 … 8 = S3c0b（冻结）
 *   9 = C1  捕获瞬态
 *   10= C2  坏种子 enc+90°
 *   11= C3  无感质量放行标志（冻结；1536 FAIL_FLAG_DROP）
 *   12= C3b 质量旗清零滞回（冻结；1545 PASS）
 *   13= C4v1 旧误接 FEED（冻结；1552/1612 FAIL）
 *   14= C4  过线后极小 Iq（解调后馈；含 SLEW=2）
 *   15= C4a FEED+HI=0（1723 PASS）
 *   16= C4b FEED+HI=0.25（1730 FAIL）
 *   17= C4c HI=0 + FEED_A=0.25（冻结）
 *   18= C4d A/B 软出力对照（冻结）
 *   19= C4e 控制≡C4a；VOFA=x_raw/Ud_inj/di_d（冻结）
 *   20= C4f AFTER_LOCK 先HFI再踢（2132 FAIL；冻结）
 *   21= C4g 先踢再 HFI（2142：Iq 跟住 Δθ=0；冻结）
 *   22= C4h 踢前 HF settle（1224 FAIL；冻结）
 *   23= C4i MEAS 清注入（1234 FAIL；冻结）
 *   24= C4j S3b 踢 + hold→HFI RUN（冻结）
 *   25= C4k 同 C4j + 踢后 LOG/RUN 关 Id PI（冻结；A_cmd=0.10）
 *   26= C4l 同 C4k + 踢前 θ̂+=π（FORCE_PI；1402 FAIL；冻结）
 *   27= C4m 同 C4k + A_cmd=0.238（1411：对照 FAIL；冻结，勿当基线）
 *   28= C4n 同 C4k + PRE 假锁门禁（冻结）
 *   29= C4o 同 C4n + 关 BRAKE（踢完滑行；静置捕获已签收 PASS 1930/1931）
 *   30= C4p 同 C4o + FEED_A=0.5（HI=0；1945/46 轻出力 PASS，静摩擦停）
 *   31= C4q 同 C4p + FEED_A=0.8（探静摩擦）
 *   32= C4r 同 C4q 捕获 + 速度环阶梯 100…1000 rpm
 *   33= C4s 同 C4r + AUTH HI=11 A（对齐 SMO/SPEED 电流顶）
 *   34= C4t 同 C4s + 抬 PLL_W_MAX + ω* 小权重前馈（冻结；指令前馈）
 *   35= C4u 同 C4t 但关 omega* 前馈与 SEED；仅保留 W_MAX=1100
 *   36= C4v 同 C4u + 巡航 100然后200 / 各10s
 *   37= C4w 控制≡C4v；VOFA=巡航有用通道
 *   38= C4x P1：关速度环 + FEED_A=0.8×10s（验带流锁相）
 *   39= C4y 同 C4x；FEED_A=1.8（加载压转速对照）
 *   40= C4z 控制同 C4y；两段冻角 + 解调通道（FEED=1.8）
 *   41= C4aa 控制同 C4x；两段冻角 + 解调通道（FEED=0.8 空载；冻结）
 *   42= C4ab 控制≡C4x；VOFA=踢/FEED 分段诊断（Iq*、ω差、y；不冻角）
 *   43= C4ac 控制≡C4x；RUN 6–8 s 掐 FEED 滑行 2 s（同速对照）
 *   44= S2b 同 S2：有感 100 rpm，Park=enc；RUN 40 s（中途加负载看 Iq）
 *   45= S2c 同 S2：有感阶梯 0→1000 rpm / 每档 +100、8 s；HFI 只估角
 *   46= S2d 速度环吃编码器，Park=θ̂；100 rpm 然后 200 rpm，各 20 s
 *   47= S2e 同 S2d；阶梯 100→1000 rpm / 每档 +100、8 s；ch8=积分转速
 *   48= S2f 同 S2e；x≤A 不写积分（1838 丢步；冻结）
 *   49= S2g 同 S2e 阶梯；PLL 同 S2d（每拍 Ki+Kp）
 *   50= S2h tacc 五段（1917 失锁；冻结）
 *   51= S2i；+50 表/3.5s；VH 三档 RO 换档写
 *   52= S2j；Iq=0 锁住后再固定 0.8 A，速度环关
 *   53= 第 1 轮：同 38 交接，VH0 后停（Vh=0、Id 钉 WEAK，不开 IDUP）
 *   54= 相对 53：VH_FLOOR=0.40 + HOLD 后停（对齐 1646 残 Vh，不灭注入）
 *   55= 1646 复现：残 Vh scale=0.25、不开 Id（对照）
 *   56/57= END=0（Iq 噪 FAIL）
 *   58= 残 Vh 下开 Id（丢角 FAIL）
 *   59= 相对 57：END=0.05 微地板（≈0.02 V），不开 Id【灭 Vh 基线 / 1238/1401】
 *   60–64= 开 Id / iq·ω 守卫对照（均未稳；开 Id 污染 SMO）
 *   65= 相对 59：ω* 1800→800（仅减速巡航，无反向交接）
 *   66= 相对 59：SMO→HFI 反向；爬 1500 后减速，过 1400 回 HFI【节时】
 *   67= 相对 66：RVH 放 hold 救 θ̂ + RQUAL 过门再交角（1454 FAIL 根因）
 *   68= 相对 67：RVH/RQUAL 满注入唤醒（Vh=1）+ RQUAL 去 dw 门（1506 误杀）
 *   69= 相对 68：RVH 钉 θ̂=SMO 再 RQUAL 放 PLL；去 x 上界（1515）
 *   70= 观察枪：减速~900，Vh→0.4，HFI 旁路不交权【第一步】
 *   71= 相对 70：Vh 按前向 VH0+FADE 反演抬到 0.4（仍观察）
 *       VOFA ch3=enc_raw（轴参照；SMO ω 失步后不可信）
 *   72= 相对 71：关 REV，只 SMO 1500→900【加速基线冻结；2147/2238】
 *       加速 PASS：HFI→交接→爬稳 1500；减速已知 ~1050 挂
 *   73= 相对 72：SMO 减速 |Iq| 地板（旧触发；2228 1500 丢，废）
 *   74= 相对 72：|Iq|同号地板（2251：开窗把+0.5抬到+1.5→加速失步，废）
 *   75= 相对 72：制动向地板 iq*≤−1.5（ω>0）；未签收，勿当基线
 *   76= 相对 72：D1 只放慢降速斜坡 50→20 rpm/s（加速斜坡仍 50）；RPM2_S=55
 *   77= 相对 72：SMO 段 VH_END=0（真灭注入；1038 Iq 噪 FAIL）
 *   78= 相对 72：S1 只观察 VESC 窗 want_smo（≥1050 置位 / ≤950 清零）；不改 Park
 *       VOFA ch11=want_smo(0/1)；控制≡72
 *   79= S2 硬关 VH（FAIL）；80= S2b 软交接（中途 stall）
 *   81= Rel C4x FEED：锁/qual 后软开 Id→0，VH=1，无 SMO
 *   82= Rel 81：进 RUN 即 Id*=0 常开（无 soft）；解调基线
 *   83= Rel 82：FEED 起来后钉 Iq*（auth 掉旗不掐）
 *   84= Rel 83：控制同 83；VOFA 打边沿解调中间量（离线重放）
 *   85= Rel 84：Id PI 反馈改低通 Id（解调仍裸 Id）；NO_GAIN
 *   86= Rel 84：解调前高通 Id/Iq 再算 di；电流环仍裸 Id；NO_GAIN
 *   87= Rel 84：αβ 半周差分再 Park(θ̂)；HP 关；REGRESS
 *   88= Rel 84：αβ 注入进 SVPWM；极性/未进桥，不作基线
 *   89= Rel 84：Id PI 全程不旁路（含 LOG/踢）；Id*=0；COND，不作锁基线
 *   90= Rel 84：αβ 三端点中点差分再 Park(中点 θ̂)；NO_GAIN，不作锁基线
 *   91= Rel 72：SMO 用 u−u_hfi，HFI 段不停观测；加速≡72
 *   92= Rel 91：Park 切满 SMO 时旋 Id/Iq PI
 *   93= Rel 92：Id PI 不旁路、Id*=0（对照文档§7 步1）
 *   94= V1 Rel GATE1：环后 θ̂ 注入 + 注入轴 αβ 解调
 *   95= V2 Rel GATE2：有感 100，Id PI 开，Park=enc
 *   96= V3 Rel GATE3：静置 Park=θ̂，Iq*=0（1813 补：不开速环）
 *   97= V4 Rel GATE4：q 踢 1.6 A×300 ms，无对称刹车
 *   98= V5 Rel C4p：踢完滑行再 FEED 0.5 A，Id PI 开（未签）
 *   99= VESC 尺 Rel 94：静态 δ，残差不限幅，PLL 不写 θ̂（1929 PASS）
 *   100= VESC 旁路 Rel 95：Park=enc 100 rpm（1938 FAIL 180°）
 *   101= Rel 100：先 q 踢判极性再有感 100 rpm（1949 PASS）
 *   102= Rel 96：踢完极性后 Park=θ̂、Iq*=0 静置（1959 PASS）
 *   103= Rel 98/C4p：102 时间线 + 过线 FEED 0.5 A（2007 FAIL_FEED_CHOP）
 *   104= Rel 103：V4 |e| 限幅 10→0.30（2018 PASS）
 *   105= Rel C4r：104 + 速度环 100 rpm（2026 FAIL_SPEED_SLAM）
 *   106= Rel C4q：104 时间线 + FEED 0.8 A 阶跃（2031 FAIL_CHOP）
 *   107= Rel C4d：106 + FEED_RAMP 0.8 s（2045 FAIL_CHOP）
 *   108= Rel 83：107 + FEED_HOLD（2056 电流连续、θ 进 −90° 反转）
 *   109= 控制≡108；仅 VOFA：破粘鉴相（e / eps / y / pll_int）
 *   110= Rel 108：x 进 Lq 井且 y≈0 时 θ̂ 一次 +π/2 后冻结（2138 LOG→RUN 误翻）
 *   111= Rel 110：仅 FEED 指令≥0.5 A 才许翻
 *   112= Rel 111：翻轴不清 pll_int/ω
 *   113= Rel 108：PLL Kp 39.2→1200，Ki/MAX_ERR/Wmax 不动，不翻轴
 *   114= Rel 108：Kp 回到 39.2。限幅只夹积分；残差贴满 ±0.30 时不积分
 *   115= Rel 114：Kp 仍 39.2。PLL 吃 ½atan2，A_cmd=0.216；y 残差只留遥测
 *   116= Rel 115：|ε|<0.5 才积分；限幅只夹积分；角度速率=积分+Kp·ε
 *   117= Rel 113：Kp=1200。e 为同一注入轴上、高通前的一对 Δiq
 *   118= Rel 117：I 每拍 += e·(Kp/400)，|I|≤|θ̂ 转速|，角度速率不夹 200
 *   119= Rel 118：积分直接等于 θ̂ 转速低通，不再按 e·(Kp/400) 爬
 *   120= Rel 119：速度环吃编码器，100→500 rpm 每档 5 s；Park 仍是 θ̂
 *   121= Rel 120：PLL 收到约 15 Hz。速度环吃 HFI 的 ω=I+Kp·e
 *   122= Rel 120：速度环吃 HFI 低通转速（ch6）。PLL 仍是 Kp=1200
 *   123= Rel 122：阶梯上限 500→1500 rpm，每档仍 100 rpm / 5 s
 *   124= Rel 123：速度观测 5 ms→20 ms。角度仍是 Kp=1200
 *   125= Rel 123：速度环 Kp 0.015→0.005。速度观测回到 5 ms
 *   126= Rel 125：阶梯上限 1500→2000 rpm
 *   127= Rel 125：SMO 旁路，吃 u−u_hfi。Park 仍是 HFI，注入一直开。阶梯 100→1500
 *   128= Rel 127：1300 rpm 起发布 SMO 角，1000 rpm 交回。注入一直开。
 *        300 rpm 以下补每转 12 次和 14 次的转矩纹波
 *   129= Rel 128：发布角跟速度指令比。1500 关注入，1400 再开。
 *        100→1500 后再降到 900 巡航 5 s。补偿关掉
 *   130= Rel 129：速度环吃发布角的变化率，不再吃 SMO 的转速状态
 *   131= Rel 129：100、200 各停 5 s，再 50 rpm/s 爬到 1500。
 *        减速仍是 100 rpm / 5 s，降到 0。注入只在 SMO 已发布时关
 *   132= Rel 131 的 HFI。250 rpm 以下速度反馈陷波 12/rev 与 14/rev。
 *        100→500 再降到 0，每档 5 s
 *   133= Rel 132。100 rpm 档补每转 12 次和 14 次的转矩，每路 ≤0.20 A。
 *        80–130 rpm 以外衰减掉
 *   134= Rel 132。实测转速 0..200 rpm 给速度反馈补 5 ms 相位。不跟指令
 *   135= Rel 132。实测 60..140 rpm 按角度超前每转 12 次和 14 次。
 *        200 rpm 回到陷波器
 *   136= Rel 135。解调前减去 150 ms 平均转速
 *   137= Rel 136。再超前每转 24 次。阶梯只留 100、200、100
 *   138= Rel 131。指令在 +1500 与 −1500 之间阶跃，各停 3 s，反复 5 次。
 *        发布和注入跟实测转速的绝对值
 *   139= Rel 125。100 rpm 停 15 s，再 200 rpm 停 10 s。
 *        实测转速 160 以下速度环 Kp=0.015，到 200 回到 0.005
 *   140= Rel 139。100 rpm 指令时 Kp=0.015，并按实测转速陷掉每转 24 次和 28 次。
 *        指令到 200 rpm 关陷波，Kp 回到 0.005
 *   141= Rel 138。速度观测到 ±1500 后再保持 0.5 s 才换向，反复 5 轮。
 *        发布和注入跟实测转速的绝对值【本轮】
 */
#undef M1_HFI_GATE
#define M1_HFI_GATE                     141

#endif /* CONFIG_BRINGUP_ACTIVE_H */
