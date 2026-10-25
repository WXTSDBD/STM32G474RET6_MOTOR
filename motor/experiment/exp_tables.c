/**
 * @file exp_tables.c
 * @date 2026-10-25
 * @brief 实验段表：把原来散在宏里的实验参数收成 const 数据。
 *
 * 设计约定（与 docs/方案_宏归属表_2026-10-07.md 一致）：
 *  - 本文件 **不引用任何 M1_*_ENABLE / M1_*_PARAM 实验宏**，数值全部写死在表里；
 *  - 每张表 = 一段序列；`dur_ms=0` 表示时长由 SCRIPT 决定；
 *  - `mark` 用于遥测打点，与 PC 端判据对齐；
 *  - `cfg_idx` 留给"同一次上电 A/B"（连续两段只差本字段）；
 *  - `script=NULL` 的表只有数据，尚未接执行逻辑（标 UNVERIFIED，等台架）。
 *
 * 参数出处（2026-10-25 基线，±2000 金样）：
 *   斜坡 500 rpm/s、每段保持 500 ms、目标 ±2000 rpm、
 *   Ud 标定 id=3.0 A、每步 800 ms、6 角度 60° 步进、正反各一遍。
 */
#include "exp_tables.h"

#include <stddef.h>

/* 段表字段顺序：{ dur_ms, act, ops, p0, p1, p2, cfg_idx, mark, guard_mask } */
#define SEG(dur, act, a, b, c, mark) \
    { (uint16_t)(dur), (uint8_t)(act), NULL, (a), (b), (c), 0u, (uint8_t)(mark), 0u }

/* 空配置行（实验不改运行时 cfg 时用） */
static const exp_cfg_t s_cfg_default[] = {
    { 0u, 0u, 0u, 0u, 0.0f, 0.0f },
};

/* ============================ 1. 正反转 ±2000 ============================
 * 2 轮无感（+2000/-2000 各两次）+ 1 轮有感（+2000/-2000）。
 * 每半程：斜坡 2000/500=4000 ms + 保持 500 ms ⇒ 4500 ms。
 * 有感/无感切换由执行器按 cfg 选（原 M1_HFI_REV141_PAIRS / SENSED_PAIRS）。
 */
static const exp_seg_t s_rev141_segs[] = {
    SEG(4500u, EXP_ACT_OMEGA_REF, +2000.0f, 0.0f, 0.0f, 0x11),
    SEG(4500u, EXP_ACT_OMEGA_REF, -2000.0f, 0.0f, 0.0f, 0x12),
    SEG(4500u, EXP_ACT_OMEGA_REF, +2000.0f, 0.0f, 0.0f, 0x13),
    SEG(4500u, EXP_ACT_OMEGA_REF, -2000.0f, 0.0f, 0.0f, 0x14),
    SEG(4500u, EXP_ACT_OMEGA_REF, +2000.0f, 0.0f, 0.0f, 0x21),
    SEG(4500u, EXP_ACT_OMEGA_REF, -2000.0f, 0.0f, 0.0f, 0x22),
    SEG(4500u, EXP_ACT_OMEGA_REF, +2000.0f, 0.0f, 0.0f, 0x23),
    SEG(4500u, EXP_ACT_OMEGA_REF, -2000.0f, 0.0f, 0.0f, 0x24),
    SEG(4500u, EXP_ACT_OMEGA_REF, +2000.0f, 0.0f, 0.0f, 0x31),
    SEG(4500u, EXP_ACT_OMEGA_REF, -2000.0f, 0.0f, 0.0f, 0x32),
    SEG(4500u, EXP_ACT_OMEGA_REF, +2000.0f, 0.0f, 0.0f, 0x33),
    SEG(4500u, EXP_ACT_OMEGA_REF, -2000.0f, 0.0f, 0.0f, 0x34),
};

/* ============================ 2. 编码器零偏 Ud 锁轴 ============================
 * 12 步：0/60/120/180/240/300° 正走后反向。p0=电角度(deg)，p1=Id 电流(A)。
 * 每步 800 ms；开头留 8000 ms 让 HFI 上电流程走完。
 */
static const exp_seg_t s_ud_cal_segs[] = {
    SEG(8000u, EXP_ACT_HOLD,   0.0f,   0.0f, 0.0f, 0x01), /* 等上电流程 */
    SEG(800u,  EXP_ACT_THETA_REF,   0.0f, 3.0f, 0.0f, 0x10),
    SEG(800u,  EXP_ACT_THETA_REF,  60.0f, 3.0f, 0.0f, 0x11),
    SEG(800u,  EXP_ACT_THETA_REF, 120.0f, 3.0f, 0.0f, 0x12),
    SEG(800u,  EXP_ACT_THETA_REF, 180.0f, 3.0f, 0.0f, 0x13),
    SEG(800u,  EXP_ACT_THETA_REF, 240.0f, 3.0f, 0.0f, 0x14),
    SEG(800u,  EXP_ACT_THETA_REF, 300.0f, 3.0f, 0.0f, 0x15),
    SEG(800u,  EXP_ACT_THETA_REF, 300.0f, 3.0f, 0.0f, 0x25),
    SEG(800u,  EXP_ACT_THETA_REF, 240.0f, 3.0f, 0.0f, 0x24),
    SEG(800u,  EXP_ACT_THETA_REF, 180.0f, 3.0f, 0.0f, 0x23),
    SEG(800u,  EXP_ACT_THETA_REF, 120.0f, 3.0f, 0.0f, 0x22),
    SEG(800u,  EXP_ACT_THETA_REF,  60.0f, 3.0f, 0.0f, 0x21),
    SEG(800u,  EXP_ACT_THETA_REF,   0.0f, 3.0f, 0.0f, 0x20),
};

/* ============================ 3. 速度阶梯 ============================ */
static const exp_seg_t s_speed_ladder_segs[] = {
    SEG(4000u, EXP_ACT_OMEGA_REF,  300.0f, 0.0f, 0.0f, 0x10),
    SEG(4000u, EXP_ACT_OMEGA_REF,  600.0f, 0.0f, 0.0f, 0x11),
    SEG(4000u, EXP_ACT_OMEGA_REF, 1000.0f, 0.0f, 0.0f, 0x12),
    SEG(4000u, EXP_ACT_OMEGA_REF, 1500.0f, 0.0f, 0.0f, 0x13),
    SEG(4000u, EXP_ACT_OMEGA_REF, 2000.0f, 0.0f, 0.0f, 0x14),
};

/* ============================ 4. 位置阶跃 / 5. MIT 往返 ============================ */
static const exp_seg_t s_pos_step_segs[] = {
    SEG(1500u, EXP_ACT_HOLD,  0.0f, 0.0f, 0.0f, 0x10),
    SEG(1500u, EXP_ACT_THETA_REF, 1.5707963f, 0.0f, 0.0f, 0x11), /* +90° */
    SEG(1500u, EXP_ACT_THETA_REF, 0.0f, 0.0f, 0.0f, 0x12),
};
static const exp_seg_t s_mit_rev_segs[] = {
    SEG(2000u, EXP_ACT_OMEGA_REF,  +500.0f, 0.0f, 0.0f, 0x10),
    SEG(2000u, EXP_ACT_OMEGA_REF,  -500.0f, 0.0f, 0.0f, 0x11),
    SEG(2000u, EXP_ACT_OMEGA_REF,  +500.0f, 0.0f, 0.0f, 0x12),
};

/* ============================ 6. 速度反转纯测试 / 7. uq 扫描 ============================ */
static const exp_seg_t s_speed_rev_segs[] = {
    SEG(5000u, EXP_ACT_OMEGA_REF, +2000.0f, 0.0f, 0.0f, 0x10),
    SEG(5000u, EXP_ACT_OMEGA_REF, -2000.0f, 0.0f, 0.0f, 0x11),
};
static const exp_seg_t s_uq_sweep_segs[] = {
    SEG(5000u, EXP_ACT_IQ_FF, 2.0f, 0.0f, 0.0f, 0x10),
    SEG(5000u, EXP_ACT_IQ_FF, 2.5f, 0.0f, 0.0f, 0x11),
    SEG(5000u, EXP_ACT_IQ_FF, 3.0f, 0.0f, 0.0f, 0x12),
};

/* ============================ 8. IPD 扫频 ============================ */
static const exp_seg_t s_ipd_segs[] = {
    SEG(500u, EXP_ACT_HOLD, 0.0f, 0.0f, 0.0f, 0x10),
    SEG(500u, EXP_ACT_HOLD, 0.0f, 0.0f, 0.0f, 0x11),
};

/* ============================ 9. 参数辨识（Rs/Ld/Lq/ψf） ============================
 * 每个辨识项一段；p0=激励幅值(V/A)，p1=频率(Hz) 或 目标转速(rpm)。
 * ops 尚未接（script=NULL）⇒ 只有数据，执行逻辑后续补。
 */
static const exp_seg_t s_ident_segs[] = {
    SEG(3000u, EXP_ACT_IQ_FF, 1.0f, 0.0f, 0.0f, 0x10),  /* Rs: 直流注入 */
    SEG(3000u, EXP_ACT_IQ_FF, 1.0f, 500.0f, 0.0f, 0x11),/* Ld/Lq: 高频注入 */
    SEG(4000u, EXP_ACT_OMEGA_REF, 1000.0f, 0.0f, 0.0f, 0x12), /* ψf: 1000rpm 反电势 */
};

/* ============================ 10. Bode / 11. VASI / 12. I/F / 13. 死区 / 14. QKICK ===== */
static const exp_seg_t s_bode_segs[] = {
    SEG(1000u, EXP_ACT_HOLD, 0.0f, 0.0f, 0.0f, 0x10),
    SEG(60000u, EXP_ACT_IQ_FF, 0.5f, 10.0f, 1000.0f, 0x11), /* 10..1000 Hz 扫 */
};
static const exp_seg_t s_vasi_segs[] = {
    SEG(1000u, EXP_ACT_HOLD, 0.0f, 0.0f, 0.0f, 0x10),
    SEG(20000u, EXP_ACT_IQ_FF, 1.0f, 200.0f, 0.0f, 0x11),
};
static const exp_seg_t s_if_segs[] = {
    SEG(2000u, EXP_ACT_OMEGA_REF, 100.0f, 0.0f, 0.0f, 0x10),
    SEG(4000u, EXP_ACT_OMEGA_REF, 300.0f, 0.0f, 0.0f, 0x11),
};
static const exp_seg_t s_deadband_segs[] = {
    SEG(3000u, EXP_ACT_IQ_FF, 0.5f, 0.0f, 0.0f, 0x10),
    SEG(3000u, EXP_ACT_IQ_FF, 1.0f, 0.0f, 0.0f, 0x11),
    SEG(3000u, EXP_ACT_IQ_FF, 2.0f, 0.0f, 0.0f, 0x12),
};
static const exp_seg_t s_qkick_segs[] = {
    SEG(2000u, EXP_ACT_HOLD, 0.0f, 0.0f, 0.0f, 0x10), /* PRE */
    SEG(600u,  EXP_ACT_IQ_FF, 1.6f, 0.0f, 0.0f, 0x11),/* KICK */
    SEG(500u,  EXP_ACT_HOLD, 0.0f, 0.0f, 0.0f, 0x12), /* POL_DECIDE */
};

/* ============================ 注册项 ============================ */
#define DEF(id_, name_, kind_, segs_) \
    { (id_), (name_), (kind_), (segs_), \
      (uint16_t)(sizeof(segs_) / sizeof((segs_)[0])), s_cfg_default, 1u, 0u, NULL }

const exp_def_t g_exp_rev141     = DEF(EXP_ID_REV141,      "rev141_2000rpm",  EXP_KIND_SCHEDULE, s_rev141_segs);
const exp_def_t g_exp_ud_cal     = DEF(EXP_ID_UD_CAL,      "enc_offset_ud",   EXP_KIND_SCHEDULE, s_ud_cal_segs);
const exp_def_t g_exp_speed_ladder = DEF(EXP_ID_SPEED_LADDER, "speed_ladder", EXP_KIND_SCHEDULE, s_speed_ladder_segs);
const exp_def_t g_exp_pos_step   = DEF(EXP_ID_POS_STEP,    "pos_step",        EXP_KIND_SCHEDULE, s_pos_step_segs);
const exp_def_t g_exp_mit_rev    = DEF(EXP_ID_MIT_REV,     "mit_rev",         EXP_KIND_SCHEDULE, s_mit_rev_segs);
const exp_def_t g_exp_speed_rev  = DEF(EXP_ID_SPEED_REV,   "speed_rev",       EXP_KIND_SCHEDULE, s_speed_rev_segs);
const exp_def_t g_exp_uq_sweep   = DEF(EXP_ID_UQ_SWEEP,    "open_uq_sweep",   EXP_KIND_SCHEDULE, s_uq_sweep_segs);
const exp_def_t g_exp_ipd_sweep  = DEF(EXP_ID_IPD_SWEEP,   "ipd_sweep",       EXP_KIND_SCHEDULE, s_ipd_segs);
const exp_def_t g_exp_ident      = DEF(EXP_ID_IDENT,       "param_ident",     EXP_KIND_SCRIPT,   s_ident_segs);
const exp_def_t g_exp_bode       = DEF(EXP_ID_BODE,        "current_bode",    EXP_KIND_SCRIPT,   s_bode_segs);
const exp_def_t g_exp_vasi       = DEF(EXP_ID_VASI,        "vasi_ldlq",       EXP_KIND_SCRIPT,   s_vasi_segs);
const exp_def_t g_exp_if_drag    = DEF(EXP_ID_IF_DRAG,     "if_drag",         EXP_KIND_SCRIPT,   s_if_segs);
const exp_def_t g_exp_deadband_lut = DEF(EXP_ID_DEADBAND_LUT, "deadband_lut", EXP_KIND_CALIB,    s_deadband_segs);
const exp_def_t g_exp_qkick      = DEF(EXP_ID_QKICK,       "qkick_polarity",  EXP_KIND_CALIB,    s_qkick_segs);
