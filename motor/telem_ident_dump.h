/**
 * @file telem_ident_dump.h
 * @brief Rs / Ld-Lq 辨识结果经 VOFA JustFloat 突发（头帧 + 格点 + 尾帧）。
 *
 * 需 M1_VOFA_IDENT_DUMP_ENABLE=1 且 (M1_RS_IDENT_ENABLE || M1_LD_LQ_IDENT_ENABLE)。
 */

#ifndef TELEM_IDENT_DUMP_H
#define TELEM_IDENT_DUMP_H

#include <stdint.h>

#include "motor_params_m1.h"

#ifdef __cplusplus
extern "C" {
#endif

/** ch0 魔数：辨识头帧（CSV 搜 I0≈−777777） */
#define M1_VOFA_IDENT_MAGIC_HDR     (-777777.0f)
/** ch0 魔数：辨识尾帧 */
#define M1_VOFA_IDENT_MAGIC_TAIL    (-666666.0f)
/** ch4 协议版本：2.0=500Hz+1kHz；3.0=+2kHz f2（M1_LD_LQ_IDENT_F2_ENABLE） */
#if M1_LD_LQ_IDENT_F2_ENABLE
#define M1_VOFA_IDENT_PROTO_VER     (3.0f)
#else
#define M1_VOFA_IDENT_PROTO_VER     (2.0f)
#endif

/*
 * proto 2.0 头帧：I0=magic, I1=grid_n, I2=Rs_ohm, I3=rs_ok, I4=proto,
 *       I5=n_ld_ok(coarse), I6=n_lq_ok(coarse), I7=n_ld_ok_fine,
 *       I8=n_lq_ok_fine, I9=ld_lq_ok(coarse), I10=ld_lq_ok_fine
 * 数据帧（每格点一帧）：I0=id_bias, I1=iq_bias,
 *       I2=Ld_coarse_uH, I3=Lq_coarse_uH, I4=Ld_fine_uH, I5=Lq_fine_uH,
 *       I6=ld_coarse_valid, I7=lq_coarse_valid, I8=ld_fine_valid, I9=lq_fine_valid
 * 尾帧：I0=magic, I1=grid_n, I2=checksum（有效 coarse+fine Ld/Lq_uH 之和）
 *
 * proto 3.0：头 I11=n_ld_ok_f2；数据 I10=Ld_f2_uH,I11=Lq_f2_uH；
 *       尾 I4=n_lq_ok_f2,I5=ld_lq_ok_f2；checksum 含 f2
 *
 * proto 1.0（旧）：头 I5=n_ld_ok,I6=n_lq_ok,I7=ld_lq_ok；
 *       数据 I2=Ld_uH,I3=Lq_uH,I4/I5=valid（仅 coarse，实为 500Hz）
 */

/** 辨识 DONE 后调用一次，快照结果并启动突发 */
void telem_ident_dump_arm(void);

/** 1=突发进行中 */
uint8_t telem_ident_dump_is_busy(void);

/**
 * @brief 若处于辨识突发，填充下一帧
 * @return 1=vals 有效；0=未突发或已发完
 */
int telem_ident_dump_next(float vals[], uint8_t ident_dump_ch);

#ifdef __cplusplus
}
#endif

#endif
