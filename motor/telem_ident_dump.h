/**
 * @file telem_ident_dump.h
 * @date 2026-10-06
 * @brief 辨识结果经串口突发发送。

 *
 * next 由遥测任务调用。arm 在辨识结束后调用一次。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
/** ch4 协议版本：4.0=Rs + l_hfi 标量（无 VASI 网格） */
#define M1_VOFA_IDENT_PROTO_VER     (4.0f)

/*
 * proto 4.0 头帧：I0=magic, I1=0, I2=Rs_ohm, I3=rs_ok, I4=proto,
 *       I5=Ld_uH, I6=Lq_uH, I7=Lavg_uH, I8=l_ok
 * 无数据帧；尾帧：I0=magic, I1=0, I2=checksum(Ld+Lq+Lavg_uH), I3=rs_used
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
