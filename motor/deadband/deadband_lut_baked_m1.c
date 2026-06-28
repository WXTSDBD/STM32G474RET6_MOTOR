/**
 * @file deadband_lut_baked_m1.c
 * @brief phase LUT from VOFA vofa+202606281745 (proto 3.4, geo=152, scale=1).
 */

#include "deadband_lut_baked_m1.h"

#include "deadband.h"
#include "motor_params_m1.h"

const float m1_deadband_lut_baked_amps[M1_DEADBAND_LUT_BAKED_LEN] = {
    0.056392f, 0.072504f, 0.094658f, 0.114798f, 0.132923f, 0.132923f, 0.145007f,
    0.161119f, 0.177231f, 0.195357f, 0.195357f, 0.195357f, 0.245707f, 0.245707f,
    0.255777f, 0.255777f, 0.300085f, 0.354463f, 0.414882f, 0.459190f, 0.509540f,
    0.557876f, 0.610239f, 0.652533f, 0.718995f, 0.763303f, 0.813653f, 0.855946f,
    0.950604f, 1.071444f, 1.170129f, 1.299025f,
};

const float m1_deadband_lut_baked_vals[M1_DEADBAND_LUT_BAKED_LEN] = {
    0.140669f, 0.176744f, 0.188549f, 0.238480f, 0.304234f, 0.304234f, 0.370249f,
    0.951123f, 1.025156f, 1.077249f, 1.125795f, 1.125795f, 1.216948f, 1.216948f,
    1.216948f, 1.252753f, 1.296915f, 1.333052f, 1.354739f, 1.377705f, 1.396220f,
    1.408011f, 1.416213f, 1.429650f, 1.429650f, 1.438427f, 1.443623f, 1.452880f,
    1.454543f, 1.458453f, 1.472096f, 1.472096f,
};

bool deadband_lut_baked_m1_apply(void)
{
    deadband_set_lut(m1_deadband_lut_baked_amps, m1_deadband_lut_baked_vals,
                     M1_DEADBAND_LUT_BAKED_LEN);
    deadband_set_lut_domain(0u);
    deadband_set_runtime_apply_ud(0u);
    deadband_set_lut_runtime_scale(1.0f);
#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
    deadband_set_cluster_luts(m1_deadband_lut_baked_amps, m1_deadband_lut_baked_vals,
                              m1_deadband_lut_baked_amps, m1_deadband_lut_baked_vals,
                              M1_DEADBAND_LUT_BAKED_LEN);
#endif
    deadband_set_mode(M1_DEADBAND_MODE_LUT);
    return true;
}
