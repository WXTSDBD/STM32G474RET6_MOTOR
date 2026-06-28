import math
from pathlib import Path

N = 256
TWO_PI = 2.0 * math.pi
vals = [math.sin(i * TWO_PI / N) for i in range(N)]

out = Path(__file__).resolve().parents[1] / "bringup" / "math_tables" / "math_tables_sincos.c"
lines = [
    "/** @file math_tables_sincos.c - sin[256] on [0,2pi), linear lerp; cos via pi/2 offset. */",
    '#include "math_tables.h"',
    '#include <stddef.h>',
    '#include <stdint.h>',
    "",
    "#define MATH_TABLES_TWO_PI     6.28318530718f",
    "#define MATH_TABLES_LUT_SIZE   256u",
    "#define MATH_TABLES_LUT_SCALE  (256.0f / MATH_TABLES_TWO_PI)",
    "#define MATH_TABLES_PI2_OFFSET 64.0f",
    "",
    "static const float sin_lut[256] = {",
]
for i in range(0, N, 8):
    chunk = vals[i : i + 8]
    lines.append("    " + ", ".join(f"{v:.8f}f" for v in chunk) + ",")
lines += [
    "};",
    "",
    "static float math_tables_wrap_0_2pi(float rad)",
    "{",
    "    while (rad >= MATH_TABLES_TWO_PI) {",
    "        rad -= MATH_TABLES_TWO_PI;",
    "    }",
    "    while (rad < 0.0f) {",
    "        rad += MATH_TABLES_TWO_PI;",
    "    }",
    "    return rad;",
    "}",
    "",
    "static float math_tables_sin_lut_lerp(float u)",
    "{",
    "    uint32_t i = (uint32_t)u;",
    "    float frac = u - (float)i;",
    "    float y0;",
    "    float y1;",
    "",
    "    i &= (MATH_TABLES_LUT_SIZE - 1u);",
    "    y0 = sin_lut[i];",
    "    y1 = sin_lut[(i + 1u) & (MATH_TABLES_LUT_SIZE - 1u)];",
    "    return y0 + frac * (y1 - y0);",
    "}",
    "",
    "void math_tables_sincos_f32(float rad, float *cos_out, float *sin_out)",
    "{",
    "    float u;",
    "",
    "    if (cos_out == NULL || sin_out == NULL) {",
    "        return;",
    "    }",
    "",
    "    rad = math_tables_wrap_0_2pi(rad);",
    "    u = rad * MATH_TABLES_LUT_SCALE;",
    "",
    "    *sin_out = math_tables_sin_lut_lerp(u);",
    "",
    "    u += MATH_TABLES_PI2_OFFSET;",
    "    if (u >= (float)MATH_TABLES_LUT_SIZE) {",
    "        u -= (float)MATH_TABLES_LUT_SIZE;",
    "    }",
    "    *cos_out = math_tables_sin_lut_lerp(u);",
    "}",
    "",
]
out.write_text("\n".join(lines), encoding="utf-8")
print(out)
