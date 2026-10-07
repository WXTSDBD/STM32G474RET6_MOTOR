# 宏 `#undef` 分类表（motor_params_m1.h 内，2026-10-07）

**性质**：只读扫描；**不改固件**。profile 576 处不在本表。
再生：`python tools/classify_motor_params_undef.py`

## 1. 汇总

| 项 | 值 |
|---|---:|
| `#undef` 行 | 115 |
| 波及宏名 | 70 |
| 有外部读取的行 | 110 |
| 初判 `必留` | 85 |
| 初判 `重复强写` | 29 |
| 初判 `可删` | 1 |

## 2. 建议刀序

1. 只动初判 **`重复强写`** 且人工复核通过的（≤10/批）。
2. **`可删`** 须再确认无 `#if` 依赖后再动。
3. **`必留`** 默认不动。
4. 编完比四数+hex：相同→不烧；不同→烧录窗口停。

## 3. 重复强写候选（有读取点优先，Top）

| 行 | 宏 | 强制值 | 读文件 | hot | 理由 |
|---:|---|---|---:|---:|---|
| 1544 | `M1_VOFA_IDENT_DUMP_ENABLE` | `1` | 12 | 0 | 与先前定义同值（先前行 1289）→ 可改 #ifndef |
| 1445 | `M1_DEADBAND_I_ZERO_DISABLE` | `1` | 4 | 0 | 与先前定义同值（先前行 1309）→ 可改 #ifndef |
| 1551 | `M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE` | `1` | 4 | 1 | 与先前定义同值（先前行 1272）→ 可改 #ifndef |
| 1614 | `M1_IDENT_BODE_BANDS` | `2u` | 4 | 0 | 与先前定义同值（先前行 1381）→ 可改 #ifndef |
| 1412 | `M1_DEADBAND_LUT_APPLY_UD` | `0` | 3 | 0 | 与先前定义同值（先前行 1320）→ 可改 #ifndef |
| 1572 | `M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME` | `1` | 3 | 0 | 与先前定义同值（先前行 1327）→ 可改 #ifndef |
| 1607 | `M1_IDENT_STEP_FIXED_ROUNDS` | `0u` | 3 | 0 | 与先前定义同值（先前行 1373）→ 可改 #ifndef |
| 1620 | `M1_IDENT_BODE_FIXED_ROUNDS` | `0u` | 3 | 0 | 与先前定义同值（先前行 1385）→ 可改 #ifndef |
| 1352 | `M1_ID_CAL_AMP_TABLE_LEN` | `30u` | 2 | 0 | 与先前定义同值（先前行 1293）→ 可改 #ifndef |
| 1357 | `M1_ID_CAL_ID_DWELL_S` | `0.5f` | 2 | 0 | 与先前定义同值（先前行 1297）→ 可改 #ifndef |
| 1359 | `M1_ID_CAL_ID_DWELL_LOW_S` | `0.5f` | 2 | 0 | 与先前定义同值（先前行 1299）→ 可改 #ifndef |
| 1435 | `M1_ID_CAL_ID_DWELL_S` | `0.5f` | 2 | 0 | 与先前定义同值（先前行 1358）→ 可改 #ifndef |
| 1437 | `M1_ID_CAL_ID_DWELL_LOW_S` | `0.5f` | 2 | 0 | 与先前定义同值（先前行 1360）→ 可改 #ifndef |
| 1513 | `M1_ID_CAL_AMP_TABLE_LEN` | `30u` | 2 | 0 | 与先前定义同值（先前行 1353）→ 可改 #ifndef |
| 1517 | `M1_ID_CAL_ID_DWELL_S` | `0.5f` | 2 | 0 | 与先前定义同值（先前行 1436）→ 可改 #ifndef |
| 1519 | `M1_ID_CAL_ID_DWELL_LOW_S` | `0.5f` | 2 | 0 | 与先前定义同值（先前行 1438）→ 可改 #ifndef |
| 1534 | `M1_RS_IDENT_USE_FIXED_NOMINAL` | `1` | 2 | 0 | 与先前定义同值（先前行 1267）→ 可改 #ifndef |
| 1554 | `M1_LD_LQ_IDENT_FINE_GRID_ENABLE` | `1` | 2 | 0 | 与先前定义同值（先前行 1274）→ 可改 #ifndef |
| 1557 | `M1_LD_LQ_IDENT_F_COARSE_HZ` | `500.0f` | 2 | 0 | 与先前定义同值（先前行 1277）→ 可改 #ifndef |
| 1559 | `M1_LD_LQ_IDENT_F_FINE_HZ` | `1000.0f` | 2 | 0 | 与先前定义同值（先前行 1279）→ 可改 #ifndef |
| 1562 | `M1_LD_LQ_IDENT_INJECT_LUT_ENABLE` | `1` | 2 | 0 | 与先前定义同值（先前行 1281）→ 可改 #ifndef |
| 1630 | `M1_IDENT_FIX_THETA_ENABLE` | `1` | 2 | 1 | 与先前定义同值（先前行 1398）→ 可改 #ifndef |
| 1632 | `M1_IDENT_THETA_EL_RAD` | `M1_ID_CAL_THETA_EL_RAD` | 2 | 1 | 与先前定义同值（先前行 1400）→ 可改 #ifndef |
| 1450 | `M1_DEADBAND_GEO_MERGE_VAL30_ONLY` | `1` | 1 | 0 | 与先前定义同值（先前行 1312）→ 可改 #ifndef |
| 1507 | `M1_LD_LQ_PRE_DECAY_S` | `1.0f` | 1 | 0 | 与先前定义同值（先前行 1270）→ 可改 #ifndef |
| 1521 | `M1_ID_CAL_I_REF_ABS_MAX` | `3.5f` | 1 | 0 | 与先前定义同值（先前行 1301）→ 可改 #ifndef |
| 1538 | `M1_LD_LQ_PRE_DECAY_S` | `1.0f` | 1 | 0 | 与先前定义同值（先前行 1508）→ 可改 #ifndef |
| 1354 | `M1_ID_CAL_I_MAX_A` | `1.5f` | 0 | 0 | 与先前定义同值（先前行 1295）→ 可改 #ifndef |
| 1515 | `M1_ID_CAL_I_MAX_A` | `1.5f` | 0 | 0 | 与先前定义同值（先前行 1355）→ 可改 #ifndef |

共 29 行初判为重复强写（全量见 csv/json）。

## 4. 可删候选（须复核）

| 行 | 宏 | 强制值 | 理由 |
|---:|---|---|---|
| 3250 | `M1_IF_OBS_SPEED_IQ_SIGN` | `(1.0f)` | 无外部读取且几乎无后续守卫（须人工复核后再删） |

