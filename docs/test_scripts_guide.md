········、 # 核心测试脚本说明文档

> 本文档介绍 Vehicle_AtoB 项目中的 4 个核心测试脚本，涵盖压力测试、障碍物场景测试、路径质量验收测试和路径几何质量专项测试。

---

## 目录

1. [一键运行：run_all_tests.sh](#1-一键运行run_all_testssh)
2. [test_stress.py — 大规模并行压力测试](#2-test_stresspy--大规模并行压力测试)
3. [test_obs_v2.py — 障碍物场景综合测试](#3-test_obs_v2py--障碍物场景综合测试)
4. [test_path_quality.py — 路径质量验收测试](#4-test_path_qualitypy--路径质量验收测试)
5. [test_path_quality_v2.py — 路径几何质量专项测试](#5-test_path_quality_v2py--路径几何质量专项测试)

---

## 1. 一键运行：run_all_tests.sh

位于 `scripts/run_all_tests.sh`，依次执行全部 4 个测试套件。

```bash
cd scripts/
bash run_all_tests.sh
```

执行顺序：

| 序号 | 脚本 | 说明 |
|------|------|------|
| 1/4 | `test_stress.py --profile thorough --workers 8` | 大规模压力测试（约 12000 用例） |
| 2/4 | `test_obs_v2.py` | 12 个障碍物场景 |
| 3/4 | `test_path_quality.py` | 路径质量验收（9 项 AC 指标） |
| 4/4 | `test_path_quality_v2.py` | 路径几何质量（6 个 QM 度量） |

> 脚本使用 `set -e`，任一测试失败即停止后续测试。

---

## 2. test_stress.py — 大规模并行压力测试

### 2.1 概述

利用 `multiprocessing.Pool` 满载 CPU 并行执行数千～数万随机测试用例，对规划器进行端到端验证：**规划 → 轨迹重建 → 逐点碰撞检测**。目标是暴露"假阳性碰撞"（planner 声称成功但轨迹实际穿越障碍物）。

### 2.2 运行方式

```bash
cd scripts/

# 快速烟雾测试（~300 用例）
python3 tests/test_stress.py --profile quick --workers 4

# 标准测试（~3000 用例）
python3 tests/test_stress.py --profile standard --workers 8

# 全量测试（~12000 用例，耗时较长）
python3 tests/test_stress.py --profile thorough --workers 8
```

### 2.3 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--profile` | `quick/standard/thorough` | `quick` | 测试强度档位 |
| `--workers` | `int` | CPU 核心数 | 并发 Worker 数量 |
| `--seed` | `int` | `42` | 随机种子（保证可复现） |

### 2.4 测试用例类型

| 类型 | 说明 | quick | standard | thorough |
|------|------|-------|----------|----------|
| **RandomMulti** | 随机 1~4 个障碍物 + 随机起点 | 300 | 3000 | 12000 |
| **Passage** | 参数化窄通道（缝隙宽 0.8~2.5m） | 50 | 300 | 1000 |
| **GoalBlocked** | 目标被阻塞（预期失败） | 20 | 100 | 200 |

### 2.5 用例预过滤

- **起点合法性** (`_is_start_valid`)：起点不在障碍物/走廊内，使用 `no_corridor=False` 严格模式
- **目标阻塞判断** (`_is_goal_blocked`)：目标区域被完全覆盖时，标记为 `expect_fail`
- **坐标取整**：先 `round(x, 2)` / `round(th, 3)` 再做预过滤，确保与轨迹验证使用完全相同的坐标

### 2.6 验证逻辑

对于规划成功的用例：

1. **轨迹重建**：`simulate_path(x, y, th, acts, prims)` 重放 A* 动作序列 + 拼接 RS 轨迹
2. **逐点碰撞检测**：对完整轨迹每个点调用 `check_collision(no_corridor=False)`
3. **危险边距**：计算轨迹点与障碍物的最小距离，< 0.4m 标记为 `DANGEROUS_MARGIN`
4. **碰撞溯源**：区分碰撞发生在 `rs_traj`（RS 轨迹段）还是 `acts`（A* 动作段）

### 2.7 输出报告

```
============================================================
 🚨 STRESS TEST REPORT 🚨
============================================================
Total Cases Tested : 11966
Profile            : thorough
------------------------------------------------------------
✅ SUCCESS (Safe)   : 10245 (Contains 3 dangerous margins)
🎯 EXPECTED FAIL    : 1721 (Goal blocked, handled correctly)
⚠️ UNEXPECTED FAIL  : 0 (False Negatives)
⏱️ TIMEOUT          : 0
------------------------------------------------------------
✨ TRAJ_COLLISION   : 0 (No false positives found!)
============================================================
```

- **TRAJ_COLLISION = 0** 是核心目标（无假阳性碰撞）
- 碰撞用例自动导出到 `logs/stress_collisions_*.json`，可供 GUI 复现

---

## 3. test_obs_v2.py — 障碍物场景综合测试

### 3.1 概述

覆盖 **13 个典型障碍物场景**，包括开放场地、单障碍物、长墙、窄通道、U 型陷阱、不可达目标等，系统性验证规划器在各种障碍物配置下的正确性。

### 3.2 运行方式

```bash
cd scripts/

# 运行全部场景（默认 planner v1）
python3 tests/test_obs_v2.py

# 指定 planner v2
python3 tests/test_obs_v2.py --planner v2

# 同时测试 v1 和 v2
python3 tests/test_obs_v2.py --planner both

# 只运行某个场景
python3 tests/test_obs_v2.py --scenario S04_narrow_passage

# 调整单用例超时
python3 tests/test_obs_v2.py --timeout 60
```

### 3.3 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--planner` | `v1/v2/both` | `v1` | 测试的规划器版本 |
| `--scenario` | `str` | `None` | 只运行指定场景 |
| `--timeout` | `int` | `30` | 单用例超时秒数 |

### 3.4 场景列表

| 编号 | 场景名称 | 障碍物数 | 用例数 | 预期结果 | 说明 |
|------|----------|----------|--------|----------|------|
| S01 | Open Field | 0 | 4 | 成功 | 无障碍，纯 RS 可解 |
| S02 | Single Block | 1 | 4 | 成功 | 单障碍物挡直线路径，需绕行 |
| S03 | Long Wall | 1 | 4 | 成功 | 长墙 y∈[-3.5,1.5]，缺口在 y>1.5 |
| S04 | Narrow Passage | 2 | 3 | 成功 | 1.2m 窄缝（车辆半径 0.1m） |
| S05 | U-Trap | 3 | 3 | 成功 | U 型陷阱，开口朝 +x |
| S06 | Unreachable | 1 | 2 | 失败 | 全高墙阻断通路 |
| S07a-c | Random Seeds | ~5 | 2×3 | 成功 | 3 组随机障碍物（保留通道） |
| S08 | Start==Goal | 0 | 4 | 成功 | 起点在目标区内/附近 |
| S09 | Dead-end Reverse | 3 | 3 | 成功 | 死胡同，需倒车逃出 |
| S10 | Corridor Straight | 2 | 3 | 成功 | 平行货架走廊 |
| S11 | L-Barrier | 2 | 2 | 成功 | L 型障碍，需变向 |
| S12 | Goal Covered | 1 | 2 | 失败 | 目标被障碍物覆盖 |
| S13 | Force Fallback | 1 | 1 | 成功 | 低 expand limit 验证回退机制 |

### 3.5 验证维度

每个成功用例会额外检查：

- **RS 轨迹无碰撞** (`_validate_rs_traj`)：RS 段每个采样点通过碰撞检测
- **终点达标** (`_validate_endpoint`)：终点落在目标区 `(x≤2.30, |y|≤0.25, |θ|≤5°+0.05)`

### 3.6 输出

- 每个场景输出详细表格（位置、是否成功、是否匹配预期、用时、展开节点数、规划层级）
- 日志保存到 `logs/obs_v2_test_*.log`
- 最终汇总匹配率

---

## 4. test_path_quality.py — 路径质量验收测试

### 4.1 概述

路径质量全面验收测试，包含 **9 项验收标准 (AC-1 ~ AC-9)**，覆盖 RS 路径精度、规划成功率、规划时延、超限拒绝、边界正确性等维度。

### 4.2 运行方式

```bash
cd scripts/

# 运行全部板块
python3 tests/test_path_quality.py

# 仅运行 RS 精度板块
python3 tests/test_path_quality.py --section rs

# 仅运行 plan_path 集成测试
python3 tests/test_path_quality.py --section plan

# 仅运行 plan_path_robust 全场景
python3 tests/test_path_quality.py --section robust

# 详细模式（打印 RS_SAMPLE/RS_EXPAND 日志）
python3 tests/test_path_quality.py --verbose
```

### 4.3 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--section` | `rs/plan/robust/all` | `all` | 只运行指定板块 |
| `--verbose` / `-v` | flag | `False` | 打印详细日志 |

### 4.4 验收标准 (AC)

| AC 编号 | 名称 | 阈值 | 板块 |
|---------|------|------|------|
| AC-1 | `rs_sample_path` 终点位置误差 | ≤ 0.05m（8 用例全部通过） | 板块 1 |
| AC-2 | `rs_sample_path` 终点角度误差 | ≤ 3°（8 用例全部通过） | 板块 1 |
| AC-3 | `plan_path` 返回 True 时终点满足目标 | 100%（0 失败） | 板块 2 |
| AC-4 | `plan_path_robust` 标准场景成功率 | ≥ 8/8 | 板块 2/3 |
| AC-5 | 标准场景平均规划时间（\|y\|≤0.5m） | ≤ 500ms | 板块 2/3 |
| AC-6 | 大偏航角两阶段规划用时 | ≤ 3000ms 且成功 | 板块 3 |
| AC-7 | 超限场景（\|y\|>MAX_PLANNABLE_Y）立即拒绝 | ≤ 10ms | 板块 3 |
| AC-8 | 所有返回路径的点均在工作区内 | 100%（零碰撞） | 板块 2/3 |
| AC-9 | MAX_PLANNABLE_Y 边界正确性 | 刚好在内可规划，刚好超出拒绝 | 板块 3 |

### 4.5 测试板块

#### 板块 1：RS 精度测试 (AC-1 / AC-2)

8 组 `rs_sample_path` 用例，验证 RS 曲线采样终点的位置精度和角度精度。

#### 板块 2：plan_path 标准集成 (AC-3 / AC-4 / AC-5)

8 组 `plan_path` 用例，组合测试 `use_rs` 和 `no_corridor` 参数，验证终点到达、成功率和时延。

#### 板块 3：plan_path_robust 全场景 (AC-4 ~ AC-9)

- **3-a** 标准场景（\|y\|≤0.5m, \|θ\|≤40°）
- **3-b** 大偏航角两阶段（\|θ\|>40°）
- **3-b2** 中等 y 偏差两阶段（0.5m<\|y\|≤MAX_PLANNABLE_Y）
- **3-c** 超限场景（\|y\|>MAX_PLANNABLE_Y，期望立即拒绝）
- **3-d** MAX_PLANNABLE_Y 边界精确性测试 (AC-9)
- **3-e** 走廊约束 vs NoCorr 对比

### 4.6 输出

最终汇总每个 AC 的 PASS/FAIL 状态，返回退出码：`0` 全部通过，`1` 存在失败。

---

## 5. test_path_quality_v2.py — 路径几何质量专项测试

### 5.1 概述

专门针对"规划成功但路径极其不合理"的隐式失效问题。通过 **6 个质量度量维度 (QM-1 ~ QM-6)** 检测路径几何质量，防止绕行、穿墙、过度后退等问题。

### 5.2 运行方式

```bash
cd scripts/

# 正常模式
python3 tests/test_path_quality_v2.py

# 回归验证模式（禁用规划器质量过滤，证明测试能抓到坏路径）
python3 tests/test_path_quality_v2.py --regress
```

### 5.3 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--regress` | flag | `False` | 禁用规划器质量过滤，验证测试套件的检测能力 |

### 5.4 质量度量 (QM)

| QM 编号 | 名称 | 阈值 | 说明 |
|---------|------|------|------|
| QM-1 | 全局绕行比 | ≤ 5.0 | 路径总长 / 欧氏距离 |
| QM-2 | 最大横向偏移 | ≤ max(\|sy\|, \|gy\|) + 3.0m | 防止大幅横向偏离 |
| QM-3 | 最大后退距离 | ≤ start_x + 3.0m | 防止过度回退 |
| QM-4 | 穿墙采样点数 | ≤ 5 | 检测轨迹是否穿越墙壁后方 |
| QM-5 | RS 段独立绕行比 | ≤ 4.0 | RS 段长度 / RS 段欧氏距离 |
| QM-6 | 挡位切换上限 | ≤ 8 | 前进/后退挡位切换次数 |

### 5.5 测试板块

#### Block A: 回归用例 (Regression Cases)

3 个已知 Bug 复现用例，包含截图 Bug、极端角度、负偏移等场景。

| 用例 | 起点 | 角度 | 障碍物 |
|------|------|------|--------|
| REG-01 Screenshot Bug | (4.18, 2.03) | -119.2° | 中心方块 |
| REG-02 Extreme Angle | (5.0, 2.5) | -150.0° | 中心方块 |
| REG-03 Neg Offset | (3.5, -2.0) | 160.0° | 偏移方块 |

#### Block B: 参数扫描 (Parametric Sweep)

6 个大角度 × 3 个起点 × 2 组障碍物 = **36 个用例**，系统性扫描极端工况。

- 角度：[-180°, -135°, -90°, 90°, 135°, 180°]
- 起点：(4.0,-1.0), (5.0,0.0), (6.0,1.5)
- 障碍物：CenterBlock、NarrowPass

#### Block C: 规划层级基线 (Level Baselines)

验证不同规划层级的路径质量：

| 用例 | 预期层级 | 起点 | 障碍物 |
|------|----------|------|--------|
| L1 Pure RS | Level-1 | (4.0, 0.0, 0°) | 无 |
| L1.8 2D Skel | Level-1.8 | (4.5, 0.0, 0°) | 小方块 |
| L2 A* | Level-2 | (3.0, 1.0, 90°) | 窄墙 |

### 5.6 输出

```
================================================================================
 RESULT: 42/42 — ALL PASS
================================================================================
```

- 退出码 `0`：全部通过，`1`：存在失败
- `--regress` 模式下，预期部分用例会 FAIL（证明测试可检测坏路径）

---

## 附录：测试文件位置

```
Vehicle_AtoB/scripts/
├── run_all_tests.sh                    # 一键运行全部测试
└── tests/
    ├── test_stress.py                  # 大规模并行压力测试
    ├── test_obs_v2.py                  # 障碍物场景综合测试
    ├── test_path_quality.py            # 路径质量验收测试（AC-1~AC-9）
    └── test_path_quality_v2.py         # 路径几何质量专项测试（QM-1~QM-6）
```
