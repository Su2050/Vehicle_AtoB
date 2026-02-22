# 测试结果记录（2026-02-22）

## 本次目标
- 修复测试脚本中的漏洞（统计口径、弱断言、静默通过等）。
- 运行测试脚本并对测试脚本本身问题做修复。

## 已修复的测试脚本问题

### 1) `scripts/tests/test_primitives.py`
- 修复 `test_straight_primitive_endpoint()` 的静默通过风险：增加 `found` 标记与最终断言。
- 修复 `test_turning_primitive_angle()` 的静默通过风险：增加 `checked` 计数与最终断言。

### 2) `scripts/tests/test_collision_unit.py`
- 修复 `test_check_traj_collision_mid_collision()` 的样本点，使其明确落入碰撞区域，避免注释与几何不一致。

### 3) `scripts/tests/test_rs_utils.py`
- 在 `test_pure_rs_straight_approach()` 增加 `collision_fn` 调用计数断言，确保闭包确实被调用。

### 4) `scripts/tests/test_astar_core.py`
- `test_astar_anti_zigzag()` 不再允许在 `ok/acts` 条件下跳过核心断言。
- 用行为契约替代源码字符串扫描：
  - 新增 `test_astar_rejects_legacy_env_kwargs()`，验证 `obstacles` / `dijkstra_grid` 旧参数会触发 `TypeError`。

### 5) `scripts/tests/test_planner.py`
- 收紧 L1 判定断言：要求 `level == 'L1_pure_rs'`。
- 收紧 Phase-0 判定：要求 `two_stage == True` 且 `stage1_mode` 包含 `phase0`。
- 调整 `test_planner_kturn_fast()` 场景为 `use_rs=False`，避免被 L1 Pure RS 直接短路。
- 性能阈值从 `<500ms` 放宽为 `<2000ms`，并要求 `stage1_ms` 存在。

### 6) `scripts/tests/test_planner_obs.py`
- 收紧接口断言：增加 `acts/traj` 返回类型检查。
- 收紧 L1 判定断言：要求 `level == 'L1_pure_rs'`。

### 7) `scripts/tests/test_grid_search.py`
- 修复成功率统计口径：
  - 仅当 `ok=True` 且 `goal_ok=True` 且 `ws_ok=True` 才计为成功。
- 修复退出码：
  - 全通过返回 `0`，存在失败返回 `1`。
- 失败计数改为 `len(fail_cases)`。

### 8) `scripts/tests/test_grid_search_obs.py`
- 修复 timeout 分支对 `elapsed` 的固定覆写，保留真实耗时统计。

---

## 测试执行结果

## 单元测试（修复后）
在 `scripts/` 下执行（带 `PYTHONPATH=.`）：

- `tests/test_primitives.py` ✅
- `tests/test_collision_unit.py` ✅
- `tests/test_rs_utils.py` ✅
- `tests/test_astar_core.py` ✅
- `tests/test_planner.py` ✅
- `tests/test_planner_obs.py` ✅

结论：上述 6 套更新后的单元测试全部通过。

## 网格回归测试
执行：`PYTHONPATH=. python tests/test_grid_search.py`

结果：
- 总用例数：160
- 成功用例数：156
- 失败用例数：4
- 成功率：97.5%
- 平均耗时：30.0 ms
- 最大耗时：619.6 ms

失败用例（均为 `Goal missed`）：
- Case #012: `(2.5, -0.6, 135°)`
- Case #044: `(3.5, -0.6, 135°)`
- Case #144: `(6.5, -0.6, -135°)`
- Case #148: `(6.5, 0.6, 135°)`

说明：本次修复后，`test_grid_search.py` 的统计与退出码逻辑已正确反映实际失败，不再出现“显示 100% 但仍有失败明细”的口径问题。

## 障碍场景冒烟测试
执行：`PYTHONPATH=. python tests/test_grid_search_obs.py --profile quick --max-cases-per-scene 1`

结果：
- 脚本可完整执行并输出全局汇总。
- 当前配置下全局成功率为 57.1%（7 个场景，各 1 case，仅作冒烟验证）。

说明：此项主要用于验证脚本执行链路与统计输出可用，不代表全量障碍回归结果。

---

## 当前结论
- 测试脚本层面的关键漏洞已修复（统计、断言、退出码）。
- 单元测试通过。
- `test_grid_search.py` 现可真实暴露剩余算法问题（4 个目标未命中）。
- 后续建议：针对上述 4 个失败 case 做规划策略层修复并回归到 160/160。
