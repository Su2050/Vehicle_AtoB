# 路径质量隐式失效修复与测试补全

**日期**: 2026-02-24  
**分支**: `feature/reeds-shepp-heuristic`

## 1. 问题现象

在 viz.py 中手动测试时发现：起点 `(4.18, 2.03, -119.2°)` + 障碍物时，规划器返回 `ok=True`，但可视化路径严重不合理——紫色 RS 轨迹从目标点附近笔直延伸到屏幕底部，绕行距离达原始直线距离的 17 倍。

状态栏显示：`Found: 161 acts | 0 expanded | 3666.3ms [RS Expand, 2-Stage]`

## 2. 根因分析

### 2.1 直接原因

规划器的三个代码路径缺少路径质量过滤：

| 代码路径 | 问题 | 修复前行为 |
|----------|------|-----------|
| `_make_rs_expand_fn_multi()` | RS 扩展回调只检碰撞，不检路径合理性 | 接受绕行 17 倍的 RS 路径 |
| Level-1.8 `plan_2d_fallback` 返回值 | 直接返回，不检质量 | 返回穿墙或超长骨架轨迹 |
| K-turn 前置处理 | 无挡位震荡约束 | 产生 92 次换挡的抽风轨迹 |

对比：Level-1 和 Level-1.5 已有 `_path_goes_behind_wall()` 和 `_path_is_unreasonable()` 两个过滤器，但 Level-1.8、Level-2 的 RS 扩展遗漏了。

### 2.2 失效分类

这是一类**隐式质量失效**——系统没有崩溃，不报错，不碰撞，目标到达，从所有既有断言的角度看是 PASS。但路径几何上荒谬，对真实叉车不可接受。

### 2.3 测试盲区

审查全部 20+ 测试文件后发现：所有测试的断言 oracle 只有 `(ok, collision_free, goal_reached)` 三元组。**没有任何测试检查路径的几何质量**（长度、绕行比、横向偏移、换挡次数）。

触发条件需要三者同时满足：
1. `|theta| > 90°`（大角度偏转）
2. 存在障碍物（阻断短 RS 路径）
3. K-turn 后落在 RS 扩展半径内（触发 RS 直连）

现有测试覆盖了这个输入空间（`test_grid_search_obs.py` 测了 180° + 障碍物），但断言太弱。

## 3. 修复内容

### 3.1 规划器修复 (`planner_obs_v2.py`)

**修改 1**：RS 扩展回调增加质量过滤

```python
# _make_rs_expand_fn_multi() 中，碰撞检测前增加：
if _path_goes_behind_wall(traj):
    continue
if _path_is_unreasonable(traj, cx, cy):
    continue
```

**修改 2**：Level-1.8 返回值增加质量过滤

```python
if fb2d_ok and not _path_goes_behind_wall(fb2d_traj) \
        and not _path_is_unreasonable(fb2d_traj, sx2, sy2):
```

**修改 3**：新增 `_count_gear_shifts()` 函数 + K-turn 换挡门控

```python
prefix_too_many_shifts = _count_gear_shifts(prefix_acts) > 8
```

当 K-turn 前置处理产生过多挡位切换时，跳过 L1.8 和 L2，直接走 L3 兜底。

**修改 4**：`_path_is_unreasonable()` 分母保护

将 `direct` 替换为 `max(direct, 2.0)`，防止起终点极近时绕行比分母爆炸导致误拒。

### 3.2 所有修改均在 `planner_obs_v2.py` 内完成，未修改其他文件。

## 4. 新增测试

### 4.1 文件

`scripts/tests/test_path_quality_v2.py` — 路径几何质量专项测试套件

### 4.2 质量度量维度（QM-1 ~ QM-6）

| 指标 | 含义 | 阈值 |
|------|------|------|
| QM-1 | 全局绕行比 `path_length / max(euclidean, 2.0)` | <= 5.0 |
| QM-2 | 最大横向偏移 `max(\|y\|)` | <= max(\|start_y\|, \|goal_y\|) + 3.0m |
| QM-3 | 最大后退距离 `max(x)` | <= start_x + 3.0m |
| QM-4 | 穿墙采样点数 (x<1.92 且 \|y\|>0.5) | <= 5 |
| QM-5 | RS 段独立绕行比 | <= 4.0 |
| QM-6 | 挡位切换次数 | <= 8 |

### 4.3 测试板块

| 板块 | 用例数 | 用途 |
|------|--------|------|
| Block A: 回归用例 | 3 | 精确复现截图 bug + 变体 |
| Block B: 参数扫描 | 36 | 6 角度 × 3 起点 × 2 障碍布局 |
| Block C: 级别基线 | 3 | 各规划级别的质量基准 |
| **合计** | **42** | |

### 4.4 运行方式

```bash
cd scripts/

# 正常模式 — 验证修复后路径质量
python tests/test_path_quality_v2.py

# 回归模式 — 禁用质量过滤，证明测试能抓到坏路径
python tests/test_path_quality_v2.py --regress
```

## 5. 验证结果

### 正常模式（修复后）

```
RESULT: 42/42 — ALL PASS
```

所有回归 case 和参数扫描 case 均通过。质量过滤器成功阻止了坏路径被返回。

### 回归模式（模拟修复前）

```
RESULT: 24/42 — SOME FAILED
```

18 个 case 暴露了质量问题。REG-01（截图原始 bug case）的具体度量：

```
FAIL | REG-01 Screenshot Bug | QM1(Detour 6.1>5.0), QM5(RSDetour 6.1>4.0)
```

绕行比 6.1 倍、RS 段绕行比 6.1 倍——两个指标同时触发断言失败。

## 6. 经验教训

1. **路径质量是正确性的一部分**，不只是优化指标。一条绕行 17 倍的路径即使碰撞自由也不可接受。
2. **同一约束在不同代码路径上必须一致执行**。L1/L1.5 有质量过滤器，L1.8/L2 的 RS 扩展遗漏了，属于经典的约束执行不一致。
3. **测试的断言强度决定了它能发现什么**。覆盖了输入空间但断言太弱（只检 ok/collision/goal），等于没测。
