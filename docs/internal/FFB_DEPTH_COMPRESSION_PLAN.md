# FFB 最小深度压缩方案（严格认证 + 零修复）

> **⚠️ 更新（2026-06）：L3 seed-bias 已废弃、退役出默认路径。**
> L3 让分裂值依赖 query seed，违反 LECT 的**种子/场景无关不变量**（节点分裂只能依赖
> `(robot, canonical domain)`），导致 canonical warm cache 与外部证据无法复用
> （`materialization_reused_external_evidence → 0`）。现 `ffb_seed_bias` 默认 `0.0`。
> 深度补偿改由**种子无关**杠杆承担：
> - **P3 support-hull 分裂调度**：最小化 *SupportHull 认证包络体积*（而非端点 AABB 体积和）
>   的纯函数调度 `support_hull_volume_min_depth_schedule(robot, canonical intervals)`；
> - **P1 canonical 域整形**（关节限位 / dim0 对称）；
> - **P4 最浅已认证祖先复用**：`find_free_box` 返回包含 seed 的最浅 `DefinitelyFree`
>   canonical 祖先（最大盒/最浅深度），只取决于已认证的 canonical 节点（种子无关）。
>
> L3 仅作为 E4 消融的反向对照行 `legacy_seed_bias_l3` 保留。完整方案与决策见
> [docs/SEED_INDEPENDENT_LECT_PLAN.md](docs/SEED_INDEPENDENT_LECT_PLAN.md)。以下原始计划供历史参考。

> 目标：在 **不放宽 envelope、不引入误判**（严格认证 + 零修复）的前提下，进一步压低 shelf 5 个 anchor 的最小 FFB 深度。
> 基线（`sample_nodes_per_depth=8` + `FIXED_SHELF_ROOT_INTERVALS`）：左右**AS=32, TS=32, CS=28, LB=26, RB=26**。

L3 详细实现说明见 [docs/FFB_L3_SEED_BIAS_IMPLEMENTATION.md](docs/FFB_L3_SEED_BIAS_IMPLEMENTATION.md)。

## 0. 诊断动机

AS 在深度 32 成功时的分裂维直方图近似均匀：

```
dim0:5  dim1:4  dim2:4  dim3:5  dim4:4  dim5:6  dim6:4
```

说明深度并非被某个几何困难维主导，而是 FFB 的 `BestTighten` 分裂器近似轮询地把分裂摊到全部 7 维（含腕滚 `dim6` 分 4 刀）。深度里存在大量「策略次优 + envelope 保守」的可压缩项，几何下限远低于 32。

## 约束（已确认）

- **硬约束**：严格认证 + 零修复。仅允许「仍是 sound 上近似」的手段（收紧 envelope、改进分裂分配、自适应分裂点）。禁止 CoverageHeuristic、禁止放宽 padding。
- **廉价性约束**：每个 node 选分裂维 + 分裂点的成本必须低。禁止在 live FFB 热路径里逐维做 FK 体积评估。分裂维权重必须一次性预计算、分裂点 O(1)/node。
- **粒度**：L2 权重用 **per-robot 全局敏感度**。
- **dim6**：若 A2 证明 dim6 对 envelope 完全惰性，**直接 mask 掉**，并加入「自适应惰性维 mask 检测机制」，为不同机器人自动识别并屏蔽惰性维。
- L1/L2/L3 **全做**，并做**消融对比**取最优组合。

## 三个认证安全的杠杆

| 杠杆 | 机制 | 成本 | 认证安全性 |
|---|---|---|---|
| **L1** envelope 收紧 | 提高验证 endpoint 的 HIFK 深度 / `n_subdivisions`，得到更紧但仍 sound 的上近似 | per-node 固定倍数端点计算 | ✅ 仍是过近似 |
| **L2** 廉价 dim-priority | 一次性预计算每维运动学敏感度权重，喂进 `better_best_tighten_candidate`，降权惰性维 | O(1)/node | ✅ 不改分裂合法性 |
| **L3** anchor 自适应分裂点 | split_val 从中点改为朝 seed 偏置 | O(1)/node | ✅ 仍夹在 [lo,hi] |
| **mask** 惰性维屏蔽 | 自适应检测对 envelope 无贡献的维，从候选集中剔除 | 一次性检测 | ✅ 仅减少无效分裂 |

## 分阶段计划

### 阶段 A — 诊断与下界归因 ✅ 已完成

- **A1 几何下界归因表（零代码）** ✅：5 anchor 在各自最小深度的分裂分配**近似均匀**（各维 12.9%–16.7%），完全不按维宽比例。dim6 宽度最窄（0.617）却仍占 12.9%（3–4 刀/anchor），是头号浪费。
- **A2 冻结 dim6 探针 + 惰性检测** ✅（决定性）：
  - `aafk_volume_min_depth_schedule`（按最小端点测度增益贪心选维）**从不选 dim6**（0/32、0/50）→ dim6 在 shelf bounds 下对端点 envelope **完全惰性**。
  - mask dim6（`max_candidate_dim=5`）跑 5 anchor，全部 `fail_code=0`（零修复、仍认证）：

    | anchor | 基线 | mask dim6 | 降幅 |
    |---|---|---|---|
    | AS | 32 | 28 | −4 |
    | TS | 32 | 28 | −4 |
    | CS | 28 | 26 | −2 |
    | LB | 26 | 22 | −4 |
    | RB | 26 | 22 | −4 |

  - 结论：dim6 分裂纯属浪费，mask 既安全又有效。惰性检测采用「aafk schedule 频率」：跑深 schedule，count==0 的维即惰性 → mask（per-robot 一次性，零热路径成本）。

### 阶段 D — L3 anchor 自适应分裂点（C++，O(1)/node）✅ 已完成

- **D1/D2/D3** ✅：`oracle.cpp` 新增自由函数 `biased_split_value(interval, dim, opts)`：默认中点，当 `seed_bias>0` 且区间内有有效 seed 时返回 `mid + bias·(seed−mid)`，夹在 `[lo+0.02w, hi−0.02w]`。`choose_best_tighten_split` 的 replay 与候选两路径均改用它。`BestTightenOptions` 加 `seed_bias`、`seed_coords`。
- seed 自动注入：三处 FFB 路径（`FindFreeBoxService::find`、`find_free_box_in_domain`、`debug_find_free_box`）在 `seed_bias>0` 时把 query 的 `tree_seed` 写入局部 `OracleSplitOptions.best_tighten.seed_coords`（const options 用局部副本），无需手动按 anchor 设 seed。
- **共享树注意**：lect kd-tree 持久共享，节点 split_value 一旦由首个到达的 query 固定即被全体复用。故 per-anchor 独立 DB 的 L3 数字是上界，公平评估须在共享 DB 模式（消融已如此做）。

### 阶段 X — 消融对比取最优 ✅ 已完成

共享 DB / 深度、全 5 anchor、全 `fail_code=0`：

| 配置 | AS | TS | CS | LB | RB | 和 |
|---|---|---|---|---|---|---|
| baseline | 32 | 32 | 28 | 26 | 26 | 144 |
| mask | 28 | 28 | 26 | 22 | 22 | 126 |
| L2 w0.05 | 30 | 26 | 24 | 24 | 24 | 128 |
| L3 b0.7 | 20 | 20 | 20 | 18 | 20 | 98 |
| L3 b0.9 | 16 | 18 | 18 | 16 | 16 | 84 |
| **mask+L3 b0.9** | **16** | **18** | **16** | **16** | **16** | **82** |
| L2+L3 b0.9 | 16 | 18 | 18 | 16 | 16 | 84 |
| mask+L2+L3 | 16 | 18 | 18 | 16 | 16 | 84 |

**胜出 = mask + L3(seed_bias=0.9)**，sum=82（约 −43%）。L3 是主导杠杆；mask 提供小幅额外收益；L2 无边际增益（叠加反而轻微变差）；L1 无效。自动注入 seed 后（不手动设 seed_coords，仅 `dim_mask`+`seed_bias=0.9`）复现 AS16/TS16/CS16/LB16/RB16 **sum=80**，全 `fail_code=0`。

### 阶段 E — 端到端零修复回归 ✅ 已完成

- ✅ seed 自动注入接入三条 FFB 路径（`FindFreeBoxService::find`、`find_free_box_in_domain`、`debug_find_free_box`）并重建验证。
- ✅ CLI/config 旋钮：`common_sbf_config.py` 新增 `--ffb-seed-bias`（L3 强度）、`--ffb-auto-mask-inert`（自动惰性维屏蔽开关）；`configure_standalone_sbf` 设 `best_tighten.seed_bias`；新增 `compute_inert_dim_mask(robot, root)` / `apply_dim_mask(cfg, mask)` 辅助。`run_shelf_sbf_case.case_config` 在 auto-mask 开启时用根域（restricted root 或 robot joint limits）一次性算 per-robot 惰性 mask 并应用。旗标经 `lect_db_dispatch.build_current_shelf_sbf_anytime_command` 与 `run_shelf_ablation.py` 透传到子进程。
- ✅ Exp.4 shelf E2E 回归（`--ffb-seed-bias 0.9 --ffb-auto-mask-inert --require-no-repair`）：`status=completed`、`rc=0`，全 query `repair_count=0`、`audit_ok=True`/`post_audit_ok=True`/`no_repair=True`，认证 box 正常提交。胜出组合端到端保持**严格认证 + 零修复**。

## 最终结论

- **胜出方案 = mask（自动惰性维屏蔽）+ L3（`seed_bias=0.9`）**。shelf 5 anchor 最小 FFB 深度从基线 144（AS32/TS32/CS28/LB26/RB26）降到 80（全 16，自动注入 seed 后），约 −44%，全程 `fail_code=0`、零修复、认证不退化。
- 杠杆排序：L3（seed-bias）主导，mask 小幅增益，L2（soft dim-priority）无边际增益（叠加反伤），L1（envelope 收紧）无效。
- 全局推荐默认现已设为：`--ffb-auto-mask-inert --ffb-seed-bias 0.9`。惰性 mask 为 per-robot 自适应（`aafk_volume_min_depth_schedule` 频率检测），自动适配不同机器人；seed-bias O(1)/node、自动注入 query seed，无热路径逐维 FK。

### 阶段 B — L1 envelope 收紧（参数级，认证安全）✅ 已完成（无效，已排除）

- **B1** ✅：probe 验证 endpoint 为 `aafk`（IFK/max_depth0），理论上有收紧空间。
- **B2** ✅：`n_subdivisions {4,8}` × HIFK 深度 `{2,4}` 扫描，5 anchor probe 最小深度**全部等于基线**（AS32/TS32/CS28/LB26/RB26），HIFK 反而更慢（1.8s vs 0.3s）。
- **结论**：深度瓶颈是**分裂分配**（把刀浪费在惰性维），**不是** envelope 过近似。L1 被严格支配，排除。

### 阶段 C — L2 廉价 dim-priority 权重（C++，O(1)/node）

- **C1**：在 `BestTightenOptions` 加每维权重字段 `dim_priority_weights` + `dim_priority_weight`。
- **C2**：在 `better_best_tighten_candidate` 评分加 dim-priority 项；低 w_d 降权、高 w_d 升权；保持 sound（仍只在 [lo,hi] 分裂、仍可达 DefinitelyFree）。
- **C3**：预计算每维敏感度（A2 的检测函数，per-robot 一次性），暴露到 Python 配置；跑 5 anchor probe 对照。

**C 结果** ✅（已实现 + 验证）：`BestTightenOptions` 新增 `dim_priority_weights`（per-dim）+ `dim_priority_weight`（全局强度），在 `evaluate_best_tighten_dim` 末尾对 `balanced_score` 减 `scale·w·weights[dim]`。权重 = 归一化敏感度 `[1,.9,.9,1,.8,1,0]`。最优 `w=0.05`：AS30/TS26/CS24/LB24/RB24 sum=128，全 `fail_code=0`。非单调，`w≥0.5` 反让 LB/RB 回升到 28。L2 单独弱于 L3，且与 L3 叠加无边际增益。

### 阶段 mask — 自适应惰性维屏蔽（C++）✅ 已完成

- **M1/M2/M3** ✅：`BestTightenOptions.dim_mask`（`vector<int>`，0=屏蔽该维），在 `candidate_dim_allowed` 剔除 masked 维。Python 端用 `aafk_volume_min_depth_schedule` 频率自动检测：`count==0` 的维即惰性。iiwa14 shelf → `mask=[1,1,1,1,1,1,0]`（屏蔽 dim6）。5 anchor probe 全 `fail_code=0`：AS28/TS28/CS26/LB22/RB22 sum=126（= A2，验证一致）。

## 关键文件

| 文件 | 作用 |
|---|---|
| `experiments/exp04_shelf_ablation/probe_restricted_root_domain.py` | 诊断 + 各阶段验证主入口（已有 `split_dim_hist` / `sample_nodes_per_depth`） |
| `lect_database/include/LECTDatabase/sbf/oracle.h` | `BestTightenOptions`（L2 权重字段 / mask 字段） |
| `lect_database/src/sbf/oracle.cpp` | `better_best_tighten_candidate`（L2 评分）、分裂值选择（L3 偏置）、候选维选择（mask） |
| `safe_box_forest/src/find_free_box.cpp` | FFB 主循环；L3 把 seed 传到分裂值选择 |
| `link_interval_envelope/include/sbf/envelope/endpoint_source.h` | `hifk_max_depth`（L1） |
| `link_interval_envelope/include/sbf/envelope/envelope_type.h` | `n_subdivisions`（L1） |
| `safe_box_forest/experiments/sbf_old/common_sbf_config.py` | envelope/endpoint/BestTighten 参数 CLI 入口 |
| `experiments/common/run_shelf_sbf_case.py` | 阶段 E 零修复回归入口 |

## 验证标准

1. 每杠杆后重跑 probe 5 anchor，断言 `found=true` / `fail_code=0` / `hit_unknown_depth_cap=false`，最小深度 ≤ 基线。
2. 记 `grow_ms` / 每 node endpoint eval 计数，确认 L1 per-node 成本与 L2/L3 O(1) 承诺成立（热路径无逐维 FK 体积评估）。
3. 阶段 E：Exp.4 shelf case `--require-no-repair=True` 全 5 anchor 跑通，repair=0、认证覆盖不退化。
4. 任何深度下降都须伴随 fail_code=0 与零修复；不接受放宽安全性换深度。

## 决策

- 硬约束：严格认证 + 零修复；只用 sound 手段。
- L2 权重：per-robot 全局敏感度。
- dim6 惰性则直接 mask + 自适应检测机制。
- 范围外：CoverageHeuristic、envelope 放宽、per-region root。
