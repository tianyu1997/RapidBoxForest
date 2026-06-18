# Seed/场景无关 LECT 优化计划（替换 L3 seed-bias）

状态：执行中（2026-06-01 起）。本文件取代 `docs/FFB_L3_SEED_BIAS_IMPLEMENTATION.md` 的方向性结论。

## 1. 动机与根因（已实测确认）

LECT 的本质：kd-tree 节点的切分（split_dim、split_value）必须**只由 `(robot, canonical 域)` 决定**，
与 query seed、与具体场景无关。只有这样节点才能终身复用、external evidence 缓存才能命中。

L3 违反了这一不变量：

- `biased_split_value`（`lect_database/src/sbf/oracle.cpp`）把 split_value 从中点拉向 query seed
  （`value = mid + β·(seed − mid)`）。lect_database 的 kd-tree 是持久共享的——节点首次被切就
  固定 split_dim/split_value，于是 L3 把**第一个 query 的 seed**冻结进了共享 canonical 树。
- L3 还专门把 seed 那侧子盒切窄 → box 变小（用户反馈的核心问题）。

第二层根因（独立于 L3，但同样破坏复用）：grower 的 BestTighten 用 `best_tighten_depth_dims_`
记录“每深度的切分维”，但该向量**未从 schedule 预置**，而是由**首个到达该深度的 query 自由选维**
惰性填充（`observe_best_tighten_choice`）。prewarm 与 live 的首查顺序不同 → 维序列分歧 →
node_path 不匹配 → external evidence key（按 node_path 匹配）永不命中。

实测（depth25 单 case）：10576 次 materialization，`reused_external_evidence = 0`，
external_lookup 花 14ms 纯开销零命中，warm cache 对 build 零加速。

## 2. 用户决定（硬约束）

1. **去 seed-bias 导致的 FFB 深度回升可接受**；但 split policy 必须与 seed/场景**完全解耦**（不可妥协）。
2. 新 schedule 的 envelope 测度 = **support-hull 体积**（与运行时 SupportHull 认证 envelope 同源）。
3. 保留 **L2 `dim_priority_weights`**（per-robot 运动学敏感度，seed 无关）作为同分维 tie-break。
4. canonical 域 = **关节限位 + dim0 对称**（场景无关），放弃 shelf-anchor restricted root。
5. E4 消融**保留旧消融项**，并新增必要新项。

## 3. 优化目标

- 主指标（seed/场景无关）：同一深度下，box 的 swept-envelope（support-hull）体积尽量小。
- 验证指标（仅测结果，场景相关）：shelf 场景中同一批 seed 取得 DefinitelyFree box 的 FFB 深度尽量小。
- 不变量测试：固定 `(robot, 域)`，任意 query / 任意 seed 顺序下，重复构树得到**完全一致**的
  node_path 与 split_value（identity / fingerprint 稳定）。

## 4. 阶段

### P1 — 回退 L3 至 seed 无关（解耦，必做）
- 所有配置 `--ffb-seed-bias` 默认改为 `0.0`：
  `safe_box_forest/experiments/sbf_old/common_sbf_config.py`、`experiments/common/lect_db_dispatch.py`、
  `experiments/exp04_shelf_ablation/run_shelf_ablation.py`。
- 移除 3 处 seed 注入：
  `safe_box_forest/src/find_free_box.cpp`、`safe_box_forest/src/safe_box_forest.cpp`、
  `safe_box_forest/python/bindings.cpp`（删除把 `tree_seed` 写入 `split.best_tighten.seed_coords` 的块）。
- 保留 `--ffb-auto-mask-inert`（per-robot inert 维 mask，seed 无关）。
- 保留 L2 `dim_priority_weights`（seed 无关 tie-break）。
- 可选：删除 `biased_split_value` 的 seed 分支或整函数化简为 midpoint（保留 sector boundary 优先逻辑）。
- 验证：任意 query 下 `split_value` 恒等于中点（或一致的 sector boundary）；identity 测试通过。

### P2 — grower 路径保真（让 box 成为 canonical 节点）
- 在 `DatabaseBoxOracle` 两个构造函数体内，从
  `database_.split_policy_descriptor().depth_dimensions` **预置** `best_tighten_depth_dims_`
  （当 strategy == AAFKVolumeMin / depth_dimensions 非空时）。
- 效果：每深度切分维完全由 canonical FixedDepthSchedule 决定（与首查顺序、seed 无关）；
  `choose_best_tighten_split` 走 replay 分支，配合 seed_bias=0 用中点值 → box 成 canonical 节点。
- worker 已从 master 复制 `best_tighten_depth_dims_`（oracle.cpp:1552），自动继承。
- 验证：depth25 controlled probe `materialization_reused_external_evidence > 0` 且 build 比 no-cache 快。

### P3 — support-hull envelope 体积 schedule（seed/场景无关）
- 现状：`aafk_volume_min_depth_schedule`（link_interval_envelope）用轴对齐 endpoint-AABB 体积，
  对 base 关节旋转不敏感、middle-axis heavy、dim6 starvation。
- 新增 support-hull 体积测度：对候选 box 计算各活跃 link 的 support-hull（凸支撑包）体积之和，
  一次性按 `(robot, canonical 域)` 预计算每深度最优维序列。纯 `(robot, domain)` 函数。
- 暴露新 split strategy / schedule 构造选项（如 `support_hull_volume_min_depth_schedule`），
  在 `common_sbf_config.make_aafk_volume_min_split_policy` 旁加平行构造器，CLI 可选。
- 验证：P0 主指标（深度-envelope 体积曲线）新 < 旧；P0 场景指标 FFB 深度不劣于（目标优于）旧 L3。

### P4 — 最浅已认证祖先复用（最大盒 / 浅深度）
- `find_free_box` 返回包含 seed 的**最浅 DefinitelyFree canonical 祖先**，而非降到固定深度。
- seed 无关（只取决于哪些 canonical 节点已认证）。
- 验证：场景 batch 的“已认证最大盒体积”上升、平均命中深度下降。

### P5 — E4 消融重构 + 文档
- baseline 改为 seed 无关 canonical warm cache（seed_bias=0 + 预置 schedule + support_hull schedule）。
- 保留旧消融项；新增：schedule 准则（AABB vs support-hull）、域（关节限位 vs +dim0 对称）、
  路径保真（预置 depth_dims 开/关）、seed_bias（0 vs 旧 0.9，作为反向对照证明解耦收益）。
- 更新 `experiments/04_shelf_ablation_plan.md`、`docs/FFB_DEPTH_COMPRESSION_PLAN.md`；
  给 `docs/FFB_L3_SEED_BIAS_IMPLEMENTATION.md` 加废弃说明。

## 5. 验证命令骨架

depth25 复用 probe（必须显式传 `--rbf-cache-root`）：

```
python3 experiments/common/run_shelf_sbf_case.py --case-name probe_use \
  --out-json <p> --database-path <cache>/active_probe --rbf-cache-root <cache> \
  --endpoint-source aafk --lect-split-policy aafk_volume_min --rbf-envelope support_hull \
  --warm-cache-label <label> --lect-root-intervals "<ROOT>" \
  --rbf-max-depth 25 --ffb-depth 25 --connector-pave-depth 25 \
  --component-connect-ffb-max-depth 25 --rbf-ffb-start-depth 10 \
  --threads 8 --seeds-list 0 --timeout-ms 60000 --use-external-evidence
```

构建（避免全量重编触发 bindings keep_kdop 漂移）：

```
cmake --build build-rbf-only-exec --target sbf_core -j 4
cmake --build build-rbf-only-exec --target _sbf_cpp -j 4
```

## 6. 风险

- 去 seed-bias 可能让 FFB 深度回升（当初加 L3 的原因）。补偿杠杆：P3 更紧 support-hull schedule、
  P1 域整形、P4 最浅祖先复用。用户已确认深度回升可接受，前提是解耦不可妥协。
- support-hull 体积 schedule 构造成本较高，但为一次性预计算，可承受。
