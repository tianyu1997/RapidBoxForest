# Experiment 4: Shelf Planning And Ablation

## 目的

在 shelf/IIWA 场景评估推荐 SBF 配置和关键机制贡献。Baseline 固定为 `warm + AAFK + SupportHull + 8 threads + AAFKVolumeMin split policy`。

## 实验设置

1. 场景：Marcucci shelf / IIWA。
2. Query：AS、TS、CS、LB、RB anchor pairs，与旧 TABLE I 和 fig3 对齐。
3. Baseline：warm LECT cache、AAFK、SupportHull、8 threads、AAFKVolumeMin split policy。
4. 消融：no LECT cache、CritSample、AABB、AABB->SH chain、single thread、round-robin split、AAFKVolumeMin+dim6 hybrid、**support-hull schedule（新）**、**legacy seed-bias L3 反向对照（新/废弃）**。
5. 每个配置使用相同 seeds、query 顺序、timeout 和 audit 口径。

### 种子/场景无关不变量（seed-independent invariant）

LECT kd-tree 节点的分裂 `(split_dim, split_value)` 必须只依赖 `(robot, canonical domain)`，
**绝不**依赖 query seed/scene。这保证 canonical warm cache 可跨 query/场景复用、外部证据可命中。
本轮重构将旧 L3 seed-bias（违反该不变量）从默认路径退役，并以新的种子无关分裂准则补偿
FFB 深度：

- **support-hull schedule（新增项，factor=`lect_split_schedule_criterion`）**：分裂调度最小化
  **SupportHull 扫掠包络体积**（认证所用的包络），而非更松的端点 AABB 体积和。调度是
  `(robot, canonical root intervals)` 的纯函数，保持种子无关。对照旧 AABB 准则
  （baseline `aafk_volume_min`）评估"准则"这一因素。
- **legacy seed-bias L3（新增反向对照项，factor=`seed_bias_reverse_control`，已废弃）**：
  重新打开 `ffb_seed_bias=0.9`，使分裂偏向 query seed。该行**故意违反**种子无关不变量
  （per-query 认证盒变为种子相关），仅用于量化解耦的代价/收益，不作为推荐配置。
- **最浅已认证祖先复用（P4）**：`find_free_box` 沿 seed 路径在第一个 `DefinitelyFree`
  canonical 祖先处返回（最大盒/最浅深度），只取决于哪些 canonical 节点已认证（种子无关）。
  诊断计数：`ffb.free_ancestor_hits`、`ffb.free_ancestor_depth_sum/_max`、
  `ffb.free_ancestor_log_volume_sum`，用于度量"平均命中深度下降、已认证盒体积上升"。

## 如何进行

1. 冻结 scene、robot、query anchor manifest。
2. Baseline 先 prewarm/build cache，再运行 queries；warm load 与 cold build 分开记录。
3. 每个消融一次只改变一个因素。
4. 保存 path、audit trace、collision/refine counters、LECT diagnostics 和 RSS snapshots。
5. 输出 ablation 表和完整 per-query artifact。

## 统计口径

1. `Build/prewarm time`：生成或加载可复用结构的时间。
2. `Query time`：从 start/goal 到返回候选 path，不含最终 audit。
3. `Path length`：通过 final audit 的 success-only mean/median/p95。
4. `Speedup`：明确相对 baseline 或 no-cache。
5. Mechanism counters：materializations、cache hit rate、split count、envelope eval count、refine count、audit collision failures。

## 预期结果

1. Baseline 预期最稳。
2. No LECT cache 增加在线 materialization 成本。
3. CritSample 可能更紧，但认证和 audit failure 需要单独解释。
4. AABB 快但过滤弱；SupportHull 现在统一走纯 GJK 窄相位，不再保留单独的 AABB->SH chain 配置。
5. Single thread 拖慢 build/prewarm；round-robin 增加 tree/refine 成本。6. support-hull schedule:相对 baseline AABB 准则,FFB 命中深度更浅、已认证盒体积更大
   (P0 主指标:深度-包络体积曲线更优),同时保持种子无关与外部证据复用 `reuse>0`。
7. legacy seed-bias L3:可能 FFB 深度更浅,但破坏种子无关性 → canonical cache/外部证据复用率
   下降(预期 `materialization_reused_external_evidence` 显著低于种子无关行),作为反向对照
   证明"解耦 + support-hull 补偿"优于"靠 seed-bias 压深度"。
## 初始脚本

`exp04_shelf_ablation/run_shelf_ablation.py` 先生成完整消融矩阵，并执行当前 runner 的 SupportHull 纯 GJK 基线。删除 keep_kdop 后，原先的 `AABB->SH chain` 行已经并入默认 `support_hull` 路径；no-cache、true round-robin 和完整 warm AAFKVolumeMin cache hook 仍标记为后续 native hook。
