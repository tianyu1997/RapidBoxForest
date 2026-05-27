# Experiment 4: Shelf Planning And Ablation

## 目的

在 shelf/IIWA 场景评估推荐 SBF 配置和关键机制贡献。Baseline 固定为 `warm + AAFK + SupportHull + 8 threads + AAFKVolumeMin split policy`。

## 实验设置

1. 场景：Marcucci shelf / IIWA。
2. Query：AS、TS、CS、LB、RB anchor pairs，与旧 TABLE I 和 fig3 对齐。
3. Baseline：warm LECT cache、AAFK、SupportHull、8 threads、AAFKVolumeMin split policy。
4. 消融：no LECT cache、CritSample、AABB、AABB->SH chain、single thread、round-robin split。
5. 每个配置使用相同 seeds、query 顺序、timeout 和 audit 口径。

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
4. AABB 快但过滤弱；AABB->SH chain 预期在平均 collision 成本上低于 pure SH，同时保留更强过滤。
5. Single thread 拖慢 build/prewarm；round-robin 增加 tree/refine 成本。

## 初始脚本

`exp04_shelf_ablation/run_shelf_ablation.py` 先生成完整消融矩阵，并执行旧 runner 能表达的配置；其中 `AABB->SH chain` 直接映射到 legacy `support_hull` 路径并强制 `--no-support-hull-keep-kdop`，因为底层共享碰撞实现本身已经是 `AABB broadphase -> SupportHull narrow phase`。no-cache、true round-robin 和完整 warm AAFKVolumeMin cache hook 仍标记为后续 native hook。
