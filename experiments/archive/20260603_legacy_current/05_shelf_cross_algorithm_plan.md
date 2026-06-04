# Experiment 5: Shelf Cross-Algorithm Comparison

## 目的

复现旧 TABLE I 和 fig3 风格，对比 SBF、IRIS-NP+GCS、PRM、RRTConnect、BIT* 在 shelf/IIWA 场景的 anytime trade-off。

## 实验设置

1. 场景和 query 与 Experiment 4 完全一致。
2. SBF 使用 Experiment 4 baseline。
3. Baselines：IRIS-NP+GCS、PRM、RRTConnect、BIT*。
4. OMPL planning、simplify 和 final audit 使用 `0.01` joint-space segment step。
5. RRTConnect one max-timeout attempt；BIT* fixed-timeout invocation，保存 audited monotone incumbent trace。

## 如何进行

1. 为每个算法提供统一 runner wrapper。
2. SBF/IRIS/PRM 记录 reusable build 和 online query。
3. RRTConnect/BIT* 记录 query 或 checkpoint trace。
4. 保存完整 anytime curve，再生成 TABLE I 风格 query-by-query readable slice。
5. 生成 fig3 风格 trade-off 图和 amortized per-query cost。

## 统计口径

1. `Build (s)`：可复用结构构建；RRTConnect/BIT* 无 build 时记 0 或留空并脚注。
2. `Query (s)`：在线规划时间，success-only 平均。
3. `Path`：final audit 后路径长度，success-only 平均。
4. `Anytime point`：charged time 与 audited path length。
5. `Amortized cost`：`(build + sum(query_i)) / N`。

## 预期结果

1. SBF 以较低 reusable build 和较低 online query 获得接近最佳 path。
2. RRTConnect 快但路径质量和方差较弱。
3. BIT* 随时间改善，但短预算不一定稳定优于 SBF。
4. PRM 需要较高 build 才稳定。
5. IRIS-NP+GCS 路径质量强，但 build/query 成本高。

## 初始脚本

`exp05_shelf_cross_algorithm/run_shelf_cross_algorithm.py` 调度 SBF、IRIS/PRM/BIT* 和 RRTConnect 的旧 runner，默认生成 dry-run manifest，`--execute` 后顺序运行。
