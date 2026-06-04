# Experiment 3: LECT Performance

## 目的

建立独立 LECT 微基准，测量 query、load、split、save、materialize、spill/read flat payload 等常用操作耗时，以及节点内存和序列化开销。

## 实验设置

1. 数据集：至少 shelf/IIWA baseline LECT cache；可追加 random robot caches。
2. Cache 模式：cold load、warm load、fresh writable、read-only replay、incremental save、full save。
3. Envelope/channel：AAFK+SH baseline，AABB、AABB->SH chain、CritSample 作为对照。
4. Tree 规模：1k、10k、100k 或按现有 cache 实际规模分桶。
5. 操作：query、batch query、split、materialize evidence、load、save full、save incremental、spill/read flat payload。

## 如何进行

1. 准备固定 cache 和 query workload manifest。
2. 单操作微基准每次只测一个 API 或阶段。
3. 在 load 前后、split 前后、query batch 后、save 前后、spill 前后采样 RSS。
4. 记录 node_count、leaf_count、payload bytes、per-node bytes p50/p90/p95、mmap file size、heap overlay bytes。
5. 输出 cache hit/miss、materialization count、dirty nodes、bytes written。

## 统计口径

1. Query：p50/p90/p95/p99、QPS、batch size。
2. Load/save/split：total time、per-node time、per-byte time。
3. Memory：VmRSS、VmHWM、PSS 可选；mmap 与 anonymous heap 分开解释。
4. Incremental save：dirty nodes、bytes written、full-save equivalent bytes、speedup。

## 预期结果

1. Warm/read-only cache query 显著快于在线 materialization。
2. Incremental save 在 dirty node 少时远快于 full save。
3. Load warm filesystem cache 明显快于 cold load。
4. SH per-node bytes 高于 AABB；AABB->SH chain 还会增加 staged metadata，但可能降低平均 collision/refine 成本。

## 初始脚本

`exp03_lect_microbench/run_lect_microbench.py` 先实现外部命令式 measurement harness，记录 RSS、wall time 和 artifact。C++ 层若尚未暴露细粒度 LECT counters，后续在该脚本下补 native hook。
