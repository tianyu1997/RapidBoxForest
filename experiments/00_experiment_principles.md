# 实验总体口径与原则

本文档定义新一轮 RapidBoxForest 实验的共同口径。每个具体实验可以扩展字段，但不得改变这里的审计、统计和复现原则。

## 目标

1. 将 envelope、LECT、planning 和跨算法比较拆成独立实验，避免把微基准结论和端到端 planning 结论混在一起。
2. 所有 runner 只产出原始 JSON/JSONL/CSV artifact；表格、图和论文摘录由 analyzer 从原始 artifact 生成。
3. 每个实验都必须能先运行 smoke test，再运行 full matrix。

## 复现元数据

每次运行必须记录：

1. `git_sha`、运行命令、工作目录、runner 路径和 run id。
2. OS、Python 版本、CPU、机器名、环境变量中影响线程或 cache 的关键项。
3. build 目录、Python binding 路径、robot、scene、seed、线程数、cache 路径。
4. 输出文件路径、输入 manifest 路径和依赖的旧 artifact hash。

## 时间口径

1. 微基准默认 warm-up 5 次，有效重复 30 次；高成本 source 或 planner 可以降到 10 次，但必须写入 metadata。
2. Planning 至少拆分 `build/prewarm`、`load_cache`、`online_query`、`audit`、`save_cache`、`analysis/report`。
3. 主报告使用 wall-clock seconds；endpoint/link 微基准可额外输出 microseconds。
4. 子进程 runner 必须记录返回码、wall time、stdout/stderr 尾部和失败原因。

## 路径与审计口径

1. 最终路径质量只使用固定分辨率 audit 后通过的路径。
2. 统一沿用旧论文口径：OMPL planning、simplify 和 final audit 使用 `0.01` joint-space segment step。
3. `Query (s)` 和 `Path` 的主表数值为 success-only average，同时保留 all-run success rate、timeout rate 和 audit failure rate。
4. RRTConnect 使用 one max-timeout attempt；BIT* 使用 fixed-timeout invocation 并记录 audited monotone incumbent checkpoints。

## 内存与 LECT 口径

1. 至少记录 VmRSS、VmHWM、cache file size、serialized payload bytes、node count。
2. LECT 实验额外记录 per-node bytes p50/p90/p95、dirty nodes、bytes written、cache hit/miss、materialization count。
3. mmap payload 与匿名堆内存必须分开解释；不能只用 RSS 推断节点真实内存。

## 输出规范

建议 JSON 顶层字段：

```json
{
  "experiment": "expXX_name",
  "run_id": "...",
  "status": "ok|failed|dry_run|partial",
  "params": {},
  "environment": {},
  "artifacts": {},
  "rows": [],
  "summary": {}
}
```

## 选点与图表原则

1. Anytime 图保存完整 checkpoint curve。
2. 表格只作为 readable slice；不能删除完整曲线。
3. 随机场景表格沿用旧 TABLE II 的规则：在最短 audited path 的 8% 窗口内，最小化 path/log-time utility。
4. Shelf 表格建议使用同一规则；如改用固定 checkpoint，必须在表注中说明。

## Smoke Test 原则

1. Endpoint/link smoke：1 个 width、1 个 box、最少 source/variant 组合。
2. Planning smoke：1 个 seed、1 个 query 或最短 query set、短 timeout。
3. Cross-algorithm smoke：允许只生成 dry-run manifest；高成本 IRIS/GCS 不在默认 smoke 中执行。
