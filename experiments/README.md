# RapidBoxForest Experiments

本目录是新一轮实验的根入口。旧脚本仍保留在 `safe_box_forest/experiments/sbf_old/`，新脚本优先以清晰的实验包组织，并在必要时复用旧 pipeline。

## 文档

1. `00_experiment_principles.md` — 总体统计、审计、复现和输出口径。
2. `01_endpoint_envelope_plan.md` — Endpoint envelope 微基准计划。
3. `02_link_envelope_s1_plan.md` — 固定 `S=1` 的 Link envelope 计划。
4. `03_lect_performance_plan.md` — LECT 操作耗时与内存计划。
5. `04_shelf_ablation_plan.md` — Shelf planning 和消融计划。
6. `05_shelf_cross_algorithm_plan.md` — Shelf 跨算法比较计划。
7. `06_random_robot_plan.md` — 随机场景与跨机器人计划。

## Runner

所有 runner 默认写入 `outputs/new_experiments/<experiment>/`。先用 `--dry-run` 或 `--smoke --dry-run` 检查 manifest，再执行长实验。

```bash
python3 experiments/exp01_endpoint_envelope/run_endpoint_envelope.py --smoke --dry-run
python3 experiments/exp02_link_envelope_s1/run_link_envelope_s1.py --smoke --dry-run
python3 experiments/exp03_lect_microbench/run_lect_microbench.py --dry-run
python3 experiments/exp04_shelf_ablation/run_shelf_ablation.py --dry-run
python3 experiments/exp05_shelf_cross_algorithm/run_shelf_cross_algorithm.py --dry-run
python3 experiments/exp06_random_robot/run_random_robot.py --dry-run
```

## 当前实现状态

1. Exp.1 复用旧 endpoint pipeline，并补充 AAFK alias、smoke 和 manifest。
2. Exp.2 复用旧 link pipeline，强制 `S=1`，并补充 `S=1` cascade variant 解析。
3. Exp.3 已有外部命令式 measurement harness；native LECT counters 需要后续 C++ hook。
4. Exp.4 已有完整消融矩阵 manifest；旧 runner 无法表达的 no-cache 和 true round-robin 被显式标记为待补 hook。
5. Exp.5/Exp.6 已有跨算法/随机场景 dispatcher，可统一生成命令和输出路径。
