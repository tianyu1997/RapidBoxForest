# Exp04 当前状态总结（2026-05-29）

## 2026-05-30 补充：E4 默认参数切换到 0.20 / 0.25 / 0.35 后的重跑摘要

本次补跑使用的新默认 grower 参数为：

- `rrt_goal_bias = 0.20`
- `intertree_goal_bias = 0.25`
- `component_connect_prob = 0.35`

对应产物位于：

- `outputs/new_experiments/exp04_e4_ccprob035_20260530/baseline_warm_aafk_support_hull_8t_aafk_volume_min.json`
- `outputs/new_experiments/exp04_e4_ccprob035_20260530/no_lect_cache_online_envelopes.json`
- `outputs/new_experiments/exp04_e4_ccprob035_20260530/single_thread.json`

### E4 关键重跑表

| Case | Threads | Fast total (s) | Fast grow / connector (ms) | Fast final islands | High total (s) | High grow / connector (ms) | High final islands | component-connect success / attempts (fast, high) | BiRRT invocations (fast, high) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| baseline_warm_aafk_support_hull_8t_aafk_volume_min | 8 | 0.273 | 56.1 / 157.6 | 1 | 0.330 | 234.3 / 57.5 | 1 | 15/93, 44/281 | 5, 4 |
| no_lect_cache_online_envelopes | 8 | 0.249 | 60.2 / 137.2 | 1 | 0.326 | 227.9 / 57.2 | 1 | 15/93, 44/281 | 5, 4 |
| single_thread | 1 | 0.594 | 89.3 / 206.1 | 2 | 0.858 | 463.7 / 198.0 | 2 | 18/71, 73/289 | 5, 4 |

补充观察：

- 这三条结果都仍然使用了 BiRRT connector，而不是 no-BiRRT 变体；直接证据是对应 stage diagnostics 中 `connector.birrt_invocations` 非零。
- baseline 和 no_lect_cache_online_envelopes 在新默认下都能在 `high` 档稳定收敛到 1 island，说明把 `component_connect_prob` 从 0.45 降到 0.35 后，没有把压力简单转移成更差的最终连通性。
- single_thread 仍明显退化，说明这一行的主瓶颈依旧是并行度，而不是当前 goal bias / component-connect 参数。
- 本次重跑中的 incremental FK 统计已正常计数，不再是旧版本里的假零：例如 baseline 的 `oracle.materialization_incremental_fk` 为 75，`grower.worker_oracle.materialization_source_incremental_state` 在 `fast/high` 分别达到 978 / 3104。

### 与旧的 0.45 baseline 对照

旧对照产物：

- `outputs/new_experiments/exp04_e4_baseline_no_external_20260530_fix/baseline_warm_aafk_support_hull_8t_aafk_volume_min.json`

最关键的差异不在 `fast`，而在 `high`：

| Baseline variant | component_connect_prob | Fast total (s) | Fast connector (ms) | Fast final islands | High total (s) | High connector (ms) | High final islands |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline_0.45 | 0.45 | 0.201 | 50.5 | 1 | 0.694 | 181.9 | 2 |
| baseline_0.35 | 0.35 | 0.273 | 157.6 | 1 | 0.330 | 57.5 | 1 |

解释：

- 单次 seed 下，`0.35` 的 `fast` 档不一定比 `0.45` 更快。
- 但 `0.35` 明显改善了 `high` 档的连通稳定性和 connector 成本：`high total` 从 `0.694s` 降到 `0.330s`，`connector_ms` 从 `181.9ms` 降到 `57.5ms`，最终 islands 从 `2` 变成 `1`。
- 这和前面的 multiseed 结论一致：`0.20 / 0.25 / 0.35` 的价值主要体现在高档位的稳定性，而不是追求某一次 `fast` 的最小单点值。

### Ablation manifest 产物登记修复

`experiments/exp04_shelf_ablation/run_shelf_ablation.py` 之前只把子进程 `measurement` 写进 `runs`，没有在执行后把 artifact 回填，所以 manifest 里会出现 `status/artifact = null` 的脏记录。

现已修复为：

- 每个 row 显式记录 `artifact_path`
- 执行后基于 `returncode + artifact 是否存在` 回填 `runs.status`
- 同时写出 `runs.artifact`

最小验证产物：

- `outputs/new_experiments/exp04_manifest_registration_check_20260530/shelf_ablation_manifest.json`

该验证 manifest 中，baseline row 已正确登记为：

- `status = completed`
- `artifact = outputs/new_experiments/exp04_manifest_registration_check_20260530/baseline_warm_aafk_support_hull_8t_aafk_volume_min.json`

## 背景

这轮工作的目标已经从“继续盲调 cache / envelope / connector”收敛成两件更具体的事：

1. 用可复现的单 case 回路量清楚当前 grower / envelope 改动到底带来了什么真实影响。
2. 给 connector 建一个足够窄、可重复、能稳定复现 timeout 的 benchmark，再决定是否动 BiRRT 超时和采样策略。

当前所有关键验证都已经在 WSL 下完成，使用的构建目录是 `build-wsl`，且 `build-wsl/CMakeCache.txt` 显示当前构建类型为 `Release`，因此本轮结论不是 Debug/Release 混淆造成的假信号。

---

## 已建立的测量回路

### 1. fast 单 case probe

- 脚本：`experiments/exp04_shelf_ablation/measure_fast_case_delta.py`
- 产物：`outputs/new_experiments/exp04_fast_single_case_probe/baseline_fast_probe.json`
- 用途：
  - 只跑 exp04 的 `fast` stage。
  - 自动和历史 reference artifact 对比。
  - 直接看 `grow_ms`、`connector_ms`、`planning_s`、`audit_ok`，以及 envelope / endpoint / grower seed cache 的命中情况。

### 2. 真实 shelf 场景 connector benchmark

- 脚本：`experiments/exp04_shelf_ablation/connector_birrt_shelf_benchmark.py`
- 80ms 基线：`outputs/new_experiments/exp04_connector_birrt_benchmark/shelf_timeout80.json`
- 250ms 对照：`outputs/new_experiments/exp04_connector_birrt_benchmark/shelf_timeout250.json`
- 用途：
  - 直接在 `IIWA + shelf obstacles + combined queries` 上跑 `rrt_connect_path`。
  - 按 query / seed 统计 success 与 timeout-like failure。
  - 让 connector 的后续优化不再依赖完整 forest build。

### 3. toy C++ connector benchmark（辅助）

- 可执行入口：`safe_box_forest/tests/connector_birrt_benchmark.cpp`
- 结论：
  - 这个 benchmark 已经证明“connector core 的单独测量框架”可用。
  - 但 toy scene 仍然过于容易直连，没能稳定产出有价值的 timeout pair。
  - 后续 connector 调参应以真实 shelf benchmark 为主，而不是继续在 toy 几何上耗时间。

---

## 当前实测结果

### 1. fast probe：当前代码的最新状态仍明显异常

来自 `baseline_fast_probe.json` 的最新 cache-on 结果：

- `grow_ms = 586.112`，reference 为 `86.611`。
- `connector_ms = 70.543`，reference 为 `131.881`。
- `planning_s = 1.363`，reference 为 `0.340`。
- `wall_s = 1.363`，reference 为 `0.340`。
- `unique_box_count = 132`，reference 为 `129`。
- `audit_ok = false`。
- `envelope_cache_hit_rate = 0.0`。
- `endpoint_reuse_rate = 0.0103`。
- `grower_component_seed_cache_hit_rate = 95 / 138 = 0.688`。

含义很明确：

- connector 比 reference 更快，但这并没有把整体规划时间拉回正常区间。
- 真正拖慢总时间的是 grow / materialization 路径。
- envelope cache 仍然完全没有命中，因此“理论上应该大幅提速”的预期目前没有被数据支持。

### 2. materialization / envelope 诊断信号明显恶化

fast probe 中最值得关注的不是 box 数量变化，而是 materialization 相关耗时暴涨：

- `grower.worker_oracle.materialization_envelope_compute_time_us = 5763.529`，reference 为 `1120.176`。
- `grower.worker_oracle.materialization_external_lookup_time_us = 3350.659`，reference 为 `2460.468`。
- `grower.worker_oracle.materialization_envelope_read_time_us = 199.311`，reference 为 `0.0`。
- `oracle.materialization_envelope_compute_time_us = 2642.047`，reference 为 `71.167`。
- `oracle.materialization_external_lookup_time_us = 7469.262`，reference 为 `85.204`。
- `oracle.materialization_envelope_read_time_us = 31.604`，reference 为 `0.0`。

同时，下面这些值并没有对应变好：

- `grower.worker_oracle.materialization_reused_cached_envelope = 0.0`
- `oracle.materialization_reused_cached_envelope = null / 实际未形成命中收益`
- `envelope_cache_hit_rate = 0.0`

这说明当前更像是“增加了额外 envelope/materialization 读写与计算成本”，但没有换来复用收益。

### 3. connector benchmark 已经锁定稳定敏感 pair

80ms 基线（`shelf_timeout80.json`）：

- `TS->CS`：`3/3` 全部 timeout-like failure，median `80.166ms`。
- `CS->LB`：`2/3` 成功，`1/3` timeout-like failure，median `36.084ms`。
- `AS->TS`：`3/3` 成功，median `48.909ms`。
- `LB->RB`、`RB->AS`：稳定成功，且几乎都是亚毫秒到 1ms 级。

250ms 对照（`shelf_timeout250.json`）：

- `TS->CS`：`2/3` 成功，`1/3` timeout-like failure，median `202.761ms`。
- `CS->LB`：仍然 `1/3` timeout-like failure。
- 其余 pair 基本稳定成功。

这组结果已经足够支撑后续 connector 调参：

- `TS->CS` 是主敏感 pair。
- `CS->LB` 是次敏感 pair。
- `LB->RB`、`RB->AS` 可以视为“容易样本”，用来验证改动没有把简单问题搞坏。

---

## 已做过的关键判别实验

### grower component seed cache 不是主因

为了验证 `grower.component_connect_seed_cache` 是否是这次回归的根因，已经做过一次最直接的判别实验：

- 临时关闭该 cache。
- 重新增量构建。
- 重跑同一个 fast 单 case probe。

结果：

- cache-off 时 `grow_ms` 仍然约为 `496.247ms`。
- `audit_ok` 依然是 `false`。
- 绝对值比 cache-on 略低，但仍远高于 `86.611ms` 的 reference。

结论：

- 这层 cache 可能会影响行为细节或带来一定波动。
- 但它不是当前 `grow_ms` 暴涨和 `audit_ok=false` 的主导根因。
- 因此不应继续把主要精力投入在“先撤掉 grower seed cache 再看”这条线上。

---

## 当前判断

### 1. 现在最大的未知点不在 connector，而在 oracle/materialization

当前数据组合非常一致：

- connector 已经比 reference 更快。
- fast probe 的总耗时却显著更差。
- envelope cache 命中仍为 0。
- worker oracle 和主 oracle 的 materialization / envelope / external lookup 耗时都显著膨胀。

因此，当前最合理的判断是：

> 这轮性能回归的主导因素，更可能在 oracle/materialization 路径，而不是 connector，也不是 grower component seed cache 本身。

### 2. “cache / envelope 理论上应该大幅提速”目前没有被数据验证

目前实测看到的是：

- envelope cache 基础设施已经存在。
- 但在 fast build 路径上命中率仍然是 0。
- 读取时间和计算时间增加了，但没有形成复用红利。

所以现在更务实的表述应该是：

- cache / envelope 还没有证明自己能带来显著收益；
- 在命中率实际提升之前，不应再预设它们会自然带来大幅加速。

### 3. connector 现在已经有了足够窄、足够真实的调参回路

之前不能动 connector，是因为没有稳定 benchmark。

现在这个前提已经改变：

- `connector_birrt_shelf_benchmark.py` 已经能稳定复现 `TS->CS` 的 timeout-like failure。
- 同时有 80ms 和 250ms 两档基线，可用于区分“只是预算不够”还是“搜索策略本身差”。

这意味着 connector 的下一步可以动，但必须围绕这个 benchmark 来动，而不是直接回到完整 forest 上盲试。

---

## 建议的下一步行动

### 优先级 1：先深挖 oracle/materialization 回归

这是当前最值得投入的主线。

建议动作：

1. 继续把 materialization 路径拆细，重点盯：
   - cache lookup
   - external evidence lookup
   - envelope compute
   - envelope collision
   - envelope read
2. 对比当前实现和 reference 行为，确认到底是：
   - 调用次数变多了；
   - 单次成本变高了；
   - 额外读路径引入了固定开销；
   - 还是 worker/main oracle 的逻辑分叉导致重复工作。
3. 重点核实新加的 worker-local envelope cache 是否出现了“零命中但每次都付 lookup/read 成本”的情况。

### 优先级 2：围绕真实 shelf benchmark 做 connector 微调

建议只围绕 `TS->CS` 和 `CS->LB` 做单变量实验，不要同时改多项：

1. 先测 timeout 预算的敏感性。
2. 再测采样半径、step size、goal bias 这类直接影响搜索效率的参数。
3. 如果要动桥接前置碰撞检查，也必须先看 `TS->CS` 是否受益。

接受改动的标准应当是：

- 80ms 下 `TS->CS` 的成功率或 median 表现变好；
- 250ms 下没有明显把原本可解的情况搞坏；
- 简单 pair（如 `LB->RB`、`RB->AS`）没有退化。

### 优先级 3：把 fast probe 当成每次修改后的回归门禁

后续每次真正改动之后，都应该至少重跑：

1. `measure_fast_case_delta.py`
2. `connector_birrt_shelf_benchmark.py --timeout-ms 80`
3. `connector_birrt_shelf_benchmark.py --timeout-ms 250`

关注的核心指标应固定为：

- `grow_ms`
- `connector_ms`
- `planning_s`
- `audit_ok`
- `envelope_cache_hit_rate`

### 当前不建议继续投入的方向

- 不建议继续在 toy C++ benchmark 的 obstacle preset 上反复调几何。
- 不建议继续把主要怀疑放在 grower component seed cache 上。
- 不建议在没有 benchmark 约束的情况下直接改 connector 正式逻辑。
- 不建议再用“理论上 cache/envelope 应该更快”代替实际命中与耗时数据。

---

## 建议保留并持续使用的文件

### 核心脚本

- `experiments/exp04_shelf_ablation/measure_fast_case_delta.py`
- `experiments/exp04_shelf_ablation/connector_birrt_shelf_benchmark.py`
- `safe_box_forest/tests/connector_birrt_benchmark.cpp`（辅助用）

### 核心产物

- `outputs/new_experiments/exp04_fast_single_case_probe/baseline_fast_probe.json`
- `outputs/new_experiments/exp04_connector_birrt_benchmark/shelf_timeout80.json`
- `outputs/new_experiments/exp04_connector_birrt_benchmark/shelf_timeout250.json`

---

## 一句话结论

当前阶段最重要的变化不是“已经优化完了”，而是：

- fast 单 case 回路已经证明问题仍然存在，而且主问题不在 connector；
- 真实 shelf connector benchmark 已经就位，后续可以有约束地优化 BiRRT；
- 下一步最该做的是查清 oracle/materialization 为什么在 envelope 仍零命中的前提下显著变贵。