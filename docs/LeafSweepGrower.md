# LeafSweepGrower 使用教程

`LeafSweepGrower` 是一个只做 box 验证、不做规划、不建连通图的 grower。它从指定 `start_depth` 的 LECT frontier 出发，对待定 box 做广度优先验证，最终输出场景级：

- `free_boxes`
- `collision_boxes`

这里的 `collision_boxes` 表示“到 `max_depth` 仍无法证明 free / collision possible”，不是严格 occupied proof。

## 核心接口

Python:

```python
import sbf

robot = sbf.load_iiwa14_robot()
obstacles = list(sbf.make_combined_obstacles())

cfg = sbf.SBFConfig()
forest = sbf.SafeBoxForest(robot, cfg)

leaf_cfg = sbf.LeafSweepConfig()
leaf_cfg.obstacle_cluster_gap = 0.0
leaf_cfg.n_threads = 1
leaf_cfg.validation_batch_size = 256
leaf_cfg.timeout_ms = 0.0
leaf_cfg.store_group_results = True

result = forest.build_leaf_sweep(
    obstacles,
    start_depth=10,
    max_depth=18,
    config=leaf_cfg,
)

print(len(result.free_boxes), len(result.collision_boxes))
```

C++:

```cpp
rbf::LeafSweepConfig leaf_cfg;
leaf_cfg.obstacle_cluster_gap = 0.0;
leaf_cfg.n_threads = 1;
leaf_cfg.validation_batch_size = 256;
leaf_cfg.timeout_ms = 0.0;
leaf_cfg.store_group_results = true;

auto result = forest.build_leaf_sweep(obstacles, 10, 18, leaf_cfg);
```

## 算法语义

给定 `start_depth` 和 `max_depth`：

1. 从 root 开始 materialize 到 `start_depth`。
2. 将 `start_depth` frontier 中的所有节点加入 pending queue。
3. 依次验证 pending node。
4. 若 `Free`，加入 free set，不再下探。
5. 若非 `Free` 且 `depth >= max_depth`，加入 collision set。
6. 若非 `Free` 且未到 `max_depth`，按 split policy 分裂，子节点加入 pending queue。
7. pending 为空后，输出最终场景级 free/collision set。

`build_leaf_sweep()` 会更新 forest 的 scene，并将 `forest.boxes()` / raw boxes 设置为最终 `free_boxes`。`collision_boxes` 只通过返回值获取。

## 多障碍物处理

默认会按 obstacle AABB 聚类：

```python
leaf_cfg.obstacle_cluster_gap = 0.0
```

含义：

- `0.0`：AABB 相交或接触才聚为一组。
- 更大值：距离小于该 gap 的 obstacle 也会聚为一组。
- 很大值，例如 `1000.0`：通常会把所有 obstacle 合成一个 group。

多 group 时：

- 每个 group 单独 sweep。
- 最终 `free_boxes` 是各 group free 覆盖区域的交集。
- 最终 `collision_boxes` 是各 group collision 覆盖区域的并集。

`use_virtual_topology=True` 当前要求单 obstacle group。因此 exp04 warm 快速验证通常设置 `obstacle_cluster_gap=1000.0`。

## Warm/d23 快速模式

对于 exp04 shelf ablation 的 d23 warm cache，推荐使用 virtual topology：

```python
leaf_cfg.use_virtual_topology = True
leaf_cfg.parallel_virtual_validation = True
leaf_cfg.n_threads = 8
leaf_cfg.validation_batch_size = 65536
leaf_cfg.store_group_results = False
leaf_cfg.obstacle_cluster_gap = 1000.0
```

含义：

- 不在 active LECT 中逐节点 split。
- 直接按 d23 split schedule 在内存中生成 virtual topology。
- 通过 d23 external evidence 做 exact lookup。
- parallel batch validation 使用独立 read-only oracle worker，并共享 master LECT topology 视图。
- LeafSweep sweep 期间会关闭 oracle envelope cache，以保证 serial/parallel/cold/warm 验证口径一致；该 cache 在 exp04 中命中很少，关闭后结果稳定且更快。

推荐命令：

```bash
PYTHONPATH=build-exp04/python:safe_box_forest/python:link_interval_envelope/python:lect_database/python \
python3 experiments/exp04_shelf_ablation/leaf_sweep_d23_probe.py \
  --cases warm \
  --start-depth 10 \
  --max-depth 18 \
  --threads 8 \
  --validation-batch-size 65536 \
  --obstacle-cluster-gap 1000 \
  --no-store-group-results \
  --no-endpoint-evidence-cache \
  --use-virtual-topology \
  --parallel-virtual-validation \
  --out-dir outputs/new_experiments/exp04_leaf_sweep_d10_d18_warm_fast_virtual_parallel
```

当前 exp04 shelf d10-d18 验证结果：

- serial virtual warm: `free_boxes=16795`, `collision_boxes=126924`, wall 约 `1.08s`
- parallel virtual warm, 8 threads, batch 65536: `free_boxes=16795`, `collision_boxes=126924`, wall 约 `0.52s`

## 配置字段

```python
leaf_cfg = sbf.LeafSweepConfig()
```

`obstacle_cluster_gap`

控制障碍物聚类距离。若只关心最终整体场景，且希望减少多 group 合成开销，可以设大一些，将障碍物视为单组。

`n_threads`

执行线程数。`0` 会由 facade clamp 到至少 1。warm virtual parallel 模式建议从 `8` 和 `validation_batch_size=65536` 开始。当前 exp04 d10-d18 下，`16` threads 会因为 external lookup mutex contention 和线程调度成本变慢。

`validation_batch_size`

并行 virtual validation 的 batch 粒度。过小会增加调度开销，因为当前 `ThreadPoolExecutor::parallel_for` 每个 batch 都会创建 worker threads。exp04 d10-d18 下推荐 `65536`；`512` 明显偏小。

`timeout_ms`

`0.0` 表示不设 timeout。超时后剩余 pending 节点按 conservative collision 处理。

`store_group_results`

若只需要最终 `free_boxes` / `collision_boxes`，建议设为 `False`，减少 group 结果保存和 Python 侧传输开销。

`pre_split_to_max_depth`

实验项。提前把 active LECT split 到 `max_depth`。exp04 warm 场景实测变慢，不推荐。

`use_virtual_topology`

warm/d23 推荐项。跳过 active LECT split，复用 d23 split schedule 语义。当前只支持单 group。

`parallel_virtual_validation`

warm/d23 推荐项。需要 `use_virtual_topology=True` 且 `n_threads > 1`。当前 exp04 d10-d18 serial/parallel 结果一致。

## 耗时瓶颈与调参

当前 exp04 shelf d10-d18 warm virtual parallel 的最快实测配置：

```text
threads=8
validation_batch_size=65536
store_group_results=False
endpoint_evidence_cache=False
use_virtual_topology=True
parallel_virtual_validation=True
```

该配置下典型结果：

```text
wall ~= 0.52s
free_boxes = 16795
collision_boxes = 126924
node_validations = 286414
```

线程数扫描，batch 65536：

```text
threads=2   wall ~= 0.639s
threads=4   wall ~= 0.566s
threads=8   wall ~= 0.521s
threads=16  wall ~= 0.652s
```

batch size 扫描，threads 8：

```text
512      wall ~= 0.726s
1024     wall ~= 0.665s
2048     wall ~= 0.634s
4096     wall ~= 0.613s
8192     wall ~= 0.585s
16384    wall ~= 0.564s
32700    wall ~= 0.575s
65536    wall ~= 0.536s
131000   wall ~= 0.561s
```

主要瓶颈：

- external evidence lookup 是最大瓶颈。parallel worker 累计 external lookup time 很高，主要包含全局 mutex 等待。
- batch 调度成本明显。batch 从 `512` 增大到 `65536` 可将 wall 从约 `0.72s` 降到约 `0.53s`。
- envelope compute 是第二类实算成本，但并行后已被摊薄。
- collision check 不是瓶颈，累计约 `88ms` worker time。
- initialize 和 compose 都是亚毫秒级，可以忽略。

后续可优化方向：

- 去掉或细化 external evidence lookup 的全局 mutex。理想状态是只锁必要的 shared exact-box index，或者给每个 worker 独立 read snapshot。
- 在 virtual topology 下直接构造 d23 evidence key，避免 `endpoint_for_box_exact(intervals)` 的 exact-box tree lookup。
- 将 `ThreadPoolExecutor` 改成 persistent thread pool，避免每个 batch 创建线程。

## 读取统计信息

```python
print(result.total_ms)
print(result.initialize_ms)
print(result.group_sweep_ms)
print(result.compose_ms)
print(dict(result.diagnostics))

counters = dict(forest.oracle_counters())
print(counters["node_validations"])
print(counters["materialization_reused_external_evidence"])
```

并行 virtual validation 的 worker oracle counters 会写入 `result.diagnostics`，例如：

- `leaf_sweep.worker_oracle.node_validations`
- `leaf_sweep.worker_oracle.materialization_reused_external_evidence`
- `leaf_sweep.worker_oracle.validate_node_total_time_us`

## 统计 depth 分布

```python
from collections import Counter

def box_depth(box):
    node_id = int(box.tree_id)
    return node_id.bit_length() - 1 if node_id > 0 else 0

free_depth = Counter(box_depth(box) for box in result.free_boxes)
collision_depth = Counter(box_depth(box) for box in result.collision_boxes)

print(dict(sorted(free_depth.items())))
print(dict(sorted(collision_depth.items())))
```

注意：应使用 `box.tree_id` 统计 depth，不要使用 `box.id`。`box.id` 是结果集合内局部 id，不是 LECT node id。

exp04 d10-d18 parallel warm 当前 free depth 示例：

```text
d10=169
d11=155
d12=215
d13=605
d14=486
d15=979
d16=1567
d17=4501
d18=8118
```

collision depth 示例：

```text
d17=1
d18=126923
```

## 注意事项

- `start_depth <= max_depth`，否则抛 `invalid_argument`。
- `max_depth` 会 clamp 到 oracle `max_tree_depth()-1`。
- `collision_boxes` 是 conservative collision-possible set，不是严格 occupied proof。
- d23 warm cache 的 envelope 可能由更深子节点 hull bottom-up 得到，通常比 cold 当前节点直接 materialize 更紧，因此 warm/cold 的 free/collision 数量可能不同。
- active endpoint cache 若开启，会 lazy 写 active LECT evidence；cold 场景可能非常慢。当前 leaf sweep 统计推荐关闭 active endpoint cache，仅使用 d23 external evidence。
