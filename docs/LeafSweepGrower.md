LeafSweepGrower 使用教程

LeafSweepGrower 是一个不做规划、不建连通图的验证型 grower。它从指定 start_depth 的 LECT leaf frontier 出发，对所有待定 box 做广度优先验证，最终输出场景级：

free_boxes
collision_boxes
其中 collision_boxes 表示“到 max_depth 仍无法证明 free / collision possible”，不是严格 occupied proof。

核心接口

Python:

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
C++ facade:

rbf::LeafSweepConfig leaf_cfg;
leaf_cfg.obstacle_cluster_gap = 0.0;
leaf_cfg.n_threads = 1;
leaf_cfg.validation_batch_size = 256;
leaf_cfg.timeout_ms = 0.0;
leaf_cfg.store_group_results = true;

auto result = forest.build_leaf_sweep(obstacles, 10, 18, leaf_cfg);
算法语义

给定 start_depth 和 max_depth：

从 root 开始 materialize 到 start_depth。
将 start_depth frontier 中的所有节点加入 pending queue。
依次验证 pending node：
若 Free，加入 free set，不再下探。
若非 Free 且 depth >= max_depth，加入 collision set。
若非 Free 且未到 max_depth，按 split policy 分裂，子节点加入 pending queue。
pending 为空后，输出最终场景级 free/collision set。
多障碍物处理

默认会按 obstacle AABB 聚类：

leaf_cfg.obstacle_cluster_gap = 0.0
含义：

0.0：AABB 相交或接触才聚为一组。
更大值：距离小于该 gap 的 obstacle 也会聚为一组。
很大值，例如 1000：通常会把所有 obstacle 合成一个 group。
多 group 时：

每个 group 单独 sweep。
最终 free_boxes 是各 group free 覆盖区域的交集。
最终 collision_boxes 是各 group collision 覆盖区域的并集。
推荐 warm/d23 快速模式

对于 exp04 shelf ablation 的 d23 warm cache，推荐使用 virtual topology：

leaf_cfg.use_virtual_topology = True
leaf_cfg.store_group_results = False
leaf_cfg.obstacle_cluster_gap = 1000.0
含义：

不在 active LECT 中逐节点 split。
直接按 d23 split schedule 在内存中生成 virtual topology。
通过 d23 external evidence 做 exact lookup。
适合 warm-only 验证，速度显著快。
推荐命令：

PYTHONPATH=build-exp04/python python3 experiments/exp04_shelf_ablation/leaf_sweep_d23_probe.py \
  --cases warm \
  --no-endpoint-evidence-cache \
  --no-store-group-results \
  --start-depth 10 \
  --max-depth 18 \
  --threads 1 \
  --validation-batch-size 512 \
  --obstacle-cluster-gap 1000 \
  --use-virtual-topology \
  --out-dir outputs/new_experiments/exp04_leaf_sweep_d10_d18_warm_fast_virtual_serial
实测 exp04 shelf：

warm virtual topology:
wall_s ~= 1.51
free_boxes = 16802
collision_boxes = 126905
重要配置说明

LeafSweepConfig 字段：

leaf_cfg = sbf.LeafSweepConfig()

leaf_cfg.obstacle_cluster_gap = 0.0
控制障碍物聚类距离。若只关心最终整体场景、且想减少多 group 合成开销，可以设大一些，将障碍物视为单组。

leaf_cfg.n_threads = 1
当前稳定推荐值是 1。并行 virtual validation 已实验实现，但结果存在小幅非确定差异，暂不建议作为正式结果。

leaf_cfg.validation_batch_size = 256
验证 batch 大小。串行模式下影响较小；并行实验模式中影响任务粒度。

leaf_cfg.timeout_ms = 0.0
0.0 表示不设 timeout。

leaf_cfg.store_group_results = False
若只需要最终 free_boxes/collision_boxes，建议设为 False，减少 group 结果复制和 JSON 输出开销。

leaf_cfg.pre_split_to_max_depth = False
实验项。提前把 active LECT split 到 max_depth。在 exp04 warm 场景实测变慢，不推荐。

leaf_cfg.use_virtual_topology = True
warm/d23 推荐项。跳过 active LECT split，复用 d23 topology 语义。

leaf_cfg.parallel_virtual_validation = False
实验项。速度更快，但当前结果非确定，不推荐正式使用。

读取统计信息

结果中包含：

print(result.total_ms)
print(result.initialize_ms)
print(result.group_sweep_ms)
print(result.compose_ms)
print(dict(result.diagnostics))
也可以从 forest 读取 oracle counters：

counters = dict(forest.oracle_counters())
print(counters["node_validations"])
print(counters["materialization_reused_external_evidence"])
统计 depth 分布

from collections import Counter

def box_depth(box):
    node_id = int(box.tree_id)
    return node_id.bit_length() - 1 if node_id > 0 else 0

free_depth = Counter(box_depth(box) for box in result.free_boxes)
collision_depth = Counter(box_depth(box) for box in result.collision_boxes)

print(dict(sorted(free_depth.items())))
print(dict(sorted(collision_depth.items())))
注意：应使用 box.tree_id 统计 depth，不要使用 box.id。box.id 是结果集合内局部 id，不是 LECT node id。

exp04 d10-d18 示例结果

warm virtual topology fast path：

free depth:
d10=169
d11=156
d12=326
d13=666
d14=868
d15=1367
d16=3820
d17=7816
d18=1602
注意事项

start_depth <= max_depth，否则会抛异常。
max_depth 会受 LECT max_tree_depth 限制。
collision_boxes 是 conservative collision-possible set，不是严格 occupied proof。
d23 warm cache 的 envelope 可能由更深子节点 hull bottom-up 得到，通常比 cold 当前节点直接 materialize 更紧，因此 warm/cold 的 free/collision 数量可能不同。
active endpoint cache 若开启，会 lazy 写 active LECT evidence；cold 场景可能非常慢。当前 leaf sweep 统计推荐关闭 active endpoint cache，仅使用 d23 external evidence。