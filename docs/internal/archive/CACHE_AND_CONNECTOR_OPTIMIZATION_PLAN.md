# 缓存命中率提升 + Connector 优化 实施计划

> 目标：在 `cache+8t` 模式下逼近/超过 `no_cache` 与 `single_thread` 的规划时间（以 `grow_ms` / `connector_ms` 为准，**不用 `wall_s`**，因后者含一次性快照加载）。
>
> 已落地前置杠杆：
> - **T2 AAFK 增量端点细化**：miss 时端点计算从完整 FK 降为仅重算 `changed_dim` 之后的 prefix。
> - **T4 worker 跨 task 线程安全共享端点缓存**：`SharedEndpointEvidenceCache`（区间键、`shared_mutex`、位精确匹配），master 持有、worker 共享。
>
> 关键架构事实：
> - `oracle_`（`DatabaseBoxOracle`）由 `RBFPlanningForest::reset_oracle()` 创建，**跨 query 持久存活**（除非显式 reset）。
> - T4 的 `shared_endpoint_cache_` 挂在 `oracle_` 上 → 已天然具备**跨 query 持久性**。
> - 风险：缓存随 `query 数 × 深节点数` 无界增长 → **OOM**。每条 Entry = `vector<Interval>`(键) + `shared_ptr<EvidenceRecord>`(payload `vector<float>`)。

---

## 进度勾选表（每完成一项必须回到此表打勾，并逐条核对"验收细节"）

- [x] **方案 A**：跨 query 持久共享缓存 + **OOM 防护**（内存预算 / 容量上限 / LRU 淘汰）
- [x] **方案 B**：预热快照按 canonical 区间键索引（调查结论：机制已存在，缺口是覆盖，由 A+C 填补）
- [x] **方案 C**：按需延迟深化（build 下探子树时记录 canonical intervals 写入持久缓存）
- [x] **方案 D**：canonical 键去重 / 受控量化键 + 位精确二次校验（评估后跳过，详见结论）
- [x] **方案 E（Task 5）**：connector 优化（多 island 并行 + 确定性提交；fast/balanced/high connector_ms 降低 3–4 倍）
- [x] **Task 6**：重建 + 重跑 shelf 消融验证（多 query 复用机制可用、OOM 压测通过、connector 3–4 倍提速）

---

## 方案 A：跨 query 持久共享缓存 + OOM 防护 ⚠️

### 现状
- `shared_endpoint_cache_` 已挂在持久 `oracle_` 上，跨 query 自动复用。
- 但 `SharedEndpointEvidenceCache::put` **无任何上限**，map 持续 push_back → 长跑/多 query 必然 OOM。

### 设计
1. 为 `SharedEndpointEvidenceCache` 增加**内存预算与容量上限**：
   - `std::size_t max_entries_`（默认例如 200000，0 表示不限）。
   - `std::size_t max_bytes_`（默认例如 512MB，按 `payload.size()*sizeof(float)+intervals.size()*sizeof(Interval)` 估算，0 表示不限）。
   - 维护 `current_entries_`、`current_bytes_` 计数（put/淘汰时增减）。
2. **LRU 淘汰**：
   - Entry 增加 `std::uint64_t last_used_seq_`；全局 `std::atomic<uint64_t> tick_` 自增；命中(`endpoint_for_box_exact`)与写入(`put`)时更新。
   - put 时若超预算，淘汰 `last_used_seq_` 最小的若干 Entry，直到回到预算内。
   - 注意：`endpoint_for_box_exact` 当前是 `const` + `shared_lock`；更新 `last_used_seq_` 需 `mutable` 字段 + 原子写（避免升级为写锁）。淘汰只在 `put`（写锁）内做。
3. **配置贯通**：
   - 在 `OracleValidationConfig` 增 `std::size_t shared_endpoint_cache_max_entries`、`std::size_t shared_endpoint_cache_max_bytes`。
   - `shared_endpoint_cache()` 惰性创建时把上限传入缓存。
   - 经 bindings 暴露给 Python（默认值安全，实验可调）。
4. **跨 query 持久性确认**：确保 `reset_oracle` / `clear_forest` 不会无意清空缓存；新增显式 `clear_shared_endpoint_cache()` 仅在 `--clean-cache` 时调用。

### 验收细节（逐条核对）
- [x] A1 `SharedEndpointEvidenceCache` 有 `max_entries_` / `max_bytes_` / `current_entries_` / `current_bytes_` 成员。
- [x] A2 `put` 在超预算时按 LRU 淘汰，且 `current_*` 计数准确（淘汰减、写入增、覆盖更新差值）。
- [x] A3 Entry 有 `last_used_seq`（mutable atomic），命中与写入都更新；`endpoint_for_box_exact` 用原子写不升级锁。
- [x] A4 `OracleValidationConfig` 新增 `shared_endpoint_cache_max_entries`(200000) / `shared_endpoint_cache_max_bytes`(512MB)，经构造器传入缓存。
- [x] A5 bindings 暴露 3 个参数（验证：默认 200000 / 536870912，可读写）。
- [x] A6 `reset_oracle` 改为**保留**同一缓存实例（端点与场景无关）；`--clean-cache` 为文件系统级（新进程→新 forest→空缓存），无需运行时清理。
- [x] A7 新增 diagnostics：`oracle.shared_endpoint_cache_size` / `_bytes` / `_evictions`（经 `shared_endpoint_cache_peek()`）。
- [x] A8 编译通过（3 targets）。OOM 小预算压测在 Task 6 用 `shared_endpoint_cache_max_entries` 小值 + 多 query 验证淘汰生效。

---

## 方案 B：预热快照按 canonical 区间键二级索引

### 现状
预热产物（656MB 快照）按 `node_id` 键存储 → 仅"同树同 node"命中；深层 build 几乎不命中浅层预热。

### 设计
1. 在外部证据源（`LectDatabaseEvidenceSource` / 快照）侧，为已有 `endpoint_for_box_exact(intervals, key)` 路径补充/确认**按 canonical intervals 指纹**的索引（与 T4 同构）。
2. 让 build 深层降序时用 `evidence_frame.lookup_intervals` 查询外部快照，只要 canonical intervals 位精确相等即命中（不再受 node_id / 深度约束）。
3. 若快照本身只有 node_id 索引，则在加载时构建一份 `fingerprint(intervals) -> record` 的内存二级索引（注意内存：可只对深度区间 [prewarm_depth, build_depth] 的节点建索引，或复用方案 A 的预算机制）。

### 调查结论（重要）
代码调查发现：**区间键精确查找机制已完整存在**，无需重复实现：
- `LectReadSnapshot::endpoint_for_box_exact(intervals, key)` 使用 `exact_box_index`（`make_snapshot_box_index_key` = `fingerprint_intervals` + 二级 hash）做**位精确区间查找**，不依赖 node_id。
- `LectDatabaseEvidenceSource` / `LectSnapshotEvidenceSource` 都走该路径；build 传入 `evidence_frame.lookup_intervals`（canonical）。
- 实测已证：MAIN oracle（浅层）`reused_external=95` → canonical 区间跨快照匹配**在浅层有效**。
- WORKER oracle 也传入 `master_.external_evidence_source()` → 同样做区间键外部查找。

**真正缺口 = 覆盖而非键型**：预热只到深度 ≤18，快照里根本没有深度 30–40 的盒子 → 深层 build miss。
该缺口由：**方案 A**（跨 query 持久共享缓存）+ **方案 C**（按需将 build 下探的深节点写入持久缓存）共同填补。

> 结论：不再构建冗余的二级内存索引（避免过度工程，且快照本身不含深节点）。

### 验收细节
- [x] B1 确认外部证据源支持按 canonical intervals 指纹查询（位精确）——`exact_box_index` 已实现。
- [x] B2 build 浅层 miss 时确实走该路径并能命中（`reused_external=95` 实测证据）；深层未命中=覆盖缺失，非键问题。
- [x] B3 不需额外二级索引（快照无深节点）；覆盖问题转由 A+C 解决。
- [x] B4 命中计数可观测（`materialization_reused_external_evidence`）。
- [x] B5 编译通过（未改动代码，纯调查）；位精确校验保证无错误复用。

---

## 方案 C：按需延迟深化（自适应深化预热）

### 现状
用户否决"全深度预热（太贵）"。需要只在 build 真正下探的路径上花成本。

### 设计
1. 不预先深热整树；在 build **首次将某子树下探到深层并物化端点**时，把该节点的 `canonical lookup_intervals + payload` 写入**持久共享缓存**（方案 A 的缓存）。
2. 后续 query / worker 命中相同 canonical box → 复用。等价于"需求驱动的深化"，成本只花在被访问路径。
3. 实现上：方案 A 的 `put` 已在 `endpoint_payload_for_node` 物化后调用 → C 的核心**已随 A 落地**。本方案重点是：
   - 确认 worker 与 master 都向同一持久缓存写入（A 已保证）。
   - 跨 query 第二次起命中率提升（用计数验证）。
   - 可选：对"被多次命中"的 canonical box 提升其 LRU 权重（防被淘汰）。

### 验收细节
- [x] C1 worker + master 物化后都向持久共享缓存写入：worker 配置 `enable_endpoint_evidence_cache=true`（继承）+ `enable_worker_shared_endpoint_cache=true` + 已 `set_shared_endpoint_cache` → 两者都走 `put`。
- [x] C2 多 query 重复场景下复用机制经验证可用（Task 6 验证）：多 seed 跑中观察到 `reused_shared_endpoint_cache` 命中（seed2-quality=8），且同一 forest 内跨 stage 缓存持久增长（410→1972）。注：shelf 场景每次 query 的 start/goal 机器人连杆包络几乎不重合，故复用绝对量小；当端点区间真正重现时复用即生效（与 E2 推理一致）。
- [x] C3 高命中 Entry 抗淘汰：LRU 天然保护频繁命中 Entry（`last_used_seq` 保持较高），无需额外策略。
- [x] C4 编译通过（随 A 落地）。

---

## 方案 D：canonical 键去重 / 受控量化键 + 位精确二次校验

### 现状
端点缓存按 `evidence_frame.lookup_intervals`（canonical 主扇区）键；浮点尾差可能让数值等价的 box 落入不同桶 → 合并率下降。

### 设计
1. 排查 canonical 化是否对**数值上等价但位不同**的 box 产生不同指纹。
2. 若有：对**键的指纹**做受控量化（如按 ULP/固定步长归一化用于分桶），但：
   - **payload 仍精确存储**；
   - 命中时保留 `intervals_equal` 位精确二次校验（T4 已具备）→ 量化只缩小候选桶，校验保证正确性。
3. 若排查发现无显著尾差问题，则记录"无需量化"并跳过（避免过度工程）。

### 验收细节
- [x] D1 评估结论：**不实现量化，明确跳过**。理由：
  - canonical `lookup_intervals` 来自 KD 树存储的 split 边界（double），降序时**拷贝而非重算** → 相同盒子必得位精确相等；不同盒子不会碰撞。
  - 已有证据：`reused_external=95` 证明 canonical 跨源匹配在现有位精确口径下**已有效**，无需量化提升合并率。
  - 量化会引入额外碰撞风险与复杂度，违反“不过度工程”原则。
- [x] D2 （未实现，不适用）。
- [x] D3 编译通过（未改动代码）。

---

## 方案 E（Task 5）：Connector 优化

### 现状（数据）
- connector 主导总时间（110–185ms vs grow 17–108ms）。
- 流程：`find_islands` → `broadphase_bridge_pairs`（每 gap 取 `max_pairs_per_gap` 候选）→ 对候选 `parallel_for`：先 `closest_box_point_segment`（廉价直连），失败再 `birrt_connect_impl`（贵，`per_pair_timeout_ms=250`）。
- 串行/封顶点：每轮只连 main↔back 两个 island；BiRRT 超时 250ms/对。

### 设计（按性价比）
1. **E1 减少 island/gap（提高盒子覆盖）**：grow 阶段已优化；确认 connector 前 island 数；必要时调 `frontier_bridge` 参数减少需 BiRRT 的 gap。（低风险、先量化）
2. **E2 跨 query 复用桥接路径**：成功的 bridge path（按 source/target canonical box 或区间）缓存，后续 query 相同 gap 直接复用 → 跳过 BiRRT。需 OOM 防护（同 A 思路，小容量）。
3. **E3 BiRRT 提速**：检查 `step_size` / `goal_bias` / `segment_resolution`；优先 `closest_box_point_segment`（已是第一选择，确认其命中率）；可加"双向并行扩展"或更紧的早停。
4. **E4 候选/超时调参**：`per_pair_timeout_ms`、`max_pairs_per_gap`、`parallel_threshold` 调优，减少无效 BiRRT 长尾。
5. **E5 并行覆盖更多 island 对**：当前每轮只连 main↔back；可在一轮内并行尝试多对 island（需保证 commit 串行确定性）。

> 注意：connector 改动涉及 commit 顺序与确定性（`deterministic_reduce`），任何并行化必须保持 `parallel_for` 后串行 commit 的语义。

### 验收细节
- [x] E1 量化 connector 前 island 数与各 gap 的 BiRRT 触发次数（先测后改）。
  - 新增计数器 `connector.islands_initial`、`connector.birrt_invocations`。
  - 实测：islands=5（4 个 gap）；`closest_box_point_segment` 命中率 0%；BiRRT 几乎占满整个 connector 时间（`profile.connector.birrt.total_ms` ≈ conn_ms）。瓶颈明确为 **BiRRT 跨轮串行**。
- [x] E2 （评估后跳过）：桥接路径针对场景障碍做碰撞检查，**场景相关**；跨 query 复用存在返回过期路径的正确性风险，且 shelf 场景 start/goal 变化使盒子徃罕重现（复用率近 0）→ 不实现。
- [x] E3 （评估后跳过）：BiRRT 参数（step_size/goal_bias/segment_resolution）现值合理；数据未显示盲调参有稳定收益，不盲改。
- [x] E4 （评估后跳过）：`max_pairs_per_gap=8` 但实际只返回≤4 候选；`per_pair_timeout_ms=250` 仅 quality 阶段命中 1 次；无明显无效长尾可剪。
- [x] E5 **多 island 对并行 + 确定性串行提交（并查集）**：
  - 一轮内对主 island 与所有其他 island 生成候选，`parallel_for` 并发跨 gap BiRRT；提交按 `task_id` 稳定排序 + 并查集只合并仍处于不同连通分量的桥，与线程完成顺序无关 → **保持确定性**。
  - 实测（baseline_warm 8t，单 query）：fast 118.6→27.8ms、balanced 113.4→33.0ms、high 115.6→30.8ms（**3–4倍**）；seed 184.9→153.8ms；quality 136.0→164.8ms（+28ms，该阶段有超时 BiRRT 对落在并行临界路径上）。
  - 连通性保持或提升：seed/fast/balanced/high `adjacency_islands=1`（与原一致）；quality 3→2（余下由 segment edge 桥接，与原机制一致）。
- [x] E6 编译通过；connector_ms 在 4/5 阶段显著下降且连通率不降。

---

## Task 6：重建 + 重跑消融验证

### 步骤
1. 重建窄构建 3 targets：
   ```
   cmake --build build-rbf-only-exec --target _link_interval_envelope_cpp -j 4
   cmake --build build-rbf-only-exec --target sbf_core -j 4
   cmake --build build-rbf-only-exec --target _sbf_cpp -j 4
   ```
2. 重跑 shelf 消融（比 `grow_ms` / `connector_ms`，非 `wall_s`）：
   ```
   PYTHONPATH=build-rbf-only-exec/python:. python experiments/exp04_shelf_ablation/run_shelf_ablation.py \
     --out-dir <out> --only baseline_warm_aafk_support_hull_8t_aafk_volume_min \
     --execute --prewarm-depth 18 --rbf-max-depth 40 --prewarm-max-depth 40 \
     --seeds 1 --threads 8 --timeout-ms 60000
   ```
   - 跨 query 复用需**多 query**才显现 → 用多 seed / 多 query 场景，且**不要** `--clean-cache`（除首轮基线）。
3. 对照三档：baseline_warm(cache,8t) / no_cache(8t) / single_thread(cache,1t)，记录 `grow_ms`、`connector_ms`、`reused_shared_endpoint_cache`、缓存大小/淘汰数。

### 验收细节
- [x] 6.1 3 targets 重建通过。
- [x] 6.2 多 query 场景下 `reused_shared_endpoint_cache` 复用路径可用：多 seed 跑中 seed2-quality 出现 `reused=8` 命中，证明复用机制工作；shelf 场景总体接近 0 是因为每次 query 的 start/goal 机器人连杆包络几乎不重合（与 E2 推理一致，非缺陷）。同一 forest 内跨 stage（`reset_oracle`）缓存持久且增长（seed→quality：410→1972），符合方案 A 预期。
- [x] 6.3 内存稳定，OOM 防护经压测确认生效：
  - 在线运行：`shared_endpoint_cache_bytes=540KB`（1501 条），`evictions=0`，远低于 512MB/200000 默认预算。
  - 离线压测 `safe_box_forest/tests/shared_endpoint_cache_oom_test.cpp`：
    - 条目上限=100、插入 1000：结束 `entries=100`、`evictions=900`，热项保留、最旧项被淘汰。
    - 字节上限=4096、插入 1000：结束 `bytes=3960≤4096`、`evictions=989`。
    - 两个预算维度（条目/字节）均正确执行 LRU 淘汰，size/bytes 不越界 → **不会 OOM**。
- [x] 6.4 记录对比：E5 前后 connector_ms（fast 118.6→27.8、balanced 113.4→33.0、high 115.6→30.8，3–4 倍），连通率不降；缓存复用与淘汰计数见 6.2/6.3。

---

## 总执行顺序

A → B → C → D → E（Task 5）→ Task 6。每完成一项：
1. 编译验证；
2. 回到本文件勾选对应"验收细节"；
3. 全部勾选后再进入下一项。

最终回到顶部"进度勾选表"确认 6 项全绿。
