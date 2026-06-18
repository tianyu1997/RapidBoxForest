以下评审意见按 **IEEE Transactions on Robotics（TRO）审稿标准**来写，重点关注：原创性、技术严谨性、实验可信度、可复现性、论述清晰度，以及后续修改优先级。本文档依据你上传的 `sbf_tro_2026.pdf` 评审。

# 总体评价

这篇论文提出 **RapidBoxForest（RBF）**：一种面向多查询机械臂规划的、基于 link-envelope 的 C-space box forest 方法。论文的核心想法是将连续碰撞检测中已有的 endpoint-bounding / swept-capsule 包络思想，转化为 **关节空间盒子验证 oracle**；再通过盒子增长、森林图、typed corridor graph 和 Lifelong Envelope Cache Tree（LECT）实现可复用的多查询规划结构。

论文的方向是有价值的。它不是试图替代 RRTConnect、BIT* 或 IRIS/GCS，而是定位在“重复查询、场景局部变化、低构建成本”的中间区域。这个问题设置对工业 workcell、bin-picking、shelf manipulation、重复 pick-and-place 等场景确实有意义。论文也比较诚实地限定了 novelty boundary：endpoint-to-link envelope 不是新的几何恒等式，贡献在于把该 pipeline 用作 C-space free-region construction primitive，并组织为可查询、可更新的 box-region graph。

但是，以 TRO 标准来看，当前稿件还存在比较明显的问题：**理论保证偏弱、保守模式与性能模式的界限仍然不够清楚、实验公平性和可复现性需要加强、对 cache 成本和预计算成本的报告不足、动态更新实验还不是端到端规划实验**。我会建议 **Major Revision**。如果按更严格的 TRO 一审标准，也可以理解为 “Reject and Resubmit after substantial revision”，但我认为文章有潜力，经过系统性补强后可以达到期刊论文水平。

# 建议审稿结论

**Recommendation: Major Revision**

**主要理由：**
方法有明确工程价值和一定系统创新性，但目前的证据还不足以支撑 TRO 级别的强结论。尤其是 RBF 的优势很大程度依赖 LECT warm cache、特定 profile、query repair、segment witness 和非完全保守的 filtering pipeline；这些机制在实验中混合出现，使读者难以判断：到底是 box-envelope 方法本身带来了优势，还是缓存、profile tuning、在线修补、baseline 设置共同造成了结果。

---

# 论文优点

## 1. 问题定位清楚，有实际意义

论文明确瞄准 repeated-query manipulation：机器人运动学固定、障碍环境大体固定或局部变化、需要反复求解 start-goal 查询。这类场景下，单次 RRTConnect 的低 latency 并不总是最优，重型 IRIS/GCS 的构建成本又可能过高。RBF 提出的 “cheap reusable volumetric structure” 是合理的中间设计点。

## 2. 对 novelty boundary 的表述相对诚实

文章没有声称 swept capsule endpoint bound 本身是新理论，而是承认其来自 CCD / articulated-body collision detection 文献，并强调新的贡献是将其用在 **joint-box validation** 和 **reusable region graph construction** 中。这一点比很多规划论文更稳健。

## 3. 系统设计较完整

论文不仅有 link interval envelope，还设计了：

* LECT：kinematics-keyed endpoint/link-envelope evidence cache；
* scene-level RBF cache：validated boxes + typed corridor graph；
* adaptive leaf sweep；
* restricted refinement；
* forest consolidation；
* typed graph query；
* local obstacle insertion/removal maintenance。

这些组件形成了一个完整的规划系统，而不是单一 oracle。

## 4. 保证条件写得比较谨慎

Theorem 1 明确声明：只有路径保持在 conservatively validated box union 内时，才继承 collision-free certificate。论文也承认 repaired、post-processed 或 performance-mode path 需要 query validation。这种条件式 soundness 比无条件宣称“安全”更严谨。

## 5. 实验覆盖了多个层次

实验包括 Shelf+IIWA ablation、跨算法对比、random multi-robot scenes、endpoint/link envelope mechanism study、LECT cache cost、dynamic updates。结构上比单一 benchmark 更完整。

---

# 主要问题与修改建议

## Major Comment 1：保守证书模式与性能模式仍然混杂，需要彻底分离

论文多次区分 “conservative certificate” 与 “performance-mode filter”，但实验中使用的注册 profile 包含 SupportHull、AABB broadphase、d23 external endpoint evidence、query bridge repair、residual segment witnesses、final simplification、strict audit 等多个环节。读者很难判断表 II–IV 中的 RBF 成功到底来自：

1. 完全保守的 box-union corridor；
2. performance-mode boxes + final validation；
3. segment witnesses；
4. online bridge repair；
5. OMPL simplification；
6. 或者这些因素的组合。

**建议修改：**

增加一张核心表，强制把每个实验结果分成以下类别：

| Result category | Conservative boxes only | Eε used? | Es segment witness used? | Eπ portal used? | Performance-mode filter? | Post-processing leaves box union? | Final audit required? |
| --------------- | ----------------------: | -------: | -----------------------: | --------------: | -----------------------: | --------------------------------: | --------------------: |

并在所有主实验表中增加至少三列：

* **Box-contained success rate**：路径完全在 conservative box union 内的成功率；
* **Segment-witness fraction**：当前已有 Seg.，但建议明确 Es 使用次数、长度比例、是否经过 0.01 rad audit；
* **Repair-dependent success rate**：如果禁用 online repair / segment bridge，成功率是多少。

现在论文中提到 theorem 只覆盖 conservative box-contained corridor，但主实验指标主要是 final validated success。这会让理论保证和实验成功之间有 gap。TRO 审稿人会重点质疑这一点。

---

## Major Comment 2：LECT warm cache 的预计算成本没有被充分计入，导致 build-time 结论可能偏乐观

表 II 中 RBF-SH d23 build time 为 0.070–0.071 s，而 No external LECT/HIFK-5 build time 为 7.428–7.663 s。这说明注册 profile 的低构建时间高度依赖外部 d23 LECT evidence replay。论文也承认了这一点，但目前对 d23 cache 的生成成本、存储空间、加载成本、适用范围、cache hit rate、跨场景 amortization 没有充分报告。

**这是非常关键的问题。**
如果一个 planner 的 0.07 s build 实际上依赖一个昂贵的预计算数据库，那么必须回答：

* d23 LECT snapshot 是如何生成的？
* 总生成时间是多少？
* 存储占用是多少？
* 每个机器人是否都需要单独 cache？
* 换一个 URDF、link radius、active link set、endpoint source、split policy 后 cache 是否失效？
* local obstacle update 是否仍需要同样的 cache？
* 使用多少个 scenes / queries 后，cache 预计算才 amortize 到比 PRM 或 IRIS/GCS 有优势？
* cache loading 是否计入 build？
* cache miss 时性能如何退化？

**建议新增一节：Cache economics / amortization analysis。**
给出如下表格：

| Robot | LECT depth | Nodes | Evidence records | Disk size | Memory after load | Precompute time | Snapshot load time | Build time warm | Build time cold | Break-even queries/scenes |
| ----- | ---------: | ----: | ---------------: | --------: | ----------------: | --------------: | -----------------: | --------------: | --------------: | ------------------------: |

此外，主结果应至少同时报告：

* **RBF-cold**：无外部 LECT；
* **RBF-warm**：有 LECT，但计入 cache loading；
* **RBF-precomputed-amortized**：把 cache 预计算按 N scenes 或 N queries 摊销。

否则 “0.070 s reusable build” 容易被审稿人认为是不公平的 warm-start 数字。

---

## Major Comment 3：与 PRM、RRTConnect、BIT*、IRIS/GCS 的比较公平性需要加强

当前跨算法对比的信息很丰富，但存在几个潜在不公平点。

### 3.1 RBF 使用 8-thread，OMPL baseline 似乎主要是 single-query standard interface

论文说明 RBF timing runs 使用 eight-thread budget，而 OMPL baselines 通过 standard single-query interfaces 运行，除非 planner 暴露内部并行。这样在 build-time 和 online-time 上不完全公平。TRO 审稿人通常会要求：

* 单线程 RBF vs 单线程 baselines；
* 多线程 RBF vs 可并行 baselines；
* 或者至少报告 CPU time 而不只是 wall-clock time。

**建议：** 增加 single-thread RBF 表，或者报告 total CPU time / wall time 两种指标。

### 3.2 RBF profile 是经过 Shelf+IIWA ablation 选择的，但 baseline 是否同样 tuning 不清楚

RBF 有 b100、d23、leaf10/FFB56/endpoint110 等 profile tuning。PRM、RRTConnect、BIT* 的参数虽然给了一些，但不清楚是否做了同等强度的 grid search / budget sweep。特别是 PRM 的 build time 与 roadmap quality 对参数非常敏感。

**建议：**
为每个 baseline 给出 budget-quality curves，而不只是选定点。Figure 4 和 Figure 5 已经有部分曲线，但需要更明确说明：

* 每个 planner 的调参搜索空间；
* 选择规则是否完全一致；
* 是否使用相同的 path simplification budget；
* 是否所有 planner 都按同样的 success / audit 规则筛选。

### 3.3 BIT* / PRM 的 checkpoint selection 可能偏向路径质量而非真实在线策略

表 IV 说明 PRM 和 BIT* 是对每个 query/seed 选择 “fastest strict-audited checkpoint within 1.08× of best path”。这个选择是事后 oracle 式的：真实在线运行时通常不知道哪个 checkpoint 会在 1.08× 内。虽然这可能对 baseline 有利，但也使比较语义复杂。

**建议：**
同时报告两种版本：

* **Online-policy version**：固定 timeout 或固定 early stopping policy；
* **Oracle-selected checkpoint version**：当前方式，作为 upper-bound/context。

这样结果更清楚。

### 3.4 IRIS-NP+GCS 只在 Shelf 中出现，random scenes 中缺席

论文解释 IRIS-NP+GCS 在 random q10x10 中没有可靠 strict-audit success。但这会让 “convex region planner comparison” 偏弱。TRO 读者会希望看到更强或更现代的 convex-region baseline，至少说明失败原因：

* 是构建超时？
* 是 region generation 失败？
* 是 GCS infeasible？
* 是 path audit 不通过？
* 是实现限制？

**建议：**
增加 IRIS/GCS 失败统计表，哪怕不放主结果，也应放 appendix。否则审稿人会认为比较不完整。

---

## Major Comment 4：动态更新实验不是端到端规划实验，当前结论应收窄或补实验

表 VIII 报告 insertion update 0.031–0.050 s、removal update 0.034–0.071 s，并相对 warm rebuild 有 24.5–55.9× speedup。但论文也明确说这是 adaptive leaf sweep only，没有 query bridge、connector、OMPL simplification 或 final path audit。

这只能证明 **scene-cache maintenance 的局部操作快**，不能证明 **动态环境中的规划 query latency / success / path quality** 也同样好。

**建议补充端到端动态更新实验：**

设计如下 protocol：

1. 固定 robot + scene + 10 queries；
2. build initial RBF forest；
3. 插入障碍；
4. local update；
5. 对所有 queries 重新规划；
6. strict audit；
7. 与 full rebuild + query、PRM rebuild + query、RRTConnect one-shot query 对比；
8. 再删除障碍重复以上流程。

报告：

| Method | Update/build time | Query success | Online/q | Total update+query | L/L* | Box-contained success | Repair fraction |
| ------ | ----------------: | ------------: | -------: | -----------------: | ---: | --------------------: | --------------: |

如果暂时无法补充，应把当前动态更新结论改为更保守的表述：
“RBF accelerates isolated adaptive leaf-sweep maintenance”，而不是泛化为 “dynamic obstacle update planner speedup”。

---

## Major Comment 5：缺少 completeness / coverage 性质讨论

文章目前只有 conditional soundness theorem。对于 planner，TRO 审稿人会问：

* 方法是否 resolution complete？
* 如果 Dmax → ∞、budget → ∞，是否能覆盖所有有 clearance 的路径？
* Axis-aligned boxes + conservative envelopes 会不会因为 wrapping / over-approximation 永远无法验证某些可行窄通道？
* 如果 endpoint envelope 是保守但过宽的，是否存在 feasible region 但所有包含它的 canonical boxes 都被判 collision？
* segment witness 是否恢复 probabilistic completeness？

论文可以不证明完整的 probabilistic completeness，但需要明确理论位置。

**建议新增一个 subsection：Completeness and failure modes。**

至少给出如下结论：

* RBF conservative box forest 在有限预算下不 complete；
* 在固定 envelope source 和固定 axis-aligned split policy 下，即使无限预算，如果最小 cell 尺度趋近 0 且 FK/envelope bound 收敛，则对具有正 clearance 的路径可望 resolution-complete；
* 但若存在 non-vanishing over-approximation、非保守 performance filter、segment bridge 限制、或最大深度限制，则不保证；
* 当前实现依赖 online repair / segment witness 来补偿覆盖不足。

这会显著增强论文的理论透明度。

---

## Major Comment 6：self-collision、attached object、任务约束被排除，限制了 manipulation 说服力

Assumption 1 说明 formal certificates 只涉及 active rounded links 与 workspace obstacles；self-collision、attached objects、task-specific forbidden regions 只有启用时才包含。这在理论上可以接受，但对 TRO manipulation paper 来说，self-collision 和 grasped object collision 非常重要。

**建议：**

至少补充一个实验或消融：

* 开启 self-collision 检查后 RBF build / query / success 如何变化；
* 携带简单 attached box / cylinder object 时 envelope 如何处理；
* 如果当前方法不支持 attached object 的保守 envelope，应明确列为 limitation，而不是只在 assumption 中轻轻带过。

否则审稿人可能认为实验环境过于简化，尤其是 Shelf+IIWA 应用中，真实 shelf manipulation 往往涉及末端执行器和物体。

---

## Major Comment 7：方法细节太多但关键实现仍不够可复现

论文包含大量参数：Dmax、Dskip、d23、b100、leaf10、FFB56、endpoint110、skip-to-depth 32、paving depth 52、bridge attempts 12、bridge-RRT 1600 iterations、sample step 0.08 rad、repair cap 24、AABB broadphase、SupportHull GJK 等。
问题不是参数多，而是读者难以知道哪些是必要的，哪些是经验调参结果，哪些对性能最敏感。

**建议：**

增加一个 “Default parameter table”：

| Parameter | Value in Shelf | Value in random scenes | Meaning | Sensitivity | Chosen by |
| --------- | -------------: | ---------------------: | ------- | ----------- | --------- |

并增加 sensitivity plot：

* Dmax / LECT depth vs build time / success；
* box budget vs success / query time；
* repair cap vs success / segment fraction；
* SupportHull vs LinkAABB under same cache；
* cache depth vs storage / replay speed / build speed。

目前表 II–VII 已经有机制研究，但没有把关键 profile 参数系统化。

---

# 次要问题与具体文字修改建议

## Minor Comment 1：摘要信息密度过高

摘要中塞入了方法、cache、formal claim、多个实验数值。建议压缩，突出 3 点：

1. endpoint-to-link envelope as C-space box-validation oracle；
2. box forest + typed corridor graph for repeated queries；
3. low build/update cost demonstrated under clearly stated validation policy。

现在摘要过长，读起来像 extended abstract。

## Minor Comment 2：贡献列表需要按 “novelty / system / evaluation” 重排

当前贡献列表有 5 条，但部分内容是工程实现或实验 protocol。建议改成：

1. Geometric primitive reuse：joint-box link-envelope validation；
2. Planning system：box forest + typed corridor graph；
3. Reuse architecture：scene cache vs LECT evidence cache；
4. Local maintenance：deletion promotion / insertion refill；
5. Evaluation protocol：saved catalogs + audit-separated timing。

并明确哪些是理论贡献，哪些是系统贡献。

## Minor Comment 3：术语需要统一

文中同时使用：

* box；
* cell；
* domain；
* validated box；
* certified-free cell；
* performance-mode box；
* collision domain；
* terminal cell；
* scene-validation policy；
* query-validation path。

虽然 V 节中有定义，但全文读起来仍容易混淆。建议增加一张 terminology table，放在 Section III 或 V 开头。

## Minor Comment 4：图 1 和图 2 信息量大但可读性一般

图 1 展示 pipeline，但字体较小，视觉元素多。建议拆成两个图：

* Fig. 1：几何 envelope pipeline；
* Fig. 2：LECT + forest query/update pipeline。

当前 Fig. 2 的 C-space / workspace update illustration 很有帮助，但应在 caption 或正文中说明它只是 2-link didactic example，不代表高维实验。

## Minor Comment 5：表格只给 IQR 不给 median，不够直观

论文反复说明 numeric entries report only [Q1,Q3] interval。IQR 有用，但读者仍需要 median。建议改成：

`median [Q1, Q3]`

例如：

`0.071 [0.070, 0.071]`

这会让表 II–IV 更易读。

## Minor Comment 6：路径质量指标 L/L* 需要更清晰

L/L* 的 reference 有时是 query-level globally shortest strict-audited path，有时是 scenario-level reference。建议统一，或在表中单独标记：

* query-level reference；
* scenario-level imported reference；
* success-only aggregation。

否则不同表之间的 L/L* 可比性有限。

## Minor Comment 7：严格审计 validation time 不计入 planning time，需更透明

论文说 validation time reported separately and not charged to planning time。但主表没有总是显示 validation time。对于 safety-critical planner，final audit 是实际系统成本的一部分。建议主表增加：

* planning time；
* final simplification time；
* strict audit time；
* total returned-path time。

至少 appendix 应完整给出。

---

# 建议增加的关键实验

我认为以下实验最能提高论文通过 TRO 的概率。

## 实验 A：Conservative-only RBF vs full RBF

目的：明确理论 guarantee 对实际成功率的贡献。

配置：

1. conservative boxes only，禁用 Eε / Es / online repair / performance filters；
2. conservative boxes + Eπ portal；
3. full RBF with repair and segment witnesses；
4. full RBF but no post-processing。

报告 success、box-contained success、online time、path length、segment fraction。

## 实验 B：Cache amortization curve

横轴：number of scenes 或 number of queries。
纵轴：amortized total time，包括 LECT precompute。

比较：

* RBF cold；
* RBF warm without charging precompute；
* RBF warm with precompute amortized；
* PRM；
* RRTConnect repeated one-shot；
* BIT* repeated one-shot；
* IRIS/GCS if available。

这个实验非常关键，因为 RBF 的核心卖点是 repeated-query reuse。

## 实验 C：End-to-end dynamic update

当前动态更新只测 leaf sweep。建议补上 update 后的 actual query solve 和 strict audit。

## 实验 D：Realistic manipulation constraints

至少选择一个：

* self-collision enabled；
* attached object；
* narrow shelf with gripper；
* cluttered workcell；
* real robot execution or high-fidelity simulator replay。

TRO 对 manipulation 论文通常希望看到不只是 abstract C-space planning benchmark。

## 实验 E：Parameter sensitivity

最少报告：

* LECT depth；
* box budget；
* support-hull broadphase on/off；
* bridge attempts；
* repair cap；
* obstacle density。

这能证明方法不是高度依赖某个 hand-tuned profile。

---

# 对理论部分的具体修改建议

## 1. Proposition 1 与 Corollary 1 基本成立，但要补数值鲁棒性说明

需要说明：

* interval arithmetic / affine arithmetic 的 rounding mode；
* floating-point outward rounding 是否使用；
* obstacle AABB / mesh collision 的 conservative distance tolerance；
* SupportHull GJK 的 termination tolerance 是否会破坏保守性；
* B∞ padding 与 Euclidean ball padding之间的 conservatism 关系。

否则“conservative certificate”在实现层面可能不严格。

## 2. Theorem 1 太弱但诚实，可以保留；建议补一个 resolution-style proposition

例如：

在满足以下条件时：

* joint boxes recursively refine with diameter → 0；
* endpoint envelope over-approximation converges to exact sweep as box diameter → 0；
* obstacle set closed；
* path has positive clearance δ；
* scene-validation uses conservative envelopes；

则存在足够深度使该 path 可被 finite chain of validated boxes 覆盖。

这个命题不一定需要很强，但能帮助读者理解 RBF 的 failure modes。

## 3. Typed edges 的语义建议形式化

当前 E∩、Eε、Es、Eπ 的解释是文字化的。建议给出一个表：

| Edge type | Construction condition | Certificate inherited? | Requires query validation? | Used in theorem? |
| --------- | ---------------------- | ---------------------: | -------------------------: | ---------------: |

这样能避免理论与实现之间的歧义。

---

# 对实验表述的修改建议

## 表 II

当前表 II 很有用，但建议补：

* cache precompute excluded/included；
* exact cache size；
* box-contained success；
* number of boxes；
* segment witness count；
* strict audit time。

尤其 RBF-SH d23 与 No external LECT/HIFK-5 的 build 差距巨大，必须更突出地说明 warm cache 的代价。

## 表 III

建议把 RBF 的 build amortized total time 也直接放进表中。例如对于每个 query count K：

`total/K = build/K + online + simplification + audit`

这样读者能直接判断 repeated-query 下的优势，而不是在 Figure 4 中推断。

## 表 IV

Random scenes 的 RBF 在 Panda-Hard online IQR 到 0.649 s，且 path ratio 到 1.28。建议讨论为什么 Panda-Hard 更困难：

* geometry?
* split policy?
* obstacle distribution?
* endpoint fallback?
* repair fraction?
* SupportHull narrowphase cost?

这部分可以成为 limitation analysis。

## 表 V

Analytical 在 width 0.50 的 negative gap 不是 0，而是 -4.5e-3，但 Safe 标为 Y。需要解释这个 apparent contradiction：是 sampling-union reference 不完全可靠？还是 numerical tolerance？还是 analytical bound 相对某个 reference 有负 gap 但仍理论 conservative？这个表可能会被审稿人抓住。

## 表 VI

SupportHull 在 width 0.30 和 0.50 collision time 显著上升，分别到 4.823 µs 和 10.950 µs，而 LinkIAABB 仍约 0.3 µs。需要更明确说明 broadphase 如何避免这个问题，以及在主实验中 SupportHull narrow phase 被调用的比例。

## 表 VII

LECT snapshot 有 2,097,151 nodes/evidence，但没有 disk/memory size。必须补充，否则 cache 可行性不完整。

## 表 VIII

建议标题明确为：

“Adaptive leaf-sweep maintenance only, not end-to-end replanning.”

正文中也应避免把该结果表述成完整 dynamic planning speedup。

---

# 可能的审稿人质疑与建议回应方式

## 质疑 1：RBF 是否只是 adaptive cell decomposition + cached collision bounds？

建议回应：
承认 RBF 继承 adaptive cell decomposition，但强调具体新组合：

* endpoint-bound pipeline 被提升为 joint-box oracle；
* link-envelope representations 可切换；
* LECT 将 kinematics evidence 与 scene labels 分离；
* typed graph 明确区分 overlap/candidate/segment/portal semantics；
* local update 使用 blocker metadata 做 maintenance。

但仅靠文字不够，需要实验显示这些组件分别必要。

## 质疑 2：为什么不用 PRM with lazy collision cache？

建议补实验：
PRM + lazy edge cache + obstacle update revalidation。
对比：

* memory；
* update cost；
* query time；
* path quality；
* repeated query amortization。

RBF 的卖点是 volume reuse，不是 edge reuse。最好用实验证明 volume reuse 在局部障碍变化时更有优势。

## 质疑 3：axis-aligned boxes 在高维 C-space 中会不会太保守？

建议补分析：

* accepted box volume distribution；
* depth distribution；
* coverage estimate；
* rejected boxes 原因统计；
* envelope over-approximation vs obstacle collision 的贡献分解；
* high-DOF scaling，至少 7-DOF、6-DOF、7-DOF 已有，但需要更明确 scaling。

## 质疑 4：结果是否依赖特定 Shelf queries？

建议在 Shelf 中增加更多 random start-goal pairs，或至少说明 canonical five queries 的来源与难度分布。

---

# 建议的修改优先级

## 第一优先级：必须修改

1. 明确区分 conservative certificate success 与 final-validation success。
2. 报告 LECT precompute time、disk/memory size、cache hit rate、cold/warm/amortized results。
3. 增加 end-to-end dynamic update planning experiment，或显著收窄动态更新 claim。
4. 提高 baseline fairness：单线程/多线程、参数 tuning、fixed online policy、full timing decomposition。
5. 增加 box-contained success、repair fraction、segment witness usage、strict audit time。

## 第二优先级：强烈建议

1. 增加 completeness / resolution discussion。
2. 增加 self-collision 或 attached-object 处理实验/说明。
3. 增加 parameter sensitivity。
4. 改进表格：median [Q1,Q3]，补全 memory/storage/box count。
5. 重画 Fig. 1，简化 pipeline 图。

## 第三优先级：文字和呈现

1. 缩短摘要。
2. 重排贡献列表。
3. 增加术语表。
4. 将 appendix 与主文结果的选择规则写得更透明。
5. 统一 L/L* 定义和 timing semantics。

---

# 建议给作者的总体修改路线

我建议把论文重构成以下逻辑：

1. **Problem and scope**：RBF 是 repeated-query reusable region planner，不声称 single-query 全面优于 RRTConnect/BIT*。
2. **Geometric oracle**：endpoint envelope → link envelope → conservative box validation；明确实现 conservatism。
3. **Reuse architecture**：scene cache 与 LECT cache 分离；报告 cache economics。
4. **Planner**：box forest、typed graph、query semantics；把 E∩ / Eε / Es / Eπ 证书边界讲清楚。
5. **Theory**：conditional soundness + failure/completeness discussion。
6. **Experiments**：

   * conservative-only vs full pipeline；
   * warm/cold/amortized cache；
   * fair baseline budget curves；
   * end-to-end updates；
   * mechanism studies。
7. **Limitations**：axis-aligned conservatism、cache dependency、finite-budget incompleteness、self-collision/attached object、one-shot planner优势场景。

---

# 最终判断

这篇论文有一个值得发展的系统性想法，也有不少实验数据支撑其在 repeated-query 场景中的潜力。但以 IEEE TRO 标准，当前版本还需要更强的 **证书语义分离、cache 成本核算、公平 baseline 对比和端到端动态更新验证**。我会给出：

**Decision: Major Revision**

修改充分后，论文的定位可以从“一个看起来很快的缓存式 box planner”提升为“一个有清晰理论边界、工程成本核算完整、适合 repeated-query manipulation 的 reusable region-planning framework”。
