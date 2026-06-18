下面我按“可以直接落地到你当前稿件和代码结构”的方式展开。行号是我对你上传的 `sbf_tro_2026.tex` 源码看到的对应位置，主要参考了当前 `LECT` 设计、`FindFreeBox`、`LeafSweepRefine`、typed graph 和 conservative theorem 的结构。

你现在稿件里已经有很好的理论边界：LECT 只存 scene-independent endpoint/link-envelope evidence，scene-specific obstacle tests 和 labels 仍在 forest 层做；这一点在 LECT 设计里已经写得很清楚，尤其是 evidence/label separation。当前 `FindFreeBox` 也已经是 seed-path descent，并返回 seed path 上第一个通过 scene-validation 的 ancestor；但 leaf sweep 部分仍然枚举固定深度 virtual leaves，再逐个分类，这正是可以压缩的地方。对应源码位置大致是：LECT heap-style binary KD tree 与 depth cap 在 L721–L729，depth-synchronous split policy 在 L740–L759，`FindFreeBox` 在 L928–L974，fixed virtual leaf sweep 在 L982–L1046，typed graph 在 L1194–L1228，conservative corridor theorem 在 L1246–L1264。

---

# 总体目标：把“全层枚举”改成“自适应终端 cell 集合”

当前 leaf sweep 逻辑本质是：

[
\forall C\in \mathcal L_{d_0:d_1},\quad \text{validate }C
]

即使一个粗 parent 已经能被证明 free，也仍然会枚举它下面所有 virtual leaves。这会把本来一个 box certificate 就能表达的 free region 展开成很多 leaf boxes。

建议改成：

[
\mathcal T_{\mathrm{terminal}}
==============================

\mathcal F_{\max}
\cup
\mathcal C_{\mathrm{occ}}
\cup
\mathcal C_{\mathrm{defer}}
\cup
\mathcal C_{\mathrm{cap}}
]

其中每个 terminal cell 是以下之一：

[
\texttt{FREE},\quad
\texttt{CERT_OCCUPIED},\quad
\texttt{INCONCLUSIVE_DOMAIN},\quad
\texttt{DEFERRED},\quad
\texttt{COVERED}.
]

也就是说，leaf sweep 不再追求“到某个固定深度的一整层叶子”，而是追求“足够回答当前 build/query/update 目标的一组最大化终端 cells”。这不会改变你的 certificate：凡是进入 forest 的 box 仍然必须通过原来的 scene-validation policy。

---

# 1. Early-stop adaptive sweep：替换 fixed-depth virtual leaf sweep

## 1.1 当前问题定位

你现在的 `FindFreeBox` 设计是合理的：它沿 seed 所在路径下降，materialize endpoint/link envelopes，遇到通过 validation 的 ancestor 就返回。这正是 coarse acceptance 的基础。

但 `LeafSweepRefine` 里目前是：

```text
FORALL virtual leaves C in L_{d0:d1}:
    if C certified free:
        insert C
    else:
        record C as collision domain
```

这会导致一个大 free parent 仍然被分裂成大量 children。尤其是当 (d_1) 增加时，成本是 layer cardinality 驱动的，而不是 certificate 数量驱动的。

要改成：

```text
AdaptiveClassify(I):
    materialize evidence for I
    if I is already covered by accepted boxes:
        prune
    if I passes scene validation:
        accept I as a maximal free box
        prune descendants
    if I has sound occupied certificate:
        record occupied domain
        prune descendants
    if depth cap reached:
        record inconclusive collision domain
        stop
    if irrelevant or dominated:
        defer I
        stop
    choose split dimension
    recurse on selected child/children
```

这样一旦 (I) 被证明 free，整棵 descendant subtree 都不会被创建。

---

## 1.2 不要让 `D_skip=32` 误伤 adaptive sweep

你稿件的 registered shelf profile 里写到 `FindFreeBox` keeps binary descent with skip-to-depth 32，而 leaf sweep profile 是 virtual leaf sweep from depth 8 to 13。这个设置对 seed-guided local repair 可能合理，但对 early-stop sweep 不合理。

建议把当前一个 `D_skip` 拆成两个参数：

[
D_{\mathrm{accept}}^{\mathrm{sweep}},
\qquad
D_{\mathrm{accept}}^{\mathrm{seed}}.
]

对于 adaptive leaf sweep：

[
D_{\mathrm{accept}}^{\mathrm{sweep}} = d_0
]

或者甚至允许 (d \ge d_0) 后立刻接受 free cell。对于 seed/query bridge repair，仍然可以保留：

[
D_{\mathrm{accept}}^{\mathrm{seed}} = 32
]

或者你当前实验 profile 里的值。

这是很关键的一点：如果 adaptive sweep 仍然调用一个带 `D_skip=32` 的 `FindFreeBox`，那么粗节点永远不能 early accept，算法会退化回深层展开。

---

## 1.3 推荐的替代算法：AdaptiveLeafSweep

可以直接把现在的 `LeafSweepRefine` 替换为下面这个结构。

```text
AdaptiveLeafSweep(T, O, d0, dmax, budget, context):
    F ← ∅
    C ← ∅
    Q ← priority queue

    for each shallow start cell I in StartCells(d0, context):
        push I into Q with Priority(I, context)

    while Q not empty and budget remains:
        I ← pop(Q)

        if CoveredByAcceptedForest(I, F):
            continue

        R ← MaterializeAndValidate(I, T, O)

        if R.status == FREE:
            InsertFreeBox(F, I, R.free_certificate)
            PruneDescendants(I)
            continue

        if R.status == CERT_OCCUPIED:
            RecordOccupiedDomain(C, I, R.occupied_certificate)
            PruneDescendants(I)
            continue

        if depth(I) == dmax:
            RecordCollisionDomain(C, I, R.blockers)
            continue

        if DominatedByNoGoodHistory(I, R, context):
            DeferDomain(C, I, R.blockers, reason=no_good)
            continue

        if not RelevantToBuildOrQuery(I, R, context):
            DeferDomain(C, I, R.blockers, reason=low_relevance)
            continue

        k ← ChooseSplitDimension(I, R, context)
        children ← Split(I, k)

        for J in SelectChildren(children, R, context):
            push J into Q with Priority(J, context)

    BuildOrUpdateAdjacency(F)
    return F, C
```

这里的 `context` 很重要。你的 paper 明确强调 offline build 不接收 start/goal/query label；因此不能把 query relevance 偷偷塞进 query-agnostic offline profile。建议定义三种 context：

[
\texttt{offline_coverage},\quad
\texttt{restricted_domain_refine},\quad
\texttt{online_query_repair}.
]

在 `offline_coverage` 中，relevance 来自 anchors、frontier、component connection potential、domain boundary，而不是 start/goal。在 `online_query_repair` 中，才允许 start/goal、query corridor、bridge target 影响 priority。

---

## 1.4 StartCells 设计：不要一上来从 root 全局递归

为了最小改动，第一版不要从 root 递归到整个 (\Q)。建议保留你现在的 (d_0) shallow grid，只把 (d_0\to d_1) 这段改成 adaptive。

也就是：

[
\mathcal S_0 = \mathcal L_{d_0}
]

然后对每个 (C_0\in\mathcal S_0) 调用 `AdaptiveClassify(C_0)`。如果现在 shelf profile 是 (d_0=8)，那么初始 256 个 cells 是可控的；真正避免的是继续固定展开到 13、23、32 或更深。

第一阶段可以这样做：

[
d_0 = 8,\qquad d_{\max}=13
]

和当前 baseline 对齐。第二阶段再测：

[
d_{\max}=18,\ 23,\ 32
]

这时 full sweep 会爆炸，而 adaptive sweep 只会在 blocker/frontier/portal 附近变深。

---

## 1.5 Priority 和 relevance 的具体定义

建议给每个 cell 一个 priority score：

[
\operatorname{prio}(I)
======================

w_f S_{\mathrm{frontier}}(I)
+
w_c S_{\mathrm{component}}(I)
+
w_a S_{\mathrm{anchor}}(I)
+
w_p S_{\mathrm{portal}}(I)
+
w_b S_{\mathrm{blocker}}(I)
---------------------------

w_h S_{\mathrm{history}}(I).
]

其中：

[
S_{\mathrm{frontier}}(I)=
\mathbf{1}{I \text{ touches accepted forest boundary}},
]

[
S_{\mathrm{component}}(I)
=========================

\left|{\operatorname{comp}(B): B\in \mathcal N_\tau(I)}\right|,
]

[
S_{\mathrm{anchor}}(I)
======================

\mathbf{1}{I \cap \mathcal A \neq \emptyset},
]

[
S_{\mathrm{portal}}(I)
======================

\mathbf{1}{I \text{ lies on a collision-domain/free-domain interface}}.
]

`offline_coverage` 模式下不使用 query start/goal。`online_query_repair` 模式下可以加入：

[
S_{\mathrm{query}}(I)
=====================

-\operatorname{dist}(I,\text{query corridor or bridge target}).
]

`RelevantToBuildOrQuery` 可以先用非常保守的规则：只要 cell touched frontier、contains anchor、could connect components、or lies inside selected collision domain，就 relevant。这样不会明显伤害 coverage。

---

## 1.6 Split dimension：从 depth-synchronous 改成 local score

当前 AAFK-volume-min 是 depth-synchronous：同一深度用同一 split dimension。这对 cache replay 很友好，但对 narrow passage 未必好。

建议在 adaptive sweep 中引入 node-local split score：

[
k^*
===

\arg\max_k
\left[
\alpha S_{\mathrm{kin}}(I,k)
+
\beta S_{\mathrm{blocker}}(I,k)
+
\gamma S_{\mathrm{frontier}}(I,k)
\right].
]

其中：

[
S_{\mathrm{kin}}(I,k)
\approx
\Delta \sum_\ell \operatorname{vol}(B_\ell(I))
]

表示 split joint (k) 预计能减少多少 link-envelope volume。

对于 blocker-driven splitting，可以用：

[
S_{\mathrm{blocker}}(I,k)
=========================

\sum_{(\ell,o)\in \beta(I)}
\mathbf{1}{k\in \operatorname{Ancestors}(\ell)}
\cdot
R_{k,\ell}
\cdot
\operatorname{overlap}_{\ell,o}(I).
]

意思是：如果当前 blocker 是 link (\ell) 和 obstacle (o)，那么优先 split 对 link (\ell) 有 kinematic influence 的 upstream joints。(R_{k,\ell}) 可以是 joint (k) axis 到 link (\ell) 的最大 lever arm bound。

更低风险的版本是：先保留现有 depth-synchronous schedule 作为 default，只在 validation fail 且 blocker 明确时用 blocker score 作为 tie-break。这样 cache 兼容性更好。

---

## 1.7 Children selection：不要总是两个孩子都递归

`Split(I,k)` 产生两个 children：

[
I_k^-,\quad I_k^+.
]

在纯 coverage 模式下，可以两个都入队，但 free parent early-stop 已经会大幅减少节点数。

在 restricted refinement 或 online query repair 中，建议只递归 selected children：

[
\operatorname{SelectChildren}
=============================

{J: J \cap \mathcal R_{\mathrm{target}}\neq\emptyset
\ \lor
J \text{ touches frontier}
\ \lor
J \text{ has high blocker-reduction score}}.
]

例如 query bridge 模式中，(\mathcal R_{\mathrm{target}}) 可以是 start-goal line tube、component-to-component connector tube，或者 portal-to-portal corridor band。这样深层 refinement 不会扩散到整个 collision domain。

---

## 1.8 复杂度该怎么写

原 fixed sweep：

[
N_{\mathrm{eval}}^{\mathrm{fixed}}
==================================

# |\mathcal L_{d_0:d_1}|

2^{d_1-d_0}
]

对每个 start subtree 都要付这个代价。

adaptive sweep 中，如果每个 split 最多产生两个 children，那么对于一个有限 adaptive binary tree：

[
N_{\mathrm{eval}}
\le
2|\mathcal T_{\mathrm{terminal}}|-1.
]

其中 terminal cells 包括 accepted free cells、certified occupied cells、depth-cap collision domains、deferred domains、covered domains。因此可以写成：

[
N_{\mathrm{eval}}
=================

O(
N_{\mathrm{free\text{-}maximal}}
+
N_{\mathrm{occ}}
+
N_{\mathrm{collision}}
+
N_{\mathrm{deferred}}
).
]

如果在 corridor/refinement 模式中每次只追一个 selected child，那么单个 seed 的最坏 materialized chain 是：

[
O(d_{\max}-d_0)
]

而不是 (O(2^{d_{\max}-d_0}))。

需要诚实写一句：adversarial checkerboard free space 下最坏情况仍然可能指数级；改动消除的是“无论是否需要都全层物化”的指数成本，而不是证明一个 polynomial complete planner。

---

# 2. Occupied / no-good pruning：把 blocker cache 从 update metadata 升级为 refinement controller

你现在已有 blocker set：

[
\beta(C)\subseteq {1,\ldots,|\mathcal O|}
]

用于 obstacle deletion 时选择可 promotion 的 collision domains。现在可以扩展为更丰富的 `ValidationReport`，既服务 dynamic update，也服务 adaptive sweep pruning。

---

## 2.1 先扩展 validation failure report

当前 (\beta(C)) 只记录 obstacle indices。建议改成：

[
\beta(I)
========

{(\ell,o,\mathrm{stage},m,\rho,\mathrm{affectedJoints})}.
]

其中：

* (\ell)：blocked link id；
* (o)：obstacle id 或 obstacle group id；
* `stage`：AABB broadphase fail、SupportHull/GJK fail、SDF penetration witness 等；
* (m)：overlap score、negative margin、penetration depth 或 conservative slack；
* (\rho)：cell motion bound 或 envelope looseness proxy；
* `affectedJoints`：哪些 joints 影响该 link，用于 split dimension scoring。

代码层面可以定义：

```text
ValidationReport:
    status: FREE | FAIL | CERT_OCCUPIED
    free_certificate: optional
    occupied_certificate: optional
    blockers: list[Blocker]
    min_margin: float
    overlap_score: float
    fail_stage: AABB | GJK | SDF | MIXED
```

这样 adaptive sweep 失败时不只是“失败”，而是知道“为什么失败、是否值得继续 split、应该 split 哪个 joint”。

---

## 2.2 Sound occupied certificate：只在条件足够强时启用

这个剪枝可以很强，但一定要写清楚 assumptions。不能把 “AABB overlap” 当成 occupied proof。AABB overlap 只能说明 envelope 与 obstacle overlap，不说明整个 (C)-space box 都 collision。

一个安全版本是基于 signed distance 和 material witness point。

设 obstacle (O) 有 1-Lipschitz signed distance function：

[
\phi_O(x)<0 \iff x\in \operatorname{int}(O).
]

在中心配置 (q_c) 下，如果 link (\ell) 上某个固定 material point (x_0) 满足：

[
\phi_O(T_\ell(q_c)x_0) \le -\delta,
]

并且对所有 (q\in I)，该 material point 的 workspace 位移有上界：

[
|T_\ell(q)x_0 - T_\ell(q_c)x_0|
\le
\rho_{\ell,x_0}(I),
]

那么只要：

[
\delta > \rho_{\ell,x_0}(I) + \epsilon_{\mathrm{num}},
]

就有：

[
\forall q\in I,\quad
\phi_O(T_\ell(q)x_0)<0,
]

因此 (I) 中每个 configuration 都 collision。这个 (I) 可以作为 `CERT_OCCUPIED` terminal cell，安全剪掉所有 descendants。

可以写成 lemma：

[
\phi_O(T_\ell(q_c)x_0)
+
\rho_{\ell,x_0}(I)
+
\epsilon_{\mathrm{num}}
<0
\quad\Longrightarrow\quad
I \subseteq \mathcal C_{\mathrm{obs}}.
]

### (\rho_{\ell,x_0}(I)) 的工程计算

对 revolute joint，可以用 lever-arm bound：

[
\rho_{\ell,x_0}(I)
\le
\sum_{j\in \operatorname{Anc}(\ell)}
2R_{j,\ell,x_0}\sin(\Delta_j/2),
]

其中：

[
\Delta_j
========

\max{|q_{c,j}-l_j|,\ |u_j-q_{c,j}|}.
]

小角度下也可以用更保守的：

[
2R\sin(\Delta/2)\le R\Delta.
]

对 prismatic joint：

[
\rho_j \le \Delta_j.
]

如果没有 reliable SDF / penetration witness，就不要启用 sound occupied certificate。可以先实现为 optional module：

```text
TryOccupiedCertificate(I, q_c, O):
    for each center-pose penetrating link-obstacle witness:
        compute material point x0
        compute motion bound rho
        if signed_distance(x0 at q_c, O) + rho + eps < 0:
            return CERT_OCCUPIED
    return no_certificate
```

### 论文中如何放

建议不要把 occupied certificate 放进主 theorem 的必要条件。可以作为一个 optional pruning lemma：

> Under an additional signed-distance witness assumption, occupied-domain pruning is sound: it removes only cells fully contained in obstacle collision space. If the witness is unavailable, the implementation falls back to inconclusive-domain deferral, which does not affect the conservative corridor certificate.

这样 reviewers 不会抓住 occupancy proof 的实现细节攻击你的主 soundness。

---

## 2.3 Blocker-signature dominance：heuristic no-good，不做 soundness claim

对 non-free cell (I)，定义 blocker signature：

[
\operatorname{sig}(I)
=====================

\operatorname{TopK}_{(\ell,o)}
\left[
(\ell,o,\mathrm{stage})
\right],
]

按 overlap score 或 negative margin 排序。

如果沿着连续 refinement chain：

[
I_0 \supset I_1 \supset \cdots \supset I_h
]

都有：

[
\operatorname{sig}(I_0)=\operatorname{sig}(I_1)=\cdots=\operatorname{sig}(I_h),
]

并且 overlap progress 很小：

[
\frac{
m(I_0)-m(I_h)
}{
m(I_0)+\eta
}
<
\theta_{\mathrm{prog}},
]

那么把这个 subtree 标记为 `NOGO_COOLDOWN`，暂时不再深挖。

这不是 occupied proof，所以只能写成：

> budgeted refinement heuristic.

它不影响 soundness，因为它不会把 cell 放进 forest，也不会证明 no path；它只是推迟探索。

建议参数初值：

[
h=3,\qquad
\theta_{\mathrm{prog}}=0.05,\qquad
H_{\mathrm{cool}}=100
]

其中 (H_{\mathrm{cool}}) 可以沿用你已有 hard-frontier failure deferral 思路。你当前稿件已经在 forest growth 里有 failed seed 达到 (K_{\mathrm{fail}}) 后 deferral (H_{\mathrm{cool}}) 的机制；这里可以把它推广到 subtree/domain level。

触发重新尝试的条件：

[
\text{higher budget}
\quad\lor\quad
\text{larger active depth}
\quad\lor\quad
\text{obstacle deletion affects }\beta(I)
\quad\lor\quad
\text{portal/query repair explicitly requests this domain}.
]

这和你现有 dynamic update 中“删除 obstacles 后，(\beta(C)) 为空则 eligible for promotion”的逻辑自然兼容。

---

## 2.4 Connectivity dominance：即使 free 也不一定有用

这是最适合 multi-query forest 的剪枝。核心判断是：

> 如果一个 cell 即使 eventually free，也不会改变当前 graph 的 useful connectivity，就不要优先深挖。

定义邻接 component 集合：

[
\mathcal K(I)
=============

{\operatorname{comp}(B): B\in\mathcal F,\ B\sim_\tau I}.
]

其中 (B\sim_\tau I) 使用你当前 query graph 的 overlap/touch/tolerance adjacency predicate。

规则：

1. 如果存在 accepted box (B) 满足：

[
I\subseteq B,
]

则直接 `COVERED`，不 validation，不 split。

2. 如果：

[
|\mathcal K(I)|\ge 2,
]

则 (I) 有潜在 component-connecting value，priority 提高。

3. 如果：

[
|\mathcal K(I)|=1
]

且 (I) 不包含 anchor、不在 portal boundary、不接近 query repair target、不打开新的 frontier face，则把它 `DEFERRED_CONNECTIVITY`。

4. 如果：

[
|\mathcal K(I)|=0
]

且不是 offline coverage anchor cell，则低 priority 或 defer。

这可以提前阻止很多“只是在同一个 connected component 内部填洞”的深层 refinement。

注意：这会影响 coverage/path quality，但不影响 conservative correctness。论文里可以说：

> connectivity dominance is used only to allocate finite refinement budget; it does not label a cell as occupied or free.

---

# 3. Sparse Patricia / skip tree：不要物化完整 binary path

## 3.1 当前 LECT 表示的问题

你现在 LECT 是 heap-style breadth-first indexing：

[
p \mapsto 2p+1,\ 2p+2,
]

并且有 maximum depth (D_{\max})。这对完整二叉树很方便，但对深层 sparse refinement 不友好：一旦要去 depth 32 或 52，容易创建很多中间节点，即使中间节点没有独立价值。

建议把 LECT node key 从 heap index 改为 dyadic address。

---

## 3.2 Dyadic address：用 per-dimension level vector 表示 cell

定义：

[
A(I)=
\left(
(\ell_1,i_1),\ldots,(\ell_n,i_n)
\right),
]

其中 joint (j) 的区间是：

[
I_j(A)
======

\left[
q_j^- + \frac{i_j}{2^{\ell_j}}(q_j^+-q_j^-),
\quad
q_j^- + \frac{i_j+1}{2^{\ell_j}}(q_j^+-q_j^-)
\right].
]

这样 cell 大小由 level vector 决定：

[
\boldsymbol{\ell}
=================

(\ell_1,\ldots,\ell_n).
]

split joint (k) 时只增加：

[
\ell_k \leftarrow \ell_k+1.
]

这比单一 global depth 更适合 manipulator narrow passages，因为 passage 可能只需要 joint 2 和 joint 5 变细，而不是每个 dimension 都跟着 schedule 变细。

---

## 3.3 Patricia compression：只存 branching/terminal nodes

建议的数据结构：

```text
SparseLectNode:
    addr: DyadicAddress
    interval: JointBox
    split: optional SplitDecision
    evidence_ref: optional EvidenceRef
    child_minus: optional NodeRef
    child_plus: optional NodeRef
    status: materialized | virtual | terminal
```

但注意：scene label 不放在 LECT node 里。scene-specific status 放到 forest/domain cache 里：

```text
SceneCellRecord:
    addr: DyadicAddress
    validation_report: ValidationReport
    scene_status: FREE | OCC | INCONCLUSIVE | DEFERRED | COVERED
    forest_box_id: optional
    domain_id: optional
```

LECT 仍然只管 evidence；forest/domain cache 管 scene labels。这样保持你原来的 evidence/label separation。

Patricia compression 的意思是：如果从 depth 13 直接跳到某个 depth-vector 对应 cell，不需要创建中间 19 层 nodes。只要有 address 和 interval，就可以 materialize evidence：

```text
MaterializeEvidence(addr):
    I ← Interval(addr)
    key ← EvidenceKey(robot, endpoint_source, rep, active_links, radii, I)
    if key exists in LECTDB:
        replay evidence
    else:
        compute endpoint/link envelope
        optionally write back
```

---

## 3.4 jump_child：直接跳到目标 descendant

对于 seed (q) 和目标 level vector (\boldsymbol{\ell}')，直接计算 descendant address：

[
i_j'
====

\left\lfloor
\frac{q_j-q_j^-}{q_j^+-q_j^-}
2^{\ell_j'}
\right\rfloor,
]

并 clamp 到：

[
0\le i_j'\le 2^{\ell_j'}-1.
]

伪代码：

```text
JumpCellContaining(q, level_vector):
    for each joint j:
        t ← (q[j] - qmin[j]) / (qmax[j] - qmin[j])
        idx[j] ← clamp(floor(t * 2^level[j]), 0, 2^level[j]-1)
    return DyadicAddress(level, idx)
```

这可以替代“从当前 node 一层层 descend 到 depth 32”的做法。对于 `FindFreeBox`，可以先检查 coarse ancestors；如果根据 skip policy 必须去深层，也可以直接 jump 到目标 level，而不是物化完整 chain。

---

## 3.5 Split policy 与 replay compatibility

你现在稿件里写到 build fixes one descriptor so replay compatibility is well-defined。这句话在 C-LECT 里要微调：

原来：

[
\text{cache key} \sim \text{heap index + split descriptor}
]

建议变成：

[
\text{cache key}
================

H(
\text{robot model},
\text{endpoint source},
\text{envelope rep},
\text{active links},
\text{link radii},
\text{joint interval bounds/address}
).
]

也就是说，evidence 的 replay compatibility 主要由 exact interval 决定，而不是由它是怎么被 split 出来的决定。split descriptor 影响 future traversal，但不改变某个 interval 的 endpoint evidence validity。

如果 parent evidence 是由 child-hull propagation 得来的，则 record provenance：

[
\operatorname{parent_record}
============================

H(\operatorname{child_addr}_1,\ldots,\operatorname{child_addr}_m).
]

这样 provenance 不会和 direct IFK/HIFK evidence 混淆。

---

## 3.6 分阶段实现，避免一次性重写 LECTDB

建议工程上分三步：

**Stage 1：VirtualCell overlay。**
保留旧 LECTDB，不改持久化格式。新增一个 `VirtualCell` 类型，用 interval bounds/address 表示 cell。adaptive sweep 先用 virtual cells 做 validation，只有需要 evidence 时再向旧 LECT 请求或在线 materialize。

**Stage 2：SparseNodeMap。**
新增：

```text
unordered_map<DyadicAddress, EvidenceRef>
```

只存 materialized cells。旧 heap index 仍可作为一种 `AddressKind=HeapSchedule` 支持。

**Stage 3：Per-dimension level vector。**
把 split depth 改成 (\boldsymbol{\ell})。这一步收益最大，但对缓存 key、adjacency index、实验对齐影响也最大，建议放在第二版 ablation。

---

# 4. Portal edge compression：深层路径不暴露为全局 vertices

## 4.1 当前 graph 的自然扩展

你当前 graph 是：

[
G=(V,E_{\cap}\cup E_{\varepsilon}\cup E_s),
]

其中 (E_\cap) 是 overlap/touch box edges，(E_\varepsilon) 是 tolerance-gap candidate，(E_s) 是 explicit collision-checked segment witness。现在建议增加：

[
E_{\mathrm{portal}}.
]

新的 graph：

[
G=(V,E_{\cap}\cup E_{\varepsilon}\cup E_s\cup E_{\mathrm{portal}}).
]

区别是：

* (E_s)：一条 checked segment 或 sampled witness；
* (E_{\mathrm{portal}})：一条 internal validated box-chain certificate。

也就是说，portal edge 不是 shortcut segment；它是隐藏在 collision domain 内部的一串 conservative boxes。

---

## 4.2 Portal 的定义

对一个 shallow collision domain (C)，找它和外部 forest 的接触区域。

定义 portal 集合：

[
\mathcal P(C)
=============

{
p=(C,B,f):
B\in\mathcal F,\
B\sim_\tau C,\
f\text{ is an interface face or boundary patch}
}.
]

每个 portal 记录：

```text
Portal:
    domain_id: C
    boundary_box: B
    component_id: comp(B)
    face_id / boundary patch
    seed set near interface
```

只需要考虑不同 connected components 之间的 portal pairs：

[
\operatorname{comp}(B_i)\ne \operatorname{comp}(B_j).
]

如果两个 portals 已经属于同一 component，连接它们通常不会改变 global graph connectivity，可以低 priority 或跳过，除非为了 path shortening 做 optional refinement。

---

## 4.3 Domain-internal portal search

对每个 selected pair：

[
(p_{\mathrm{in}},p_{\mathrm{out}})
]

只在 (C) 内部运行 adaptive refinement：

[
B_r \subseteq C.
]

伪代码：

```text
BuildPortalEdges(F, C_cache, budget):
    for each collision domain C in priority order:
        P ← DetectPortals(C, F)

        for selected portal pair (pin, pout):
            chain ← AdaptivePortalSearch(C, pin, pout)

            if chain is conservative and connects pin to pout:
                id ← StoreInternalCorridor(C, chain)
                Add E_portal edge between pin.boundary_box and pout.boundary_box
```

`AdaptivePortalSearch` 内部可以复用前面的 `AdaptiveClassify`：

```text
AdaptivePortalSearch(C, pin, pout):
    local_graph ← empty
    Q ← cells/seeds near pin

    while Q not empty and budget remains:
        I ← pop(Q)
        require I ⊆ C

        R ← MaterializeAndValidate(I)

        if R.status == FREE:
            insert I into local_graph
            if local_graph connects pin to pout:
                return ExtractBoxChain(local_graph, pin, pout)

        else if R.status == CERT_OCCUPIED:
            prune

        else if depth cap or no-good:
            record local blocker

        else:
            split I and push selected children toward pout

    return failure
```

输出 chain：

[
B_1,\ldots,B_m\subseteq C
]

满足：

[
B_1 \sim B_{\mathrm{in}},
\quad
B_i \sim B_{i+1},
\quad
B_m \sim B_{\mathrm{out}},
]

并且每个 (B_i) 都通过 conservative scene-validation。

---

## 4.4 Global graph 只存 compressed edge

不要把 (B_1,\ldots,B_m) 插入 global forest vertices。存成：

```text
PortalCorridor:
    corridor_id
    domain_id
    pin, pout
    internal_boxes: [B1, ..., Bm]
    free_cert_refs: [cert1, ..., certm]
    intersection_witnesses: optional
    conservative: bool
```

global graph edge：

```text
E_portal:
    u = pin.boundary_box_id
    v = pout.boundary_box_id
    corridor_id
    cost
    type = portal
```

这样 global graph search 看到的是：

[
B_{\mathrm{in}}
\rightarrow
B_{\mathrm{out}}
]

一条 edge；只有当 query path 真的使用这条 edge 时，才展开：

[
B_{\mathrm{in}},B_1,\ldots,B_m,B_{\mathrm{out}}.
]

---

## 4.5 Portal edge 的 soundness 写法

给 (E_{\mathrm{portal}}) 一个 certificate-preserving 条件：

一个 portal edge (e=(B_u,B_v)) 是 conservative portal edge，当且仅当它携带 finite chain：

[
\Pi_e=(B_1,\ldots,B_m)
]

满足：

[
B_i \text{ passed conservative validation},
]

[
B_i\subseteq C,
]

[
B_u\sim B_1,\quad
B_i\sim B_{i+1},\quad B_m\sim B_v,
]

其中 (\sim) 是和 (E_\cap) 一样的 exact overlap/touch adjacency，而不是 loose tolerance gap。

然后定义 expansion operator：

[
\operatorname{expand}(e)
========================

(B_u,B_1,\ldots,B_m,B_v).
]

定理可以加一个 corollary：

> Any graph path using (E_\cap) and conservative (E_{\mathrm{portal}}) edges can be expanded into an (E_\cap)-style sequence of conservatively validated boxes. Therefore the existing conditional box-corridor theorem applies to the expanded path.

证明很简单：把每个 portal edge 替换成它携带的 internal box chain。展开后的路径完全落在 conservatively validated box union 内，所以沿用你当前 theorem。

如果 portal chain 中用了 performance-mode boxes，则不能继承 conservative theorem，应该像 (E_s) 或 (E_\varepsilon) 一样进入 query-validation path。

---

## 4.6 Portal internal boxes 的 membership 问题

隐藏 internal boxes 会带来一个实际问题：如果某个 query start/goal 恰好落在 portal corridor 内部，但这些 boxes 没有进 global forest，membership lookup 可能找不到它。

解决方案有两个：

**低风险方案：** global membership 只查 global forest。若 start/goal 不在 global forest，则触发 local query repair；不主动查 portal internals。

**更完整方案：** 维护一个轻量 `PortalInteriorIndex`。查询时：

1. 先查 global forest；
2. 若失败，再查 portal internal boxes；
3. 如果命中 internal box (B_i)，临时展开对应 portal corridor，并创建 temporary query vertex attached to (B_i)。

第二种更完整，但实现复杂一些。第一版可以不做，论文里只说 portal edges are expanded when used by graph search。

---

# 5. 四个机制之间的组合方式

建议最终结构叫：

[
\textbf{C-LECT: Compressed Adaptive LECT}
]

它不是一个单独 trick，而是四层压缩：

[
\text{early-stop sweep}
+
\text{no-good pruning}
+
\text{sparse dyadic tree}
+
\text{portal edge compression}.
]

推荐的数据流是：

```text
AdaptiveLeafSweep
    ↓
maximal free boxes + inconclusive / occupied / deferred domains
    ↓
restricted domain refinement
    ↓
portal-corridor search inside selected domains
    ↓
global typed graph with E_cap, E_eps, E_s, E_portal
    ↓
query graph search
    ↓
lazy expansion of portal edges only if used
```

关键 invariant：

[
\text{Only validated boxes or validated internal box chains can contribute to a conservative path certificate.}
]

剪掉、defer、no-good、occupied domains 都不会进入 conservative path，因此不会破坏 soundness。

---

# 6. 论文中建议怎么改

## 6.1 修改 LECT section

把当前：

> LECT is a dynamically grown binary KD tree ...

改成：

> LECT is a sparse dyadic evidence tree, with the heap-style binary schedule as one compatible address mode.

可以保留旧描述作为 implemented baseline，然后新增 C-LECT paragraph：

```latex
\paragraph{Sparse dyadic addressing.}
The compressed variant keys evidence by robot-compatible joint intervals rather
than by materialized heap nodes. A cell is represented by a dyadic address ...
Intermediate nodes on an unbranched path need not be materialized.
```

然后强调：

[
\text{evidence key} \neq \text{scene label}.
]

这和你现有 separation 完全一致。

---

## 6.2 替换 LeafSweepRefine algorithm

当前 `LeafSweepRefine` 可以改为两个算法：

1. `AdaptiveLeafSweep`
2. `AdaptiveClassify`

主文里说：

> The fixed virtual leaf layer is a special case obtained by forcing every inconclusive node to split until (d_1).

这样你可以把旧算法纳入新算法：

[
\text{FixedLeafSweep}
=====================

\text{AdaptiveLeafSweep with no early free stop and full child expansion}.
]

但从实现上当然启用 early stop。

---

## 6.3 新增 optional occupied-pruning lemma

放在 forest construction 或 appendix：

```latex
\begin{lemma}[Signed-distance occupied pruning]
...
\end{lemma}
```

同时写清楚：

> The planner does not require this oracle; without it, failed cells are treated as inconclusive domains.

这样避免 reviewer 认为你依赖一个很强的 penetration oracle。

---

## 6.4 修改 graph edge definition

当前：

[
G=(V,E_{\cap}\cup E_{\varepsilon}\cup E_s)
]

改成：

[
G=(V,E_{\cap}\cup E_{\varepsilon}\cup E_s\cup E_{\pi})
]

其中 (E_\pi) 或 (E_{\mathrm{portal}}) 是 compressed portal edge。

给它一句定义：

> An (E_{\pi}) edge stores a finite internal chain of conservatively validated boxes inside a retained collision domain. It is certificate-preserving only through expansion of that chain.

然后 theorem 后加 corollary：

```latex
\begin{corollary}[Portal-expanded conservative corridor]
If every portal edge on a graph path carries a conservative internal box chain,
then expanding those edges yields a path contained in the union of conservatively
validated boxes. Hence the conditional conservative box-corridor theorem applies.
\end{corollary}
```

---

# 7. 实验计划：必须证明不是只换了表示

建议加一个 ablation table，指标不要只报 query time，也要报 tree/graph compression。

对 shelf + IIWA 和 random scenes 分别跑：

1. Fixed leaf sweep baseline；
2. Early-stop free only；
3. Early-stop + blocker/no-good deferral；
4. Early-stop + sparse dyadic addressing；
5. Early-stop + portal edges；
6. Full C-LECT。

建议报告：

[
N_{\mathrm{virtual\ leaves}},
\quad
N_{\mathrm{materialized\ cells}},
\quad
N_{\mathrm{validated\ free}},
\quad
N_{\mathrm{collision\ domains}},
\quad
N_{\mathrm{deferred}},
\quad
N_{\mathrm{LECT\ evidence\ lookups}},
\quad
N_{\mathrm{global\ graph\ vertices}},
\quad
N_{E_{\mathrm{portal}}},
\quad
\text{query success},
\quad
\text{audited path length}.
]

特别应该画两个 histogram：

1. accepted free box depth distribution；
2. materialized cell depth distribution。

如果 early-stop 有效，你会看到很多 accepted boxes 停在 shallow depths，而不是集中在 (d_1)。如果 portal compression 有效，你会看到 global graph vertices 显著下降，但 successful queries 保持或提升。

---

# 8. 推荐实现顺序

我建议按这个顺序做，风险最低、收益最快。

## Step 1：只做 early-stop adaptive sweep

不改 LECTDB，不改 graph edge，只替换 leaf sweep 的枚举方式。

目标：证明 materialized/evaluated cells 下降。

需要改：

```text
LeafSweepRefine
→ AdaptiveLeafSweep + AdaptiveClassify
```

保留当前 validation oracle 和 conservative theorem。

## Step 2：加 richer blocker report + no-good cooldown

扩展 (\beta(C))，让它不只是 obstacle id，而是 link-obstacle-stage-margin record。

目标：减少重复深挖同一个 blocker。

注意：不要 claim sound occupied。只说 heuristic budget allocation。

## Step 3：加 sparse address overlay

先不改持久化 DB，只在 runtime 用 `DyadicAddress` + `VirtualCell` 避免中间节点物化。

目标：让 depth 32/52 不自动创建完整 path。

## Step 4：加 optional sound occupied certificate

只有当 SDF 或 reliable penetration witness 可用时启用。

目标：在 shelf/wall/deep penetration 场景剪掉整块 impossible regions。

没有可靠 witness 就跳过，不影响主算法。

## Step 5：加 portal edge

先只对 restricted refinement domains 做 portal-to-portal internal chains。

目标：减少 global graph vertices，同时保留 box-chain certificate。

这是论文卖点最强的一步，但工程复杂度也最高，尤其是 query path expansion 和 dynamic update。

---

# 9. 最终建议的写法定位

我会把这部分定位成：

> C-LECT does not change the box-validation oracle or the conservative corridor certificate. It changes only which cells are materialized, which failed cells are revisited, and how locally deep certified corridors are represented in the scene graph.

核心 claim：

[
\text{fixed-depth leaf sweep cost}
==================================

O(2^{d_1-d_0})
]

被替换为：

[
\text{adaptive sweep cost}
==========================

O(
N_{\mathrm{free\text{-}maximal}}
+
N_{\mathrm{occ}}
+
N_{\mathrm{collision}}
+
N_{\mathrm{deferred}}
)
]

再通过 portal edge 把深层 local chain 从 global graph 中隐藏：

[
B_1,\ldots,B_m
\quad\leadsto\quad
e_{\mathrm{portal}}=(p_{\mathrm{in}},p_{\mathrm{out}},\mathrm{chain_id}).
]

这样你的文章逻辑会很顺：LECT 可以继续很深，但深度不再强迫全局二叉树、全局 forest 和全局 graph 一起指数增长。
