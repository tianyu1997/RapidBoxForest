可以，而且我认为这应该成为你现在这套方法的**主线改法**之一。它比“离线尽量覆盖 query path”更合理：离线不需要猜中整条 query path，而是构建一个**由粗到细的连通骨架**。粗层先判断哪些大 cell 已经能 certified free，哪些只是 mixed/unknown；然后优先尝试把 free components 连起来。只有当某个粗层连接必须穿过 mixed cell 时，才在那个 mixed cell 内部细化。

我建议把它命名为：

[
\textbf{HiPaC: Hierarchical Partition Connectivity}
]

或者：

[
\textbf{HCR: Hierarchical Connectivity Refinement}
]

它和你现有的 LECT / RBF 很契合：LECT 仍然负责 cell 的 envelope evidence；RBF 不再做固定深度 leaf sweep，而是在多分辨率 partition graph 上做 connectivity-driven refinement。

---

# 1. 核心思想

当前 leaf sweep 的逻辑更像：

[
\text{enumerate cells} \rightarrow \text{classify free/non-free} \rightarrow \text{build graph}.
]

分级 partition 连通应改成：

[
\text{coarse partition} \rightarrow \text{build abstract connectivity graph}
\rightarrow \text{find unresolved component gaps}
\rightarrow \text{only refine gaps}.
]

也就是：

```text
粗层：
    大 cell 如果 certified free，直接作为大 free region
    大 cell 如果不能 certified free，不说它 occupied，只标为 mixed / unknown

连通层：
    先在粗层 free cells 之间建图
    找哪些 free components 已经连通，哪些还隔着 mixed cells

细化层：
    只细化那些可能连接重要 components 的 mixed cells
    如果细化后出现 free child chain，就把它压缩成 portal edge 或局部 box chain
```

最重要的变化是：**refinement 的单位不再是“所有 leaf”，而是“当前最有连通价值的 mixed domain”。**

---

# 2. Cell 状态设计

对每个 partition cell (C)，不要只用 free / not-free 二分类。建议定义：

[
\sigma(C)\in
{
\texttt{FREE},
\texttt{CERT_OCCUPIED},
\texttt{MIXED},
\texttt{DEFERRED},
\texttt{REFINED}
}.
]

含义如下。

## 2.1 FREE

[
C\subseteq \mathcal C_{\mathrm{free}}
]

即整个 cell 通过 conservative envelope-disjointness validation。这个 cell 可以直接进入 certified forest。

## 2.2 CERT_OCCUPIED

可选状态。只有在你有 sound occupied certificate 时才使用，例如 signed-distance penetration witness 加 motion bound。否则不要轻易使用。

## 2.3 MIXED

这是最常见的失败状态。含义是：

[
C \text{ 没有被证明 free}
]

而不是：

[
C\subseteq \mathcal C_{\mathrm{obs}}.
]

这个区别非常重要。粗层大 cell 的 envelope 很松，validation fail 很可能只是保守外包络太粗，不代表 cell 中没有可行通道。

## 2.4 DEFERRED

当前 budget 下暂不细化，但保留 blocker / adjacency / component 信息。以后 query 或 higher budget 可以重新激活。

## 2.5 REFINED

该 parent cell 已经被 child partition 替代。parent 本身不进入 certified path，但可以存一个 connectivity summary。

---

# 3. 多分辨率 partition 结构

定义分辨率层级：

[
\mathcal P^0,\mathcal P^1,\ldots,\mathcal P^L.
]

(\mathcal P^0) 是粗 partition，(\mathcal P^{\ell+1}) 是对 selected cells 的局部细化。

不要全局从 (\mathcal P^\ell) 展开到 (\mathcal P^{\ell+1})。应该是 lazy refinement：

[
C\in\mathcal P^\ell
\quad\rightarrow\quad
\operatorname{children}(C)\subset \mathcal P^{\ell+1}
]

只对重要 mixed cell 创建 children。

如果使用 dyadic cell，可以表示为：

[
C =
\prod_{j=1}^n
\left[
q_j^- + \frac{i_j}{2^{\ell_j}}(q_j^+-q_j^-),
q_j^- + \frac{i_j+1}{2^{\ell_j}}(q_j^+-q_j^-)
\right].
]

这里最好使用 per-dimension level vector：

[
\boldsymbol{\ell}=(\ell_1,\ldots,\ell_n),
]

而不是单一 depth。因为 narrow passage 可能只需要某几个 joint 细化。

---

# 4. 两张图：certified graph 和 optimistic graph

这是分级连通法最关键的设计。

## 4.1 Certified graph

[
G^+ = (V^+,E^+)
]

只包含已经 certified free 的 cells，以及由内部 certified box chain 支撑的 portal edges。

这个图可以返回 path certificate。

也就是说，query 最终成功路径必须落在 (G^+) 对应的 box union 或 certified portal expansion 中。

## 4.2 Optimistic graph

[
G^\sim = (V^\sim,E^\sim)
]

包含：

* certified free cells；
* mixed cells；
* unresolved portal candidates；
* deferred but potentially useful domains。

这个图不能直接返回路径。它只用于回答：

> 如果我想连通 component A 和 component B，最可能需要细化哪些 mixed cells？

也就是说，(G^\sim) 是 refinement planner，不是 motion planner。

这点非常重要。不要把 mixed cell 当作 free cell 用，只把它当作“可能值得细化的 coarse tunnel”。

---

# 5. 粗到细连通算法

可以把原来的 `LeafSweepRefine` 替换成下面这个框架。

```text
HierarchicalConnectivityRefine(T, O, P0, Lmax, budget):
    F ← ∅                         // certified free cells
    D ← ∅                         // mixed / collision domains
    G_cert ← empty graph
    G_opt  ← empty graph

    // 1. classify coarse cells
    for C in coarse partition P0:
        R ← ClassifyCell(C, T, O)

        if R.status == FREE:
            insert C into F
            insert C into G_cert and G_opt

        else if R.status == CERT_OCCUPIED:
            record occupied C
            insert blocked marker if useful

        else:
            mark C as MIXED
            record blockers β(C)
            insert C into G_opt only

    build adjacency in G_cert and G_opt

    // 2. connectivity-driven refinement
    while budget remains:
        pair ← SelectDisconnectedComponentPair(G_cert, G_opt)

        if pair == none:
            break

        Π ← FindOptimisticCoarsePath(pair, G_opt)

        if Π == none:
            mark pair currently unreachable
            continue

        C* ← SelectMixedCellToRefine(Π)

        RefineMixedCell(C*, T, O, G_cert, G_opt, budget)

    return F, D, G_cert, G_opt
```

区别在于：算法不是“把所有 cell 细化到 depth (d)”；而是反复问：

[
\text{当前哪些 free components 最值得连？}
]

然后只细化连接它们所需的 mixed cells。

---

# 6. mixed cell 内部如何细化

假设 coarse mixed cell (C) 邻接两个 free components：

[
K_a,\quad K_b.
]

我们希望知道：

[
C \text{ 内部是否存在一条 certified child-cell chain 连接 } K_a \text{ 和 } K_b.
]

定义两个 portals：

[
p_a = C \cap \partial K_a,
\qquad
p_b = C \cap \partial K_b.
]

在 (C) 内部运行局部细化：

```text
RefineMixedCell(C, entry_component, exit_component):
    split C into selected children
    classify each child

    build local child graph using certified free children

    if local child graph connects entry portal to exit portal:
        store internal chain
        add certified portal edge to G_cert
        summarize C as connected for that portal pair
        return success

    else:
        choose the next mixed child on the best local optimistic path
        recursively refine that child
```

如果找到 child chain：

[
B_1,\ldots,B_m\subset C,
]

满足：

[
B_1\sim K_a,\quad
B_i\sim B_{i+1},\quad
B_m\sim K_b,
]

且每个 (B_i) 都 certified free，那么不要一定把所有 (B_i) 暴露成 global graph vertices。可以压缩成：

[
e_C=(K_a,K_b,\text{chain_id}).
]

这就是你前面提到的 portal edge compression 和 hierarchical partition 的结合版本。

---

# 7. Parent cell 存 connectivity summary，而不是存所有 children

对一个 refined parent cell (C)，建议存一个局部 connectivity summary：

```text
CellSummary(C):
    status: FREE | MIXED | REFINED | CERT_OCCUPIED
    boundary_ports: ports touching neighboring components
    certified_portal_pairs: {(p_i, p_j) -> chain_id}
    unresolved_portal_pairs: {(p_i, p_j)}
    blockers: β(C)
    children: optional sparse children
```

如果 (C) 已经被证明整体 free，那么 summary 很简单：

[
\forall p_i,p_j\in \partial C,\quad p_i \leftrightarrow p_j.
]

如果 (C) 不是整体 free，但内部有一条 child chain 连接两个边界 portal，则只记录这个 portal pair：

[
(p_i,p_j)\in \operatorname{Conn}(C).
]

这样你不需要把深层 child boxes 全部放到 global forest。global graph 只看到：

[
K_a \xleftrightarrow[]{e_C} K_b.
]

当 query path 真正使用这个 edge 时，再展开 chain。

---

# 8. 分级 partition 如何解决“离线 budget 不帮助 online”的问题

之前的问题是：离线增加 budget 可能只是覆盖更多随机 free volume，而 query path 仍然没被碰到。

分级连通法改变了 budget 的用途。

以前：

[
\text{budget} \rightarrow \text{more boxes}.
]

现在：

[
\text{budget} \rightarrow \text{more connected components / more portals resolved}.
]

这更接近在线 query 的需求。在线 query 通常只需要：

[
q_s
\rightarrow
\text{nearest free component}
\rightarrow
\text{component graph}
\rightarrow
\text{goal component}
\rightarrow
q_g.
]

因此离线不需要覆盖整条 query path，只要建立足够多的 component-level certified connections。

特别是在 shelf / manipulation 场景里，很多 query 会共享同一批 bottleneck portals。分级连通法会优先把这些 portals 打通；一旦打通，后续大量 query 都受益。

---

# 9. Query 阶段如何使用分级连通图

在线 query 可以改成 lazy hierarchical search。

```text
HierarchicalQuery(qs, qg):
    As ← AttachOrRefine(qs)
    Ag ← AttachOrRefine(qg)

    while online budget remains:
        Π ← Search(G_cert ∪ G_opt, As, Ag)

        if Π contains only certified free cells and certified portal edges:
            return ExpandCertifiedPath(Π)

        U ← first unresolved mixed cell or portal candidate on Π

        result ← RefineToResolve(U)

        if result creates certified connection:
            update G_cert
        else:
            increase cost of U or mark deferred

    return fallback / failure
```

关键点：

* `G_opt` 允许 query 找到“可能的粗通道”；
* 但只要 path 中还有 mixed cell，就不能直接返回；
* 每次只细化当前最可能打通的 unresolved cell；
* 成功后把结果写回 scene cache；
* 下一次相似 query 直接受益。

这相当于 box-based LazyPRM / HPA* 思路，但每条最终路径仍然可以保留 conservative box certificate。

---

# 10. 选择要细化哪个 mixed cell

对于 optimistic path：

[
\Pi = C_1,C_2,\ldots,C_m,
]

其中部分 (C_i) 是 mixed。不要简单 refine 第一个 mixed cell，建议选择 bottleneck score 最大的：

[
C^*
===

\arg\max_{C_i\in \Pi\cap\texttt{MIXED}}
S(C_i).
]

一个可用 score：

[
S(C)
====

w_c S_{\mathrm{component}}(C)
+
w_p S_{\mathrm{portal}}(C)
+
w_q S_{\mathrm{query}}(C)
-------------------------

## w_b S_{\mathrm{blocker}}(C)

w_v \log \operatorname{vol}(C).
]

其中：

[
S_{\mathrm{component}}(C)
=========================

\left|{\operatorname{comp}(B): B\sim C,\ B\in G^+}\right|.
]

如果 (C) 邻接多个 free components，它很可能是重要 connector。

[
S_{\mathrm{portal}}(C)
]

表示它是否位于 shelf opening、narrow passage、collision-domain/free-domain interface。

[
S_{\mathrm{query}}(C)
]

只在 online query 或 workload-aware offline build 中使用。

[
S_{\mathrm{blocker}}(C)
]

来自 blocker signature。如果同一个 blocker 多层不消失，可以降低该 cell 的优先级，或者换 split dimension。

---

# 11. Split 方式：不要均匀切所有维度

在高维机械臂里，coarse-to-fine partition 如果每层把所有维度都加倍，会爆炸：

[
2^n
]

对于 7-DOF 太贵。

所以 refined mixed cell 时，建议一次只 split 一个或两个最相关维度：

[
k^*
===

\arg\max_k
\left[
\alpha S_{\mathrm{kin}}(C,k)
+
\beta S_{\mathrm{blocker}}(C,k)
+
\gamma S_{\mathrm{portal}}(C,k)
\right].
]

然后：

[
C \rightarrow C^-_k \cup C^+_k.
]

这和 LECT 的 binary KD tree 更兼容，也符合你当前系统。

粗层可以稍微均匀一些，细层必须 anisotropic：

```text
coarse level:
    use scheduled / balanced splits

refinement level:
    split blocker-sensitive joint
    split portal-opening joint
    split high-envelope-volume joint
```

这样 “分辨率由粗到细” 不等于全维 uniform grid，而是 adaptive partition。

---

# 12. 离线 build 的三种模式

这个分级连通法可以有三种强度。

## 12.1 Query-agnostic scene skeleton

完全不使用 query，只连接场景中的 free components。

```text
目标：
    发现 coarse free components
    打通高价值 collision-domain portals
    维护 scene-level skeleton
```

优点：最干净。
缺点：如果 query distribution 很偏，仍然可能连错区域。

## 12.2 Workload-aware component connectivity

不使用测试 query，但使用 task anchors，例如 home、pregrasp、retreat、handover、bin、shelf slot IK samples。

```text
目标：
    确保 anchor components 之间尽量连通
```

这会明显提高 online 受益程度。

例如定义 task anchor sets：

[
\mathcal A_1,\ldots,\mathcal A_K.
]

离线目标变成：

[
\operatorname{connect}(\mathcal A_i,\mathcal A_j)
\quad
\text{for high-weight task pairs}.
]

这比“覆盖 query path”更实际。

## 12.3 Lifelong online refinement

在线 query 发现的新 connection 写回 cache：

[
G^+_{t+1}
=========

G^+_t
\cup
\operatorname{CertifiedRepair}(q_s^t,q_g^t).
]

这样系统越用越强。分级 graph 里的 mixed cells 会被真实 query 不断解析成 certified portal edges。

---

# 13. 一个完整算法草案

可以写成论文算法：

```text
Algorithm: HierarchicalPartitionConnectivity

Input:
    LECT T
    obstacle set O
    coarse partition P0
    max level Lmax
    component-pair queue R
    budget B

Output:
    certified graph G+
    optimistic refinement graph G~
    hierarchical cell summaries H

1. Initialize G+, G~, H.

2. For each coarse cell C in P0:
       R ← ClassifyCell(C, T, O)

       if R is FREE:
           AddCertifiedCell(C, G+, G~)

       else if R is CERT_OCCUPIED:
           MarkBlocked(C, H)

       else:
           MarkMixed(C, β(C), H)
           AddMixedNode(C, G~)

3. Build coarse adjacency in G+ and G~.

4. While budget remains:
       (Ka, Kb) ← SelectComponentPair(R, G+)

       if Ka and Kb are already connected in G+:
           continue

       Π ← ShortestOptimisticPath(Ka, Kb, G~)

       if Π does not exist:
           mark pair unresolved
           continue

       C ← SelectMixedCellOnPath(Π)

       result ← RefineForConnectivity(C, Ka, Kb, T, O)

       if result found certified child chain:
           AddPortalEdge(Ka, Kb, result.chain, G+)
           UpdateSummary(C, result)

       else if result produced children:
           ReplaceMixedParentByChildren(C, result.children, G~)

       else:
           Defer(C)

5. Return G+, G~, H.
```

---

# 14. `RefineForConnectivity` 的内部算法

```text
RefineForConnectivity(C, Ka, Kb, T, O):
    Pin  ← interface between C and Ka
    Pout ← interface between C and Kb

    Q ← priority queue seeded near Pin and Pout

    while local budget remains:
        U ← pop(Q)

        R ← ClassifyCell(U, T, O)

        if R is FREE:
            Add U to local certified graph

            if local graph connects Pin to Pout:
                chain ← ExtractCertifiedChain(Pin, Pout)
                return CONNECTED(chain)

        else if R is CERT_OCCUPIED:
            mark U blocked

        else if level(U) < Lmax:
            k ← ChooseSplitDimension(U, R.blockers)
            children ← Split(U, k)
            push children into Q

        else:
            mark U unresolved

    return UNRESOLVED
```

这其实就是把 “restricted refinement inside collision domains” 改成了 “restricted refinement for component connectivity”。

---

# 15. 和 portal edge compression 的关系

分级 partition 连通和 portal edge 是天然配套的。

在 coarse graph 里，mixed cell (C) 是一个粗通道候选。

在细化后，如果 (C) 内部找到了：

[
B_1,\ldots,B_m,
]

那不要把所有 (B_i) 都塞进 global graph。只记录：

[
e_C =
(p_{\mathrm{in}},p_{\mathrm{out}},\text{chain_id}).
]

global graph 只增加一个 edge：

[
K_a \xleftrightarrow[]{e_C} K_b.
]

最终 query path 使用这个 edge 时，再展开：

[
p_{\mathrm{in}},B_1,\ldots,B_m,p_{\mathrm{out}}.
]

这样可以同时达到两个目标：

1. 细层能找到 narrow passage；
2. global graph 不因为细层 partition 爆炸。

---

# 16. 和你当前理论的兼容性

这个改法不会破坏 conservative theorem，只要保持下面 invariant：

[
G^+ \text{ 中的每个 vertex 或 portal edge 都有 conservative certificate}.
]

对于 vertex：

[
C\in V^+
\quad\Longrightarrow\quad
C \text{ passed conservative validation}.
]

对于 portal edge：

[
e=(u,v)\in E_{\mathrm{portal}}
]

必须携带 internal chain：

[
B_1,\ldots,B_m,
]

且：

[
\forall i,\quad B_i \text{ passed conservative validation}.
]

查询返回前，将 portal edge 展开成 child box chain。展开后路径仍然落在 validated box union 内，因此原来的 box-corridor theorem 仍然成立。

需要避免的错误是：

[
\texttt{MIXED cell} \neq \texttt{FREE cell}.
]

mixed cell 可以参与 optimistic graph search，但不能直接参与 certified path。

---

# 17. 这套方法对 online query 的直接收益

原来 online query 如果 start/goal 不在 forest 或者中间没有 box corridor，只能局部救火。

分级 partition 后，online 至少可以拿到一个 coarse explanation：

```text
start component
    → coarse free cell
    → mixed domain C1
    → coarse free cell
    → mixed domain C2
    → goal component
```

这意味着 online repair 不再盲目搜索，而是有明确目标：

[
\text{resolve } C_1,C_2,\ldots
]

每个 (C_i) 都是一个 bounded domain，且带有 entry / exit portals。这比在整个 (\mathcal Q) 中找 path 便宜得多。

也就是说，offline 即使没有完全打通 query path，也会给 online 留下有用结构：

1. nearest components；
2. unresolved mixed connectors；
3. blocker information；
4. portal candidates；
5. coarse optimistic route。

这就是离线 budget 能转化为 online 收益的关键。

---

# 18. 实验应该怎么验证

建议比较四个版本：

1. fixed leaf sweep；
2. early-stop adaptive sweep；
3. hierarchical partition connectivity；
4. hierarchical partition connectivity + portal compression。

不要只看 box 数量，要看连通指标。

建议报告：

[
N_{\mathrm{certified\ components}},
]

[
N_{\mathrm{connected\ anchor\ pairs}},
]

[
N_{\mathrm{resolved\ portal\ pairs}},
]

[
P_{\mathrm{attach}},
]

[
P_{\mathrm{samecomp}},
]

[
N_{\mathrm{online\ mixed\ cells\ refined}},
]

[
T_{\mathrm{online\ repair}}.
]

真正要证明的是：

[
\text{offline budget} \uparrow
\quad\Rightarrow\quad
\text{resolved component connections} \uparrow
\quad\Rightarrow\quad
\text{online repair} \downarrow.
]

而不是：

[
|\mathcal F| \uparrow.
]

---

# 19. 最小落地版本

最小改法如下：

1. 保留现有 LECT 和 validation oracle。
2. 把 fixed `LeafSweepRefine` 改成 coarse partition classification。
3. 对 non-free cell 不再直接当 collision domain 结束，而是加入 optimistic graph。
4. 建立 certified graph (G^+) 和 optimistic graph (G^\sim)。
5. 对 disconnected free components，找 optimistic path。
6. 只细化 optimistic path 上的 mixed cells。
7. 找到 child free chain 后，压缩为 portal edge。

这版不需要一开始实现 sound occupied certificate，也不需要改动全部缓存系统。

---

# 20. 结论

可以使用分级 partition 连通，而且我认为这是比“继续增加离线覆盖 budget”更正确的方向。

一句话总结：

[
\boxed{
\text{先在粗层寻找可连通结构；只在连通受阻的 mixed cells 中细化；细层成功后压缩为 certified portal edge。}
}
]

这样 LECT 的深度仍然可以存在，但它只服务于少数真正影响连通性的 mixed domains，而不是把整个 free space 按固定深度展开。对 online query 来说，离线产物也不再只是一些随机 free boxes，而是一个可搜索、可解释、可增量解析的多分辨率连通图。
