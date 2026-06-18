我建议把当前 **Table I** 从“RBF 相对于各类方法的文字性说明表”，改成一个更客观的 **taxonomy table**。现在的表格已经能表达 RBF 的定位，但主要问题是：列名偏主观，单元格里句子较长，`Strength / Limitation / Relation to RBF` 三列之间有信息重叠，而且 RBF 自己那一行的 “Conservative boxes” 作为 limitation 不够明确。当前 Table I 在第 2 页，比较了 PRM/LazyPRM、RRTConnect/RRT、BIT*/FMT*、IRIS/GCS 和 RBF 五类方法。

# 核心优化思路

Table I 最好回答一个问题：

**RBF 到底处在 reusable manipulation planning 的哪个设计点？**

因此表格列应围绕以下几个审稿人最关心的维度展开：

1. **复用对象是什么？**
   Roadmap、tree、sample graph、convex regions、box forest、LECT evidence 等。

2. **复用对象是否有 C-space 体积？**
   这是 RBF 区别于 PRM/RRT/BIT* 的关键。

3. **碰撞证据附着在哪个粒度上？**
   是 edge、segment、sample graph、convex region，还是 joint box。

4. **场景局部变化后还能复用什么？**
   这是 RBF 的另一个核心卖点：scene-level box cache 与 kinematics-level LECT 分离。

5. **主要 trade-off 是什么？**
   不是简单写 “Strength / Limitation”，而是写清楚方法为什么适合某类场景、又为什么不覆盖所有场景。

---

# 建议替换版 Table I

下面这个版本比当前表更清晰，也更符合 TRO 审稿人阅读习惯。

```latex
\begin{table*}[t]
\centering
\caption{Taxonomy of reusable structures in manipulation planning.
``Vol.'' indicates whether the persistent object represents positive-volume
C-space regions. The table positions RBF by its reused object, validation
granularity, and scene-edit semantics rather than claiming universal
dominance over other planners.}
\label{tab:reuse_taxonomy}
\scriptsize
\setlength{\tabcolsep}{3.5pt}
\renewcommand{\arraystretch}{1.15}
\begin{tabularx}{\textwidth}{l l c l l X}
\toprule
Family &
Persistent object &
Vol. &
Validation / certificate unit &
Scene-edit reuse &
Main trade-off \\
\midrule
PRM / LazyPRM &
Roadmap vertices and edges &
No &
Local edges or path segments &
Topology can be reused; affected edges require rechecking &
Effective multi-query graph, but reusable evidence is zero-volume. \\

RRTConnect / RRT &
Query tree &
No &
Incremental path segments &
Little persistent reuse across queries &
Strong one-shot discovery; RBF turns frontier growth into persistent box growth. \\

BIT* / FMT* &
Batch sample graph or tree &
No &
Candidate graph edges &
Mainly query-centric; limited scene-level reuse &
Improves path quality with time, but spends effort on query-local edge evaluation. \\

Adaptive cell decomposition &
C-space cells &
Yes &
Cells under exact, sampled, or interval tests &
Explicit cells can be locally relabeled when maintained &
Volumetric representation, but high-DOF manipulation needs a cheap cell-validation oracle. \\

IRIS / GCS &
Certified convex regions and graph &
Yes &
Convex free-space regions &
Regions are mostly scene-coupled; edits may require reinflation or recertification &
Optimization-ready, high-quality regions, but construction can be expensive. \\

\textbf{RBF} &
\textbf{Box forest + typed graph + LECT evidence} &
\textbf{Yes} &
\textbf{Joint boxes via link envelopes; final path audit when required} &
\textbf{Scene boxes are maintained locally; LECT replays kinematic evidence} &
\textbf{Low-build volumetric reuse with lower regional expressivity and conditional certificates.} \\
\bottomrule
\end{tabularx}
\end{table*}
```

---

# 为什么这个版本更清晰

## 1. 用 “Persistent object” 替代 “Reuse object”

`Reuse object` 语义没错，但 `Persistent object` 更强调多查询 planner 真正保存下来的东西。对 RBF 来说，这能自然区分：

* scene-level persistent object：box forest + typed corridor graph；
* kinematics-level persistent object：LECT envelope evidence。

当前表中 “Box forest + LECT” 虽然写了，但没有让读者立刻看出它是两个不同层级的 reuse。

---

## 2. 增加 “Vol.” 一列，突出 RBF 的核心定位

RBF 与 PRM、RRT、BIT* 的最大区别不是“更快”，而是：

**它复用的是有体积的 C-space regions，而不是 zero-volume edges 或 trees。**

所以建议加一列：

```latex
Vol.
```

含义是：

> whether the persistent object represents positive-volume C-space regions.

这样一眼就能看出：

| 方法               | Vol. |
| ---------------- | ---- |
| PRM / LazyPRM    | No   |
| RRTConnect / RRT | No   |
| BIT* / FMT*      | No   |
| IRIS / GCS       | Yes  |
| RBF              | Yes  |

这能直接说明 RBF 是介于 sampling graph 和 convex-region planner 之间的设计点。

---

## 3. 用 “Validation / certificate unit” 替代 “Strength / Limitation”

当前表里 `Strength` 和 `Limitation` 有点主观。比如：

* PRM 的 limitation 写成 “Edge checks; zero volume”；
* IRIS/GCS 的 limitation 写成 “Costly inflation”；
* RBF 的 limitation 写成 “Conservative boxes”。

这些是对的，但比较粗。TRO 审稿人更关心：**collision evidence 到底附着在哪个对象上？**

所以建议改成：

```latex
Validation / certificate unit
```

这样可以清楚说明：

* PRM：edge / segment；
* RRT：incremental segment；
* BIT* / FMT*：candidate graph edge；
* IRIS/GCS：convex region；
* RBF：joint box via link-envelope validation；
* repaired / smoothed RBF path：final audit required。

这和你论文中的理论保证直接对齐。

---

## 4. 增加 “Scene-edit reuse”，强化 local update 贡献

RBF 的贡献不仅是 repeated query，还包括 scene edits 后的 local maintenance。当前 Table I 没有很好体现这一点。

建议加一列：

```latex
Scene-edit reuse
```

这列能说明：

* PRM：roadmap topology 可保留，但受影响 edges 要重新检查；
* RRT：基本不能复用；
* IRIS/GCS：regions 与 obstacle layout 强耦合，局部编辑可能需要 reinflation；
* RBF：scene boxes 局部 invalidation/refill，LECT evidence 不随 obstacle 改变而失效。

这会让 Table I 和后文 dynamic update 实验更好地呼应。

---

## 5. 增加 “Adaptive cell decomposition” 行

我强烈建议加这一行。

当前论文后文承认 RBF 继承 adaptive cell decomposition / subdivision planning 的思想，但 Table I 没有把这一类方法列出来。对审稿人来说，这其实是最接近 RBF 的传统范式之一。如果不列，可能会被认为你在 Table I 中只选了对 RBF 有利的比较对象。

建议加：

```latex
Adaptive cell decomposition
```

其 trade-off 可以写成：

> Volumetric representation, but high-DOF manipulation needs a cheap cell-validation oracle.

这能自然引出 RBF 的真正贡献：

> RBF is not merely cell decomposition; it supplies a link-envelope oracle and a two-level cache structure for high-DOF manipulation.

---

# 当前 Table I 中具体应改的地方

## PRM / LazyPRM 行

当前写法：

> Roadmap / Multi-query graph / Edge checks; zero volume / RBF reuses validated boxes rather than only edges.

建议改为：

> Roadmap vertices and edges / No volume / local edge or segment validation / topology reusable but affected edges require rechecking.

这样更客观，也避免 “RBF reuses...” 这种每行重复的写法。

---

## RRTConnect / RRT 行

当前写法：

> Query tree / Fast discovery / Little persistent reuse / RBF turns frontier growth into reusable box growth.

这行基本合理，但可以更精确：

> Query tree / No volume / incremental segment checks / little persistent reuse across queries.

`Fast discovery` 是优点，但不是 taxonomy 维度；可以放到 trade-off 里。

---

## BIT* / FMT* 行

当前写法：

> Sample graph / Anytime improvement / Query-centric checking / RBF prioritizes reusable low-cost regions.

建议改成：

> Batch sample graph or tree / No volume / candidate graph edges / query-centric; limited scene-level reuse.

这样更清楚地说明 BIT* 和 FMT* 的 persistent object 不是 reusable free-space region。

---

## IRIS / GCS 行

当前写法：

> Convex regions / Optimization-ready graph / Costly inflation / RBF uses cheaper boxes instead of rich convex sets.

建议改成：

> Certified convex regions and graph / Yes volume / convex free-space regions / mostly scene-coupled; edits may require reinflation or recertification.

这样更准确。IRIS/GCS 的问题不只是 “costly inflation”，还包括局部 obstacle edit 后 region certificate 的维护成本。

---

## RBF 行

当前写法：

> Box forest + LECT / Volume reuse / Conservative boxes / CCD-style link envelopes validate C-space boxes; scene and kinematic caches remain separate.

这里最大问题是 `Conservative boxes` 作为 limitation 太含糊。建议明确写成：

> Low-build volumetric reuse with lower regional expressivity and conditional certificates.

或者：

> Axis-aligned boxes are less expressive than convex polytopes; repaired or filtered paths require final validation.

这样比 “Conservative boxes” 更准确，也更诚实。

---

# 如果想保持更短，可以用精简版

如果你担心 `table*` 太宽，可以用下面这个五列表版本：

```latex
\begin{table*}[t]
\centering
\caption{Qualitative taxonomy of reusable manipulation-planning paradigms.
RBF is positioned by the object it reuses and the granularity at which
collision evidence is stored.}
\label{tab:reuse_taxonomy_short}
\scriptsize
\setlength{\tabcolsep}{4pt}
\renewcommand{\arraystretch}{1.15}
\begin{tabularx}{\textwidth}{l l c l X}
\toprule
Family & Reused object & C-space volume & Check / certificate unit & Main distinction \\
\midrule
PRM / LazyPRM & Roadmap & No & Edges or local segments &
Reusable graph connectivity, but no volumetric free-space regions. \\

RRTConnect / RRT & Query tree & No & Incremental segments &
Fast one-shot discovery with little persistent reuse. \\

BIT* / FMT* & Batch sample graph/tree & No & Candidate edges &
Anytime query improvement, but still edge-centric. \\

Adaptive cell decomposition & C-space cells & Yes & Cells &
Volumetric, but high-DOF manipulation needs a cheap validation oracle. \\

IRIS / GCS & Convex regions + graph & Yes & Convex regions &
Optimization-ready regions with higher construction and edit cost. \\

\textbf{RBF} & \textbf{Box forest + typed graph + LECT} & \textbf{Yes} &
\textbf{Joint boxes + audited paths} &
\textbf{Cheap volumetric reuse via link envelopes, with lower expressivity and conditional certificates.} \\
\bottomrule
\end{tabularx}
\end{table*}
```

这个版本更适合 Introduction，因为它短、直观、不会抢正文篇幅。

---

# 表题也建议改

当前 caption：

> Qualitative comparison of reusable manipulation-planning paradigms.

建议改成：

```latex
\caption{Qualitative taxonomy of reusable manipulation-planning paradigms.
The comparison emphasizes the persistent object, validation granularity,
and scene-edit semantics rather than ranking planners by runtime.}
```

这样可以避免审稿人误解为“作者声称 RBF 全面优于 PRM/RRT/BIT*/IRIS”。

---

# 最推荐的修改方案

我建议采用 **六列表版本**：

| Family | Persistent object | Vol. | Validation / certificate unit | Scene-edit reuse | Main trade-off |
| ------ | ----------------- | ---: | ----------------------------- | ---------------- | -------------- |

并做三点关键变化：

1. **删除 `Strength / Limitation / Relation to RBF` 三列**，换成更客观的技术维度。
2. **新增 `Vol.` 和 `Scene-edit reuse` 两列**，直接突出 RBF 的核心定位。
3. **新增 `Adaptive cell decomposition` 行**，避免遗漏与 RBF 最接近的传统方法。

这样 Table I 会从“宣传式对比表”变成“设计空间定位表”，更容易被 TRO 审稿人接受。
