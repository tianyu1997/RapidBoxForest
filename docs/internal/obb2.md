目标可以明确改成：

segment edge / RRT bridge 不再只是一条 collision-checked 线，而是尽量转成一个或多个 certified C-space OBB tubes。
	​


这样一次 query repair 生成的东西不再只是 single-use segment witness，而是一个可复用的体积区域。关键有两个问题：

OBB 的方向怎么选；
OBB 如何快速膨胀到“足够大但还能 certify”的尺寸。

我建议把 OBB 作为 query-local volumetric bridge certificate，优先用于：

segment edge 的体积化；
RRT bridge path 的 tube cover；
CS→LB、LB→RB 这类 bad transition；
successful query repair 的 sliced promotion。

不要一开始全局替换 AABB forest。全局 forest 仍用 AABB / partition；OBB 作为 hidden region 或 typed bridge edge 存在。

1. 先在 scaled joint space 里定义 OBB

不要直接在原始 joint 坐标里做 OBB。应该先定义一个 scaled coordinate：

y=S(q−q
ref
	​

),

其中 S 是 joint scaling matrix。最简单可以取：

S
jj
	​

=
q
j
max
	​

−q
j
min
	​

1
	​

.

更好一点，可以用 link-motion scaling：

S
jj
	​

∝
ℓ
max
	​

joint j 对 link ℓ 的 workspace motion sensitivity.

之后 OBB 在 y-space 里表示：

R
y
	​

=c+Udiag(h)[−1,1]
n
.

映射回 joint space：

q=q
ref
	​

+S
−1
y.

这样 OBB 的方向和半径不会被某个量纲较大的 joint 人为支配。

2. OBB 方向选择：不要只用 PCA，要用 “path tangent + risk-aware transverse axes”

对于一个 segment edge：

q
a
	​

→q
b
	​

,

第一根轴应该是 path tangent：

u
1
	​

=
∥S(q
b
	​

−q
a
	​

)∥
S(q
b
	​

−q
a
	​

)
	​

.

这是 OBB 的长轴。

对于 RRT bridge path：

q
0
	​

,q
1
	​

,…,q
m
	​

,

如果 path window 比较直，可以仍然用 endpoint tangent：

u
1
	​

=
∥S(q
m
	​

−q
0
	​

)∥
S(q
m
	​

−q
0
	​

)
	​

.

如果 path window 有弯曲，则用 PCA 的第一主方向：

u
1
	​

=principal eigenvector of 
i
∑
	​

w
i
	​

(y
i
	​

−
y
ˉ
	​

)(y
i
	​

−
y
ˉ
	​

)
⊤
.

但是，横向轴不要直接用 PCA 的剩余轴。PCA 的小特征值方向经常是数值噪声，对包络松紧没有实际意义。横向轴应该按“哪个方向膨胀更容易通过 validation”来选。

3. 横向轴：用 risk matrix 选低风险方向

对当前 OBB 中心 q
c
	​

，构造一个 local risk matrix：

R=λI+
(ℓ,o)∈β
∑
	​

w
ℓo
	​

J
ℓ
⊤
	​

n
ℓo
	​

n
ℓo
⊤
	​

J
ℓ
	​

+μ
ℓ
∑
	​

J
ℓ
⊤
	​

J
ℓ
	​

.

含义：

J
ℓ
	​

：link endpoint / keypoint 对 joint 的 Jacobian；
n
ℓo
	​

：当前 blocker link-obstacle 的 workspace 接触方向或近似分离方向；
J
ℓ
⊤
	​

n
ℓo
	​

：在 C-space 里沿哪个方向动会让该 blocker 最敏感；
R 越大，表示该方向越不适合膨胀。

然后在长轴 u
1
	​

 的正交子空间里求：

P=I−u
1
	​

u
1
⊤
	​

,
PRP.

横向轴取 PRP 的小特征值方向：

u
2
	​

,u
3
	​

,…,u
n
	​

=low-risk eigenvectors.

也就是说：

长轴沿 transition；横向轴沿低 blocker / 低 link-motion sensitivity 的方向。
	​


这比纯 PCA 更适合生成大 OBB，因为你膨胀的是最不容易撞障碍、也最不容易让 envelope 变松的方向。

4. 最稳做法：生成多个 orientation candidates，快速试膨胀后选最好的

我建议不要只生成一个 OBB 方向。对每个 bad transition / segment / RRT window，生成几个候选：

Segment-tangent + risk-aware transverse；
PCA axes of local RRT samples；
Segment-tangent + partition-axis transverse；
Segment-tangent + blocker-aware transverse；
AABB baseline orientation，即 U=I。

然后给每个候选一个很小的 validation budget，例如 8 到 16 次 cell/OBB validation，跑一次 quick grow，计算：

score=αlogvol(R)+βN
segments_covered
	​

+γN
bridge_edges_replaced
	​

−ηN
validations
	​

−ρN
hidden
	​

.

选择 score 最大的 orientation，再进入完整 grow。

伪代码：

ChooseOBBOrientation(path_window, blockers):
    candidates ← []

    candidates.push(SegmentTangentRiskAxes(path_window, blockers))
    candidates.push(PCAAxes(path_window))
    candidates.push(SegmentTangentPartitionAxes(path_window))
    candidates.push(SegmentTangentBlockerAxes(path_window, blockers))
    candidates.push(AxisAligned())

    best ← none

    for U in candidates:
        O ← MinimalOBBContainingPath(path_window, U)
        Oquick ← QuickGrowOBB(O, small_budget)

        if Oquick.valid and Score(Oquick) > Score(best):
            best ← Oquick

    return best.orientation

这会比手动押注某一种方向稳很多。

5. 初始 OBB：必须先包含原始 segment / RRT path

给定 orientation U，把 path samples 投影到 OBB 坐标：

z
i
	​

=U
⊤
(y
i
	​

−c).

最小包含 path 的 OBB 半径是：

h
k
min
	​

=
2
1
	​

(
i
max
	​

z
i,k
	​

−
i
min
	​

z
i,k
	​

)+ϵ
k
	​

.

中心也可以用投影区间中点反推：

c=
k
∑
	​

2
max
i
	​

z
i,k
	​

+min
i
	​

z
i,k
	​

	​

u
k
	​

.

对单个 segment，最简单是：

c=
2
1
	​

(y
a
	​

+y
b
	​

),
h
1
min
	​

=
2
1
	​

∥y
b
	​

−y
a
	​

∥+ϵ
∥
	​

,
h
k>1
min
	​

=ϵ
⊥
	​

.

其中 ϵ
⊥
	​

 不要为 0，否则它只是 segment witness，不是可复用体积。可以从：

epsilon_perp = 0.005 ~ 0.02 rad in scaled joint metric

开始，之后自动膨胀。

6. OBB 的最大半径上界：joint limits + local scope

不要无限膨胀。先算 hard upper bound。

设 joint-space OBB 为：

q=q
c
	​

+Adiag(h)ξ,ξ∈[−1,1]
n
,

其中：

A=S
−1
U.

第 j 个 joint 的最大偏移是：

r
j
	​

(h)=
k
∑
	​

∣A
jk
	​

∣h
k
	​

.

要保证 OBB 在 joint limits 内：

r
j
	​

(h)≤min(q
c,j
	​

−q
j
min
	​

, q
j
max
	​

−q
c,j
	​

).

如果还想限制它不跑出当前 HiPaC parent mixed cell / transition scope，也用同样形式加约束：

q∈C
scope
	​

.

所以每次膨胀前都检查：

k
∑
	​

∣A
jk
	​

∣h
k
	​

≤m
j
	​

.

这一步很重要。否则 OBB 可能长到 query 无关区域，validation 变贵且没有收益。

7. 快速膨胀：exponential grow + binary search + coordinate polish

不要从很小半径一点点加。推荐三阶段。

7.1 Phase A：整体 transverse exponential grow

保持长轴半径 h
1
	​

 至少包含 segment/path，先同时膨胀横向半径：

h
2:n
	​

←γh
2:n
	​

,γ=1.5 or 2.

每次调用：

ValidateOBB(O)

如果通过，继续乘 γ。如果失败，在 last-good 和 first-fail 之间做 binary search。

伪代码：

GrowTransverseIsotropic(O):
    good ← O

    while budget remains:
        cand ← scale_transverse(good, gamma)

        if violates_hard_bounds(cand):
            break

        if ValidateOBB(cand):
            good ← cand
        else:
            return BinarySearch(good, cand)

    return good

这能很快找到一个接近可行边界的 transverse tube。

7.2 Phase B：逐轴 coordinate grow

整体膨胀后，再逐个轴增长。轴顺序按低风险到高风险：

u
k
	​

 with smaller u
k
⊤
	​

Ru
k
	​

 first.

对每个轴：

GrowAxis(O, k):
    lo ← current h_k
    hi ← hard upper bound for h_k

    exponential search until fail
    binary search last good / first fail

这样可以得到高度 anisotropic 的 OBB：

安全方向半径很大；
blocker-sensitive 方向半径小；
整体 volume 比 isotropic tube 大很多。
7.3 Phase C：blocker-aware polish

如果 validation fail 返回 blocker signature：

β(O)={(ℓ,o,overlap,slack)},

计算每个 OBB 轴对 blocker 的敏感度：

s
k
	​

=
(ℓ,o)∈β
∑
	​

∣n
ℓo
⊤
	​

J
ℓ
	​

A
k
	​

∣h
k
	​

.

失败后不要简单整体缩小，而是：

freeze 高 s
k
	​

 的轴；
继续尝试低 s
k
	​

 轴；
或者 split 高 s
k
	​

 轴。

这样能快速得到“能 certify 的最大 anisotropic region”。

8. 什么叫“尽量大的 OBB”

不要追求全局最大体积，这个问题太贵。建议定义一个工程上可证明的 maximality：

在固定 orientation、固定 scope、固定 validation budget 下，OBB 是 coordinate-wise maximal。
	​


也就是返回时满足：

for every active axis k:
    trying to increase h_k by (1 + eta) either
    violates hard bounds, exceeds budget, or fails validation

这足够写论文，也足够工程上稳定。

可以记录：

maximal_reason[k] =
    JOINT_LIMIT |
    SCOPE_LIMIT |
    VALIDATION_FAIL |
    BUDGET_LIMIT |
    BLOCKER_FROZEN

这样 debug 很方便。

9. Segment edge 如何用 OBB 覆盖

当前 segment edge E
s
	​

 是 explicit segment witness。建议新增一种 edge：

E
obb
	​


或者更一般：

E
region
	​

.

对一个 segment edge：

e
s
	​

=(B
a
	​

,B
b
	​

,q
a
	​

,q
b
	​

),

其中：

q
a
	​

∈B
a
	​

,q
b
	​

∈B
b
	​

.

构造一个 OBB tube：

R
e
	​

⊃[q
a
	​

,q
b
	​

].

如果 OBB validation 成功：

R
e
	​

⊆C
free
	​

,

则用它替代 segment edge：

E_s  ->  E_obb

并存 overlap witnesses：

q
a
	​

∈B
a
	​

∩R
e
	​

,
q
b
	​

∈R
e
	​

∩B
b
	​

.

这样它不再只是 sampled segment witness，而是一个 conservative volumetric bridge。

伪代码：

TryCoverSegmentEdgeWithOBB(edge):
    qa, qb ← edge.witness_endpoints

    U ← ChooseOBBOrientation({qa, qb}, edge.blockers)
    O ← MinimalOBBContainingSegment(qa, qb, U)

    O ← GrowOBBMaximal(O, scope=edge.local_scope)

    if ValidateOBB(O):
        return OBBEdge(
            source_box=edge.source,
            target_box=edge.target,
            region=O,
            witnesses={qa, qb}
        )

    return original_segment_edge

如果单个 OBB 不通过，可以递归 split segment：

[q
a
	​

,q
b
	​

]→[q
a
	​

,q
m
	​

]∪[q
m
	​

,q
b
	​

].

然后尝试两个更短的 OBB。
这比保留很多 segment witnesses 更可复用。

10. RRT bridge 如何用 OBB 覆盖

对 RRT bridge path：

π=q
0
	​

,q
1
	​

,…,q
m
	​

,

不要尝试一个 OBB 覆盖整条 path。应该用 greedy longest valid window。

CoverRRTBridgeWithOBBs(path):
    i ← 0
    regions ← []

    while i < m:
        j ← GrowWindowEnd(path, i)

        O ← FitOBBToPathWindow(q_i ... q_j)
        O ← GrowOBBMaximal(O)

        if O valid:
            regions.push(O)
            i ← j
        else:
            if j == i+1:
                keep original segment edge q_i -> q_{i+1}
                i ← i+1
            else:
                shrink j and retry

    regions ← MergeAdjacentOBBs(regions)

    return regions

GrowWindowEnd 可以这样做：

从 j=i+1 开始；
指数增加 window 长度；
每次 fit OBB + quick validate；
第一次失败后 binary search 最大 j。

这样能自动把 RRT path 压缩成尽量少的 OBB tubes。

11. RRT window 的方向选择

对 path window：

q
i
	​

,…,q
j
	​

,

先判断曲率。如果局部 path 很直：

∠(q
k+1
	​

−q
k
	​

, q
k
	​

−q
k−1
	​

)<θ
max
	​

,

用 endpoint tangent：

u
1
	​

=
∥S(q
j
	​

−q
i
	​

)∥
S(q
j
	​

−q
i
	​

)
	​

.

如果 path 弯曲明显，则用 PCA：

u
1
	​

=first PCA axis of {Sq
i
	​

,…,Sq
j
	​

}.

但 window 弯曲过大时，不要强行一个 OBB；直接 split。经验上：

if max_turn_angle > 20°~30°:
    split window

否则 OBB 会为了覆盖弯曲 path 变得很胖，validation 反而更难。

12. OBB validation 的两种模式
12.1 快速落地：OBB by AABB cover

把 OBB 在局部坐标 ξ 中切成小块：

[−1,1]
n
=
r
⋃
	​

X
r
	​

.

每个 X
r
	​

 映射回 joint space后取 AABB：

I
r
	​

=bbox
q
	​

(q
c
	​

+AX
r
	​

).

用现有 AABB validation 验证每个 I
r
	​

。如果全部通过，则 OBB free。

优点：最安全，复用现有 pipeline。
缺点：高维斜 OBB 可能需要较多 tiles。

适合第一版。

12.2 更强版本：OBB Taylor / zonotope envelope

直接在 OBB 坐标里做 FK envelope：

q=q
c
	​

+Aξ,ξ∈[−1,1]
n
.

对 endpoint：

p(q)≈p(q
c
	​

)+J(q
c
	​

)Aξ+R(ξ).

得到 endpoint zonotope：

E=p(q
c
	​

)+J(q
c
	​

)A[−1,1]
n
⊕[−ρ,ρ].

然后继续生成 link envelope / SupportHull / GJK narrow phase。

这个版本能真正保留 joint correlation，最适合 segment / RRT bridge OBB。
如果你想让 OBB 明显优于 AABB，这是最终应该做的版本。

13. 用 OBB 覆盖 segment / RRT bridge 的关键收益

当前 segment edge 或 RRT bridge 是：

一条线

只能服务当前 query。
OBB cover 后变成：

一块 certified tube

可以服务相似 query。

尤其对 CS→LB、LB→RB 这种 transition：

q
2
	​

↑, q
4
	​

↓, q
6
	​

↑

这类 correlated motion，AABB 会包含很多无关 joint combinations：

q
2
	​

 已到终点,q
4
	​

 仍在起点,q
6
	​

 在中间

而 OBB tube 不包含这些组合。
所以 OBB envelope 会比 AABB envelope 紧很多，更容易 certify 成较大的局部 corridor。

14. Promotion 逻辑也要改成 OBB-first

当 heavy bridge 或 RRT bridge 成功后，不要再尝试整条 path promotion。应该这样：

PromoteBridgeWithOBB(path):
    bad_windows ← ExtractBadTransitions(path)

    for W in bad_windows:
        if not PromotionGate(W):
            continue

        obb_cover ← CoverRRTBridgeWindowWithOBB(W)

        if obb_cover.success:
            InsertOBBCorridor(obb_cover)

Promotion gate 可以很严格：

require transition_type in {CS_TO_LB, LB_TO_RB}
require window_parent_cells <= 2
require window_segment_count >= 4
require estimated_bridge_edges_replaced >= 16
require max_obb_validations <= 64 or 96
disable FFB fallback

这样不会再出现：

PromCell ≈ 512, PromAdd = 0

而是大量候选被便宜过滤，少数局部 transition 被 OBB tube 成功吸收。

15. 推荐新增 edge / region 类型

可以把内部结构泛化为：

enum class CspaceRegionKind {
    JointAABB,
    JointOBB
};

enum class VolumetricEdgeKind {
    BoxCorridor,
    PortalCorridor,
    SegmentOBBCorridor,
    RRTBridgeOBBCorridor,
    TransitionOBBCorridor
};

每个 OBB corridor 存：

struct CspaceOBBRegion {
    Vec q_center;
    Mat axes_scaled;      // columns are U in scaled joint space
    Vec halfwidth;
    int source_transition_id;
    int scope_cell_or_cluster;
};

struct OBBCorridor {
    VolumetricEdgeKind kind;
    std::vector<CspaceOBBRegion> regions;
    std::vector<CertificateRef> certs;
    std::vector<OverlapWitness> witnesses;
    int replaced_segment_edges_estimate;
    int validation_count;
};

对于 graph path，OBB corridor 使用时展开为：

R
1
	​

,…,R
k
	​


每个 R
i
	​

 都是 certified free convex region。相邻 region 之间用 witness 连接。

16. OBB 和原来的 segment edge theorem 关系

如果 OBB validation 成功，那么这个 edge 已经不应该再算普通 E
s
	​

。它可以进入 conservative volumetric corridor 类。

原来：

E
s
	​

=explicit segment witness

现在：

E
obb
	​

=certified convex C-space region chain.

只要每个 OBB region 都有 conservative link-envelope certificate：

R
i
	​

⊆C
free
	​

,

且相邻 region overlap 或有 witness：

R
i
	​

∩R
i+1
	​


=∅,

那么它和 box corridor 一样可以展开成 conservative corridor。
如果 OBB validation 失败，则保留原来的 segment edge，不影响正确性。

17. 快速膨胀的完整伪代码
GrowMaximalOBB(path_window, scope, blockers, budget):
    // 1. Choose orientation
    U ← ChooseOBBOrientation(path_window, blockers)

    // 2. Build minimal OBB containing path window
    O ← MinimalOBB(path_window, U)

    // 3. Compute hard bounds
    h_max ← ComputeHardBounds(O, joint_limits, scope)

    // 4. Validate minimal tube
    if not ValidateOBB(O):
        if window has more than one segment:
            return SPLIT_WINDOW
        else:
            return FAIL_KEEP_SEGMENT

    // 5. Isotropic transverse grow
    O ← GrowTransverseIsotropic(O, h_max, budget)

    // 6. Coordinate-wise grow
    axes ← sort_axes_by_low_risk(U, blockers)

    for k in axes:
        O ← GrowSingleAxis(O, k, h_max[k], budget)

    // 7. Blocker-aware polish
    O ← TryGrowUnblockedAxes(O, blockers, budget)

    return O

ValidateOBB 必须只在成功时产生 certificate。失败只用于调整，不用于证明 occupied。

18. 推荐实验配置

建议新增三组 ablation。

18.1 Segment edge OBB cover
--segment-edge-obb-cover
--obb-orientation best-of
--obb-grow coordinate-max
--obb-max-validations 64

指标：

segment_edges_before
segment_edges_after
segment_obb_edges_added
segment_obb_used_in_final_path
segment_obb_validations
18.2 RRT bridge OBB cover
--rrt-bridge-obb-cover
--rrt-bridge-obb-greedy-window
--obb-max-window-segments 16
--obb-max-validations-per-window 96

指标：

rrt_bridge_segments
rrt_bridge_obb_regions
rrt_bridge_obb_success_rate
bridge_edges_replaced
query_bridge_added
online/q
18.3 Transition OBB cover
--hipac-online-transition-obb
--transition-types CS_TO_LB,LB_TO_RB
--transition-obb-max-parent-cells 2
--transition-obb-max-validations 64

指标：

transition_obb_attempts
transition_obb_success
transition_obb_used_in_path
query_bridge_added
segment_edges

最重要的成功信号是：

query_bridge_added

和：

segment_edges

下降，而不是单纯 OBB 数量增加。

19. 默认策略建议

我建议优先级如下：

第一优先级：segment edge → OBB tube

每条已有 segment edge 都可以尝试 OBB 覆盖。
成功则升级成 volumetric edge；失败保留 segment edge。

这是最小侵入式改动。

第二优先级：RRT bridge path → greedy OBB corridor

RRT 生成成功 path 后，用 greedy longest-window OBB cover 压缩成少数 OBB tubes。
这能把一次性 RRT bridge 变成 reusable corridor。

第三优先级：bad transition → OBB prebridge

对 CS→LB、LB→RB 这种高成本 transition，在 heavy bridge 前尝试 OBB tube。
成功则直接减少 heavy bridge 工作量。

第四优先级：OBB Taylor-zonotope envelope

先用 AABB cover 证明概念，再上真正 correlated OBB envelope。
后者才是 OBB 相对 AABB 的最大收益来源。

20. 一句话总结

OBB 的方向应该这样选：

长轴沿 segment / RRT window / bad transition tangent，横向轴沿低 blocker、低 link-motion sensitivity 方向。
	​


OBB 的尺寸应该这样长：

先取包含原始 segment/path 的最小 OBB，再在 joint-limit 和 local scope 内做 exponential + binary + coordinate-wise validation growth。
	​


segment edge 和 RRT bridge 都可以升级为：

certified OBB tube corridor
	​


成功时替代 segment witness，失败时回退原 segment/RRT bridge。这样不会破坏现有 planner，但能把 query repair 生成的桥从“一条一次性线段”变成“可复用的大体积局部自由空间”。