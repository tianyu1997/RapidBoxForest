# C-LECT Paper Change Proposal

This proposal maps the `docs/improve.md` manuscript edits without modifying
`paper/sbf_tro_2026.tex`.

The proposed manuscript name is C-LECT: Compressed Adaptive LECT.

## LECT Section

Add a paragraph introducing sparse dyadic evidence addressing:

```latex
\paragraph{Sparse dyadic addressing.}
The compressed variant keys evidence by robot-compatible joint intervals rather
than by materialized heap nodes. A cell is represented by per-dimension dyadic
levels and indices, so intermediate nodes on an unbranched path need not be
materialized. Scene-specific labels remain outside LECT; only endpoint and link
envelope evidence is cached.
```

## LeafSweepRefine Algorithm

Replace the fixed virtual leaf layer with `AdaptiveLeafSweep` and
`AdaptiveClassify`.  The fixed-depth sweep should be described as a special case
that forces every inconclusive node to split until the terminal depth.

## Optional Occupied-Pruning Lemma

Add the signed-distance witness lemma only as optional pruning:

```latex
\begin{lemma}[Signed-distance occupied pruning]
If a material point on link $\ell$ has signed distance
$\phi_O(T_\ell(q_c)x_0)$ to obstacle $O$ at the cell center and its workspace
motion over cell $I$ is bounded by $\rho_{\ell,x_0}(I)$, then
$\phi_O(T_\ell(q_c)x_0)+\rho_{\ell,x_0}(I)+\epsilon_{\rm num}<0$
implies $I$ is fully occupied.
\end{lemma}
```

State explicitly that the planner falls back to inconclusive domains when this
witness is unavailable.

## Graph Edge Definition

Change the graph edge set to:

```latex
G=(V,E_{\cap}\cup E_{\varepsilon}\cup E_s\cup E_{\pi})
```

where `E_pi` is a conservative portal edge carrying a finite internal chain of
validated boxes.  Add a corollary stating that expanding every portal edge
recovers a standard validated box corridor.
