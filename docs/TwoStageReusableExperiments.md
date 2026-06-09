# Two-Stage Reusable RBF Experiments

This note fixes the experiment semantics for the TRO 2026 Exp.4--Exp.6
planner comparisons.

## Planner Semantics

RBF is reported as a reusable planner with two stages.

- Offline build uses only the robot, native planning space, obstacles, and
  LECT oracle/cache. It does not receive query starts, goals, waypoint labels,
  or query-derived priority points.
- Online query receives the saved offline forest and the query batch. Endpoint
  anchoring, query bridge, local corridor repair, graph search, and path audit
  diagnostics are online work.
- Canonical symmetry is internal to LECT. Experiment scripts, planner inputs,
  saved scene catalogs, and reported paths remain in native joint space.

The RBF build call must pass `priority_points=[]` in the two-stage profile.
The manifest must report:

- `offline_query_agnostic_build=true`
- `qroot_pairs_total=0`
- `qroot_uncovered_endpoints=0`

Any nonzero query-root build diagnostic in the offline stage is an experiment
setup error.

## Offline Anchors

Offline random anchors are query-independent seed configurations used to improve
free-space coverage before online queries arrive.

The default generator is deterministic for `(seed, box budget)` and samples
candidate configurations inside the active native planning space. Candidates are
filtered by point collision, then selected for broad coverage using joint-limit
margin, shallow LCA separation under the active split schedule, and normalized
center distance. The default count is 16 selected anchors from 512 candidates.

The C++ build stage certifies selected anchors with FFB before committing root
boxes. Anchor diagnostics record requested and committed roots, FFB success/fail
counts, volume statistics for committed boxes, and island counts before/after
anchor insertion.

## Timing

Use these timing fields consistently.

- `offline_build_s`: wall-clock offline time, including offline anchor
  candidate generation/selection and the C++ forest build.
- `offline_build_profile_s`: C++ forest build profile only.
- `online_batch_s`: total online adaptation plus graph query time for the query
  batch.
- `online_solve_s`: online adaptation plus graph-query solve time, excluding
  fixed OMPL simplification.
- `online_simplify_s`: fixed OMPL simplification time actually consumed by
  successful online query post-processing.
- `online_per_query_s`: `online_batch_s / query_count`.
- `online_solve_per_query_s`: `online_solve_s / query_count`.
- `online_simplify_per_query_s`: `online_simplify_s / query_count`.
- `planning_s`: `offline_build_s + online_batch_s`.
- `amortized_s_k{K}`: `offline_build_s / K + online_per_query_s`.

Final audit time is reported separately and is not included in planning time.
The main paper口径 uses a common 0.01 s OMPL simplification budget for RBF,
PRM, RRTConnect, and BIT*. Appendix/trade-off artifacts sweep
`0, 0.005, 0.01, 0.05` s to show path-quality polish behavior. The 0.05 s
setting is treated as a full-polish upper-budget comparison, not the primary
low-latency online metric.

## Experiment Roles

Exp.4 Shelf Ablation:
Build one Shelf+IIWA RBF per seed/budget/ablation, then run the five shelf
queries online. Tables report Build, Solve/q, Simplify/q, Online/q, Amort@5,
Path, Seg., Boxes, and query-level SR.

Exp.5 Shelf Cross-Algorithm:
RBF uses the Exp.4 two-stage profile. PRM and IRIS/GCS are reusable planners
when available. RRTConnect and BIT* are one-shot online baselines with
`build=0`. The main curve is amortized time/query over K.

Exp.6 Random Multi-Robot:
Use `random_scene_catalog_v7.json`. Each saved obstacle scene contains fixed
obstacles and fixed query records. RBF and PRM build once per obstacle scene;
RRTConnect and BIT* solve each saved query independently. The selected RBF
trade-off point must have full query success, then minimize online per-query
time subject to the registered path-quality rule.

## Artifact Rule

Smoke runs should write to `/tmp` or another scratch output directory. Do not
copy smoke-generated tables or figures into `paper/generated/`. Regenerate
paper-facing assets only after the registered Exp.4--Exp.6 reruns complete.
