# Experiment Principles

## Evidence Chain

The manuscript claim is a planning-systems claim: RapidBoxForest offers a
low-latency reusable box-region planning design point with fixed-resolution
final-audited paths. The experiments must therefore report both the planning
trade-off and the mechanism costs that make the trade-off credible.

## Reproducibility

Every run records:

1. git SHA, command line, runner path, run id, working directory;
2. Python version, platform, hostname, thread-related environment variables;
3. robot, scene, seed, cache path, scene catalog path, and build directory;
4. raw artifact paths and generated table/figure paths.

## Timing

Report wall-clock seconds. `planning_time` excludes final audit. At minimum:

1. `build_s`: reusable structure construction;
2. `query_s`: online query plus allowed method-specific post-processing;
3. `audit_s`: final fixed-step audit;
4. `asset_s`: analysis/table/figure generation, never charged to planning.

## Depth Semantics

All paper-facing cache, leaf-sweep, free-box-finder, and connector/paving depth
values are defined in the relevant LECT tree, not in an external native planning
tree. For robots with symmetry reduction, the canonical LECT tree owns the root
interval, split policy, node ids, box depth, and evidence identity.
Native-space planning, query endpoints, paths, and final audit remain outside
LECT and must not change the meaning of any depth parameter.

This means:

1. `d23` means depth 23 in the canonical LECT root recorded by the cache
   manifest.
2. `leaf_start_depth`, `leaf_max_depth`, `deep_ffb_depth`,
   `ffb_start_depth`, `connector_pave_depth`, and `rbf_max_depth` are LECT
   node depths in the active LECT tree used to generate or certify boxes. They
   are not native-sector-expanded depths.
3. External native/all-sector planning must reuse canonical evidence through
   the LECT evidence adapter's internal canonical map and inverse evidence
   transform; it must not reinterpret `d23` as a native-tree depth.
4. Never compensate for native sector expansion by adding artificial leading
   sector splits and renaming the result as a comparable depth. Such a cache is
   a different LECT tree and must be reported with a different explicit label.
5. Exp.4 Shelf+IIWA prewarm is capped at canonical `d23`. Scripts must refuse
   larger prewarm depths unless a new appendix experiment explicitly studies
   cache-depth scaling.
6. Depth fields must be named and reported as either `lect_*_depth` or
   explicitly documented as LECT tree depths. They must be reported separately
   from native path metrics and must not be used to name the cache unless they
   are the actual canonical prewarm depth.

## Path Quality And Success

Only paths that pass the common fixed-step final audit are eligible for main
path-length statistics. Report success-only mean path length and all-run success
rate. Also report timeout and audit-failure rates when available.

## Random Scenes

Random scenes are generated once and saved. Full runs use `reuse` or `verify`
catalog mode. Missing scenes in full mode are errors.

## SBF Segment Accounting

SBF segment bridges are fallback witnesses. Every SBF planning artifact must
separate box-overlap graph edges from segment edges and report raw no-post
segment fraction before final simplification.

The registered RBF algorithm default is `exp04_leaf_sweep_rrt_b100_d34`: leaf
sweep coverage followed by restricted deep refinement and a bounded RRT grower.
Shelf+IIWA baseline
rows add the d23 warm-cache override; random multi-robot rows do not inherit the
Shelf cache or anchors. Segment edges may be added by the build-stage connector,
but they are charged to planning time and reported as raw segment fraction;
query-time shortcuts and final collision shortcutting are disabled for the main
no-post rows.

## Paper Assets

Tables and figures are generated from artifacts into `paper/generated/`.
The manifest `paper/generated/tro_table_generation_manifest.json` is the source
of truth for table/figure provenance.
