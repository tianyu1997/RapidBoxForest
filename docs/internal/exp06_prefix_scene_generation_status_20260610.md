# Exp.6 Prefix Scene Generation Status

Date: 2026-06-10

This note summarizes the current work on strict layered random-scene catalogs for Exp.6. The active goal is to generate reusable prefix scenes for iiwa, UR5, and Panda where the same query set is evaluated under easy, medium, and hard obstacle prefixes, and where RRTConnect/BIT* first-solution times satisfy difficulty-specific median gates.

## Target Semantics

- Random scenes must be saved and reused through a catalog.
- Each scene group uses the same query set for easy, medium, and hard.
- Difficulty is selected by obstacle prefix count, not by regenerating unrelated scenes.
- Planner difficulty is gated by strict post-hoc collision checking at `0.01`.
- BIT* and RRTConnect difficulty gates use observed first strict solution time, not timeout budget.
- Query length upper bound is now disabled unless explicitly requested.
- C-space boxes are mapped to workspace obstacles through CritSample-based envelope mapping, not by using pure C-space obstacles directly.

## Code Changes Made

### Query Upper Bound

Changed query distance handling in:

- `experiments/common/generate_prefix_mapped_workspace_catalog.py`
- `experiments/common/augment_prefix_catalog_queries.py`
- `experiments/common/generate_mapped_workspace_catalog.py`

Current behavior:

- `--query-max-l2` defaults to `inf`.
- `--query-max-l2 <= 0` also disables the upper bound.
- Manifest writes:
  - `query_max_l2: null`
  - `query_max_l2_unbounded: true`

This directly implements the latest requirement that the previous query upper bound `5.0` can be removed.

### Prefix Catalog Generator

Updated `generate_prefix_mapped_workspace_catalog.py`:

- Added `query_max_l2_limit`.
- Added support for saving `workspace_mapping.ordered_obstacles`.
- Added `path_blocking_obstacle_bounds(initial_obstacles=...)`.
- Fixed an `initial_obstacles` list handling bug in path blocking.
- Added progress bars for hard-query and post-query phases.
- Added post-query gating modes:
  - `none`: only sample free queries, final prefix validation decides.
  - `hard`: each new query must pass the hard-prefix window.
  - `all`: each new query must individually pass easy/medium/hard windows.
  - `group`: each candidate is checked by adding it to the current accepted query set and testing the growing aggregate median.
- Added post-only window overrides:
  - `--post-query-rrt-median-windows`
  - `--post-query-bitstar-median-windows`

The post-only windows allow stricter candidate filtering than the final paper-facing difficulty window. Example: require candidate hard BIT* `0.200-0.750s` while final hard remains `0.100-0.750s`.

### Query Augmentation Tool

Updated `augment_prefix_catalog_queries.py`:

- Added same `query_max_l2` unbounded semantics.
- Added `--post-query-check-mode {all,hard,group,none}`.
- Added `--post-query-local-prob`.
- Added post-only RRT/BIT* window overrides.
- Manifest records the query augmentation policy and window overrides.
- `record_ordered_obstacles` now uses saved `workspace_mapping.ordered_obstacles` when available.

### Validation

Syntax checks passed:

```bash
python3 -m py_compile \
  experiments/common/generate_mapped_workspace_catalog.py \
  experiments/common/generate_prefix_mapped_workspace_catalog.py \
  experiments/common/augment_prefix_catalog_queries.py
```

## UR5 Result

Valid q10 strict prefix catalog:

- Catalog: `outputs/new_experiments/tro2026/exp06/probe_ur5_p90_q10_hard_catalog.json`
- Summary: `outputs/new_experiments/tro2026/exp06/probe_ur5_p90_q10_hard_summary.json`

Measured prefix medians:

| Robot | Difficulty | Obstacles | Queries | RRTConnect median first strict solution | BIT* median first strict solution |
|---|---:|---:|---:|---:|---:|
| UR5 | easy | 0 | 10 | 0.00119 s | 0.00509 s |
| UR5 | medium | 10 | 10 | 0.08844 s | 0.07136 s |
| UR5 | hard | 12 | 10 | 0.08635 s | 0.16236 s |

This satisfies the current windows:

- easy: BIT* `<0.05s`
- medium: BIT* `0.05-0.10s`
- hard: BIT* `>0.10s`

## iiwa Findings

### Direct-Asc 600 Attempt

Command family: `probe_iiwa_directasc600`.

Outcome:

- Found a hard query.
- Failed to find a strict easy/medium/hard prefix triple.

Key observation:

- Direct-ascending obstacle ordering caused a difficulty jump.
- Most prefixes remained easy.
- Later prefixes jumped directly to hard or infeasible.
- Medium was missing.

Observed representative candidates:

- easy at low counts: BIT* about `0.005s`
- hard around count `555`: RRTConnect `0.143s`, BIT* `0.150s`
- no medium candidate in the required windows

Conclusion:

Direct hit or volume sorting alone is not a stable difficulty curriculum for iiwa.

### Path-Blocking q1 Attempt

Catalog:

- `outputs/new_experiments/tro2026/exp06/probe_iiwa_pathblock_catalog.json`

Valid q1 prefix result:

| Robot | Difficulty | Obstacles | Queries | RRTConnect median first strict solution | BIT* median first strict solution |
|---|---:|---:|---:|---:|---:|
| iiwa | easy | 0 | 1 | 0.00125 s | 0.00510 s |
| iiwa | medium | 9 | 1 | 0.08897 s | 0.07683 s |
| iiwa | hard | 10 | 1 | 0.20870 s | 0.10203 s |

Conclusion:

Path-blocking obstacle ordering can produce a valid continuous prefix ladder for iiwa, but it is expensive. The q1 run took about `257s` because each path-blocking insertion evaluates planner probes.

### iiwa q10 Hard-Only Augmentation

Attempted output:

- `probe_iiwa_pathblock_q10_hard_catalog.json`

Outcome:

- Accepted 9 additional queries quickly under hard-only gating.
- Final q10 prefix validation failed.

Failure reason:

- Individual hard-prefix checks passed.
- The final shared-query aggregate median did not satisfy hard BIT*.
- Hard BIT* median fell to about `0.082s`, below the `0.100s` threshold.

Conclusion:

Hard-only gating is too loose when the final statistic is the median over 10 shared queries.

### iiwa q10 All-Mode Augmentation

Attempted output:

- `probe_iiwa_pathblock_q10_all_catalog.json`

Outcome:

- Very low acceptance rate.
- Stopped before completion.

Failure reason:

- Requiring every candidate query to individually pass easy, medium, and hard windows is too strict.
- It does not match the final aggregate-median semantics well.

Conclusion:

All-mode is too conservative and inefficient.

### iiwa q10 Group-Mode Augmentation

Attempted output:

- `probe_iiwa_pathblock_q10_group_catalog.json`

Outcome:

- Very low acceptance rate.
- Stopped before completion.

Failure reason:

- Each candidate requires three aggregated planner probes over the growing query set.
- Cost was about `3.7-4.5s` per candidate.
- No accepted candidate after the initial observed sample.

Conclusion:

Group-mode has the correct semantics but is currently too expensive and too restrictive for this iiwa catalog.

### iiwa q10 Hard-Only + Path-Blocking Extension

Attempted output:

- `probe_iiwa_pathblock_q10_extend_catalog.json`

Outcome:

- q10 sampling completed.
- Final prefix validation failed.
- One path-blocking extension round added 4 obstacles.
- Final prefix validation still failed.

Important observation:

- RRTConnect became slower as expected.
- BIT* did not become monotonically harder.
- BIT* median stayed below the hard threshold at the high obstacle counts.

Representative final observed values after extension:

- count 9: RRTConnect `0.224s`, BIT* `0.042s`
- count 10: RRTConnect `0.146s`, BIT* `0.049s`
- count 11: RRTConnect `0.255s`, BIT* `0.084s`
- count 14: RRTConnect `0.250s`, BIT* `0.062s`

Conclusion:

Adding path-blocking obstacles after q10 sampling slows RRTConnect but does not reliably slow BIT*. The generator needs to target BIT* difficulty directly, not just direct path blocking or RRT path blocking.

### iiwa q10 Stricter Post Hard Gate

Attempted output:

- `probe_iiwa_pathblock_q10_posthard02_catalog.json`

Policy:

- Candidate hard-prefix BIT* gate: `0.200-0.750s`
- Final hard BIT* gate remains `0.100-0.750s`

Status at document time:

- The run had accepted 2 additional queries by around attempt 131.
- It was still running when this document was written.
- Process observed via `ps -ef`:
  - `timeout` PID: `2401533`
  - Python PID: `2401534`

Interpretation:

The stricter post gate has nonzero acceptance but is slower. It is the most promising current direction for iiwa q10 because it adds margin above the final hard threshold.

## Distribution-Selection Update

The fixed easy/medium/hard time-window policy is now deprecated as the main
catalog-selection rule. It remains useful diagnostic metadata, but final Exp.6
catalog generation should select obstacle prefixes by distribution separation
on a shared q10 query set.

Implemented changes:

- Added `--prefix-selection-mode {distribution,window}` to:
  - `experiments/common/generate_prefix_mapped_workspace_catalog.py`
  - `experiments/common/augment_prefix_catalog_queries.py`
- The default mode is `distribution`.
- Added distribution-separation parameters:
  - `--distribution-medium-ratio`, default `5.0`
  - `--distribution-hard-ratio`, default `2.0`
  - `--distribution-hard-not-faster-factor`, default `0.8`
  - `--distribution-require-strong-planner`, default enabled
- Added selector metadata to each selected record:
  - `difficulty_probe.policy = distribution_separation_v1`
  - `difficulty_probe.distribution_metrics`
  - `difficulty_probe.distribution_selection`
  - `difficulty_probe.distribution_prefix_scan`
  - `workspace_mapping.prefix_selection_mode`
  - `workspace_mapping.obstacle_prefix_count`
  - `workspace_mapping.obstacle_prefix_difficulty`
  - `workspace_mapping.distribution_separation`
- Added unit tests in:
  - `experiments/common/test_distribution_prefix_selector.py`

Selector semantics:

- Inputs are ordered obstacles, one shared query set, planner seeds, and strict
  audit step.
- For each candidate prefix count, the generator measures first strict solution
  times for RRTConnect and BIT*.
- Times are scored in log space with a `0.005s` floor.
- RRTConnect and BIT* medians are normalized and combined into a composite
  difficulty score.
- The selected easy/medium/hard prefix counts must satisfy:
  - composite median order: `easy < medium < hard`
  - composite median ratios: `medium/easy >= 5`, `hard/medium >= 2`
  - quantile separation: `Q75(easy) < Q50(medium)` and `Q75(medium) < Q50(hard)`
  - hard cannot be clearly faster than medium for either reference planner
  - at least one reference planner must satisfy stronger adjacent separation

Validation:

```bash
python3 -m py_compile \
  experiments/common/generate_prefix_mapped_workspace_catalog.py \
  experiments/common/augment_prefix_catalog_queries.py \
  experiments/common/test_distribution_prefix_selector.py

python3 -m unittest experiments.common.test_distribution_prefix_selector
```

Both checks pass.

UR5 q10 distribution smoke:

```bash
python3 experiments/common/augment_prefix_catalog_queries.py \
  --input outputs/new_experiments/tro2026/exp06/probe_ur5_p90_q10_hard_catalog.json \
  --out /tmp/ur5_distribution_validate.json \
  --summary-json /tmp/ur5_distribution_validate_summary.json \
  --queries-per-scene 10 \
  --prefix-selection-mode distribution \
  --prefix-fine-until 12 \
  --prefix-mid-step 1 \
  --prefix-coarse-step 10 \
  --planner-seeds 1 \
  --rrt-probe-timeout-s 0.5 \
  --bitstar-probe-timeout-s 0.75 \
  --bitstar-probe-checkpoint-interval-s 0.005 \
  --bitstar-probe-mode trace \
  --bitstar-samples-per-batch 100 \
  --strict-audit-step 0.01 \
  --min-probe-success-fraction 0.5
```

Result:

| Robot | Difficulty | Prefix count | Composite median | RRTConnect median | BIT* median |
|---|---:|---:|---:|---:|---:|
| UR5 | easy | 0 | 0.00505 s | 0.00116 s | 0.00511 s |
| UR5 | medium | 5 | 0.02642 s | 0.00816 s | 0.04628 s |
| UR5 | hard | 12 | 0.11720 s | 0.09287 s | 0.17463 s |

Distribution checks:

- `medium/easy = 5.23`
- `hard/medium = 4.44`
- both RRTConnect and BIT* satisfy the stronger adjacent-separation check
- all strict probe success fractions were `1.0`

Next required work:

- Use the new distribution selector as the main q10 pipeline for iiwa.
- Run Panda q3 diagnostics first, then q10 final only if the obstacle mapping
  shows separable distributions.
- Revalidate UR5 with `planner_seeds=3` before registering it as the final
  paper catalog.
- Merge only accepted q10 distribution-selected records into the saved Exp.6
  catalog used by non-RBF baselines.

## Q10 Strict Confirm Results

Completed sequential confirm runs with saved catalogs and strict audited
reference-planner probes.

Final merged candidate catalog:

- `outputs/new_experiments/tro2026/exp06/distribution_q10_three_robot_strict_catalog.json`
- Summary:
  `outputs/new_experiments/tro2026/exp06/distribution_q10_three_robot_strict_summary.json`

Source catalogs:

- UR5:
  `outputs/new_experiments/tro2026/exp06/distribution_ur5_q10_seed3_strict_confirm_catalog.json`
- iiwa:
  `outputs/new_experiments/tro2026/exp06/distribution_iiwa_q10_seed20260610_seed3_confirm_catalog.json`
- Panda:
  `outputs/new_experiments/tro2026/exp06/distribution_panda_q10_links234567_seed3_confirm_catalog.json`

Confirm settings:

- `queries_per_scene = 10`
- `planner_seeds = 3`
- `strict_audit_step = 0.01`
- `min_probe_success_fraction = 1.0`
- RRTConnect/BIT* probe rows must be strict-audited successes.
- Panda uses the wider active-link profile `allowed_link_idxs=[2,3,4,5,6,7]`.

Final prefix selections:

| Robot | Easy prefix | Medium prefix | Hard prefix | Medium/Easy | Hard/Medium | Strong planner(s) |
|---|---:|---:|---:|---:|---:|---|
| UR5 | 0 | 3 | 12 | 6.11 | 2.45 | RRTConnect |
| iiwa | 0 | 7 | 10 | 5.25 | 5.29 | RRTConnect |
| Panda | 0 | 5 | 7 | 10.14 | 3.12 | RRTConnect, BIT* |

Composite first-solution medians:

| Robot | Easy | Medium | Hard |
|---|---:|---:|---:|
| UR5 | 0.00505 s | 0.03082 s | 0.07566 s |
| iiwa | 0.00505 s | 0.02649 s | 0.14010 s |
| Panda | 0.00504 s | 0.05117 s | 0.15957 s |

Reference-planner medians:

| Robot | Difficulty | RRTConnect | BIT* |
|---|---|---:|---:|
| UR5 | easy | 0.00116 s | 0.00510 s |
| UR5 | medium | 0.01548 s | 0.05436 s |
| UR5 | hard | 0.07802 s | 0.11674 s |
| iiwa | easy | 0.00116 s | 0.00510 s |
| iiwa | medium | 0.01288 s | 0.05420 s |
| iiwa | hard | 0.27112 s | 0.07896 s |
| Panda | easy | 0.00115 s | 0.00509 s |
| Panda | medium | 0.10865 s | 0.02617 s |
| Panda | hard | 0.33857 s | 0.06221 s |

Validation command:

```bash
python3 - <<'PY'
import json
from pathlib import Path
p = Path('outputs/new_experiments/tro2026/exp06/distribution_q10_three_robot_strict_catalog.json')
data = json.loads(p.read_text())
by = {}
for rec in data['records']:
    by.setdefault(rec['robot'], {})[rec['difficulty']] = rec
for robot, rows in by.items():
    for diff in ['easy', 'medium', 'hard']:
        rec = rows[diff]
        assert len(rec['queries']) == 10
        m = rec['difficulty_probe']['distribution_metrics']
        assert m['rrtconnect']['success_fraction'] == 1.0
        assert m['bitstar']['success_fraction'] == 1.0
    e, m, h = [rows[d]['difficulty_probe']['distribution_metrics']['composite'] for d in ['easy', 'medium', 'hard']]
    assert e['median_s'] < m['median_s'] < h['median_s']
    assert m['median_s'] / max(e['median_s'], 0.005) >= 5.0
    assert h['median_s'] / max(m['median_s'], 0.005) >= 2.0
    assert e['q75_s'] < m['median_s']
    assert m['q75_s'] < h['median_s']
PY
```

Important intermediate observations:

- UR5 seed3 confirm with `0.75s` BIT* timeout gave hard BIT* success fraction
  `0.9667`; rerunning with `1.0s` timeout and `min_probe_success_fraction=1.0`
  produced a valid strict confirm.
- iiwa q10 with the first arbitrary post-sampled query set failed distribution
  separation. Changing the post-sampling seed to `20260610` produced a q10 set
  that passed seed1 screening and seed3 strict confirm.
- Panda `allowed_link_idxs=auto` produced q1 separation but was unstable for q3.
  The wider profile `[2,3,4,5,6,7]` produced q3 separation and q10 strict
  confirm.

## Exp.6 Runner Registration and OMPL Baseline Run

The Exp.6 runner is now registered to use the distribution-selected saved
catalog by default:

- Default catalog:
  `outputs/new_experiments/tro2026/exp06/distribution_q10_three_robot_strict_catalog.json`
- Default catalog mode: `reuse`
- Supported top-level catalog schema:
  `exp06_distribution_prefix_catalog_v1`
- In reuse/verify mode, the runner infers available contiguous scene seeds from
  the saved catalog. This prevents `phase=paper` from silently planning
  nonexistent seeds when the registered catalog intentionally contains one
  shared q10 scene group per robot.

Registration validation:

```bash
python3 experiments/exp06_random_robot/run_random_robot.py \
  --phase paper \
  --dry-run \
  --methods rrtconnect,bitstar \
  --robots iiwa,ur5,panda \
  --difficulties easy,medium,hard \
  --queries-per-scene 10 \
  --out-dir outputs/new_experiments/tro2026/exp06_distribution_baselines_registered_dryrun
```

Dry run result:

- Planned rows: `18`
- Scene seeds: `[0]`
- Methods: `rrtconnect`, `bitstar`
- Catalog path: registered distribution q10 strict catalog

Full RRTConnect and BIT* baseline run:

```bash
python3 experiments/exp06_random_robot/run_random_robot.py \
  --phase paper \
  --methods rrtconnect,bitstar \
  --robots iiwa,ur5,panda \
  --difficulties easy,medium,hard \
  --queries-per-scene 10 \
  --rrt-timeout-s 1.0 \
  --rrt-range 0.35 \
  --bitstar-timeout-s 1.0 \
  --bitstar-checkpoint-interval-s 0.005 \
  --bitstar-samples-per-batch 100 \
  --bitstar-rewire-factor 5.0 \
  --no-bitstar-stop-on-solution-improvement \
  --audit-segment-step 0.01 \
  --audit-collision-tolerance 0.0 \
  --ompl-simplify-time-s 0.01 \
  --out-dir outputs/new_experiments/tro2026/exp06_distribution_baselines_1s
```

Output files:

- `outputs/new_experiments/tro2026/exp06_distribution_baselines_1s/random_robot_manifest.json`
- `outputs/new_experiments/tro2026/exp06_distribution_baselines_1s/random_robot_summary.csv`

RRTConnect results:

| Robot | Difficulty | Success | Planning median | Path length mean |
|---|---|---:|---:|---:|
| iiwa | easy | 10/10 | 0.0120 s | 5.5201 |
| iiwa | medium | 10/10 | 0.1183 s | 11.7045 |
| iiwa | hard | 10/10 | 2.6164 s | 16.9813 |
| UR5 | easy | 10/10 | 0.0128 s | 8.1090 |
| UR5 | medium | 10/10 | 0.1393 s | 9.4399 |
| UR5 | hard | 10/10 | 0.8172 s | 9.5048 |
| Panda | easy | 10/10 | 0.0115 s | 3.8144 |
| Panda | medium | 10/10 | 1.1218 s | 11.6922 |
| Panda | hard | 10/10 | 3.4641 s | 12.5189 |

BIT* first full-success checkpoints and final `1.0s` points:

| Robot | Difficulty | First 10/10 checkpoint | First length | Final 1.0s length |
|---|---|---:|---:|---:|
| iiwa | easy | 0.005 s | 5.5201 | 5.5201 |
| iiwa | medium | 0.525 s | 9.9538 | 9.1698 |
| iiwa | hard | 0.220 s | 14.3307 | 10.0833 |
| UR5 | easy | 0.005 s | 8.1090 | 8.1090 |
| UR5 | medium | 0.215 s | 9.5718 | 9.2196 |
| UR5 | hard | 0.255 s | 9.8711 | 9.5235 |
| Panda | easy | 0.005 s | 3.8144 | 3.8144 |
| Panda | medium | 0.065 s | 11.7298 | 8.2599 |
| Panda | hard | 0.135 s | 14.1222 | 10.8727 |

Notes:

- A preliminary `0.5s` BIT* run left `iiwa/medium` at `9/10`, so the full
  baseline curve was extended to `1.0s`.
- The `1.0s` BIT* run produced 200 checkpoint stages from `0.005s` to `1.0s`.
- Path length among full-success BIT* checkpoints is monotone non-increasing
  under the runner's incumbent-retention logic; zero monotonicity violations
  were detected.
- Early BIT* checkpoint failures are `empty_path` before first connection and
  should not be used as paper display points. Paper figures should display only
  full-success checkpoints, plus the first full-success point where relevant.

## Key Technical Conclusions

### Difficulty Is Not Monotonic in Obstacle Count

The current prefix construction assumes that adding more ordered obstacles creates progressively harder scenes. This is only approximately true. BIT* difficulty can decrease after adding obstacles because:

- the added obstacle may block a difficult homotopy and expose an easier one,
- RRTConnect and BIT* respond differently to the same obstacle set,
- direct path obstruction is not a reliable proxy for BIT* first-solution time,
- narrow workspace obstacles can affect one query strongly and another weakly.

Therefore, each accepted prefix must be validated with planner probes, not inferred from obstacle count alone.

### q1 Validity Does Not Transfer to q10

The iiwa q1 path-blocking catalog is valid, but augmenting it to q10 changes the median behavior. A query set must be generated with the final aggregate statistic in mind.

The safest current options are:

1. Use stricter per-query hard BIT* gates so the final median has margin.
2. Generate a larger candidate pool with stored per-query probe times, then select a subset of 10 offline.
3. Build obstacle prefixes after the full q10 query set is fixed, rather than building q1 first and augmenting later.

### Path Blocking Helps RRTConnect More Than BIT*

The extension experiment showed RRTConnect medians increase substantially, but BIT* medians remain low or non-monotonic. This means future generator scoring should optimize BIT* first-solution time directly.

## Recommended Next Steps

1. Let or rerun `posthard02` to completion if compute time is acceptable.
   - It directly tests whether stricter hard BIT* candidate filtering fixes q10.
   - If it succeeds, use the resulting iiwa catalog.

2. If `posthard02` fails or is too slow, implement candidate-pool selection:
   - Generate 30-50 hard-prefix candidate queries.
   - Store each candidate's easy/medium/hard RRTConnect and BIT* probe times.
   - Select 10 queries whose aggregate medians satisfy all windows.
   - Then run final `find_prefixes` once.

3. For Panda, start with diagnostics instead of full q10:
   - First produce q1 strict prefix.
   - Check whether direct sorting, mixed sorting, or path-blocking creates a continuous prefix.
   - Only then run q10 augmentation.

4. After iiwa/UR5/Panda q10 catalogs are valid:
   - merge them into one saved Exp.6 catalog,
   - run Exp.6 non-RBF baselines on the saved catalog,
   - update Exp.6 paper assets.

## Current Artifact Inventory

Valid:

- `outputs/new_experiments/tro2026/exp06/probe_ur5_p90_q10_hard_catalog.json`
- `outputs/new_experiments/tro2026/exp06/probe_ur5_p90_q10_hard_summary.json`
- `outputs/new_experiments/tro2026/exp06/probe_iiwa_pathblock_catalog.json`

Failed or diagnostic:

- `probe_iiwa_directasc600_*`: no medium prefix.
- `probe_iiwa_pathblock_q10_hard_*`: final hard median too low.
- `probe_iiwa_pathblock_q10_all_*`: acceptance too low.
- `probe_iiwa_pathblock_q10_group_*`: acceptance/cost too high.
- `probe_iiwa_pathblock_q10_extend_*`: RRTConnect slowed, BIT* still too fast.

In progress at document time:

- `probe_iiwa_pathblock_q10_posthard02_*`

## q10x10 Prefix Catalog Revision

The paper-facing Exp.6 catalog target is now stricter than the earlier q10
single-scene catalog:

- 3 robots: iiwa, UR5, Panda.
- 3 difficulties: easy, medium, hard.
- 10 scene seeds per robot/difficulty.
- 10 queries per scene.
- Total final catalog size: 90 records and 900 method-level queries.

For each fixed `(robot, scene_seed)`, the three difficulty records must share
the exact same 10 query records. Difficulty is represented only by a strict
obstacle prefix:

```text
easy_count < medium_count < hard_count
medium_obstacles[:easy_count] == easy_obstacles
hard_obstacles[:medium_count] == medium_obstacles
```

Candidate probes are not valid paper records. A record is accepted only after
strict RRTConnect and BIT* first-solution probes pass with success fraction
`1.0` under `0.01` audit step.

### New Implementation Entry Points

- Candidate/confirm/assemble driver:
  `experiments/common/run_q10x10_prefix_generation.py`
- Strict final catalog assembler and validator:
  `experiments/common/assemble_q10x10_prefix_catalog.py`
- Selector tests:
  `experiments/common/test_distribution_prefix_selector.py`
- Catalog structure tests:
  `experiments/common/test_q10x10_catalog_validation.py`

The final catalog path registered by Exp.6 is:

```text
outputs/new_experiments/tro2026/exp06/distribution_q10x10_three_robot_strict_catalog.json
```

The old single-scene catalog is now diagnostic only:

```text
outputs/new_experiments/tro2026/exp06/distribution_q10_three_robot_strict_catalog.json
```

### Validation Behavior

`run_random_robot.py --phase paper --scene-catalog-mode reuse` now rejects a
catalog with fewer than 10 contiguous scene seeds per selected robot/difficulty
unless `--allow-fewer-catalog-scenes` is explicitly provided for diagnostics.

The reuse/verify path also validates saved prefix catalogs based on catalog
contents, not just the runner's `--scene-profile`. This prevents a prefix
catalog from being accidentally accepted as an independent-scene catalog.

### Smoke Results

Completed checks:

```bash
python3 -m py_compile \
  experiments/common/generate_prefix_mapped_workspace_catalog.py \
  experiments/common/augment_prefix_catalog_queries.py \
  experiments/common/random_scene_catalog.py \
  experiments/exp06_random_robot/run_random_robot.py \
  experiments/common/assemble_q10x10_prefix_catalog.py \
  experiments/common/run_q10x10_prefix_generation.py

PYTHONPATH=. python3 experiments/common/test_distribution_prefix_selector.py
PYTHONPATH=. python3 experiments/common/test_q10x10_catalog_validation.py
```

Runner dry-run now plans 90 runs for RRTConnect under the default three-robot
paper setting:

```bash
python3 experiments/exp06_random_robot/run_random_robot.py \
  --phase paper --dry-run --methods rrtconnect
```

The old 9-record catalog is rejected for paper mode because it contains only
one scene seed per robot/difficulty.
