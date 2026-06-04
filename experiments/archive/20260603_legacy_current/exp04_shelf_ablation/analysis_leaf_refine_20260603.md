# Exp04 Leaf-Refine Ablation Analysis

Current artifact:

`outputs/new_experiments/exp04_leaf_refine_ablation_opt_box200_full_20260603/`

Protocol:

- Exp.4 Shelf+IIWA combined scene, five fixed query pairs.
- Fast baseline uses `leaf8->14 + deep box200/d28` with read-only d23 external
  evidence replay.
- Quality trade-off also reports `box400/d28` and `box500/d24`.
- All non-baseline comparison rows disable external evidence and active endpoint
  evidence cache.
- Query shortcut boxes and collision shortcut are disabled.
- Planning time excludes query audit time. Success means every query passes the
  strict final audit.

## Results

| row | ok | planning median ms | route median | segment median | target3 max | notes |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| baseline d23 SH box200 | 8/8 | 175.7 | 12.760 | 0.021 | 0.023 | fastest audited baseline |
| baseline d23 SH box400 | 8/8 | 259.5 | 12.760 | 0.021 | 0.023 | same route, higher refinement cost |
| baseline d23 SH box500/d24 | 8/8 | 380.7 | 12.158 | 0.000 | 0.000 | zero-segment quality point |
| no-cache SH box200 | 8/8 | 257.1 | 13.363 | 0.119 | 0.043 | same refinement budget, no cache |
| no-cache AABB box200 | 8/8 | 258.0 | 13.140 | 0.121 | 0.045 | similar runtime/path, more target3 segment |
| no-cache SH single-thread | 8/8 | 353.2 | 13.363 | 0.119 | 0.043 | leaf sweep cost dominates slowdown |
| no-cache SH no anchor roots | 0/8 | 247.2 | NA | NA | NA | fails CS->LB and LB->RB in every seed |
| no-cache SH serial leaf | 8/8 | 357.6 | 13.363 | 0.119 | 0.043 | isolates virtual leaf parallelism |

Cache diagnostics validate the intended cache split:

- baseline seed0: `leaf_sweep.worker_oracle.materialization_reused_external_evidence = 19876`
- no-cache SH seed0: `leaf_sweep.worker_oracle.materialization_reused_external_evidence = 0`
- no-cache SH seed0: `leaf_sweep.worker_oracle.materialization_external_lookup_time_us = 0`

## Supported Claims

1. The optimized Exp.4 baseline is strict-audit reliable.

   All baseline trade-off rows pass `8/8` seeds. The box200 point is the fastest
   audited point. The box500/d24 point provides a zero-segment, shorter-path
   quality point.

2. d23 external evidence replay improves the full planning curve in this shelf
   protocol.

   At the same box200/d28 leaf-refine budget, baseline d23 SupportHull is faster
   than no-cache SupportHull (`175.7 ms` vs `257.1 ms`) and produces a shorter
   route (`12.760` vs `13.363`). Diagnostics confirm real external evidence
   reuse in the baseline and no external reuse in the comparison row.

3. The AABB comparison no longer supports a simple SupportHull path-length
   advantage at box200.

   No-cache AABB and no-cache SupportHull are essentially tied in runtime
   (`258.0 ms` vs `257.1 ms`). AABB has a slightly shorter median route
   (`13.140` vs `13.363`) but slightly higher segment usage, including target3
   segment fraction (`0.045` vs `0.043`). The defensible statement is therefore
   that the optimized shelf result is driven by the d23 SupportHull baseline and
   anchor-guided restricted refinement, not by universal SupportHull dominance
   over AABB.

4. Priority-anchor roots are a necessary mechanism in the current restricted
   refinement design.

   Disabling anchor roots fails `0/8`, specifically the `CS->LB` and `LB->RB`
   transitions. This is a semantic mechanism result, not a random failure.

5. Parallel virtual leaf validation is material to the reported runtime.

   The serial leaf row and single-thread row run around `354-358 ms`, compared
   with `257 ms` for the no-cache 8-thread row. The main delta is leaf sweep
   time (`~126-129 ms` serial vs `~27 ms` parallel).

## Claims Not Supported

1. The box200 baseline should not be described as box-only.

   It passes strict audit, but its route uses a small amount of segment edge
   length (`2.1%` overall, `2.3%` target3 max). Use the box500/d24 point if the
   manuscript needs a zero-segment result.

2. The data do not support "SupportHull is always better than AABB".

   At the no-cache box200 budget, AABB has nearly the same runtime and slightly
   shorter median route, while SupportHull has slightly lower segment fraction.
   The paper should avoid a broad representation dominance claim from this row.

3. The no-anchor row should not be treated as a weak tuning point.

   It fails deterministically across all seeds. This directly supports keeping
   unrestricted priority-anchor roots in restricted domains.

## Paper-Facing Recommendation

Use `baseline_d23_sh_8t_leaf8_14_box200_d28` as the fast baseline point and
report `baseline_d23_sh_8t_leaf8_14_box500_d24` as the zero-segment quality
point. Frame AABB as a close no-cache representation comparison rather than as a
clear loser; the robust mechanism claims are d23 replay, anchor roots, and
parallel virtual leaf validation.
