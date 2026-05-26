# Mandatory Revision Experiment Results, 2026-05-18

This file records the mandatory follow-up experiments selected under the tight
time and page budget. All outputs are under `cpp/SBF/outputs/paper/`.

## M1. Validation Closure and Failure Taxonomy

Command:

```bash
cd /home/tian/桌面/box_aabb/cpp/SBF
/home/tian/miniconda3/envs/sbf/bin/python experiments/tro2026_followup_01_validation_closure.py \
  --outputs outputs/paper \
  --out-json outputs/paper/tro2026_mandatory_validation_closure.json \
  --out-csv outputs/paper/tro2026_mandatory_validation_closure.csv \
  --out-md outputs/paper/tro2026_mandatory_validation_closure.md
```

Key result:

- Scanned 12 paper-facing artifacts.
- Full scanned artifact pool: 8809 paths, 8684 audit-pass records, audit SR 0.9858.
- Paper-wide safety accounting artifact: 668 counted paths, 663 audit-pass paths.
- The five counted audit failures come from the direct merger+GCS negative-control row.
- Audited GCS unsafe solved attempts are tracked separately as rejected attempts and are not credited as planning successes.
- Corridor-containment fields are absent from the current artifacts: `corridor_known_count = 0`.

Conclusion: the current paper can report final-audited success and the closure
scan, but cannot report measured corridor-certified coverage without a rerun
that records path containment in the validated box union.

## M2. Reporting-Rule and Path-Gap Sensitivity

Command:

```bash
cd /home/tian/桌面/box_aabb/cpp/SBF
/home/tian/miniconda3/envs/sbf/bin/python experiments/tro2026_followup_03_reporting_sensitivity.py \
  --outputs outputs/paper \
  --out-json outputs/paper/tro2026_mandatory_reporting_sensitivity.json \
  --out-md outputs/paper/tro2026_mandatory_reporting_sensitivity.md
```

Key result:

- Postprocessed 820 anytime stage points across 50 scenario-method groups.
- The tabulated rows are unchanged for path-tolerance values 5%, 8%, 10%, and 15%.
- A zero-tolerance shortest-path-only rule changes 25/50 rows.
- RBF-SH path gap to the shortest tabulated method:
  - median: 0.0562 rad,
  - p90: 0.2960 rad,
  - max: 0.4549 rad on Shelf+IIWA,
  - best or tied in 3/10 scenario rows.

Conclusion: the current 8% tabulated-row rule is stable over a practical
tolerance range. The table remains a compact numeric slice; the full curves are
still the primary evidence.

## M3. Audit-Resolution Sweep

Command:

```bash
cd /home/tian/桌面/box_aabb/cpp/SBF
/home/tian/miniconda3/envs/sbf/bin/python experiments/tro2026_followup_04_audit_resolution_sweep.py \
  --out-json outputs/paper/tro2026_mandatory_audit_resolution_sweep.json \
  --out-md outputs/paper/tro2026_mandatory_audit_resolution_sweep.md
```

Key result on the current random anytime artifact:

| Segment step | Counted success paths | Invalid records | Unique invalid paths | Unique scene/method/tasks |
|---:|---:|---:|---:|---:|
| 0.020 | 3240 | 6 | 5 | 5 |
| 0.010 | 3240 | 0 | 0 | 0 |
| 0.005 | 3240 | 502 | 79 | 60 |
| 0.002 | 3240 | 678 | 111 | 80 |

Debugging note:

- The first run reported only raw invalid record counts; this overstates the
  issue because repeated checkpoint/incumbent records can share the same path.
- `experiments/audit_random_anytime_artifact.py` was extended to record path
  signatures and unique invalid counts.
- The final result still shows real audit-resolution sensitivity: finer steps
  find invalid stored paths across RBF/SBF and OMPL baselines.

Conclusion: the current paper should not claim robustness to finer audit steps
or continuous safety for these final-audited paths. The valid claim is the fixed
0.01 segment-step final audit used by all methods.

## Paper Integration

Integrated into `doc/paper/tro_rewrite_2026/sbf_tro_2026.tex`:

- `Reporting Protocol`: added validation closure, paper-wide failure taxonomy,
  corridor-field absence, and audit-resolution sweep result.
- `Tabulated Rows and Omitted Sensitivities`: added reporting-rule stability and
  path-gap summary.
- `Discussion` and `Conclusion`: replaced stale future-work phrasing with the
  limited checks now performed and the remaining larger follow-ups.

## Remaining Deferred Experiments

Not run in this tight pass:

- Larger random-scene seed-count sweep.
- Explicit corridor-certified coverage rerun with containment fields.
- Grower strategy ablation.
- Worker-count scaling.
- Narrow-clearance stress test.

These remain useful reviewer-defense experiments but require heavier reruns or
new instrumentation beyond this mandatory package.