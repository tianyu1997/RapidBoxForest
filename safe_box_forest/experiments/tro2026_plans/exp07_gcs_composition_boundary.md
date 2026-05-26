# Exp.7 GCS Composition Boundary And Protected Adapter

## Claim

Direct optimization over provisional SBF boxes is unsafe. A GCS adapter is acceptable only when the GCS-produced path itself passes strict audit; otherwise fallback must be counted explicitly.

## Hypotheses

- Direct or overlap-expanded GCS may solve but fail audit.
- Audited path-tube/protected adapters reduce unsafe shortcuts.
- Final safety accounting must distinguish GCS-only audited success from SBF fallback.

## Existing Runners

- `experiments/paper_04_audited_corridor_gcs.py`
- `experiments/paper_07_merger_protected_study.py`
- `gcs-science-robotics/gcs/linear.py`

## Protocol

1. Build SBF corridor on Marcucci shelf+IIWA.
2. Try raw SBF box corridor GCS.
3. Try overlap-expanded corridor.
4. Try audited path-tube/protected adapter.
5. Audit every returned GCS path.
6. Record final source: GCS-only, GCS-with-repair, SBF fallback, or failure.

## Comparison Groups

- Direct/provisional GCS negative control.
- Raw SBF box corridor.
- Overlap-expanded corridor.
- Audited path-tube/protected adapter.
- Native SBF corridor.
- IRIS-GCS subset if stable.

## Metrics

- GCS solve rate.
- GCS-only audit pass rate.
- Fallback trigger rate.
- Final audit SR.
- Solved-but-unsafe count.
- Path length, solve time, adapter build time.
- Failure cause: overlap gap, certificate gap, numeric failure, timeout.

## Visualizations

- Outcome Sankey.
- Failure-cause table.
- Unsafe direct path vs protected audited path overlay.

## Smoke Command

```bash
$PYTHON_SBF experiments/paper_04_audited_corridor_gcs.py \
  --query AS->TS \
  --threads 2 \
  --quality-min-connected-boxes 64 \
  --post-connect-time-budget-ms 100 \
  --max-tube-regions 200 \
  --allow-failures \
  --out-json outputs/paper/tro2026_exp07_gcs_smoke.json \
  --out-paths-json outputs/paper/tro2026_exp07_gcs_smoke_paths.json
```

## Full Command

```bash
$PYTHON_SBF experiments/paper_04_audited_corridor_gcs.py \
  --query all \
  --threads 8 \
  --quality-min-connected-boxes 128 \
  --post-connect-time-budget-ms 450 \
  --corridor-refine \
  --bridge-repaired-queries \
  --try-sbf-box-corridor \
  --try-overlap-expanded-box-corridor \
  --try-path-tube \
  --gcs-audit-step 0.04 \
  --allow-failures \
  --out-json outputs/paper/tro2026_exp07_gcs_full.json \
  --out-paths-json outputs/paper/tro2026_exp07_gcs_full_paths.json
```

## Acceptance Criteria

- No GCS row is counted successful unless the GCS path itself passes audit.
- Fallbacks are explicit and not conflated with GCS-only success.
- Direct unsafe outcomes are framed as negative controls.
