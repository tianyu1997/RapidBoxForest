# Final Safety Accounting And Failure Analysis

## Purpose

This is a paper-wide verification section, not an ordinary planner benchmark. It aggregates all final experiment artifacts and enforces one success definition: final strict collision audit must pass.

## Existing Runner

- `experiments/paper_11_soundness_audit_suite.py`

## Protocol

1. Run all formal experiment smoke/full artifacts first.
2. Point the safety suite at `outputs/paper`.
3. Aggregate SBF, random-scene, GCS, protected merger, and baseline artifacts that contain path/audit fields.
4. Add audit-resolution sensitivity from Protocol Gate B on representative paths and all failure/repair cases.
5. Generate one paper-wide safety table and one failure taxonomy table/figure.

## Categories

- `audit_passed`: final path passed strict audit.
- `audit_failed`: final path exists but failed audit.
- `solved_but_unsafe`: solver returned a path but strict audit failed.
- `repaired_success`: repair/fallback produced an audited path.
- `fallback_success`: fallback produced an audited path after attempted method failed.
- `timeout_or_no_path`: no path within budget.
- `not_applicable`: artifact has no path semantics; should not enter path totals.

## Metrics

- Total paths considered.
- Audit pass count and rate.
- Solved-but-unsafe count.
- Repair trigger/success/failure rate.
- Fallback source counts.
- Failure categories by experiment and method.
- Audit-resolution changed-decision count.

## Visualizations

- Paper-wide safety accounting table.
- Failure taxonomy stacked bar.
- Repair success vs repair budget curve if repair budget is swept.

## Smoke Command

```bash
$PYTHON_SBF experiments/paper_11_soundness_audit_suite.py \
  --outputs outputs/paper \
  --out-json outputs/paper/tro2026_safety_accounting_smoke.json
```

## Full Command

```bash
$PYTHON_SBF experiments/paper_11_soundness_audit_suite.py \
  --outputs outputs/paper \
  --out-json outputs/paper/tro2026_safety_accounting_full.json
```

## Acceptance Criteria

- No solver-only result enters planner SR.
- GCS-only success and SBF fallback success are separate.
- Any remaining unsafe assumptions are listed explicitly in the paper.
- If audit sensitivity changes decisions, the final paper uses the stable finer audit policy.
