# Mandatory Revision Experiments, 2026-05-18

This plan keeps only the experiments that are necessary under the current tight
time and page budget. The goal is not to broaden every benchmark, but to close
the reviewer-facing risks that can be answered from existing paper artifacts or
with small targeted reruns.

## Scope Decision

The current paper claims a final-audited multi-query planning design point. The
main Shelf+IIWA and random-scene anytime curves remain the primary evidence.
The mandatory follow-up package therefore focuses on validation, robustness of
reporting choices, and failure accounting. Large new random-scene seed sweeps,
new robots, real-robot demos, and broad narrow-clearance benchmarks are deferred
unless the mandatory package exposes a contradiction.

## Experiments To Run Now

### M1. Validation Closure and Failure Taxonomy

Purpose: reconcile the paper-wide final-audit denominator and classify any
reported failures.

Entry point:

```bash
cd /home/tian/桌面/box_aabb/cpp/SBF
/home/tian/miniconda3/envs/sbf/bin/python experiments/tro2026_followup_01_validation_closure.py \
  --outputs outputs/paper \
  --out-json outputs/paper/tro2026_mandatory_validation_closure.json \
  --out-csv outputs/paper/tro2026_mandatory_validation_closure.csv \
  --out-md outputs/paper/tro2026_mandatory_validation_closure.md
```

Outputs: path counts, audit pass counts, solved-but-unsafe counts, corridor-field
availability, and a failure taxonomy by artifact.

Acceptance: totals must be explainable. Unknown failure classes are allowed only
if the source artifact lacks the needed fields, and must be reported as such.

### M2. Reporting-Rule and Path-Gap Robustness

Purpose: show that the paper's conclusions are not created by a particular
tabulated-row extraction rule.

Entry point to be added if not already present:

```bash
cd /home/tian/桌面/box_aabb/cpp/SBF
/home/tian/miniconda3/envs/sbf/bin/python experiments/tro2026_followup_03_reporting_sensitivity.py \
  --outputs outputs/paper \
  --out-json outputs/paper/tro2026_mandatory_reporting_sensitivity.json \
  --out-md outputs/paper/tro2026_mandatory_reporting_sensitivity.md
```

Statistics:

- Sweep path-tolerance values `{0, 0.05, 0.08, 0.10, 0.15}` when full checkpoint
  records are available.
- Report row-movement counts, selected build/query/path ranges, and whether the
  selected method ordering changes.
- Report path-gap summaries against the best available audited baseline in each
  comparable scenario/query group: median, p90, max, and count.

Acceptance: if row extraction changes the tabulated ordering, the paper must
state that the table is only a compact slice and rely on full curves. If path
gaps contain large outliers, list them explicitly.

### M3. Audit-Resolution Robustness From Existing Artifacts

Purpose: check whether stored final-audited paths can be re-audited at stricter
segment steps. This is mandatory if the artifacts contain raw paths; otherwise
the mandatory result is a field-availability report plus a clear limitation.

Preferred entry points:

```bash
cd /home/tian/桌面/box_aabb/cpp/SBF
/home/tian/miniconda3/envs/sbf/bin/python experiments/audit_random_anytime_artifact.py --help
/home/tian/miniconda3/envs/sbf/bin/python experiments/paper_11_soundness_audit_suite.py --help
```

Target segment steps: `{0.02, 0.01, 0.005, 0.002}`.

Acceptance: no silent pass/fail changes. If the current artifacts do not store
enough raw path and scene information for re-audit, record that limitation and
do not claim a measured sweep.

### M4. Paper Integration

Purpose: add one compact paragraph or small appendix table to the active paper
summarizing M1--M3 without expanding the main claims.

Integration rule:

- Main text: one short paragraph in the Experiments reporting protocol or
  Discussion.
- Appendix/table generator: add generated tables only if they fit without
  disrupting the main figures.
- Keep the full anytime curves as the primary evidence; do not reintroduce
  selected-operating-point wording.

## Deferred Experiments

The following are valuable but not mandatory for this tight revision pass:

- Expanded random-scene seed count across all robots and difficulties.
- Full grower strategy ablation beyond the reporting sensitivity check.
- Worker-count scaling.
- Dedicated narrow-clearance benchmark.
- Dynamic-update breadth on UR5/Panda.
- Stronger LECT prewarm/cache-regime reruns.

These are deferred because they require substantial reruns or new scene design,
whereas the mandatory package directly addresses the most likely reviewer
questions about the current evidence.

## Final Validation

After experiments and paper integration:

```bash
cd /home/tian/桌面/box_aabb/cpp/SBF
/home/tian/miniconda3/envs/sbf/bin/python experiments/tro2026_generate_tables.py --mode main \
  --out-dir /home/tian/桌面/box_aabb/cpp/SBF/doc/paper/tro_rewrite_2026/generated
cd /home/tian/桌面/box_aabb/cpp/SBF/doc/paper/tro_rewrite_2026
xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
xelatex -interaction=nonstopmode -halt-on-error sbf_tro_2026.tex
```

Also grep the final manuscript for stale selected-point / selected operating
point / matched-scene-only wording before closing the revision.