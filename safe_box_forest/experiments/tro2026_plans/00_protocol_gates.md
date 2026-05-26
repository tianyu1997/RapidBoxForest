# Protocol Gates

## Purpose

The protocol gates are not numbered experiments. They freeze how every experiment is run, how success is counted, and how safety sensitivity is checked after the main evidence is collected.

## Gate A: Artifact And Statistics Protocol

### Required Artifact Fields

Every final JSON artifact should contain:

- `experiment`: stable experiment identifier.
- `params`: all planner, baseline, audit, and runtime parameters.
- `environment`: host, build directory, Python environment, thread budget, date, and command if available.
- `scenes`: robot, obstacles, query generation, and seeds.
- `trials`: per-seed or per-query rows, including failures and timeouts.
- `summary`: paper-facing aggregate statistics.
- `audit`: final strict audit policy and pass/fail accounting.
- `failures`: structured failure taxonomy, not only text logs.
- `provenance`: source artifact dependencies and script name.

### Success Definitions

- `solver_success`: optimizer or graph search returned a path.
- `audit_success`: final strict collision audit passed.
- `planner_success`: final strict collision audit passed. This is the only success rate that can be used as planner SR in the paper.
- `solved_but_unsafe`: solver returned a path but final strict audit failed.
- `fallback_success`: fallback produced an audited path; count separately from the attempted method.

### Statistics

- Success rates: Wilson confidence interval.
- Timing and path length: median, IQR, p95, and sample count.
- Paired comparisons: same scene/query/seed; bootstrap 95% CI for paired differences or ratios.
- Path length: report only over audited successful paths; keep failures in SR denominators.

## Gate B: Audit Sensitivity

### Timing

Run after the formal experiments have produced representative successes, repairs, and failures. Do not make this the opening experiment.

### Protocol

Re-audit representative paths at multiple resolutions, such as:

- current default `audit_resolution=32` or `gcs_audit_step=0.04`.
- coarser and finer alternatives, e.g. 16/32/64/128 or 0.08/0.04/0.02/0.01 depending on the runner.

### Metrics

- Decision changes between adjacent audit resolutions.
- Audit pass rate by method and resolution.
- Audit wall time by resolution.
- Number of paths whose final paper status changes under finer audit.

### Paper Output

Use one compact table in the final safety section or appendix: selected audit resolution, path count, changed-decision count, and audit overhead. If the changed-decision count is nonzero, the main results must use the finer stable resolution.
