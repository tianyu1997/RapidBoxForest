# Experiment Plan Proposal

This proposal follows the `rbf-experiment-ladder` rule: later experiments must
inherit the upstream validated profile unless a row explicitly isolates a
factor.

## Ablation Rows

1. Fixed leaf sweep baseline.
2. Early-stop free only.
3. Early-stop plus blocker/no-good deferral.
4. Early-stop plus sparse dyadic addressing.
5. Early-stop plus portal edges.
6. Full C-LECT.

## Required Metrics

- `N_virtual_leaves`
- `N_materialized_cells`
- `N_validated_free`
- `N_collision_domains`
- `N_deferred`
- `N_LECT_evidence_lookups`
- `N_global_graph_vertices`
- `N_E_portal`
- query success rate
- strict audit pass rate
- success-only mean path length
- accepted free box depth histogram
- materialized cell depth histogram

## Validation Gates

- Conservative rows may only report paths built from validated boxes or
  expanded conservative portal chains.
- Occupied pruning must be disabled unless signed-distance witness assumptions
  are met.
- Planning time excludes final audit.
- Simplification budget and audit resolution must match the current global
  paper convention.

