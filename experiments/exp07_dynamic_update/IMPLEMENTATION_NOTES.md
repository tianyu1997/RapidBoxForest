# Exp.7 Dynamic Update Implementation Notes

## Difference From Archived Exp.7

Archived reference:
`experiments/archive/20260603_legacy_current/exp07_dynamic_update/run_dynamic_update.py`.

Current runner:
`experiments/exp07_dynamic_update/run_dynamic_update.py`.

The two scripts are not measuring the same protocol:

- The archived runner samples a dedicated IIWA d23 canonical inverse-root scene
  catalog and stores hard scenes with obstacle prefixes for easy/medium/hard.
- The current runner reuses the generic saved random-scene catalog used by
  Exp.6, with robot/all-sector planning semantics and per-difficulty records.
- The archived runner constructs source and target scenes from the same hard
  obstacle list by prefix insertion/deletion. The current runner loads separate
  difficulty records and compares their obstacle counts.
- The archived runner uses an Exp.4 ablation row
  `exp07_d23_sh_8t_leaf8_14_box200_d28` with `deep_max_boxes=200`,
  `deep_ffb_depth=28`, `leaf_threads=8`, and several dynamic-update-specific
  repair parameters.
- The current runner uses the shared RBF default profile and `deep_max_boxes`
  defaults from `experiments/common/rbf_defaults.py`.
- The archived runner allows a post-update endpoint segment fallback for failed
  insertion audits. The current runner disables warm rebuild fallback and only
  uses endpoint segment fallback after a failed insertion audit.
- The archived table reports many diagnostic columns. The current Table VIII is
  intentionally single-column and keeps only Transition, SR, Update, Warm, and
  Speedup.

Therefore a direct numeric comparison between the archived Table VIII and the
current Table VIII is not meaningful. Large differences are expected unless the
current runner is switched back to the archived d23-prefix scene protocol and
the archived Exp.4 dynamic-update row.

