# Superseded Consolidation Plan

This historical plan has been superseded by the workspace-level refactor plan at
`../../docs/WORKSPACE_REFACTOR_PLAN.md`.

The active architecture is:

```text
link_interval_envelope -> lect_database -> safe_box_forest
```

The workspace root owns integrated CMake orchestration, while this module owns
only the database core, SBF adapter target, and online cache target.