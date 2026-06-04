# Random Scene Catalog Schema

Random scenes used by Exp.6 and Exp.7 are saved before planner execution. The
catalog is the shared input for SBF, IRIS-NP+GCS, PRM, RRTConnect, BIT*, and
dynamic-update runs.

## Top-Level Object

```json
{
  "schema": "tro2026_random_scene_catalog_v1",
  "robots": ["iiwa", "ur5", "panda"],
  "difficulties": ["easy", "medium", "hard"],
  "scene_seeds": 50,
  "scene_profile": "balanced",
  "seed_base": 9176,
  "obstacle_counts": {"easy": 4, "medium": 8, "hard": 12},
  "obstacle_scales": {"easy": 0.12, "medium": 0.16, "hard": 0.20},
  "records": []
}
```

## Record Object

Each record is keyed by `robot:difficulty:scene_seed`.

```json
{
  "schema": "tro2026_random_scene_catalog_v1",
  "robot": "iiwa",
  "difficulty": "medium",
  "scene_seed": 0,
  "generator_seed": 9176,
  "scene_profile": "balanced",
  "start": [0.0],
  "goal": [1.0],
  "obstacles": [[xmin, ymin, zmin, xmax, ymax, zmax]],
  "endpoint_clearance_margin_m": 0.24,
  "fixed_robot_clearance_margin_m": 0.025,
  "direct_segment_blocked": true,
  "segment_resolution": 96
}
```

## Lifecycle Modes

`generate`: overwrite/create the catalog from generator parameters.

`auto`: reuse existing matching records and generate missing records.

`reuse`: require all requested records to exist.

`verify`: require all requested records to exist and do not write new records.

Full paper runs must use `reuse` or `verify`; `generate` is allowed only for
catalog preparation.

## Reproducibility Rule

Planner artifacts must record the catalog path, schema, generator parameters,
and scene keys used. A method may not silently call the random generator when a
catalog lookup fails in full mode.

