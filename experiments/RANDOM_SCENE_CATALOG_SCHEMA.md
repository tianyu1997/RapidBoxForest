# Random Scene Catalog Schema

Random scenes used by Exp.6 are saved before planner execution. The catalog is
the shared input for SBF, IRIS-NP+GCS, PRM, RRTConnect, and BIT* runs.

## Top-Level Object

```json
{
  "schema": "tro2026_random_scene_catalog_v7",
  "robots": ["iiwa", "ur5", "panda"],
  "difficulties": ["easy", "medium", "hard"],
  "scene_seeds": 50,
  "scene_profile": "timed_probe_independent",
  "seed_base": 9176,
  "obstacle_counts": {"easy": 4, "medium": 10, "hard": 16},
  "obstacle_scales": {"easy": 0.12, "medium": 0.20, "hard": 0.26},
  "difficulty_probe": {
    "planner": "OMPL_RRTConnect",
    "policy": "first_solution_time_window_or_direct_obstruction",
    "windows_s": {"easy": [0.0, 0.05], "medium": [0.05, 0.20], "hard": [0.20, 1.00]},
    "direct_obstruction_fraction_windows": {"easy": [0.02, 0.16], "medium": [0.10, 0.35], "hard": [0.22, 1.01]}
  },
  "records": []
}
```

## Record Object

Each record is keyed by `robot:difficulty:scene_seed`.

```json
{
  "schema": "tro2026_random_scene_catalog_v7",
  "robot": "iiwa",
  "difficulty": "medium",
  "scene_seed": 0,
  "generator_seed": 9176,
  "scene_profile": "timed_probe_independent",
  "queries": [
    {
      "label": "q0",
      "start": [0.0],
      "goal": [1.0],
      "difficulty_probe": {
        "ok": true,
        "accepted_by": "time_window",
        "solve_s": 0.012,
        "direct_obstruction": {
          "samples": 97,
          "collision_samples": 18,
          "collision_fraction": 0.186
        }
      }
    }
  ],
  "start": [0.0],
  "goal": [1.0],
  "obstacles": [[xmin, ymin, zmin, xmax, ymax, zmax]],
  "endpoint_clearance_margin_m": 0.12,
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

## Difficulty Semantics

The v7 paper catalog uses `timed_probe_independent`. Each robot/difficulty/seed
scene is generated independently, then each query must have a valid
`difficulty_probe`. A query is accepted if an RRTConnect probe produces a
strictly collision-free exact path and either:

- its first-solution solve time falls in the difficulty window, or
- its straight-line joint interpolation has a direct-obstruction collision
  fraction in the difficulty window.

The second criterion is explicit clearance/obstruction evidence. It prevents the
catalog generator from filtering all scenes to the old "RRTConnect solves within
250 ms" condition, while keeping generation reproducible and finite for all
three robots.

## Reproducibility Rule

Planner artifacts must record the catalog path, schema, generator parameters,
and scene keys used. A method may not silently call the random generator when a
catalog lookup fails in full mode.
