from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common import run_shelf_sbf_case as case
from experiments.common.experiment_io import run_id, write_json


DEFAULT_REFERENCE_JSON = (
    REPO_ROOT
    / "outputs"
    / "new_experiments"
    / "exp04_shelf_ablation_full_ablation_20260528"
    / "baseline_warm_aafk_support_hull_8t_aafk_volume_min.json"
)
DEFAULT_OUTPUT_JSON = (
    REPO_ROOT
    / "outputs"
    / "new_experiments"
    / "exp04_fast_single_case_probe"
    / "baseline_fast_probe.json"
)

BUILD_KEYS = [
    "wall_s",
    "planning_s",
    "grow_ms",
    "merge_ms",
    "connector_ms",
    "unique_box_count",
    "raw_boxes",
    "final_boxes",
]

DIAGNOSTIC_KEYS = [
    "grower.rrt.make_growth_tasks",
    "grower.rrt.make_component_connect_seed_for_root",
    "grower.component_connect_attempts",
    "grower.component_connect_successes",
    "grower.component_connect_seed_cache_hits",
    "grower.component_connect_seed_cache_misses",
    "grower.worker_oracle.materializations",
    "grower.worker_oracle.envelope_collision_queries",
    "grower.worker_oracle.materialization_reused_cached_envelope",
    "grower.worker_oracle.materialization_envelope_read_time_us",
    "grower.worker_oracle.materialization_envelope_compute_time_us",
    "grower.worker_oracle.materialization_envelope_collision_time_us",
    "grower.worker_oracle.materialization_reused_external_evidence",
    "grower.worker_oracle.materialization_reused_endpoint_cache",
    "grower.worker_oracle.materialization_reused_shared_endpoint_cache",
    "grower.worker_oracle.materialization_cache_lookup_time_us",
    "grower.worker_oracle.materialization_external_lookup_time_us",
    "oracle.materializations",
    "oracle.envelope_collision_queries",
    "oracle.materialization_reused_cached_envelope",
    "oracle.materialization_envelope_read_time_us",
    "oracle.materialization_envelope_compute_time_us",
    "oracle.materialization_envelope_collision_time_us",
    "oracle.materialization_reused_external_evidence",
    "oracle.materialization_reused_endpoint_cache",
    "oracle.materialization_reused_shared_endpoint_cache",
    "oracle.materialization_cache_lookup_time_us",
    "oracle.materialization_external_lookup_time_us",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the current baseline through only the exp04 fast stage and compare it against a reference artifact."
    )
    parser.add_argument("--reference-json", type=Path, default=DEFAULT_REFERENCE_JSON)
    parser.add_argument("--out-json", type=Path, default=DEFAULT_OUTPUT_JSON)
    parser.add_argument("--stage", default="fast", choices=["seed", "fast", "balanced", "quality", "fallback"])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-root", type=Path, default=None)
    parser.add_argument("--cache-root", type=Path, default=None)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def find_reference_stage_row(payload: dict[str, Any], stage_id: str) -> dict[str, Any]:
    raw_rows = payload.get("raw_stage_rows", [])
    for item in raw_rows:
        row = item.get("row", item)
        build = row.get("build", {})
        if str(build.get("latency_stage_id", "")) == stage_id:
            return row
    for row in payload.get("rows", []):
        build = row.get("build", {})
        if str(build.get("latency_stage_id", "")) == stage_id:
            return row
    raise KeyError(f"stage {stage_id!r} not found in {payload.get('case_name', 'artifact')}")


def build_probe_args(reference_payload: dict[str, Any], reference_json: Path, run_root: Path) -> argparse.Namespace:
    params = copy.deepcopy(reference_payload["params"])
    probe_case_name = f"{reference_payload.get('case_name', 'baseline')}_current_probe"
    params["case_name"] = probe_case_name
    params["database_path"] = str(run_root / "active_cache" / probe_case_name)
    params["out_json"] = str(run_root / f"{probe_case_name}.json")
    params["clean_active_cache"] = True
    params["latency_stage_early_stop"] = False
    params["seeds"] = 1
    if "latency_profile" not in params:
        params["latency_profile"] = case.LATENCY_PROFILE_BALANCED_LOW_LATENCY
    local_cache_root = reference_json.parent / "cache"
    if local_cache_root.exists():
        params["rbf_cache_root"] = str(local_cache_root)
    return argparse.Namespace(**params)


def stage_args_for_id(base_args: argparse.Namespace, stage_id: str) -> argparse.Namespace:
    sequence = case.balanced_low_latency_stage_sequence(base_args)
    for stage_index, stage in enumerate(sequence):
        if str(stage.get("stage_id", "")) == stage_id:
            return case.stage_args(base_args, {**stage, "stage_index": stage_index})
    raise KeyError(f"stage {stage_id!r} not found in balanced_low_latency stage sequence")


def maybe_delta(current: float | int | None, reference: float | int | None) -> dict[str, float | int | None]:
    if current is None and reference is None:
        return {"current": None, "reference": None, "delta": None, "delta_pct": None}
    current_value = None if current is None else float(current)
    reference_value = None if reference is None else float(reference)
    delta = None
    delta_pct = None
    if current_value is not None and reference_value is not None:
        delta = current_value - reference_value
        if abs(reference_value) > 1e-12:
            delta_pct = 100.0 * delta / reference_value
    return {
        "current": current_value,
        "reference": reference_value,
        "delta": delta,
        "delta_pct": delta_pct,
    }


def diagnostic_value(diagnostics: dict[str, Any], key: str) -> float | None:
    value = diagnostics.get(key)
    return None if value is None else float(value)


def summarize_hit_rates(diagnostics: dict[str, Any]) -> dict[str, float | None]:
    component_cache_hits = float(diagnostics.get("grower.component_connect_seed_cache_hits", 0.0))
    component_cache_misses = float(diagnostics.get("grower.component_connect_seed_cache_misses", 0.0))
    envelope_hits = float(
        diagnostics.get(
            "grower.worker_oracle.materialization_reused_cached_envelope",
            diagnostics.get("oracle.materialization_reused_cached_envelope", 0.0),
        )
    )
    envelope_queries = float(
        diagnostics.get(
            "grower.worker_oracle.envelope_collision_queries",
            diagnostics.get("oracle.envelope_collision_queries", 0.0),
        )
    )
    external_hits = float(
        diagnostics.get(
            "grower.worker_oracle.materialization_reused_external_evidence",
            diagnostics.get("oracle.materialization_reused_external_evidence", 0.0),
        )
    )
    endpoint_hits = float(
        diagnostics.get(
            "grower.worker_oracle.materialization_reused_endpoint_cache",
            diagnostics.get("oracle.materialization_reused_endpoint_cache", 0.0),
        )
    )
    shared_hits = float(
        diagnostics.get(
            "grower.worker_oracle.materialization_reused_shared_endpoint_cache",
            diagnostics.get("oracle.materialization_reused_shared_endpoint_cache", 0.0),
        )
    )
    materializations = float(
        diagnostics.get(
            "grower.worker_oracle.materializations",
            diagnostics.get("oracle.materializations", 0.0),
        )
    )
    component_total = component_cache_hits + component_cache_misses
    return {
        "grower_component_seed_cache_hit_rate": None if component_total <= 0.0 else component_cache_hits / component_total,
        "envelope_cache_hit_rate": None if envelope_queries <= 0.0 else envelope_hits / envelope_queries,
        "endpoint_reuse_rate": None if materializations <= 0.0 else (external_hits + endpoint_hits + shared_hits) / materializations,
    }


def main() -> int:
    args = parse_args()
    reference_json = args.reference_json.resolve()
    reference_payload = load_json(reference_json)
    reference_row = find_reference_stage_row(reference_payload, args.stage)
    run_root = (args.run_root or args.out_json.parent / "run").resolve()
    base_args = build_probe_args(reference_payload, reference_json, run_root)
    if args.cache_root is not None:
        base_args.rbf_cache_root = str(args.cache_root.resolve())
    stage_args = stage_args_for_id(base_args, args.stage)

    robot = case.sbf.load_iiwa14_robot()
    obstacles = case.sbf.make_combined_obstacles()
    coverage_seeds = [list(seed) for seed in case.sbf.make_coverage_seeds(include_extra_anchors=False)]
    queries = case.sbf.make_combined_queries()
    current_row = case.run_single_seed_attempt(stage_args, robot, obstacles, coverage_seeds, queries, int(args.seed))

    reference_build = reference_row.get("build", {})
    current_build = current_row.get("build", {})
    reference_diagnostics = dict(reference_build.get("diagnostics", {}))
    current_diagnostics = dict(current_build.get("diagnostics", {}))

    build_delta = {
        key: maybe_delta(current_build.get(key), reference_build.get(key))
        for key in BUILD_KEYS
    }
    diagnostics_delta = {
        key: maybe_delta(diagnostic_value(current_diagnostics, key), diagnostic_value(reference_diagnostics, key))
        for key in DIAGNOSTIC_KEYS
    }

    payload = {
        "experiment": "exp04_fast_single_case_probe",
        "run_id": run_id("exp04_fast_single_case_probe"),
        "reference_json": str(reference_json),
        "stage_id": args.stage,
        "seed": int(args.seed),
        "current": current_row,
        "reference": reference_row,
        "comparison": {
            "build": build_delta,
            "diagnostics": diagnostics_delta,
            "current_hit_rates": summarize_hit_rates(current_diagnostics),
            "reference_hit_rates": summarize_hit_rates(reference_diagnostics),
        },
    }
    write_json(args.out_json, payload)

    print(
        f"[fast-probe] stage={args.stage} seed={args.seed} "
        f"grow_ms={current_build.get('grow_ms', 0.0):.3f} "
        f"connector_ms={current_build.get('connector_ms', 0.0):.3f} "
        f"envelope_hits={current_diagnostics.get('grower.worker_oracle.materialization_reused_cached_envelope', current_diagnostics.get('oracle.materialization_reused_cached_envelope', 0.0)):.0f}/"
        f"{current_diagnostics.get('grower.worker_oracle.envelope_collision_queries', current_diagnostics.get('oracle.envelope_collision_queries', 0.0)):.0f} "
        f"component_seed_cache={current_diagnostics.get('grower.component_connect_seed_cache_hits', 0.0):.0f}/"
        f"{current_diagnostics.get('grower.component_connect_seed_cache_hits', 0.0) + current_diagnostics.get('grower.component_connect_seed_cache_misses', 0.0):.0f}"
    )
    print(f"[fast-probe] wrote {args.out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())