from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import default_sbf_subprocess_env, environment_metadata, namespace_dict, run_id, write_json
from experiments.common.lect_db_dispatch import build_current_shelf_sbf_anytime_command


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a shelf anytime probe with connector BiRRT disabled.")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--python-executable", default=sys.executable)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--endpoint-source", default=None)
    parser.add_argument("--rbf-max-depth", type=int, default=40)
    parser.add_argument("--connector-birrt", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--bridge-boxes", type=int, default=256)
    parser.add_argument("--frontier-bridge", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--frontier-bridge-adaptive-ffb", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--frontier-bridge-ffb-max-depth", type=int, default=96)
    parser.add_argument("--frontier-bridge-candidate-limit", type=int, default=16)
    parser.add_argument("--frontier-bridge-gap-stall-iterations", type=int, default=2)
    parser.add_argument("--point-gap-tolerance", type=float, default=0.5)
    parser.add_argument("--point-gap-resolution", type=int, default=24)
    parser.add_argument("--segment-edges", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--connector-max-pairs-per-gap", type=int, default=8)
    parser.add_argument("--connector-pave-max-chain", type=int, default=None)
    parser.add_argument("--connector-pave-steps", type=int, default=None)
    parser.add_argument("--connector-pave-depth", type=int, default=None)
    parser.add_argument("--latency-profile", default="balanced_low_latency", choices=["balanced_low_latency", "stable"])
    parser.add_argument("--sample-categorical-allocation", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--sample-uniform-prob", type=float, default=None)
    parser.add_argument("--component-connect-prob", type=float, default=None)
    parser.add_argument("--component-connect-candidate-limit", type=int, default=None)
    parser.add_argument("--component-connect-ffb-depth-increment", type=int, default=None)
    parser.add_argument("--component-connect-ffb-max-depth", type=int, default=None)
    parser.add_argument("--intertree-goal-bias", type=float, default=None)
    parser.add_argument("--rrt-goal-bias", type=float, default=None)
    parser.add_argument("--unexplored-prob", type=float, default=None)
    parser.add_argument("--stage-ids", default=None)
    parser.add_argument("--stage-max-boxes", default="32,128,224,320,448")
    parser.add_argument("--stage-quality-min-connected-boxes", default="16,64,128,192,256")
    parser.add_argument("--stage-post-connect-extra-boxes", default="0,16,48,96,160")
    parser.add_argument("--stage-post-connect-time-budget-ms", default="0,200,600,1200,2400")
    return parser.parse_args()


def summarize_artifact(payload: dict[str, Any]) -> dict[str, Any]:
    stages: list[dict[str, Any]] = []
    for item in payload.get("raw_stage_rows", []):
        row = dict(item.get("row") or {})
        build = dict(row.get("build") or {})
        diagnostics = dict(build.get("diagnostics") or {})
        stages.append({
            "stage_id": str(item.get("stage_id", "")),
            "grow_ms": float(build.get("grow_ms", 0.0)),
            "connector_ms": float(build.get("connector_ms", 0.0)),
            "boxes": int(build.get("unique_box_count", 0)),
            "grow_adjacency_islands": int(build.get("grow_adjacency_islands", diagnostics.get("grower.adjacency_islands", 0.0))),
            "connector_islands_initial": int(build.get("connector_islands_initial", diagnostics.get("connector.islands_initial", 0.0))),
            "final_adjacency_islands": int(build.get("adjacency_islands", 0)),
            "bridge_boxes_added": int(diagnostics.get("connector.frontier_bridge_successes", 0.0)),
            "point_gap_segment_edges_added": int(diagnostics.get("connector.point_gap_segment_edges_added", 0.0)),
            "direct_box_segment_successes": int(diagnostics.get("connector.direct_box_segment_successes", 0.0)),
            "chain_pave_attempts": int(diagnostics.get("connector.chain_pave_attempts", 0.0)),
            "chain_pave_successes": int(diagnostics.get("connector.chain_pave_successes", 0.0)),
            "chain_pave_zero_added": int(diagnostics.get("connector.chain_pave_zero_added", 0.0)),
            "birrt_invocations": int(diagnostics.get("connector.birrt_invocations", 0.0)),
            "birrt_disabled_skips": int(diagnostics.get("connector.birrt_disabled_skips", 0.0)),
        })
    reached_single_island = [stage["stage_id"] for stage in stages if stage["final_adjacency_islands"] <= 1]
    return {
        "stages": stages,
        "reached_single_island": reached_single_island,
        "best_final_islands": min((stage["final_adjacency_islands"] for stage in stages), default=None),
    }


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_json = args.out_dir / "shelf_no_birrt_anytime.json"
    command = build_current_shelf_sbf_anytime_command(
        python_executable=str(args.python_executable),
        out_json=out_json,
        database_path=args.out_dir / "active_cache" / "shelf_no_birrt_anytime",
        case_name="shelf_no_birrt_anytime",
        threads=int(args.threads),
        seeds=int(args.seeds),
        timeout_ms=float(args.timeout_ms),
        rbf_cache_root=args.out_dir / "active_cache",
        warm_cache_label="iiwa_shelf_endpoint_only_p18_canonical_dim0q4",
        external_evidence_mode="snapshot",
        external_evidence_auto_build_snapshot=True,
        use_external_evidence=False,
        endpoint_evidence_cache=True,
        clean_active_cache=True,
        endpoint_source=str(args.endpoint_source) if args.endpoint_source is not None else "aafk",
        rbf_max_depth=int(args.rbf_max_depth),
    )
    command.extend([
        "--connector-birrt" if bool(args.connector_birrt) else "--no-connector-birrt",
        "--frontier-bridge" if bool(args.frontier_bridge) else "--no-frontier-bridge",
        "--frontier-bridge-adaptive-ffb" if bool(args.frontier_bridge_adaptive_ffb) else "--no-frontier-bridge-adaptive-ffb",
        "--frontier-bridge-ffb-max-depth",
        str(int(args.frontier_bridge_ffb_max_depth)),
        "--frontier-bridge-candidate-limit",
        str(int(args.frontier_bridge_candidate_limit)),
        "--frontier-bridge-gap-stall-iterations",
        str(int(args.frontier_bridge_gap_stall_iterations)),
        "--connector-point-gap-tolerance",
        str(float(args.point_gap_tolerance)),
        "--connector-point-gap-resolution",
        str(int(args.point_gap_resolution)),
        "--connector-bridge-boxes",
        str(int(args.bridge_boxes)),
        "--connector-max-pairs-per-gap",
        str(int(args.connector_max_pairs_per_gap)),
        "--latency-profile",
        str(args.latency_profile),
    ])
    if args.segment_edges is not None:
        command.append("--segment-edges" if bool(args.segment_edges) else "--no-segment-edges")
    if args.connector_pave_max_chain is not None:
        command.extend(["--connector-pave-max-chain", str(int(args.connector_pave_max_chain))])
    if args.connector_pave_steps is not None:
        command.extend(["--connector-pave-steps", str(int(args.connector_pave_steps))])
    if args.connector_pave_depth is not None:
        command.extend(["--connector-pave-depth", str(int(args.connector_pave_depth))])
    if args.sample_categorical_allocation is not None:
        command.append("--sample-categorical-allocation" if bool(args.sample_categorical_allocation) else "--no-sample-categorical-allocation")
    if args.sample_uniform_prob is not None:
        command.extend(["--sample-uniform-prob", str(float(args.sample_uniform_prob))])
    if args.component_connect_prob is not None:
        command.extend(["--component-connect-prob", str(float(args.component_connect_prob))])
    if args.component_connect_candidate_limit is not None:
        command.extend(["--component-connect-candidate-limit", str(int(args.component_connect_candidate_limit))])
    if args.component_connect_ffb_depth_increment is not None:
        command.extend(["--component-connect-ffb-depth-increment", str(int(args.component_connect_ffb_depth_increment))])
    if args.component_connect_ffb_max_depth is not None:
        command.extend(["--component-connect-ffb-max-depth", str(int(args.component_connect_ffb_max_depth))])
    if args.intertree_goal_bias is not None:
        command.extend(["--intertree-goal-bias", str(float(args.intertree_goal_bias))])
    if args.rrt_goal_bias is not None:
        command.extend(["--rrt-goal-bias", str(float(args.rrt_goal_bias))])
    if args.unexplored_prob is not None:
        command.extend(["--unexplored-prob", str(float(args.unexplored_prob))])
    if args.stage_ids is not None:
        command.extend(["--stage-ids", str(args.stage_ids)])
    command.extend([
        "--latency-stage-max-boxes",
        str(args.stage_max_boxes),
        "--latency-stage-quality-min-connected-boxes",
        str(args.stage_quality_min_connected_boxes),
        "--latency-stage-post-connect-extra-boxes",
        str(args.stage_post_connect_extra_boxes),
        "--latency-stage-post-connect-time-budget-ms",
        str(args.stage_post_connect_time_budget_ms),
    ])

    env = os.environ.copy()
    env.update(default_sbf_subprocess_env())
    result = subprocess.run(command, cwd=REPO_ROOT, env=env, text=True, capture_output=True)

    payload: dict[str, Any] = {
        "experiment": "exp04_shelf_no_birrt_probe",
        "run_id": run_id("exp04_shelf_no_birrt_probe"),
        "params": namespace_dict(args),
        "environment": environment_metadata(),
        "command": command,
        "returncode": int(result.returncode),
        "stdout": result.stdout,
        "stderr": result.stderr,
    }
    if out_json.exists():
        artifact = json.loads(out_json.read_text(encoding="utf-8"))
        payload["artifact"] = artifact
        payload["summary"] = summarize_artifact(artifact)
    write_json(args.out_dir / "run_manifest.json", payload)
    if result.returncode != 0:
        raise SystemExit(result.returncode)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())