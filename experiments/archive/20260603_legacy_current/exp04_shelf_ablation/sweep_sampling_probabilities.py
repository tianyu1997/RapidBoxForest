#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
PROFILE = REPO_ROOT / "experiments" / "exp04_shelf_ablation" / "profile_anchor_segment_recommended.py"
DEFAULT_OUT = REPO_ROOT / "outputs" / "new_experiments" / "exp04_sampling_sweep_20260602"


def median_field(stats: dict[str, Any], key: str, default: float = 0.0) -> float:
    value = stats.get(key, {})
    if isinstance(value, dict):
        return float(value.get("median", default))
    if value is None:
        return default
    return float(value)


def make_cases() -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    base = {
        "rrt": 0.15,
        "inter": 0.25,
        "unexp": 0.20,
        "anchor": 0.025,
        "uniform": 0.375,
        "categorical": True,
    }

    def add(name: str, family: str, **kwargs: Any) -> None:
        params = dict(base)
        params.update(kwargs)
        params["name"] = name
        params["family"] = family
        cases.append(params)

    add("cat_i25_u20_r15_a025_localbest", "baseline")
    add(
        "base_seq_i40_u35_r15_a05",
        "legacy_baseline",
        inter=0.40,
        unexp=0.35,
        anchor=0.05,
        uniform=0.0,
        categorical=False,
    )

    for inter in [0.10, 0.20, 0.30, 0.40, 0.50, 0.60]:
        add(f"seq_inter_{int(inter * 100):02d}", "seq_inter", inter=inter)
    for unexp in [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70]:
        add(f"seq_unexp_{int(unexp * 100):02d}", "seq_unexp", unexp=unexp)
    for rrt in [0.00, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40]:
        add(f"seq_rrt_{int(rrt * 100):02d}", "seq_rrt", rrt=rrt)
    for anchor in [0.00, 0.025, 0.05, 0.10, 0.15, 0.20]:
        add(f"seq_anchor_{int(anchor * 1000):03d}", "seq_anchor", anchor=anchor)

    # Categorical mode: probabilities are direct category masses. Keep total
    # mass below 1 so the remainder remains pure-uniform exploration.
    for inter in [0.20, 0.30, 0.40, 0.50]:
        for unexp in [0.15, 0.25, 0.35, 0.45]:
            for rrt in [0.05, 0.15]:
                total = inter + unexp + rrt + 0.05
                if total <= 0.95:
                    add(
                        f"cat_i{int(inter*100):02d}_u{int(unexp*100):02d}_r{int(rrt*100):02d}_a05",
                        "cat_grid",
                        inter=inter,
                        unexp=unexp,
                        rrt=rrt,
                        anchor=0.05,
                        uniform=max(0.0, 1.0 - total),
                        categorical=True,
                    )
    for uniform in [0.00, 0.10, 0.20, 0.30]:
        total = 0.35 + 0.30 + 0.10 + 0.05 + uniform
        if total <= 0.95:
            add(
                f"cat_uniform_{int(uniform*100):02d}",
                "cat_uniform",
                inter=0.35,
                unexp=0.30,
                rrt=0.10,
                anchor=0.05,
                uniform=uniform,
                categorical=True,
            )
    return cases


def command_for(args: argparse.Namespace, case: dict[str, Any], case_dir: Path) -> list[str]:
    cmd = [
        sys.executable,
        str(PROFILE),
        "--out-dir", str(case_dir),
        "--seeds-list", args.seeds_list,
        "--threads", str(args.threads),
        "--timeout-ms", str(args.timeout_ms),
        "--ffb-depth", str(args.ffb_depth),
        "--connector-pave-depth", str(args.ffb_depth),
        "--rbf-ffb-start-depth", str(args.ffb_start_depth),
        "--max-boxes", str(args.max_boxes),
        "--bootstrap-boxes", str(args.bootstrap_boxes),
        "--rrt-goal-bias", str(case["rrt"]),
        "--intertree-goal-bias", str(case["inter"]),
        "--unexplored-prob", str(case["unexp"]),
        "--anchor-target-prob", str(case["anchor"]),
        "--sample-uniform-prob", str(case["uniform"]),
        "--connector-max-pairs-per-gap", str(args.connector_max_pairs_per_gap),
        "--connector-pair-timeout-ms", str(args.connector_pair_timeout_ms),
        "--connector-rrt-iters", str(args.connector_rrt_iters),
        "--connector-rrt-timeout-ms", str(args.connector_rrt_timeout_ms),
        "--connector-bridge-boxes", "0",
        "--connector-pave-max-chain", "0",
    ]
    if bool(case["categorical"]):
        cmd.append("--sample-categorical-allocation")
    return cmd


def summarize_profile(path: Path, case: dict[str, Any], elapsed_s: float, returncode: int) -> dict[str, Any]:
    row: dict[str, Any] = {
        "name": case["name"],
        "family": case["family"],
        "categorical": bool(case["categorical"]),
        "rrt": float(case["rrt"]),
        "inter": float(case["inter"]),
        "unexp": float(case["unexp"]),
        "anchor": float(case["anchor"]),
        "uniform": float(case["uniform"]),
        "elapsed_s": elapsed_s,
        "returncode": int(returncode),
        "artifact": str(path),
    }
    if not path.exists():
        row["ok_count"] = 0
        row["n"] = 0
        return row
    payload = json.loads(path.read_text(encoding="utf-8"))
    aggregate = payload.get("aggregate", {})
    stages = aggregate.get("stage_times_ms", {})
    row.update({
        "ok_count": int(aggregate.get("ok_count", 0)),
        "n": int(aggregate.get("n", 0)),
        "success_rate": float(aggregate.get("ok_count", 0)) / max(1, int(aggregate.get("n", 0))),
        "boxes_median": median_field(aggregate, "box_count"),
        "grow_islands_median": median_field(aggregate, "grow_adjacency_islands"),
        "final_islands_median": median_field(aggregate, "adjacency_islands"),
        "build_ms_median": median_field(stages, "build_wall_ms"),
        "grow_ms_median": median_field(stages, "grow_ms"),
        "connector_ms_median": median_field(stages, "connector_ms"),
        "raw_path_median": median_field(aggregate, "raw_path_total"),
        "raw_segment_fraction_median": median_field(aggregate, "raw_path_segment_fraction_total"),
        "final_path_median": median_field(aggregate, "final_path_total"),
        "route_boxes_median": median_field(aggregate, "mission_route_box_sequence_total"),
        "route_segment_edges_median": median_field(aggregate, "mission_route_unique_segment_edge_count"),
        "segment_edges_median": median_field(aggregate, "segment_edge_count"),
    })
    return row


def write_summary(out_dir: Path, rows: list[dict[str, Any]]) -> None:
    out_json = out_dir / "sampling_sweep_summary.json"
    out_md = out_dir / "sampling_sweep_summary.md"
    out_json.write_text(json.dumps({"rows": rows}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ranked = sorted(
        rows,
        key=lambda row: (
            -float(row.get("success_rate", 0.0)),
            float(row.get("raw_segment_fraction_median", 999.0)),
            float(row.get("build_ms_median", 999999.0)),
            float(row.get("final_path_median", 999999.0)),
        ),
    )
    lines = [
        "# Exp04 sampling probability sweep",
        "",
        "| rank | case | family | mode | ok | build ms | conn ms | final len | raw seg frac | route segs | islands | probs i/u/r/a/unif |",
        "|---:|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for idx, row in enumerate(ranked, 1):
        mode = "cat" if row.get("categorical") else "seq"
        probs = (
            f"{row.get('inter', 0):.2f}/"
            f"{row.get('unexp', 0):.2f}/"
            f"{row.get('rrt', 0):.2f}/"
            f"{row.get('anchor', 0):.3f}/"
            f"{row.get('uniform', 0):.2f}"
        )
        lines.append(
            f"| {idx} | {row['name']} | {row['family']} | {mode} | "
            f"{row.get('ok_count', 0)}/{row.get('n', 0)} | "
            f"{row.get('build_ms_median', 0):.1f} | {row.get('connector_ms_median', 0):.1f} | "
            f"{row.get('final_path_median', 0):.3f} | {row.get('raw_segment_fraction_median', 0):.3f} | "
            f"{row.get('route_segment_edges_median', 0):.0f} | "
            f"{row.get('grow_islands_median', 0):.0f}->{row.get('final_islands_median', 0):.0f} | {probs} |"
        )
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sweep Exp04 grower sampling probabilities.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--seeds-list", default="0,1,2")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--timeout-ms", type=float, default=8000.0)
    parser.add_argument("--max-boxes", type=int, default=280)
    parser.add_argument("--bootstrap-boxes", type=int, default=210)
    parser.add_argument("--ffb-depth", type=int, default=28)
    parser.add_argument("--ffb-start-depth", type=int, default=15)
    parser.add_argument("--connector-max-pairs-per-gap", type=int, default=2)
    parser.add_argument("--connector-pair-timeout-ms", type=float, default=60.0)
    parser.add_argument("--connector-rrt-iters", type=int, default=5000)
    parser.add_argument("--connector-rrt-timeout-ms", type=float, default=200.0)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true", default=False)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    cases = make_cases()
    if args.limit > 0:
        cases = cases[: int(args.limit)]
    rows: list[dict[str, Any]] = []
    for index, case in enumerate(cases, 1):
        case_dir = args.out_dir / case["name"]
        profile_path = case_dir / "recommended_profile.json"
        cmd = command_for(args, case, case_dir)
        print(f"[{index}/{len(cases)}] {case['name']}", flush=True)
        if args.dry_run:
            print(" ".join(cmd))
            continue
        start = time.perf_counter()
        completed = subprocess.run(cmd, cwd=REPO_ROOT)
        elapsed_s = time.perf_counter() - start
        row = summarize_profile(profile_path, case, elapsed_s, completed.returncode)
        rows.append(row)
        write_summary(args.out_dir, rows)
    if not args.dry_run:
        write_summary(args.out_dir, rows)
        print(f"wrote {args.out_dir / 'sampling_sweep_summary.json'}")
        print(f"wrote {args.out_dir / 'sampling_sweep_summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
