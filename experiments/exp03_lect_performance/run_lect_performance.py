#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.common.experiment_io import DEFAULT_OUTPUT_ROOT, environment_metadata, namespace_dict, run_id, write_csv, write_json
from experiments.common.progress import progress


PAGE_STORE_STAGES = {
    "load.open_read_only": "Open",
    "read.node_box_disk": "Node box",
    "read.exact_box_lookup_disk": "Exact lookup",
    "read.range_query_disk": "Range query",
    "read.evidence_disk": "Evidence read",
    "write.checkpoint_dirty": "Checkpoint",
    "read.reopen_verify": "Reopen",
}

SNAPSHOT_STAGES = {
    "snapshot.load.open_read_only": "Open snapshot",
    "snapshot.read.node_box": "Node box",
    "snapshot.read.exact_box_lookup": "Exact lookup",
    "snapshot.read.endpoint_for_box_exact": "Endpoint lookup",
    "snapshot.read.endpoint_for_box_exact_hot2": "Endpoint lookup (hot)",
    "snapshot.read.range_query": "Range query",
    "snapshot.read.evidence": "Evidence read",
    "snapshot.read.evidence_hot2": "Evidence read (hot)",
}

DEFAULT_SNAPSHOT_DB = (
    DEFAULT_OUTPUT_ROOT
    / "tro2026"
    / "lectdb_defaults"
    / "cache"
    / "tro2026_ur5_p20_support_hull_d40_canonical_native_stateless"
)
DEFAULT_SNAPSHOT_LABEL = "UR5 d20 SupportHull cache"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exp.3 LECT operation and memory study.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / "tro2026" / "exp03")
    parser.add_argument("--phase", choices=["smoke", "pilot", "paper", "full", "assets"], default="smoke")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--mode", choices=["snapshot", "page-store"], default="snapshot")
    parser.add_argument("--existing-db", type=Path, default=DEFAULT_SNAPSHOT_DB)
    parser.add_argument("--snapshot-path", type=Path, default=None)
    parser.add_argument("--depth", type=int, default=8)
    parser.add_argument("--queries", type=int, default=20000)
    parser.add_argument("--evidence-records", type=int, default=20000)
    parser.add_argument("--dims", type=int, default=7)
    parser.add_argument("--payload-floats", type=int, default=84)
    parser.add_argument("--seed", type=int, default=6300)
    return parser.parse_args()


def phase_params(args: argparse.Namespace) -> tuple[int, int, int]:
    if args.phase == "smoke":
        return min(int(args.depth), 8), min(int(args.queries), 1000), min(int(args.evidence_records), 1000)
    if args.phase == "pilot":
        return min(int(args.depth), 9), min(int(args.queries), 5000), min(int(args.evidence_records), 5000)
    return int(args.depth), int(args.queries), int(args.evidence_records)


def benchmark_executable() -> Path:
    candidates = [
        REPO_ROOT / "build" / "lect_database" / "lect_database_benchmark",
        REPO_ROOT / "build-leaf-sweep" / "lect_database" / "lect_database_benchmark",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("lect_database_benchmark executable not found; build the repository first")


def read_csv_rows(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


CSV_FIELDS = [
    "operation",
    "stage",
    "operations",
    "elapsed_ms",
    "avg_us_per_op",
    "ops_per_sec",
    "nodes",
    "evidence",
    "page_reads",
    "cache_hits",
    "cache_misses",
    "ok",
]


def tex_num(value: Any, digits: int = 2) -> str:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return "--"
    if not math.isfinite(x):
        return "--"
    return f"{x:.{digits}f}"


def stage_map_for_mode(mode: str) -> dict[str, str]:
    return SNAPSHOT_STAGES if mode == "snapshot" else PAGE_STORE_STAGES


def write_tex(path: Path, rows: list[dict[str, Any]], meta: dict[str, Any]) -> None:
    mode = str(meta.get("mode", "snapshot"))
    if mode == "snapshot":
        source = str(meta.get("source_label", "persisted read snapshot"))
        nodes = int(float(rows[0].get("nodes", 0) or 0)) if rows else 0
        evidence = int(float(rows[0].get("evidence", 0) or 0)) if rows else 0
        caption = (
            "LECT read-snapshot microbenchmark on a persisted cache "
            f"({source}; {nodes:,} nodes, {evidence:,} evidence records)."
        )
    else:
        depth = int(meta.get("depth", 0) or 0)
        evidence_records = int(meta.get("evidence_records", 0) or 0)
        nodes = 2 ** (depth + 1) - 1 if depth >= 0 else 0
        caption = (
            "LECT page-store microbenchmark on a synthetic seven-dimensional "
            f"depth-{depth} tree ({nodes:,} nodes, {evidence_records:,} evidence records)."
        )
    lines = [
        r"\begin{table}[t]",
        r"\centering",
        rf"\caption{{{caption}}}",
        r"\label{tab:tro-lect-performance}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{2.2pt}",
    ]
    if mode == "snapshot":
        lines.extend([
            r"\begin{tabular}{lrrr}",
            r"\toprule",
            r"Operation & Ops & Avg. ($\mu$s) & Throughput (Mops/s) \\",
            r"\midrule",
        ])
        for row in rows:
            ops_per_sec = float(row.get("ops_per_sec", 0.0) or 0.0) / 1.0e6
            lines.append(
                f"{row['operation']} & {int(float(row.get('operations', 0) or 0))} & "
                f"{tex_num(row.get('avg_us_per_op'), 2)} & "
                f"{tex_num(ops_per_sec, 2)} \\\\"
            )
    else:
        lines.extend([
            r"\begin{tabular}{lrrrr}",
            r"\toprule",
            r"Operation & Ops & Avg. ($\mu$s) & Pages read & Hit rate (\%) \\",
            r"\midrule",
        ])
        for row in rows:
            hits = float(row.get("cache_hits", 0.0) or 0.0)
            misses = float(row.get("cache_misses", 0.0) or 0.0)
            hit_rate = 100.0 * hits / max(hits + misses, 1.0)
            lines.append(
                f"{row['operation']} & {int(float(row.get('operations', 0) or 0))} & "
                f"{tex_num(row.get('avg_us_per_op'), 2)} & "
                f"{int(float(row.get('page_reads', 0) or 0))} & "
                f"{tex_num(hit_rate, 1)} \\\\"
            )
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def run_benchmark(args: argparse.Namespace) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    depth, queries, evidence_records = phase_params(args)
    db_path = args.out_dir / "lect_benchmark_db"
    raw_csv = args.out_dir / "lect_benchmark_raw.csv"
    if args.mode == "page-store" and db_path.exists():
        shutil.rmtree(db_path)
    exe = benchmark_executable()
    if args.mode == "snapshot":
        snapshot_path = args.snapshot_path or (args.existing_db / "lect_snapshot")
        use_existing_snapshot = args.existing_db.exists() and snapshot_path.exists()
        if use_existing_snapshot:
            command = [
                str(exe),
                "--existing-db",
                "--snapshot",
                "--reuse-snapshot",
                "--read-stages-only",
                "--db",
                str(args.existing_db),
                "--snapshot-path",
                str(snapshot_path),
                "--csv",
                str(raw_csv),
                "--queries",
                str(queries),
                "--evidence-records",
                str(evidence_records),
                "--seed",
                str(int(args.seed)),
            ]
            source = str(args.existing_db)
            source_label = DEFAULT_SNAPSHOT_LABEL if args.existing_db == DEFAULT_SNAPSHOT_DB else args.existing_db.name
        else:
            if db_path.exists():
                shutil.rmtree(db_path)
            command = [
                str(exe),
                "--snapshot",
                "--db",
                str(db_path),
                "--csv",
                str(raw_csv),
                "--dims",
                str(int(args.dims)),
                "--depth",
                str(depth),
                "--queries",
                str(queries),
                "--evidence-records",
                str(evidence_records),
                "--payload-floats",
                str(int(args.payload_floats)),
                "--seed",
                str(int(args.seed)),
            ]
            source = "synthetic"
            source_label = "synthetic read snapshot"
    else:
        command = [
            str(exe),
            "--db",
            str(db_path),
            "--csv",
            str(raw_csv),
            "--dims",
            str(int(args.dims)),
            "--depth",
            str(depth),
            "--queries",
            str(queries),
            "--evidence-records",
            str(evidence_records),
            "--payload-floats",
            str(int(args.payload_floats)),
            "--seed",
            str(int(args.seed)),
        ]
        source = "synthetic page-store"
        source_label = source
    for _ in progress([command], desc="exp03 benchmark", total=1):
        completed = subprocess.run(command, cwd=str(REPO_ROOT), text=True, capture_output=True, check=False)
    raw_rows = read_csv_rows(raw_csv) if raw_csv.exists() else []
    stage_map = stage_map_for_mode(args.mode)
    selected: list[dict[str, Any]] = []
    for row in raw_rows:
        stage = str(row.get("stage", ""))
        if stage not in stage_map:
            continue
        if str(row.get("ok", "0")) not in {"1", "true", "True"}:
            continue
        selected.append({"operation": stage_map[stage], **row})
    meta = {
        "command": command,
        "returncode": int(completed.returncode),
        "stdout_tail": completed.stdout[-4000:],
        "stderr_tail": completed.stderr[-4000:],
        "raw_csv": str(raw_csv),
        "depth": depth,
        "queries": queries,
        "evidence_records": evidence_records,
        "mode": args.mode,
        "source": source,
        "source_label": source_label,
        "raw_rows": len(raw_rows),
        "selected_rows": len(selected),
    }
    if len(selected) < len(stage_map):
        missing = sorted(set(stage_map) - {str(row.get("stage")) for row in selected})
        raise RuntimeError(f"LECT benchmark did not produce required OK stages: {missing}; meta={meta}")
    return selected, meta


def planned_rows(args: argparse.Namespace) -> list[dict[str, Any]]:
    depth, queries, evidence_records = phase_params(args)
    return [
        {
            "operation": label,
            "stage": stage,
            "status": "planned",
            "depth": depth,
            "queries": queries,
            "evidence_records": evidence_records,
        }
        for stage, label in stage_map_for_mode(args.mode).items()
    ]


def main() -> int:
    args = parse_args()
    summary_csv = args.out_dir / "lect_performance_summary.csv"
    tex_path = REPO_ROOT / "paper" / "generated" / "tab_tro_lect_performance.tex"
    meta: dict[str, Any] = {}
    rows = planned_rows(args)
    if not args.dry_run:
        rows, meta = run_benchmark(args)
        write_csv(summary_csv, rows, CSV_FIELDS)
        write_tex(tex_path, rows, meta)
    payload: dict[str, Any] = {
        "experiment": "exp03_lect_performance",
        "run_id": run_id("exp03"),
        "phase": args.phase,
        "status": "dry_run" if args.dry_run else "executed",
        "environment": environment_metadata(),
        "params": namespace_dict(args),
        "summary_csv": str(summary_csv) if not args.dry_run else None,
        "table": str(tex_path) if not args.dry_run else None,
        "benchmark": meta,
        "rows": rows,
    }
    write_json(args.out_dir / "lect_performance_manifest.json", payload)
    print(f"wrote {args.out_dir / 'lect_performance_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
