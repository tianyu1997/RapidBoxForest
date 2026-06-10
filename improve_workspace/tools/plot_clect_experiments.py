#!/usr/bin/env python3
"""Generate sidecar experiment figures for docs/improve.md Section 7."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


REPO_ROOT = Path(__file__).resolve().parents[2]


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def savefig(path: Path) -> None:
    plt.tight_layout()
    plt.savefig(path, dpi=180)
    plt.close()


def plot_scaling_cells(scaling: dict[str, Any], out_dir: Path) -> Path:
    summaries = scaling["depth_summaries"]
    depths = [row["depth"] for row in summaries]
    plt.figure(figsize=(7.0, 4.2))
    plt.plot(depths, [row["fixed_materialized_cells"] for row in summaries], marker="o", label="fixed")
    plt.plot(depths, [row["early_stop_materialized_cells"] for row in summaries], marker="o", label="early-stop")
    plt.plot(depths, [row["full_materialized_cells"] for row in summaries], marker="o", label="full C-LECT")
    plt.yscale("log", base=2)
    plt.xlabel("max depth")
    plt.ylabel("materialized cells")
    plt.title("Materialized Cells vs Depth")
    plt.grid(True, which="both", alpha=0.25)
    plt.legend()
    path = out_dir / "clect_scaling_materialized_cells.png"
    savefig(path)
    return path


def plot_scaling_reductions(scaling: dict[str, Any], out_dir: Path) -> Path:
    summaries = scaling["depth_summaries"]
    depths = [row["depth"] for row in summaries]
    plt.figure(figsize=(7.0, 4.2))
    plt.plot(depths, [row["early_stop_reduction_vs_fixed"] for row in summaries], marker="o", label="early-stop")
    plt.plot(depths, [row["occupied_certificate_reduction_vs_fail_only"] for row in summaries], marker="o", label="occupied cert")
    plt.plot(depths, [row["no_good_reduction_vs_disabled"] for row in summaries], marker="o", label="no-good")
    plt.plot(depths, [row["full_reduction_vs_fixed"] for row in summaries], marker="o", label="full C-LECT")
    plt.yscale("log")
    plt.xlabel("max depth")
    plt.ylabel("reduction factor")
    plt.title("Mechanism Reduction Factors vs Depth")
    plt.grid(True, which="both", alpha=0.25)
    plt.legend()
    path = out_dir / "clect_scaling_reduction_factors.png"
    savefig(path)
    return path


def plot_scaling_graph_vertices(scaling: dict[str, Any], out_dir: Path) -> Path:
    summaries = scaling["depth_summaries"]
    depths = [row["depth"] for row in summaries]
    plt.figure(figsize=(7.0, 4.2))
    plt.plot(depths, [row["sparse_graph_vertices"] for row in summaries], marker="o", label="sparse")
    plt.plot(depths, [row["portal_graph_vertices"] for row in summaries], marker="o", label="portal")
    plt.plot(depths, [row["full_graph_vertices"] for row in summaries], marker="o", label="full C-LECT")
    plt.xlabel("max depth")
    plt.ylabel("global graph vertices")
    plt.title("Global Graph Vertices vs Depth")
    plt.grid(True, alpha=0.25)
    plt.legend()
    path = out_dir / "clect_scaling_graph_vertices.png"
    savefig(path)
    return path


def histogram_to_series(hist: dict[str, int]) -> tuple[list[int], list[int]]:
    items = sorted((int(k), int(v)) for k, v in hist.items())
    return [item[0] for item in items], [item[1] for item in items]


def plot_depth_histograms(suite: dict[str, Any], out_dir: Path, key: str, filename: str, title: str) -> Path:
    plt.figure(figsize=(7.0, 4.2))
    for row in suite["rows"]:
        hist = row.get(key, {})
        if not hist:
            continue
        depths, counts = histogram_to_series(hist)
        plt.plot(depths, counts, marker="o", label=row["variant"])
    plt.yscale("log")
    plt.xlabel("cell depth")
    plt.ylabel("count")
    plt.title(title)
    plt.grid(True, which="both", alpha=0.25)
    plt.legend(fontsize=7)
    path = out_dir / filename
    savefig(path)
    return path


def write_manifest(path: Path, figures: list[Path]) -> None:
    payload = {
        "figures": [str(figure) for figure in figures],
        "count": len(figures),
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def write_md(path: Path, figures: list[Path]) -> None:
    lines = ["# C-LECT Sidecar Figures", ""]
    for figure in figures:
        rel = figure.relative_to(REPO_ROOT)
        lines.append(f"- `{rel}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite-json", default="improve_workspace/clect_experiment_suite.json")
    parser.add_argument("--scaling-json", default="improve_workspace/clect_scaling_experiment.json")
    parser.add_argument("--out-dir", default="improve_workspace/figures")
    parser.add_argument("--manifest-out", default="improve_workspace/clect_figures_manifest.json")
    parser.add_argument("--md-out", default="improve_workspace/clect_figures.md")
    args = parser.parse_args()

    suite = load_json(REPO_ROOT / args.suite_json)
    scaling = load_json(REPO_ROOT / args.scaling_json)
    out_dir = REPO_ROOT / args.out_dir
    ensure_dir(out_dir)
    figures = [
        plot_scaling_cells(scaling, out_dir),
        plot_scaling_reductions(scaling, out_dir),
        plot_scaling_graph_vertices(scaling, out_dir),
        plot_depth_histograms(
            suite,
            out_dir,
            "accepted_free_depth_histogram",
            "clect_accepted_free_depth_histogram.png",
            "Accepted Free Cell Depth Distribution",
        ),
        plot_depth_histograms(
            suite,
            out_dir,
            "materialized_cell_depth_histogram",
            "clect_materialized_cell_depth_histogram.png",
            "Materialized Cell Depth Distribution",
        ),
    ]
    manifest_path = REPO_ROOT / args.manifest_out
    md_path = REPO_ROOT / args.md_out
    write_manifest(manifest_path, figures)
    write_md(md_path, figures)
    print(json.dumps({
        "manifest_out": str(manifest_path),
        "md_out": str(md_path),
        "figures": [str(path) for path in figures],
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
