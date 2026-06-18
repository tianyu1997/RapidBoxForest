#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]

ROOT_FILES = {
    ".gitignore",
    "CITATION.cff",
    "CMakeLists.txt",
    "LICENSE",
    "README.md",
    "environment.yml",
    "requirements-experiments.txt",
}

ALLOWED_PREFIXES = (
    ".github/",
    "docs/",
    "experiments/",
    "lect_database/",
    "link_interval_envelope/",
    "safe_box_forest/",
    "scripts/",
)

DEFAULT_EXCLUDE_PATTERNS = (
    "experiments/archive/**",
    "experiments/exp06_random_robot/run_saved_catalog_iris_gcs.py",
    "docs/archive/**",
    "docs/*PLAN*.md",
    "docs/TABLE*.md",
    "docs/obb*.md",
    "docs/paper_improve.md",
    "docs/SYMMETRY_COMPRESSION_TRAITS.md",
    "docs/TwoStageReusableExperiments.md",
    "docs/分级partition连通.md",
    "lect_database/docs/*PLAN*.md",
    "link_interval_envelope/docs/*PLAN*.md",
    "safe_box_forest/docs/*REPORT*.md",
    "safe_box_forest/docs/EXPERIMENT_REPRODUCTION.md",
    "safe_box_forest/docs/PAPER_ARTIFACTS.md",
    "safe_box_forest/experiments/sbf_old/**",
    "safe_box_forest/experiments/archive/**",
    "safe_box_forest/experiments/**",
    "safe_box_forest/experiments/tro2026_plans/**",
    "safe_box_forest/docs/*PLAN*.md",
    "safe_box_forest/docs/PLAN.md",
    "safe_box_forest/docs/STATUS.md",
    "safe_box_forest/docs/EXPERIMENT_MIGRATION_STATUS.md",
    "safe_box_forest/docs/MARCUCCI_QUALITY_AWARE_GROW_PLAN.md",
    "safe_box_forest/docs/TRO_REWRITE_EXECUTION_PLAN_20260505.md",
    "paper/**",
    "paper/sbf_old/**",
    "paper/history/**",
    "safe_box_forest/paper/sbf_old/**",
    "safe_box_forest/scripts/regentable.py",
    "experiments/exp04_shelf_leaf_rrt/compare_ffb_native_vs_canonical.py",
    "experiments/exp04_shelf_leaf_rrt/diagnose_ts_cs_gap.py",
    "experiments/exp04_shelf_leaf_rrt/run_optimized_baseline_compare.py",
    "experiments/exp04_shelf_leaf_rrt/run_validation_boundary_ablation.py",
    "experiments/exp04_shelf_leaf_rrt/scan_critsample_d23_config.py",
    "experiments/exp04_shelf_leaf_rrt/scan_full_root_depths.py",
    "experiments/exp04_shelf_leaf_rrt/study_ts_cs_box_cover.py",
    "experiments/exp07_dynamic_update/run_update_replan_diagnostic.py",
    "improve_workspace/**",
    "openai-skills/**",
    "outputs/**",
    "**/outputs/**",
    ".sbf_lect_database/**",
    "**/.sbf_lect_database/**",
    "build*/**",
    "**/build*/**",
    "__pycache__/**",
    "**/__pycache__/**",
    "*.pyc",
    "**/*.pyc",
    "*.aux",
    "*.log",
    "*.fls",
    "*.fdb_latexmk",
    "*.xdv",
    "*.out",
    "*.blg",
    "*.bcf",
    "*.run.xml",
    "*.synctex.gz",
    "docs/improve*.md",
    "docs/exp*_status_*.md",
    "docs/exp*_results_*.md",
    "docs/exp*_result_*.md",
    "docs/exp*_summary_*.md",
    "docs/EXP*_STATUS_*.md",
    "docs/EXP*_RESULTS_*.md",
    "docs/EXP*_RESULT_*.md",
    "docs/EXP*_SUMMARY_*.md",
)

ARCHIVE_PATTERNS = (
    "experiments/archive/**",
    "experiments/exp06_random_robot/run_saved_catalog_iris_gcs.py",
    "docs/archive/**",
    "docs/*PLAN*.md",
    "docs/TABLE*.md",
    "docs/obb*.md",
    "docs/paper_improve.md",
    "docs/SYMMETRY_COMPRESSION_TRAITS.md",
    "docs/TwoStageReusableExperiments.md",
    "docs/分级partition连通.md",
    "lect_database/docs/*PLAN*.md",
    "link_interval_envelope/docs/*PLAN*.md",
    "safe_box_forest/docs/*REPORT*.md",
    "safe_box_forest/docs/EXPERIMENT_REPRODUCTION.md",
    "safe_box_forest/docs/PAPER_ARTIFACTS.md",
    "safe_box_forest/experiments/sbf_old/**",
    "safe_box_forest/experiments/archive/**",
    "safe_box_forest/experiments/**",
    "safe_box_forest/experiments/tro2026_plans/**",
    "safe_box_forest/docs/*PLAN*.md",
    "safe_box_forest/docs/PLAN.md",
    "safe_box_forest/docs/STATUS.md",
    "safe_box_forest/docs/EXPERIMENT_MIGRATION_STATUS.md",
    "safe_box_forest/docs/MARCUCCI_QUALITY_AWARE_GROW_PLAN.md",
    "safe_box_forest/docs/TRO_REWRITE_EXECUTION_PLAN_20260505.md",
    "paper/sbf_old/**",
    "paper/history/**",
    "safe_box_forest/paper/sbf_old/**",
    "safe_box_forest/scripts/regentable.py",
    "experiments/exp04_shelf_leaf_rrt/compare_ffb_native_vs_canonical.py",
    "experiments/exp04_shelf_leaf_rrt/diagnose_ts_cs_gap.py",
    "experiments/exp04_shelf_leaf_rrt/run_optimized_baseline_compare.py",
    "experiments/exp04_shelf_leaf_rrt/run_validation_boundary_ablation.py",
    "experiments/exp04_shelf_leaf_rrt/scan_critsample_d23_config.py",
    "experiments/exp04_shelf_leaf_rrt/scan_full_root_depths.py",
    "experiments/exp04_shelf_leaf_rrt/study_ts_cs_box_cover.py",
    "experiments/exp07_dynamic_update/run_update_replan_diagnostic.py",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a clean public RapidBoxForest release tree.")
    parser.add_argument("--out-dir", type=Path, required=True, help="Destination directory for the public tree.")
    parser.add_argument("--force", action="store_true", help="Remove an existing destination before exporting.")
    parser.add_argument(
        "--include-archive",
        action="store_true",
        help="Include historical archive/sbf_old/paper history files. Disabled by default for public release.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the file count without copying.")
    return parser.parse_args()


def git_files() -> list[str]:
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=str(REPO_ROOT),
        check=True,
        stdout=subprocess.PIPE,
    )
    return sorted(
        path
        for path in result.stdout.decode("utf-8").split("\0")
        if path and (REPO_ROOT / path).is_file()
    )


def allowed(path: str) -> bool:
    return path in ROOT_FILES or any(path.startswith(prefix) for prefix in ALLOWED_PREFIXES)


def excluded(path: str, *, include_archive: bool) -> bool:
    patterns: Iterable[str] = DEFAULT_EXCLUDE_PATTERNS
    if include_archive:
        patterns = tuple(pattern for pattern in DEFAULT_EXCLUDE_PATTERNS if pattern not in ARCHIVE_PATTERNS)
    return any(fnmatch.fnmatch(path, pattern) for pattern in patterns)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def export_files(files: list[str], out_dir: Path) -> None:
    hashes: dict[str, str] = {}
    for rel in files:
        src = REPO_ROOT / rel
        dst = out_dir / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        hashes[rel] = file_sha256(src)
    manifest = {
        "source_repo": "RapidBoxForest public release export",
        "file_count": len(files),
        "policy": "allowlist export; generated outputs, local caches, build trees, and historical archives excluded by default",
        "files": files,
        "file_sha256": hashes,
    }
    (out_dir / "PUBLIC_RELEASE_MANIFEST.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    out_dir = args.out_dir.resolve()
    files = [path for path in git_files() if allowed(path) and not excluded(path, include_archive=args.include_archive)]
    if args.dry_run:
        print(f"would export {len(files)} files to {out_dir}")
        return 0
    if out_dir.exists():
        if not args.force:
            raise SystemExit(f"destination exists: {out_dir} (pass --force to replace it)")
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    export_files(files, out_dir)
    print(f"exported {len(files)} files to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
