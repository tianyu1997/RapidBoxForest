#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fnmatch
import json
import re
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from export_public_release import (  # noqa: E402
    DEFAULT_EXCLUDE_PATTERNS,
    allowed as export_allowed,
    check_forbidden_source_sidecars,
    excluded as export_excluded,
)


REQUIRED_RELEASE_FILES = (
    "LICENSE",
    "CITATION.cff",
    "README.md",
    "environment.yml",
    "requirements-experiments.txt",
    "docs/REPRODUCIBILITY.md",
    "docs/OPEN_SOURCE_RELEASE_CHECKLIST.md",
    "docs/cache_artifacts.example.json",
    "scripts/export_public_release.py",
    "scripts/check_public_release.py",
    "scripts/check_release_readiness.py",
    "scripts/check_cache_artifacts.py",
    "scripts/package_cache_artifacts.py",
    "scripts/fill_cache_artifact_urls.py",
    "scripts/check_paper_result_sources.py",
    "scripts/check_public_package.py",
    "scripts/package_public_release.py",
    "scripts/set_public_release_urls.py",
    "scripts/init_public_repository.py",
    "scripts/self_test_release_tools.py",
    ".github/workflows/ci.yml",
)

TRACKED_GENERATED_PATTERNS = (
    "outputs",
    "tmp",
    ".sbf_lect_database",
    "**/__pycache__",
    "*.pyc",
    "*.aux",
    "*.log",
    "*.xdv",
    "*.fdb_latexmk",
    "*.fls",
)

STALE_REFERENCE_RE = re.compile(
    r"(sbf_old|legacy_demos|SBF_OLD_DIR|paper/sbf_old|experiments/archive|sbf-standalone|improve_workspace|github\.com/tianyu1997/SafeBoxForest)",
    re.IGNORECASE,
)

PLACEHOLDER_RE = re.compile(r"(TODO|CHANGE_ME|<owner>|example\.com|example-owner)")
URL_RE = re.compile(r"^https://[^ \t\r\n]+$")
DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
SKIP_SCAN_DIRS = {
    ".git",
    ".sbf_lect_database",
    "__pycache__",
    "build",
    "build-public-final-check",
    "build-release-check",
    "outputs",
    "tmp",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a final source-tree release-readiness audit.")
    parser.add_argument("--repo-root", type=Path, default=Path("."), help="Repository root.")
    parser.add_argument("--public-tree", type=Path, default=None, help="Optional exported public tree to validate.")
    parser.add_argument(
        "--cache-manifest",
        type=Path,
        default=None,
        help=(
            "Optional filled cache artifact manifest. If omitted, only the "
            "docs/cache_artifacts.example.json template is checked and TODO "
            "placeholders are allowed."
        ),
    )
    parser.add_argument(
        "--cache-archive-dir",
        type=Path,
        default=None,
        help="Optional directory containing downloaded cache archives to verify with --cache-manifest.",
    )
    parser.add_argument(
        "--package-manifest",
        type=Path,
        default=None,
        help="Optional RapidBoxForest-public.package.json to validate with the release audit.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Require final public source metadata. Cache artifact metadata is strict only when --cache-manifest is supplied.",
    )
    parser.add_argument(
        "--tracking-only",
        action="store_true",
        help="Only verify release-critical files are present/tracked and generated artifacts are not tracked.",
    )
    return parser.parse_args()


def run_command(command: list[str], *, cwd: Path) -> tuple[int, str, str]:
    result = subprocess.run(command, cwd=str(cwd), text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    return result.returncode, result.stdout, result.stderr


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} root must be a JSON object")
    return data


def check_required_files(root: Path) -> list[str]:
    return [f"missing required release file: {rel}" for rel in REQUIRED_RELEASE_FILES if not (root / rel).exists()]


def check_required_files_tracked(root: Path, *, strict: bool) -> list[str]:
    if not strict:
        return []
    if not (root / ".git").exists():
        return ["strict release readiness requires a git checkout so required release files can be verified as tracked"]
    command = ["git", "ls-files", "--", *REQUIRED_RELEASE_FILES]
    code, stdout, stderr = run_command(command, cwd=root)
    if code != 0:
        return [f"git ls-files required release file check failed: {stderr.strip()}"]
    tracked = {line.strip() for line in stdout.splitlines() if line.strip()}
    missing = [rel for rel in REQUIRED_RELEASE_FILES if rel not in tracked]
    if missing:
        return [f"required release files are present but not git-tracked: {missing}"]
    return []


def check_public_manifest_files_tracked(root: Path, *, strict: bool) -> list[str]:
    if not strict:
        return []
    manifest_path = root / "PUBLIC_RELEASE_MANIFEST.json"
    if not manifest_path.exists():
        return ["missing PUBLIC_RELEASE_MANIFEST.json for strict tracked-file validation"]
    if not (root / ".git").exists():
        return ["strict release readiness requires a git checkout so public manifest files can be verified as tracked"]
    try:
        manifest = load_json(manifest_path)
    except Exception as exc:
        return [f"cannot parse PUBLIC_RELEASE_MANIFEST.json for tracked-file validation: {exc}"]
    files = manifest.get("files")
    if not isinstance(files, list) or not all(isinstance(item, str) for item in files):
        return ["PUBLIC_RELEASE_MANIFEST.json files must be a list of relative paths"]
    expected = set(files)
    expected.add("PUBLIC_RELEASE_MANIFEST.json")
    command = ["git", "ls-files"]
    code, stdout, stderr = run_command(command, cwd=root)
    if code != 0:
        return [f"git ls-files public manifest tracking check failed: {stderr.strip()}"]
    tracked = {line.strip() for line in stdout.splitlines() if line.strip()}
    missing = sorted(expected - tracked)
    if missing:
        return [f"public manifest files are present but not git-tracked: {missing[:40]}"]
    return []


def check_tracked_generated(root: Path) -> list[str]:
    if not (root / ".git").exists():
        return []
    command = ["git", "ls-files", *TRACKED_GENERATED_PATTERNS]
    code, stdout, stderr = run_command(command, cwd=root)
    if code != 0:
        return [f"git ls-files generated check failed: {stderr.strip()}"]
    tracked = [line for line in stdout.splitlines() if line.strip()]
    if tracked:
        return [f"generated/cache/intermediate files are tracked: {tracked[:20]}"]
    return []


def check_tracking_only_staged_scope(root: Path) -> list[str]:
    if not (root / ".git").exists():
        return []
    command = ["git", "diff", "--cached", "--name-status"]
    code, stdout, stderr = run_command(command, cwd=root)
    if code != 0:
        return [f"git diff --cached staged-scope check failed: {stderr.strip()}"]
    unexpected: list[str] = []
    for raw_line in stdout.splitlines():
        if not raw_line.strip():
            continue
        parts = raw_line.split("\t")
        status = parts[0]
        path = parts[-1]
        public_exported = export_allowed(path) and not export_excluded(path, include_archive=False)
        if public_exported:
            continue
        if status.startswith("D") and not public_exported:
            continue
        if path.startswith("tmp/") and status.startswith("D"):
            continue
        unexpected.append(raw_line)
    if unexpected:
        return [
            "tracking-only staged release set contains paths outside release-cleanup allowlist: "
            f"{unexpected[:40]}"
        ]
    return []


def cff_scalar(text: str, key: str) -> str | None:
    prefix = f"{key}:"
    for line in text.splitlines():
        if not line.startswith(prefix):
            continue
        value = line[len(prefix) :].strip()
        if len(value) >= 2 and value[0] == '"' and value[-1] == '"':
            value = value[1:-1]
        return value
    return None


def check_citation_file(root: Path, path: Path, *, strict: bool, expected_title: str | None = None) -> list[str]:
    errors: list[str] = []
    try:
        rel = path.relative_to(root).as_posix()
    except ValueError:
        rel = path.as_posix()
    if not path.exists():
        return [f"missing citation file: {rel}"]
    text = path.read_text(encoding="utf-8")
    if expected_title is not None and f'title: "{expected_title}"' not in text:
        errors.append(f"{rel} must describe {expected_title}")
    if STALE_REFERENCE_RE.search(text):
        errors.append(f"{rel} contains stale repository references")
    if not strict:
        return errors
    if PLACEHOLDER_RE.search(text):
        errors.append(f"{rel} contains placeholders")
    for key in ("repository-code", "url"):
        value = cff_scalar(text, key)
        if value is None:
            errors.append(f"{rel} missing {key}")
        elif not URL_RE.fullmatch(value):
            errors.append(f"{rel} {key} must be an HTTPS URL")
        elif PLACEHOLDER_RE.search(value):
            errors.append(f"{rel} {key} contains a placeholder")
    doi = cff_scalar(text, "doi")
    if doi is not None and PLACEHOLDER_RE.search(doi):
        errors.append(f"{rel} doi contains a placeholder")
    release_date = cff_scalar(text, "date-released")
    if release_date is not None and not DATE_RE.fullmatch(release_date):
        errors.append(f"{rel} date-released must use YYYY-MM-DD")
    return errors


def check_citations(root: Path, *, strict: bool) -> list[str]:
    errors: list[str] = []
    errors.extend(check_citation_file(root, root / "CITATION.cff", strict=strict, expected_title="RapidBoxForest"))
    errors.extend(check_citation_file(root, root / "safe_box_forest" / "CITATION.cff", strict=strict))
    return errors


def check_stale_references(root: Path) -> list[str]:
    hits: list[str] = []
    suffixes = {".cff", ".cmake", ".cpp", ".h", ".hpp", ".json", ".md", ".py", ".sh", ".tex", ".txt", ".yaml", ".yml"}
    allowlist = {
        "scripts/check_public_release.py",
        "scripts/check_release_readiness.py",
        "scripts/export_public_release.py",
        "scripts/self_test_release_tools.py",
        "docs/OPEN_SOURCE_RELEASE_CHECKLIST.md",
        "docs/REPRODUCIBILITY.md",
    }
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        rel = path.relative_to(root).as_posix()
        if any(fnmatch.fnmatch(rel, pattern) for pattern in DEFAULT_EXCLUDE_PATTERNS):
            continue
        rel_parts = path.relative_to(root).parts
        if rel in allowlist or any(part in SKIP_SCAN_DIRS or part.startswith("build-") for part in rel_parts):
            continue
        if path.name != "CMakeLists.txt" and path.suffix not in suffixes:
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        if STALE_REFERENCE_RE.search(text):
            hits.append(rel)
    return [f"stale historical references outside allowlist: {hits[:20]}"] if hits else []


def check_cache_manifest(root: Path, manifest: Path | None, *, strict: bool, archive_dir: Path | None) -> list[str]:
    manifest_path = manifest or (root / "docs/cache_artifacts.example.json")
    if not manifest_path.is_absolute():
        manifest_path = root / manifest_path
    command = [sys.executable, "scripts/check_cache_artifacts.py", str(manifest_path)]
    if manifest is None:
        command.append("--allow-placeholders")
    if archive_dir is not None:
        archive_dir_path = archive_dir if archive_dir.is_absolute() else root / archive_dir
        command.extend(["--archive-dir", str(archive_dir_path)])
    code, stdout, stderr = run_command(command, cwd=root)
    if code != 0:
        return [
            f"cache artifact manifest check failed: {manifest_path}",
            stdout[-2000:],
            stderr[-2000:],
        ]
    return []


def check_paper_manifest(root: Path) -> list[str]:
    manifest_path = root / "paper/generated/tro_table_generation_manifest.json"
    if not manifest_path.exists():
        return []
    command = [
        sys.executable,
        "scripts/check_paper_result_sources.py",
        "--manifest",
        str(manifest_path.relative_to(root)),
        "--repo-root",
        ".",
        "--verify-local",
    ]
    code, stdout, stderr = run_command(command, cwd=root)
    if code != 0:
        return [
            "paper result source manifest check failed",
            stdout[-2000:],
            stderr[-2000:],
        ]
    return []


def check_public_tree(root: Path, public_tree: Path | None, *, strict: bool) -> list[str]:
    if public_tree is None:
        return []
    command = [
        sys.executable,
        str(root / "scripts/check_public_release.py"),
        str(public_tree),
        "--run-smoke-dry-run",
    ]
    if strict:
        command.append("--strict-citation")
    code, stdout, stderr = run_command(command, cwd=root)
    if code != 0:
        return [
            f"public tree check failed: {public_tree}",
            stdout[-2000:],
            stderr[-2000:],
        ]
    return []


def check_public_package(
    root: Path,
    package_manifest: Path | None,
    cache_manifest: Path | None,
    *,
    strict: bool,
    require_cache_archives_checked: bool,
    require_release_tools_checked: bool,
) -> list[str]:
    if package_manifest is None:
        return []
    manifest_path = package_manifest if package_manifest.is_absolute() else root / package_manifest
    command = [
        sys.executable,
        str(root / "scripts/check_public_package.py"),
        str(manifest_path),
    ]
    if cache_manifest is not None:
        cache_manifest_path = cache_manifest if cache_manifest.is_absolute() else root / cache_manifest
        command.extend(["--cache-manifest", str(cache_manifest_path)])
    if require_cache_archives_checked:
        command.append("--require-cache-archives-checked")
    if require_release_tools_checked:
        command.append("--require-release-tools-checked")
    if strict:
        command.append("--strict-metadata")
    code, stdout, stderr = run_command(command, cwd=root)
    if code != 0:
        return [
            f"public package check failed: {manifest_path}",
            stdout[-2000:],
            stderr[-2000:],
        ]
    return []


def main() -> int:
    args = parse_args()
    root = args.repo_root.resolve()
    if args.cache_archive_dir is not None and args.cache_manifest is None:
        print("FAIL: --cache-archive-dir requires --cache-manifest", file=sys.stderr)
        return 1
    errors: list[str] = []
    errors.extend(check_required_files(root))
    errors.extend(check_forbidden_source_sidecars(root))
    if args.tracking_only:
        errors.extend(check_required_files_tracked(root, strict=True))
        errors.extend(check_public_manifest_files_tracked(root, strict=True) if (root / "PUBLIC_RELEASE_MANIFEST.json").exists() else [])
        errors.extend(check_tracked_generated(root))
        errors.extend(check_tracking_only_staged_scope(root))
        if errors:
            for error in errors:
                print(f"FAIL: {error}", file=sys.stderr)
            return 1
        print(f"release tracking check passed: {root}")
        return 0
    errors.extend(check_required_files_tracked(root, strict=args.strict))
    errors.extend(check_public_manifest_files_tracked(root, strict=args.strict))
    errors.extend(check_tracked_generated(root))
    errors.extend(check_citations(root, strict=args.strict))
    errors.extend(check_stale_references(root))
    errors.extend(check_cache_manifest(root, args.cache_manifest, strict=args.strict, archive_dir=args.cache_archive_dir))
    errors.extend(check_paper_manifest(root))
    errors.extend(check_public_tree(root, args.public_tree.resolve() if args.public_tree else None, strict=args.strict))
    errors.extend(
        check_public_package(
            root,
            args.package_manifest,
            args.cache_manifest,
            strict=args.strict,
            require_cache_archives_checked=args.cache_archive_dir is not None,
            require_release_tools_checked=args.public_tree is not None,
        )
    )
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1
    mode = "strict" if args.strict else "template"
    print(f"release readiness check passed ({mode} mode): {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
