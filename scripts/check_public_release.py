#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

sys.dont_write_bytecode = True


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from export_public_release import DEFAULT_EXCLUDE_PATTERNS  # noqa: E402


LOCAL_PATH_RE = re.compile(
    b"("
    + b"/home/" + b"tian"
    + b"|" + b"\xe6\xa1\x8c\xe9\x9d\xa2"
    + b"|" + b"/mnt/" + b"data"
    + b"|" + b"C:" + b"\\\\"
    + b"|" + b"Us" + b"ers/"
    + b")"
)

REQUIRED_FILES = (
    ".github/workflows/ci.yml",
    "CMakeLists.txt",
    "README.md",
    "LICENSE",
    "CITATION.cff",
    "environment.yml",
    "requirements-experiments.txt",
    "docs/cache_artifacts.example.json",
    "docs/REPRODUCIBILITY.md",
    "docs/OPEN_SOURCE_RELEASE_CHECKLIST.md",
    "scripts/check_cache_artifacts.py",
    "scripts/package_cache_artifacts.py",
    "scripts/fill_cache_artifact_urls.py",
    "scripts/check_paper_result_sources.py",
    "scripts/check_release_readiness.py",
    "scripts/check_public_package.py",
    "scripts/set_public_release_urls.py",
    "scripts/package_public_release.py",
    "scripts/init_public_repository.py",
    "scripts/self_test_release_tools.py",
    "experiments/run_tro2026.py",
    "scripts/export_public_release.py",
    "scripts/check_public_release.py",
    "safe_box_forest/CITATION.cff",
)

OPTIONAL_PAPER_FILES = (
    "paper/generated/tab_tro_endpoint_envelope.tex",
    "paper/generated/tab_tro_link_envelope.tex",
    "paper/generated/tab_tro_shelf_ablation.tex",
    "paper/generated/tab_tro_shelf_cross_algorithm.tex",
    "paper/generated/tab_tro_random_summary.tex",
    "paper/generated/tab_tro_dynamic_update.tex",
    "paper/generated/fig_tro_shelf_tradeoff.pdf",
    "paper/generated/fig_tro_shelf_cross_tradeoff.pdf",
    "paper/generated/fig_tro_random_tradeoff.pdf",
    "paper/generated/tro_table_generation_manifest.json",
    "paper/figures/framework.png",
    "paper/figures/sbf_grow_1.png",
    "paper/figures/sbf_grow_2.png",
    "paper/figures/sbf_grow_3.png",
    "paper/figures/sbf_grow_4.png",
    "paper/figures/sbf_grow_5.png",
    "paper/figures/sbf_grow_6.png",
    "paper/figures/Marcucci_scene.png",
)

FORBIDDEN_EXPORTED_PATTERNS = DEFAULT_EXCLUDE_PATTERNS

TEXT_SUFFIXES = {
    "",
    ".bib",
    ".cff",
    ".cmake",
    ".cpp",
    ".h",
    ".hpp",
    ".json",
    ".md",
    ".py",
    ".sh",
    ".tex",
    ".txt",
    ".yaml",
    ".yml",
}

BROKEN_REFERENCE_RE = re.compile(
    rb"(sbf_old|legacy_demos|SBF_OLD_DIR|paper/sbf_old|experiments/archive|sbf-standalone|github\.com/tianyu1997/SafeBoxForest)"
)
BROKEN_REFERENCE_ALLOWLIST = {
    "scripts/export_public_release.py",
    "scripts/check_public_release.py",
    "scripts/check_release_readiness.py",
    "docs/OPEN_SOURCE_RELEASE_CHECKLIST.md",
    "docs/REPRODUCIBILITY.md",
}
MARKDOWN_LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
EMPTY_EXTENSIONLESS_ALLOWLIST = {
    ".gitignore",
    "LICENSE",
    "CMakeLists.txt",
}
URL_RE = re.compile(r"^https://[^ \t\r\n]+$")
DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
PLACEHOLDER_RE = re.compile(r"(TODO|CHANGE_ME|<owner>|example\.com|example-owner)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate a clean RapidBoxForest public release tree.")
    parser.add_argument("tree", type=Path, help="Public release tree to check.")
    parser.add_argument(
        "--pythonpath",
        action="append",
        default=[],
        help="Additional PYTHONPATH entry for Python extension and smoke execute checks. May be passed multiple times.",
    )
    parser.add_argument("--run-smoke-dry-run", action="store_true", help="Run dispatcher smoke dry-run with outputs outside the release tree.")
    parser.add_argument("--check-python-extension", action="store_true", help="Verify that link_interval_envelope Python bindings are importable.")
    parser.add_argument("--run-smoke-execute", action="store_true", help="Run dispatcher smoke execute with outputs outside the release tree. Requires Python bindings.")
    parser.add_argument("--check-release-tools", action="store_true", help="Run lightweight release-tool self-tests inside the public tree.")
    parser.add_argument(
        "--check-paper-compile",
        action="store_true",
        help="Compile paper/sbf_tro_2026.tex when the checked tree intentionally includes paper sources.",
    )
    parser.add_argument("--strict-citation", action="store_true", help="Require final repository URL metadata in exported CFF citation files.")
    return parser.parse_args()


def rel_files(root: Path) -> list[str]:
    return sorted(
        str(path.relative_to(root))
        for path in root.rglob("*")
        if path.is_file() and ".git" not in path.relative_to(root).parts
    )


def matches_forbidden(path: str) -> bool:
    from fnmatch import fnmatch

    return any(fnmatch(path, pattern) for pattern in FORBIDDEN_EXPORTED_PATTERNS)


def scan_local_paths(root: Path, files: list[str]) -> list[str]:
    hits: list[str] = []
    for rel in files:
        path = root / rel
        if path.name != "CMakeLists.txt" and path.suffix not in TEXT_SUFFIXES:
            continue
        try:
            data = path.read_bytes()
        except OSError:
            continue
        if LOCAL_PATH_RE.search(data):
            hits.append(rel)
    return hits


def scan_broken_references(root: Path, files: list[str]) -> list[str]:
    hits: list[str] = []
    for rel in files:
        if rel in BROKEN_REFERENCE_ALLOWLIST:
            continue
        path = root / rel
        if path.name != "CMakeLists.txt" and path.suffix not in TEXT_SUFFIXES:
            continue
        try:
            data = path.read_bytes()
        except OSError:
            continue
        if BROKEN_REFERENCE_RE.search(data):
            hits.append(rel)
    return hits


def scan_broken_markdown_links(root: Path, files: list[str]) -> list[str]:
    hits: list[str] = []
    root_resolved = root.resolve()
    for rel in files:
        if not rel.endswith(".md"):
            continue
        path = root / rel
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            text = path.read_text(errors="ignore")
        except OSError:
            continue
        for raw_target in MARKDOWN_LINK_RE.findall(text):
            target = raw_target.strip()
            if not target:
                continue
            # Drop optional Markdown title after a space. Paths with spaces in
            # this repository are not used in release docs.
            target = target.split()[0].strip("<>")
            if (
                not target
                or target.startswith("#")
                or target.startswith("http://")
                or target.startswith("https://")
                or target.startswith("mailto:")
            ):
                continue
            target_path = target.split("#", 1)[0]
            if not target_path:
                continue
            resolved = (path.parent / target_path).resolve()
            try:
                resolved.relative_to(root_resolved)
            except ValueError:
                hits.append(f"{rel} -> {raw_target} (outside release tree)")
                continue
            if not resolved.exists():
                hits.append(f"{rel} -> {raw_target}")
    return hits


def scan_suspicious_empty_files(root: Path, files: list[str]) -> list[str]:
    hits: list[str] = []
    for rel in files:
        path = root / rel
        if rel in EMPTY_EXTENSIONLESS_ALLOWLIST:
            continue
        if path.suffix:
            continue
        try:
            if path.stat().st_size == 0:
                hits.append(rel)
        except OSError:
            continue
    return hits


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


def check_citation_metadata(root: Path, *, strict: bool) -> list[str]:
    errors: list[str] = []
    paths = [
        "CITATION.cff",
        "safe_box_forest/CITATION.cff",
    ]
    for rel in paths:
        path = root / rel
        if not path.exists():
            errors.append(f"missing citation file: {rel}")
            continue
        text = path.read_text(encoding="utf-8")
        if PLACEHOLDER_RE.search(text) and strict:
            errors.append(f"{rel} contains citation placeholders")
        if not strict:
            continue
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


def check_manifest(root: Path, files: list[str]) -> list[str]:
    errors: list[str] = []
    manifest_path = root / "PUBLIC_RELEASE_MANIFEST.json"
    if not manifest_path.exists():
        return ["missing PUBLIC_RELEASE_MANIFEST.json"]
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return [f"cannot parse PUBLIC_RELEASE_MANIFEST.json: {exc}"]
    manifest_files = set(str(item) for item in manifest.get("files", []))
    actual_source_files = set(files) - {"PUBLIC_RELEASE_MANIFEST.json"}
    missing = sorted(manifest_files - actual_source_files)
    extra = sorted(actual_source_files - manifest_files)
    if missing:
        errors.append(f"manifest lists missing files: {missing[:10]}")
    if extra:
        errors.append(f"manifest omits files: {extra[:10]}")
    if int(manifest.get("file_count", -1)) != len(manifest_files):
        errors.append("manifest file_count does not match manifest file list")
    manifest_hashes = manifest.get("file_sha256")
    if not isinstance(manifest_hashes, dict):
        errors.append("manifest is missing file_sha256 map")
    else:
        hash_files = set(str(item) for item in manifest_hashes)
        missing_hashes = sorted(manifest_files - hash_files)
        extra_hashes = sorted(hash_files - manifest_files)
        if missing_hashes:
            errors.append(f"manifest file_sha256 omits files: {missing_hashes[:10]}")
        if extra_hashes:
            errors.append(f"manifest file_sha256 has extra files: {extra_hashes[:10]}")
        bad_hashes: list[str] = []
        for rel, expected in manifest_hashes.items():
            if rel not in actual_source_files:
                continue
            if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
                bad_hashes.append(str(rel))
                continue
            digest = hashlib.sha256()
            with (root / rel).open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
            if digest.hexdigest() != expected:
                bad_hashes.append(str(rel))
        if bad_hashes:
            errors.append(f"manifest file_sha256 mismatches: {bad_hashes[:10]}")
    return errors


def check_paper_assets(root: Path, files: list[str]) -> list[str]:
    errors: list[str] = []
    file_set = set(files)
    paper_present = any(rel == "paper/sbf_tro_2026.tex" or rel.startswith("paper/") for rel in file_set)
    if not paper_present:
        return []
    missing = [rel for rel in OPTIONAL_PAPER_FILES if rel not in file_set]
    if missing:
        errors.append(f"missing required paper assets: {missing}")
        return errors
    manifest_path = root / "paper/generated/tro_table_generation_manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return [f"cannot parse paper generated manifest: {exc}"]
    generated_assets = set(str(item) for item in manifest.get("assets", []))
    required_generated = {
        Path(rel).name
        for rel in OPTIONAL_PAPER_FILES
        if rel.startswith("paper/generated/") and Path(rel).name != "tro_table_generation_manifest.json"
    }
    missing_manifest_assets = sorted(required_generated - generated_assets)
    if missing_manifest_assets:
        errors.append(f"paper generated manifest omits required assets: {missing_manifest_assets}")
    if manifest.get("placeholder_mode") is not False:
        errors.append("paper generated manifest placeholder_mode must be false for release artifacts")
    return errors


def subprocess_env(*, pythonpath: list[Path] | None = None) -> dict[str, str]:
    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    if pythonpath:
        extra = os.pathsep.join(str(path) for path in pythonpath)
        existing = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = extra if not existing else extra + os.pathsep + existing
    return env


def check_cache_artifact_template(root: Path) -> list[str]:
    env = subprocess_env()
    command = [
        sys.executable,
        "scripts/check_cache_artifacts.py",
        "docs/cache_artifacts.example.json",
        "--allow-placeholders",
    ]
    result = subprocess.run(command, cwd=str(root), env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if result.returncode == 0:
        return []
    return [
        f"cache artifact template check failed with code {result.returncode}",
        result.stdout[-2000:],
        result.stderr[-2000:],
    ]


def check_paper_result_source_manifest(root: Path) -> list[str]:
    if not (root / "paper/generated/tro_table_generation_manifest.json").exists():
        return []
    env = subprocess_env()
    command = [
        sys.executable,
        "scripts/check_paper_result_sources.py",
        "--manifest",
        "paper/generated/tro_table_generation_manifest.json",
        "--repo-root",
        ".",
    ]
    result = subprocess.run(command, cwd=str(root), env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if result.returncode == 0:
        return []
    return [
        f"paper result source manifest check failed with code {result.returncode}",
        result.stdout[-2000:],
        result.stderr[-2000:],
    ]


def run_smoke_dry_run(root: Path, *, pythonpath: list[Path]) -> list[str]:
    with tempfile.TemporaryDirectory(prefix="rbf_public_smoke_") as tmp:
        env = subprocess_env(pythonpath=pythonpath)
        command = [
            sys.executable,
            "experiments/run_tro2026.py",
            "--phase",
            "smoke",
            "--dry-run",
            "--out-dir",
            tmp,
        ]
        result = subprocess.run(command, cwd=str(root), env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        if result.returncode != 0:
            return [
                f"smoke dry-run failed with code {result.returncode}",
                result.stdout[-2000:],
                result.stderr[-2000:],
            ]
    return []


def check_python_extension(root: Path, *, pythonpath: list[Path]) -> list[str]:
    code = (
        "import sys\n"
        "try:\n"
        "    import link_interval_envelope as lie\n"
        "except Exception as exc:\n"
        "    print(f'import failed: {exc}', file=sys.stderr)\n"
        "    raise SystemExit(1)\n"
        "missing = [name for name in ('Robot', 'Interval') if not hasattr(lie, name)]\n"
        "if missing:\n"
        "    print('missing binding attributes: ' + ','.join(missing), file=sys.stderr)\n"
        "    raise SystemExit(1)\n"
        "print(getattr(lie, '__version__', 'unknown'))\n"
    )
    env = subprocess_env(pythonpath=pythonpath)
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(root),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode == 0:
        return []
    return [
        "link_interval_envelope Python extension is not importable. "
        "Build with -DRBF_WITH_PYTHON=ON and pass --pythonpath build-python-smoke/python "
        "or set PYTHONPATH to the build/python directory.",
        result.stderr[-2000:],
    ]


def run_smoke_execute(root: Path, *, pythonpath: list[Path]) -> list[str]:
    with tempfile.TemporaryDirectory(prefix="rbf_public_smoke_execute_") as tmp:
        env = subprocess_env(pythonpath=pythonpath)
        command = [
            sys.executable,
            "experiments/run_tro2026.py",
            "--phase",
            "smoke",
            "--execute",
            "--out-dir",
            tmp,
        ]
        result = subprocess.run(command, cwd=str(root), env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        if result.returncode != 0:
            return [
                f"smoke execute failed with code {result.returncode}",
                result.stdout[-2000:],
                result.stderr[-2000:],
            ]
    return []


def check_release_tools(root: Path) -> list[str]:
    env = subprocess_env()
    command = [
        sys.executable,
        "scripts/self_test_release_tools.py",
    ]
    result = subprocess.run(command, cwd=str(root), env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if result.returncode == 0:
        return []
    return [
        f"release tool self-test failed with code {result.returncode}",
        result.stdout[-2000:],
        result.stderr[-2000:],
    ]


def check_paper_compile(root: Path) -> list[str]:
    if not (root / "paper/sbf_tro_2026.tex").exists():
        return ["paper/sbf_tro_2026.tex is not included in this public source tree"]
    if shutil.which("latexmk") is None:
        return ["latexmk is not available; install a TeX distribution or skip --check-paper-compile"]
    if shutil.which("xelatex") is None:
        return ["xelatex is not available; install a XeLaTeX-capable TeX distribution or skip --check-paper-compile"]
    with tempfile.TemporaryDirectory(prefix="rbf_public_paper_") as tmp:
        env = subprocess_env()
        command = [
            "latexmk",
            "-xelatex",
            "-interaction=nonstopmode",
            "-halt-on-error",
            f"-outdir={tmp}",
            "sbf_tro_2026.tex",
        ]
        result = subprocess.run(
            command,
            cwd=str(root / "paper"),
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if result.returncode != 0:
            return [
                f"paper compile failed with code {result.returncode}",
                result.stdout[-4000:],
                result.stderr[-4000:],
            ]
        if not (Path(tmp) / "sbf_tro_2026.pdf").exists():
            return ["paper compile did not produce sbf_tro_2026.pdf"]
    return []


def main() -> int:
    args = parse_args()
    root = args.tree.resolve()
    if not root.is_dir():
        print(f"not a directory: {root}", file=sys.stderr)
        return 2
    pythonpath = [
        (root / entry).resolve() if not Path(entry).is_absolute() else Path(entry).resolve()
        for entry in args.pythonpath
    ]
    files = rel_files(root)
    errors: list[str] = []
    for rel in REQUIRED_FILES:
        if rel not in files:
            errors.append(f"missing required file: {rel}")
    forbidden = [rel for rel in files if matches_forbidden(rel)]
    if forbidden:
        errors.append(f"forbidden files present: {forbidden[:20]}")
    local_hits = scan_local_paths(root, files)
    if local_hits:
        errors.append(f"local absolute path hits: {local_hits[:20]}")
    broken_refs = scan_broken_references(root, files)
    if broken_refs:
        errors.append(f"references to excluded historical entry points: {broken_refs[:20]}")
    broken_links = scan_broken_markdown_links(root, files)
    if broken_links:
        errors.append(f"broken local Markdown links: {broken_links[:20]}")
    empty_files = scan_suspicious_empty_files(root, files)
    if empty_files:
        errors.append(f"suspicious empty extensionless files: {empty_files[:20]}")
    errors.extend(check_citation_metadata(root, strict=bool(args.strict_citation)))
    errors.extend(check_manifest(root, files))
    errors.extend(check_paper_assets(root, files))
    errors.extend(check_cache_artifact_template(root))
    errors.extend(check_paper_result_source_manifest(root))
    if args.check_release_tools:
        errors.extend(check_release_tools(root))
    if args.run_smoke_dry_run:
        errors.extend(run_smoke_dry_run(root, pythonpath=pythonpath))
    if args.check_python_extension or args.run_smoke_execute:
        errors.extend(check_python_extension(root, pythonpath=pythonpath))
    if args.run_smoke_execute and not errors:
        errors.extend(run_smoke_execute(root, pythonpath=pythonpath))
    if args.check_paper_compile and not errors:
        errors.extend(check_paper_compile(root))
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print(f"public release check passed: {len(files)} files under {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
