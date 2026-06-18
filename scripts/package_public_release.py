#!/usr/bin/env python3
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
from collections import Counter
from pathlib import Path

sys.dont_write_bytecode = True


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[0]
DEFAULT_PACKAGE_NAME = "RapidBoxForest-public"
URL_RE = re.compile(r"^https://[^ \t\r\n]+$")
DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
PLACEHOLDER_RE = re.compile(r"(TODO|CHANGE_ME|<owner>|example\.com|example-owner)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Create a checked public RapidBoxForest source release archive.")
    parser.add_argument("--out-dir", type=Path, required=True, help="Directory that will receive the archive and package manifest.")
    parser.add_argument("--tree-dir", type=Path, default=None, help="Optional destination for the exported public source tree.")
    parser.add_argument("--package-name", default=DEFAULT_PACKAGE_NAME, help="Top-level directory name inside the archive.")
    parser.add_argument("--repo-url", default=None, help="Optional public repository HTTPS URL to write into exported citation metadata.")
    parser.add_argument("--doi", default=None, help="Optional software or release DOI to write into exported citation metadata.")
    parser.add_argument("--version", default=None, help="Optional version string to write into exported citation metadata.")
    parser.add_argument("--release-date", default=None, help="Optional YYYY-MM-DD release date to write into exported citation metadata.")
    parser.add_argument(
        "--cache-manifest",
        type=Path,
        default=None,
        help="Optional filled external cache artifact manifest to validate and record in the package manifest.",
    )
    parser.add_argument(
        "--cache-archive-dir",
        type=Path,
        default=None,
        help="Optional directory containing downloaded cache archives to verify with --cache-manifest.",
    )
    parser.add_argument("--force", action="store_true", help="Replace existing tree/archive outputs.")
    parser.add_argument("--skip-checks", action="store_true", help="Create the archive without running release checkers.")
    parser.add_argument("--check-paper-compile", action="store_true", help="Also compile the paper from the exported tree before packaging.")
    parser.add_argument(
        "--strict-metadata",
        action="store_true",
        help="Require final package metadata: repo URL, version, and release date.",
    )
    return parser.parse_args()


def run(command: list[str], *, cwd: Path) -> None:
    try:
        subprocess.run(command, cwd=str(cwd), check=True)
    except subprocess.CalledProcessError as exc:
        rendered = " ".join(shlex.quote(str(part)) for part in command)
        raise SystemExit(f"command failed with exit code {exc.returncode}: {rendered}") from None


def validate_release_metadata(args: argparse.Namespace) -> None:
    if args.cache_archive_dir is not None and args.cache_manifest is None:
        raise SystemExit("--cache-archive-dir requires --cache-manifest")
    if args.strict_metadata:
        missing = [
            flag
            for flag, value in (
                ("--repo-url", args.repo_url),
                ("--version", args.version),
                ("--release-date", args.release_date),
            )
            if value in (None, "")
        ]
        if missing:
            raise SystemExit(f"--strict-metadata requires: {', '.join(missing)}")
    if args.repo_url is not None:
        if not URL_RE.fullmatch(args.repo_url):
            raise SystemExit("--repo-url must be an HTTPS URL")
        if PLACEHOLDER_RE.search(args.repo_url):
            raise SystemExit("--repo-url must be a real public URL, not a placeholder")
    for field in ("doi", "version"):
        value = getattr(args, field)
        if value is not None and PLACEHOLDER_RE.search(value):
            raise SystemExit(f"--{field.replace('_', '-')} must not contain placeholders")
    if args.release_date is not None:
        if PLACEHOLDER_RE.search(args.release_date):
            raise SystemExit("--release-date must not contain placeholders")
        if not DATE_RE.fullmatch(args.release_date):
            raise SystemExit("--release-date must use YYYY-MM-DD")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} root must be an object")
    return data


def release_manifest_path(path: Path | None) -> str | None:
    if path is None:
        return None
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return resolved.name


def refresh_public_manifest(out_dir: Path) -> None:
    manifest_path = out_dir / "PUBLIC_RELEASE_MANIFEST.json"
    existing: dict[str, object] = {}
    if manifest_path.exists():
        existing = load_json(manifest_path)
    files = sorted(
        str(path.relative_to(out_dir))
        for path in out_dir.rglob("*")
        if path.is_file()
        and path.name != "PUBLIC_RELEASE_MANIFEST.json"
        and ".git" not in path.relative_to(out_dir).parts
    )
    manifest = {
        "source_repo": existing.get("source_repo", "RapidBoxForest public release export"),
        "file_count": len(files),
        "policy": existing.get(
            "policy",
            "allowlist export; generated outputs, local caches, build trees, and historical archives excluded by default",
        ),
        "files": files,
        "file_sha256": {rel: sha256_file(out_dir / rel) for rel in files},
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def reset_path(path: Path, *, force: bool, directory: bool) -> None:
    if not path.exists():
        return
    if not force:
        raise SystemExit(f"output exists: {path} (pass --force to replace it)")
    if path.is_dir():
        shutil.rmtree(path)
    else:
        path.unlink()
    if directory:
        path.mkdir(parents=True, exist_ok=True)


def normalized_tarinfo(tarinfo: tarfile.TarInfo) -> tarfile.TarInfo:
    tarinfo.uid = 0
    tarinfo.gid = 0
    tarinfo.uname = ""
    tarinfo.gname = ""
    tarinfo.mtime = 0
    if tarinfo.isdir():
        tarinfo.mode = 0o755
    elif tarinfo.isfile():
        tarinfo.mode = 0o644
    return tarinfo


def create_deterministic_tar_gz(source_tree: Path, archive_path: Path, *, package_name: str) -> None:
    with archive_path.open("wb") as raw_stream:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw_stream, mtime=0) as gzip_stream:
            with tarfile.open(fileobj=gzip_stream, mode="w") as tar:
                root_info = tarfile.TarInfo(package_name)
                root_info.type = tarfile.DIRTYPE
                tar.addfile(normalized_tarinfo(root_info))
                for path in sorted(source_tree.rglob("*")):
                    rel = path.relative_to(source_tree)
                    arcname = (Path(package_name) / rel).as_posix()
                    info = tar.gettarinfo(str(path), arcname=arcname)
                    info = normalized_tarinfo(info)
                    if path.is_dir():
                        tar.addfile(info)
                    elif path.is_file():
                        with path.open("rb") as stream:
                            tar.addfile(info, stream)


def tar_archive_stats(archive_path: Path) -> dict[str, int]:
    with tarfile.open(archive_path, "r:gz") as tar:
        members = tar.getmembers()
    names = [member.name for member in members]
    duplicate_member_count = sum(1 for count in Counter(names).values() if count > 1)
    return {
        "tar_member_count": len(members),
        "duplicate_member_count": duplicate_member_count,
    }


def main() -> int:
    args = parse_args()
    validate_release_metadata(args)
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    tree_dir = args.tree_dir.resolve() if args.tree_dir else out_dir / args.package_name
    archive_path = out_dir / f"{args.package_name}.tar.gz"
    manifest_path = out_dir / f"{args.package_name}.package.json"

    reset_path(tree_dir, force=args.force, directory=False)
    reset_path(archive_path, force=args.force, directory=False)
    reset_path(manifest_path, force=args.force, directory=False)

    run(
        [
            sys.executable,
            "scripts/export_public_release.py",
            "--out-dir",
            str(tree_dir),
            "--force",
        ],
        cwd=REPO_ROOT,
    )
    if args.repo_url:
        command = [
            sys.executable,
            str(tree_dir / "scripts/set_public_release_urls.py"),
            "--repo-root",
            str(tree_dir),
            "--repo-url",
            args.repo_url,
        ]
        if args.doi:
            command.extend(["--doi", args.doi])
        if args.version:
            command.extend(["--version", args.version])
        if args.release_date:
            command.extend(["--release-date", args.release_date])
        run(command, cwd=REPO_ROOT)
        refresh_public_manifest(tree_dir)

    if not args.skip_checks:
        check_command = [
            sys.executable,
            "scripts/check_public_release.py",
            str(tree_dir),
            "--check-release-tools",
            "--run-smoke-dry-run",
        ]
        if args.repo_url:
            check_command.append("--strict-citation")
        run(check_command, cwd=REPO_ROOT)
        if args.check_paper_compile:
            paper_check_command = [
                sys.executable,
                "scripts/check_public_release.py",
                str(tree_dir),
                "--check-paper-compile",
            ]
            if args.repo_url:
                paper_check_command.append("--strict-citation")
            run(paper_check_command, cwd=REPO_ROOT)
        release_readiness_command = [
            sys.executable,
            "scripts/check_release_readiness.py",
            "--repo-root",
            ".",
            "--public-tree",
            str(tree_dir),
        ]
        if args.cache_manifest:
            release_readiness_command.extend(["--cache-manifest", str(args.cache_manifest)])
        if args.cache_archive_dir:
            release_readiness_command.extend(["--cache-archive-dir", str(args.cache_archive_dir)])
        run(release_readiness_command, cwd=REPO_ROOT)

    create_deterministic_tar_gz(tree_dir, archive_path, package_name=args.package_name)
    tar_stats = tar_archive_stats(archive_path)
    public_manifest = load_json(tree_dir / "PUBLIC_RELEASE_MANIFEST.json")
    cache_manifest_path = args.cache_manifest.resolve() if args.cache_manifest else None
    package_manifest = {
        "schema_version": 2,
        "package_name": args.package_name,
        "archive": archive_path.name,
        "archive_sha256": sha256_file(archive_path),
        "tar_member_count": tar_stats["tar_member_count"],
        "duplicate_member_count": tar_stats["duplicate_member_count"],
        "source_file_count": public_manifest.get("file_count"),
        "tree_file_count_including_manifest": len([path for path in tree_dir.rglob("*") if path.is_file()]),
        "public_release_manifest_sha256": sha256_file(tree_dir / "PUBLIC_RELEASE_MANIFEST.json"),
        "checks": "skipped" if args.skip_checks else "passed",
        "release_tools_checked": bool(not args.skip_checks),
        "paper_compile_checked": bool(args.check_paper_compile and not args.skip_checks),
        "repo_url": args.repo_url,
        "doi": args.doi,
        "version": args.version,
        "release_date": args.release_date,
        "cache_manifest": release_manifest_path(args.cache_manifest),
        "cache_manifest_sha256": sha256_file(cache_manifest_path) if cache_manifest_path else None,
        "cache_archives_checked": bool(args.cache_manifest and args.cache_archive_dir and not args.skip_checks),
    }
    manifest_path.write_text(json.dumps(package_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    package_check_command = [
        sys.executable,
        "scripts/check_public_package.py",
        str(manifest_path),
    ]
    if args.cache_manifest:
        package_check_command.extend(["--cache-manifest", str(args.cache_manifest)])
    if args.cache_archive_dir:
        package_check_command.append("--require-cache-archives-checked")
    if not args.skip_checks:
        package_check_command.append("--require-release-tools-checked")
    if args.skip_checks:
        package_check_command.append("--allow-skipped-checks")
    if args.strict_metadata:
        package_check_command.append("--strict-metadata")
    run(package_check_command, cwd=REPO_ROOT)
    print(f"wrote {archive_path}")
    print(f"wrote {manifest_path}")
    print(f"archive_sha256 {package_manifest['archive_sha256']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
