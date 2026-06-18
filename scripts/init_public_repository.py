#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[0]
URL_RE = re.compile(r"^https://[^ \t\r\n]+$")
DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
PLACEHOLDER_RE = re.compile(r"(TODO|CHANGE_ME|<owner>|example\.com|example-owner)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export a clean RapidBoxForest public tree and initialize it as a fresh git repository."
    )
    parser.add_argument("--out-dir", type=Path, required=True, help="Destination directory for the new public git repository.")
    parser.add_argument("--force", action="store_true", help="Replace an existing destination directory.")
    parser.add_argument("--repo-url", default=None, help="Optional public repository HTTPS URL to write into exported citation metadata.")
    parser.add_argument("--doi", default=None, help="Optional software or release DOI to write into exported citation metadata.")
    parser.add_argument("--version", default=None, help="Optional version string to write into exported citation metadata.")
    parser.add_argument("--release-date", default=None, help="Optional YYYY-MM-DD release date to write into exported citation metadata.")
    parser.add_argument("--remote-url", default=None, help="Optional git remote URL to add as origin. Defaults to --repo-url when provided.")
    parser.add_argument("--commit", action="store_true", help="Create the initial commit after staging files.")
    parser.add_argument("--commit-message", default="Initial public RapidBoxForest release", help="Initial commit message.")
    parser.add_argument("--check-paper-compile", action="store_true", help="Compile the paper from the exported tree before git initialization.")
    parser.add_argument("--skip-checks", action="store_true", help="Skip public release checks before git initialization.")
    parser.add_argument(
        "--strict-metadata",
        action="store_true",
        help="Require final public repository metadata: repo URL, version, and release date.",
    )
    return parser.parse_args()


def run(command: list[str], *, cwd: Path) -> None:
    try:
        subprocess.run(command, cwd=str(cwd), check=True)
    except subprocess.CalledProcessError as exc:
        rendered = " ".join(shlex.quote(str(part)) for part in command)
        raise SystemExit(f"command failed with exit code {exc.returncode}: {rendered}") from None


def capture(command: list[str], *, cwd: Path) -> str:
    try:
        result = subprocess.run(command, cwd=str(cwd), check=True, text=True, stdout=subprocess.PIPE)
    except subprocess.CalledProcessError as exc:
        rendered = " ".join(shlex.quote(str(part)) for part in command)
        raise SystemExit(f"command failed with exit code {exc.returncode}: {rendered}") from None
    return result.stdout.strip()


def validate_release_metadata(args: argparse.Namespace) -> None:
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
    metadata_without_repo = [
        flag
        for flag, value in (
            ("--doi", args.doi),
            ("--version", args.version),
            ("--release-date", args.release_date),
        )
        if value not in (None, "")
    ]
    if metadata_without_repo and not args.repo_url:
        raise SystemExit(f"{', '.join(metadata_without_repo)} require --repo-url so citation metadata can be written")
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


def reset_out_dir(path: Path, *, force: bool) -> None:
    if not path.exists():
        return
    if not force:
        raise SystemExit(f"destination exists: {path} (pass --force to replace it)")
    if path.is_dir():
        shutil.rmtree(path)
    else:
        path.unlink()


def add_remote(out_dir: Path, remote_url: str) -> None:
    existing = subprocess.run(["git", "remote"], cwd=str(out_dir), text=True, stdout=subprocess.PIPE, check=True)
    remotes = set(existing.stdout.split())
    if "origin" in remotes:
        run(["git", "remote", "set-url", "origin", remote_url], cwd=out_dir)
    else:
        run(["git", "remote", "add", "origin", remote_url], cwd=out_dir)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def refresh_public_manifest(out_dir: Path) -> None:
    """Refresh hashes after post-export metadata edits.

    export_public_release.py writes PUBLIC_RELEASE_MANIFEST.json before this
    script optionally stamps the final public repository URL into citation
    metadata. Keep the manifest tied to the exact public tree that will be
    committed, otherwise the release checker correctly reports stale hashes.
    """
    manifest_path = out_dir / "PUBLIC_RELEASE_MANIFEST.json"
    existing: dict[str, object] = {}
    if manifest_path.exists():
        existing = json.loads(manifest_path.read_text(encoding="utf-8"))
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
        "file_sha256": {rel: file_sha256(out_dir / rel) for rel in files},
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    validate_release_metadata(args)
    out_dir = args.out_dir.resolve()
    reset_out_dir(out_dir, force=args.force)

    run(
        [
            sys.executable,
            "scripts/export_public_release.py",
            "--out-dir",
            str(out_dir),
            "--force",
        ],
        cwd=REPO_ROOT,
    )
    if args.repo_url:
        command = [
            sys.executable,
            str(out_dir / "scripts/set_public_release_urls.py"),
            "--repo-root",
            str(out_dir),
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
        refresh_public_manifest(out_dir)

    if not args.skip_checks:
        check_command = [
            sys.executable,
            str(out_dir / "scripts/check_public_release.py"),
            str(out_dir),
            "--run-smoke-dry-run",
        ]
        if args.repo_url:
            check_command.append("--strict-citation")
        run(check_command, cwd=REPO_ROOT)
        if args.check_paper_compile:
            paper_check_command = [
                sys.executable,
                str(out_dir / "scripts/check_public_release.py"),
                str(out_dir),
                "--check-paper-compile",
            ]
            if args.repo_url:
                paper_check_command.append("--strict-citation")
            run(paper_check_command, cwd=REPO_ROOT)

    run(["git", "init", "-b", "main"], cwd=out_dir)
    # The exported tree is already allowlisted. Force-add it so source files
    # such as Python __init__.py are not dropped by release .gitignore rules.
    run(["git", "add", "-A", "-f"], cwd=out_dir)
    remote_url = args.remote_url or args.repo_url
    if remote_url:
        add_remote(out_dir, remote_url)
    if args.commit:
        run(["git", "commit", "-m", args.commit_message], cwd=out_dir)
    status = capture(["git", "status", "--short"], cwd=out_dir)
    print(f"initialized public git repository at {out_dir}")
    if remote_url:
        print(f"origin {remote_url}")
    print("staged files are ready for the first public commit" if status else "working tree is clean")
    if status:
        print(status)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
