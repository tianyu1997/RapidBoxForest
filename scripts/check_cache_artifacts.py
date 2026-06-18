#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any

sys.dont_write_bytecode = True


REPO_ROOT = Path(__file__).resolve().parents[1]
HEX_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
URL_RE = re.compile(r"^https://[^ \t\r\n]+$")
PLACEHOLDER_TOKENS = ("TODO", "CHANGE_ME", "<owner>", "example.com", "example-owner")
SUPPORTED_ARCHIVE_SUFFIXES = (".tar.gz", ".tgz", ".tar.zst")
REQUIRED_ARTIFACT_FIELDS = {
    "id",
    "role",
    "required_for",
    "robot",
    "endpoint_source",
    "envelope",
    "depth",
    "depth_semantics",
    "split_schedule",
    "canonical_mode",
    "coverage_domain",
    "expected_unpack_path",
    "build_command",
    "archive",
    "unpacked",
}
REQUIRED_ARCHIVE_FIELDS = {"file_name", "url", "sha256", "size_bytes"}
REQUIRED_UNPACKED_FIELDS = {"manifest_relative_path", "snapshot_relative_path", "directory_sha256"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate optional local LECT cache artifact metadata.")
    parser.add_argument(
        "manifest",
        type=Path,
        nargs="?",
        default=REPO_ROOT / "docs" / "cache_artifacts.example.json",
        help="Cache artifact manifest JSON. Defaults to docs/cache_artifacts.example.json.",
    )
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT, help="Repository root for local path checks.")
    parser.add_argument(
        "--allow-placeholders",
        action="store_true",
        help="Allow TODO placeholder archive URLs and checksums. Use only for the example template.",
    )
    parser.add_argument(
        "--verify-local",
        action="store_true",
        help="Verify each expected unpacked cache directory exists and contains the declared manifest/snapshot paths.",
    )
    parser.add_argument(
        "--archive-dir",
        type=Path,
        default=None,
        help="Optional directory containing downloaded cache archives to verify against archive.file_name, size_bytes, and sha256.",
    )
    parser.add_argument(
        "--write-directory-sha256",
        type=Path,
        default=None,
        help="Write computed directory SHA256 values for local cache directories to this JSON file.",
    )
    return parser.parse_args()


def has_placeholder(value: Any) -> bool:
    if isinstance(value, str):
        return any(token in value for token in PLACEHOLDER_TOKENS)
    if isinstance(value, list):
        return any(has_placeholder(item) for item in value)
    if isinstance(value, dict):
        return any(has_placeholder(item) for item in value.values())
    return False


def safe_relative_path(value: Any, *, field: str) -> str | None:
    if not isinstance(value, str) or not value:
        return f"{field} must be a non-empty string"
    path = Path(value)
    if path.is_absolute():
        return f"{field} must be repository-relative, got absolute path: {value}"
    if any(part == ".." for part in path.parts):
        return f"{field} must not contain '..': {value}"
    return None


def safe_archive_file_name(value: Any, *, field: str) -> str | None:
    if not isinstance(value, str) or not value:
        return f"{field} must be a non-empty string"
    if has_placeholder(value):
        return None
    path = Path(value)
    if path.name != value or path.is_absolute() or any(part == ".." for part in path.parts):
        return f"{field} must be a plain archive filename, got: {value}"
    if not value.endswith(SUPPORTED_ARCHIVE_SUFFIXES):
        suffixes = ", ".join(SUPPORTED_ARCHIVE_SUFFIXES)
        return f"{field} must end with one of: {suffixes}"
    return None


def stable_directory_sha256(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        rel = path.relative_to(root).as_posix()
        digest.update(rel.encode("utf-8"))
        digest.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        digest.update(b"\0")
    return digest.hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_artifact(
    index: int,
    artifact: Any,
    *,
    allow_placeholders: bool,
    repo_root: Path,
    verify_local: bool,
    archive_dir: Path | None,
) -> tuple[list[str], dict[str, str]]:
    errors: list[str] = []
    computed: dict[str, str] = {}
    if not isinstance(artifact, dict):
        return [f"artifacts[{index}] must be an object"], computed
    missing = sorted(REQUIRED_ARTIFACT_FIELDS - set(artifact))
    if missing:
        errors.append(f"{artifact.get('id', f'artifacts[{index}]')}: missing fields {missing}")
    artifact_id = artifact.get("id", f"artifacts[{index}]")
    if not isinstance(artifact.get("id"), str) or not artifact.get("id"):
        errors.append(f"artifacts[{index}]: id must be a non-empty string")
    if not isinstance(artifact.get("required_for"), list) or not artifact.get("required_for"):
        errors.append(f"{artifact_id}: required_for must be a non-empty list")
    if not isinstance(artifact.get("depth"), int) or int(artifact.get("depth", 0)) <= 0:
        errors.append(f"{artifact_id}: depth must be a positive integer")
    if not isinstance(artifact.get("canonical_mode"), bool):
        errors.append(f"{artifact_id}: canonical_mode must be boolean")
    if not isinstance(artifact.get("build_command"), list) or not all(isinstance(item, str) and item for item in artifact.get("build_command", [])):
        errors.append(f"{artifact_id}: build_command must be a non-empty list of strings")
    path_error = safe_relative_path(artifact.get("expected_unpack_path"), field=f"{artifact_id}.expected_unpack_path")
    if path_error:
        errors.append(path_error)
    elif not str(artifact["expected_unpack_path"]).startswith("outputs/"):
        errors.append(f"{artifact_id}: expected_unpack_path should live under outputs/")

    archive = artifact.get("archive")
    if not isinstance(archive, dict):
        errors.append(f"{artifact_id}: archive must be an object")
        archive = {}
    missing_archive = sorted(REQUIRED_ARCHIVE_FIELDS - set(archive))
    if missing_archive:
        errors.append(f"{artifact_id}: archive missing fields {missing_archive}")
    if "file_name" in archive:
        name_error = safe_archive_file_name(archive.get("file_name"), field=f"{artifact_id}.archive.file_name")
        if name_error:
            errors.append(name_error)
    archive_url = archive.get("url")
    if isinstance(archive_url, str) and not has_placeholder(archive_url) and not URL_RE.fullmatch(archive_url):
        errors.append(f"{artifact_id}: archive.url must be an HTTPS URL")
    archive_sha = archive.get("sha256")
    if isinstance(archive_sha, str) and not has_placeholder(archive_sha) and not HEX_SHA256_RE.fullmatch(archive_sha):
        errors.append(f"{artifact_id}: archive.sha256 must be 64 lowercase hex characters")
    size_bytes = archive.get("size_bytes")
    if not has_placeholder(size_bytes) and (not isinstance(size_bytes, int) or size_bytes <= 0):
        errors.append(f"{artifact_id}: archive.size_bytes must be a positive integer")
    if archive_dir is not None:
        file_name = archive.get("file_name")
        if not isinstance(file_name, str) or has_placeholder(file_name):
            errors.append(f"{artifact_id}: archive.file_name must be concrete when --archive-dir is used")
        else:
            archive_path = archive_dir / file_name
            if not archive_path.is_file():
                errors.append(f"{artifact_id}: missing archive file: {archive_path}")
            else:
                if isinstance(size_bytes, int) and size_bytes > 0 and archive_path.stat().st_size != size_bytes:
                    errors.append(
                        f"{artifact_id}: archive.size_bytes mismatch: "
                        f"manifest={size_bytes} actual={archive_path.stat().st_size}"
                    )
                if isinstance(archive_sha, str) and not has_placeholder(archive_sha) and HEX_SHA256_RE.fullmatch(archive_sha):
                    actual_archive_sha = sha256_file(archive_path)
                    if archive_sha != actual_archive_sha:
                        errors.append(
                            f"{artifact_id}: archive.sha256 mismatch: "
                            f"manifest={archive_sha} actual={actual_archive_sha}"
                        )

    unpacked = artifact.get("unpacked")
    if not isinstance(unpacked, dict):
        errors.append(f"{artifact_id}: unpacked must be an object")
        unpacked = {}
    missing_unpacked = sorted(REQUIRED_UNPACKED_FIELDS - set(unpacked))
    if missing_unpacked:
        errors.append(f"{artifact_id}: unpacked missing fields {missing_unpacked}")
    for key in ("manifest_relative_path", "snapshot_relative_path"):
        if key in unpacked:
            path_error = safe_relative_path(unpacked[key], field=f"{artifact_id}.unpacked.{key}")
            if path_error:
                errors.append(path_error)
    dir_sha = unpacked.get("directory_sha256")
    if isinstance(dir_sha, str) and not has_placeholder(dir_sha) and not HEX_SHA256_RE.fullmatch(dir_sha):
        errors.append(f"{artifact_id}: unpacked.directory_sha256 must be 64 lowercase hex characters")

    if has_placeholder(artifact) and not allow_placeholders:
        errors.append(f"{artifact_id}: contains TODO placeholders; publish a filled manifest or pass --allow-placeholders for the template")

    if verify_local and isinstance(artifact.get("expected_unpack_path"), str):
        cache_dir = repo_root / artifact["expected_unpack_path"]
        if not cache_dir.is_dir():
            errors.append(f"{artifact_id}: missing local cache directory: {cache_dir}")
        else:
            manifest_rel = unpacked.get("manifest_relative_path", "manifest.json")
            snapshot_rel = unpacked.get("snapshot_relative_path", "lect_snapshot")
            if isinstance(manifest_rel, str) and not (cache_dir / manifest_rel).exists():
                errors.append(f"{artifact_id}: missing unpacked manifest: {cache_dir / manifest_rel}")
            if isinstance(snapshot_rel, str) and not (cache_dir / snapshot_rel).exists():
                errors.append(f"{artifact_id}: missing unpacked snapshot: {cache_dir / snapshot_rel}")
            actual_dir_sha = stable_directory_sha256(cache_dir)
            computed[str(artifact_id)] = actual_dir_sha
            if isinstance(dir_sha, str) and not has_placeholder(dir_sha) and HEX_SHA256_RE.fullmatch(dir_sha):
                if dir_sha != actual_dir_sha:
                    errors.append(
                        f"{artifact_id}: unpacked.directory_sha256 mismatch: "
                        f"manifest={dir_sha} actual={actual_dir_sha}"
                    )

    return errors, computed


def main() -> int:
    args = parse_args()
    manifest_path = args.manifest.resolve()
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"FAIL: cannot parse {manifest_path}: {exc}", file=sys.stderr)
        return 1
    errors: list[str] = []
    if not isinstance(manifest, dict):
        errors.append("manifest root must be an object")
        artifacts: list[Any] = []
    else:
        if int(manifest.get("schema_version", 0)) != 1:
            errors.append("schema_version must be 1")
        artifacts = manifest.get("artifacts", [])
        if not isinstance(artifacts, list) or not artifacts:
            errors.append("artifacts must be a non-empty list")
            artifacts = []
    seen_ids: set[str] = set()
    computed_hashes: dict[str, str] = {}
    for index, artifact in enumerate(artifacts):
        artifact_errors, computed = validate_artifact(
            index,
            artifact,
            allow_placeholders=bool(args.allow_placeholders),
            repo_root=args.repo_root.resolve(),
            verify_local=bool(args.verify_local),
            archive_dir=args.archive_dir.resolve() if args.archive_dir is not None else None,
        )
        errors.extend(artifact_errors)
        artifact_id = artifact.get("id") if isinstance(artifact, dict) else None
        if isinstance(artifact_id, str):
            if artifact_id in seen_ids:
                errors.append(f"duplicate artifact id: {artifact_id}")
            seen_ids.add(artifact_id)
        computed_hashes.update(computed)
    if args.write_directory_sha256 is not None:
        args.write_directory_sha256.parent.mkdir(parents=True, exist_ok=True)
        args.write_directory_sha256.write_text(
            json.dumps(computed_hashes, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print(f"cache artifact metadata check passed: {len(artifacts)} artifacts in {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
