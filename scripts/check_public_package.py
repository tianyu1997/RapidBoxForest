#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import tarfile
from collections import Counter
from pathlib import Path
from typing import Any

sys.dont_write_bytecode = True


HEX_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
URL_RE = re.compile(r"^https://[^ \t\r\n]+$")
DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
PLACEHOLDER_RE = re.compile(r"(TODO|CHANGE_ME|<owner>|example\.com|example-owner)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate a RapidBoxForest public source package manifest and archive.")
    parser.add_argument("package_manifest", type=Path, help="RapidBoxForest-public.package.json.")
    parser.add_argument(
        "--cache-manifest",
        type=Path,
        default=None,
        help="Optional filled cache artifact manifest path. When provided, its SHA256 must match the package manifest.",
    )
    parser.add_argument(
        "--allow-skipped-checks",
        action="store_true",
        help="Allow package manifests with checks='skipped'. Intended only for diagnostic archives.",
    )
    parser.add_argument(
        "--strict-metadata",
        action="store_true",
        help="Require final release metadata: repo_url, version, and release_date.",
    )
    parser.add_argument(
        "--require-cache-archives-checked",
        action="store_true",
        help="Require cache_archives_checked=true in the package manifest.",
    )
    parser.add_argument(
        "--require-release-tools-checked",
        action="store_true",
        help="Require release_tools_checked=true in the package manifest.",
    )
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} root must be a JSON object")
    return data


def plain_file_name(value: Any, *, field: str) -> str | None:
    if not isinstance(value, str) or not value:
        return f"{field} must be a non-empty string"
    path = Path(value)
    if path.name != value or path.is_absolute() or any(part == ".." for part in path.parts):
        return f"{field} must be a plain filename, got: {value}"
    return None


def validate_optional_url(value: Any, *, field: str) -> list[str]:
    if value is None:
        return []
    if not isinstance(value, str) or not URL_RE.fullmatch(value):
        return [f"{field} must be an HTTPS URL"]
    if PLACEHOLDER_RE.search(value):
        return [f"{field} contains a placeholder"]
    return []


def validate_optional_text(value: Any, *, field: str) -> list[str]:
    if value is None:
        return []
    if not isinstance(value, str) or not value:
        return [f"{field} must be a non-empty string when provided"]
    if PLACEHOLDER_RE.search(value):
        return [f"{field} contains a placeholder"]
    return []


def validate_optional_date(value: Any, *, field: str) -> list[str]:
    errors = validate_optional_text(value, field=field)
    if errors or value is None:
        return errors
    if not DATE_RE.fullmatch(value):
        return [f"{field} must use YYYY-MM-DD"]
    return []


def validate_cache_manifest_reference(package: dict[str, Any], package_dir: Path, cache_manifest_arg: Path | None) -> list[str]:
    errors: list[str] = []
    cache_manifest = package.get("cache_manifest")
    cache_sha = package.get("cache_manifest_sha256")
    if cache_manifest is None and cache_sha is None:
        return errors
    if cache_manifest is None or cache_sha is None:
        return ["cache_manifest and cache_manifest_sha256 must be set together"]
    name_error = plain_file_name(cache_manifest, field="cache_manifest")
    if name_error:
        errors.append(name_error)
    if not isinstance(cache_sha, str) or not HEX_SHA256_RE.fullmatch(cache_sha):
        errors.append("cache_manifest_sha256 must be 64 lowercase hex characters")
    candidate = cache_manifest_arg
    if candidate is None and isinstance(cache_manifest, str):
        sibling = package_dir / cache_manifest
        if sibling.exists():
            candidate = sibling
    if candidate is not None:
        if not candidate.exists():
            errors.append(f"cache manifest does not exist: {candidate}")
        elif isinstance(cache_sha, str) and HEX_SHA256_RE.fullmatch(cache_sha):
            actual = sha256_file(candidate)
            if actual != cache_sha:
                errors.append(f"cache_manifest_sha256 mismatch: expected {cache_sha}, got {actual}")
    return errors


def validate_tar_archive(package: dict[str, Any], archive_path: Path) -> list[str]:
    errors: list[str] = []
    package_name = package.get("package_name")
    if not isinstance(package_name, str) or not package_name:
        return ["package_name must be a non-empty string"]
    try:
        with tarfile.open(archive_path, "r:gz") as tar:
            members = tar.getmembers()
            names = [member.name for member in members]
            duplicate_names = [name for name, count in Counter(names).items() if count > 1]
            if duplicate_names:
                errors.append(f"archive has duplicate members: {duplicate_names[:20]}")
            expected_tar_member_count = package.get("tar_member_count")
            if not isinstance(expected_tar_member_count, int):
                errors.append("tar_member_count must be an integer")
            elif expected_tar_member_count != len(members):
                errors.append(
                    f"tar_member_count mismatch: expected {expected_tar_member_count}, got {len(members)}"
                )
            expected_duplicate_member_count = package.get("duplicate_member_count")
            if not isinstance(expected_duplicate_member_count, int):
                errors.append("duplicate_member_count must be an integer")
            elif expected_duplicate_member_count != len(duplicate_names):
                errors.append(
                    "duplicate_member_count mismatch: "
                    f"expected {expected_duplicate_member_count}, got {len(duplicate_names)}"
                )
            elif expected_duplicate_member_count != 0:
                errors.append("duplicate_member_count must be 0")
            top_levels = {Path(name).parts[0] for name in names if Path(name).parts}
            if top_levels != {package_name}:
                errors.append(f"archive top-level entries must be only {package_name!r}, got {sorted(top_levels)}")
            file_members = [member for member in members if member.isfile()]
            expected_tree_files = package.get("tree_file_count_including_manifest")
            if isinstance(expected_tree_files, int) and len(file_members) != expected_tree_files:
                errors.append(
                    f"tree_file_count_including_manifest mismatch: expected {expected_tree_files}, got {len(file_members)}"
                )
            public_manifest_member = f"{package_name}/PUBLIC_RELEASE_MANIFEST.json"
            try:
                public_manifest_file = tar.extractfile(public_manifest_member)
            except KeyError:
                public_manifest_file = None
            if public_manifest_file is None:
                errors.append("archive is missing PUBLIC_RELEASE_MANIFEST.json")
            else:
                public_manifest_bytes = public_manifest_file.read()
                expected_manifest_sha = package.get("public_release_manifest_sha256")
                if not isinstance(expected_manifest_sha, str) or not HEX_SHA256_RE.fullmatch(expected_manifest_sha):
                    errors.append("public_release_manifest_sha256 must be 64 lowercase hex characters")
                else:
                    actual_manifest_sha = hashlib.sha256(public_manifest_bytes).hexdigest()
                    if actual_manifest_sha != expected_manifest_sha:
                        errors.append(
                            "public_release_manifest_sha256 mismatch: "
                            f"expected {expected_manifest_sha}, got {actual_manifest_sha}"
                        )
                try:
                    public_manifest = json.loads(public_manifest_bytes.decode("utf-8"))
                except Exception as exc:
                    errors.append(f"cannot parse archive PUBLIC_RELEASE_MANIFEST.json: {exc}")
                    public_manifest = {}
                if isinstance(public_manifest, dict):
                    source_count = package.get("source_file_count")
                    manifest_count = public_manifest.get("file_count")
                    if isinstance(source_count, int) and manifest_count != source_count:
                        errors.append(f"source_file_count mismatch: package {source_count}, public manifest {manifest_count}")
                    if isinstance(manifest_count, int) and isinstance(expected_tree_files, int):
                        if expected_tree_files != manifest_count + 1:
                            errors.append(
                                "tree_file_count_including_manifest must equal public manifest file_count + 1"
                            )
    except tarfile.TarError as exc:
        errors.append(f"cannot read archive as tar.gz: {exc}")
    return errors


def validate_package(
    package_manifest: Path,
    *,
    cache_manifest: Path | None,
    allow_skipped_checks: bool,
    strict_metadata: bool,
    require_cache_archives_checked: bool,
    require_release_tools_checked: bool,
) -> list[str]:
    errors: list[str] = []
    package = load_json(package_manifest)
    if package.get("schema_version") != 2:
        errors.append("source package schema_version must be 2")
    archive_name = package.get("archive")
    archive_name_error = plain_file_name(archive_name, field="archive")
    if archive_name_error:
        errors.append(archive_name_error)
        archive_path = package_manifest.parent / "missing-archive"
    else:
        archive_path = package_manifest.parent / str(archive_name)
    if not archive_path.exists():
        errors.append(f"archive does not exist: {archive_path}")
    else:
        archive_sha = package.get("archive_sha256")
        if not isinstance(archive_sha, str) or not HEX_SHA256_RE.fullmatch(archive_sha):
            errors.append("archive_sha256 must be 64 lowercase hex characters")
        else:
            actual = sha256_file(archive_path)
            if actual != archive_sha:
                errors.append(f"archive_sha256 mismatch: expected {archive_sha}, got {actual}")
        errors.extend(validate_tar_archive(package, archive_path))
    checks = package.get("checks")
    if checks != "passed" and not allow_skipped_checks:
        errors.append("checks must be 'passed' unless --allow-skipped-checks is set")
    release_tools_checked = package.get("release_tools_checked")
    if not isinstance(release_tools_checked, bool):
        errors.append("release_tools_checked must be boolean")
    if require_release_tools_checked and release_tools_checked is not True:
        errors.append("release_tools_checked must be true when --require-release-tools-checked is set")
    if not isinstance(package.get("paper_compile_checked"), bool):
        errors.append("paper_compile_checked must be boolean")
    if strict_metadata:
        for field in ("repo_url", "version", "release_date"):
            if package.get(field) in (None, ""):
                errors.append(f"{field} is required in --strict-metadata mode")
    errors.extend(validate_optional_url(package.get("repo_url"), field="repo_url"))
    for field in ("doi", "version"):
        errors.extend(validate_optional_text(package.get(field), field=field))
    errors.extend(validate_optional_date(package.get("release_date"), field="release_date"))
    errors.extend(validate_cache_manifest_reference(package, package_manifest.parent, cache_manifest))
    cache_archives_checked = package.get("cache_archives_checked")
    if not isinstance(cache_archives_checked, bool):
        errors.append("cache_archives_checked must be boolean")
    if cache_archives_checked is True and package.get("cache_manifest") is None:
        errors.append("cache_archives_checked=true requires cache_manifest")
    if require_cache_archives_checked and cache_archives_checked is not True:
        errors.append("cache_archives_checked must be true when --require-cache-archives-checked is set")
    return errors


def main() -> int:
    args = parse_args()
    package_manifest = args.package_manifest.resolve()
    try:
        errors = validate_package(
            package_manifest,
            cache_manifest=args.cache_manifest.resolve() if args.cache_manifest else None,
            allow_skipped_checks=bool(args.allow_skipped_checks),
            strict_metadata=bool(args.strict_metadata),
            require_cache_archives_checked=bool(args.require_cache_archives_checked),
            require_release_tools_checked=bool(args.require_release_tools_checked),
        )
    except Exception as exc:
        print(f"FAIL: cannot validate package manifest: {exc}", file=sys.stderr)
        return 1
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print(f"public package check passed: {package_manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
