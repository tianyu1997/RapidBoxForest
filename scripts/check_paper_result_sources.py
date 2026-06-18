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


HEX_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SOURCE_PATH_PREFIXES = ("outputs/", "paper/generated/", "paper/figures/")
HASH_SENTINELS = {"skipped_large_file"}
ABSOLUTE_PATH_PREFIXES = (
    "/" + "home/",
    "/" + "mnt/",
    "C:" + "\\",
    "Us" + "ers/",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate the paper table/figure provenance manifest. By default, "
            "this checks schema, paper assets, relative paths, and hash syntax "
            "without requiring large outputs/ to be present in a public source tree."
        )
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("paper/generated/tro_table_generation_manifest.json"),
        help="Paper result provenance manifest.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path("."),
        help="Repository root used to resolve repo-relative artifact paths.",
    )
    parser.add_argument(
        "--verify-local",
        action="store_true",
        help="Verify SHA256 hashes for local source files that are present.",
    )
    parser.add_argument(
        "--require-local",
        action="store_true",
        help="Require every referenced source file to exist locally. Use this for a complete artifact bundle, not for a source-only public export.",
    )
    parser.add_argument(
        "--require-source-hashes",
        action="store_true",
        help="Require every referenced outputs/ source file to have an adjacent SHA256 field.",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("manifest root must be a JSON object")
    return data


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def is_source_path(value: str) -> bool:
    return value.startswith(SOURCE_PATH_PREFIXES)


def is_valid_sha_value(value: Any) -> bool:
    if isinstance(value, str):
        return bool(HEX_SHA256_RE.fullmatch(value)) or value in HASH_SENTINELS
    if isinstance(value, dict):
        return all(is_valid_sha_value(item) for item in value.values())
    if isinstance(value, list):
        return all(is_valid_sha_value(item) for item in value)
    return False


def collect_hash_field_errors(value: Any, path: str = "$") -> list[str]:
    errors: list[str] = []
    if isinstance(value, dict):
        for key, item in value.items():
            child = f"{path}.{key}"
            if "sha256" in str(key).lower() and not is_valid_sha_value(item):
                errors.append(f"{child} is not a valid SHA256 value/container")
            errors.extend(collect_hash_field_errors(item, child))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            errors.extend(collect_hash_field_errors(item, f"{path}[{index}]"))
    return errors


def collect_absolute_path_errors(value: Any, path: str = "$") -> list[str]:
    errors: list[str] = []
    if isinstance(value, dict):
        for key, item in value.items():
            errors.extend(collect_absolute_path_errors(item, f"{path}.{key}"))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            errors.extend(collect_absolute_path_errors(item, f"{path}[{index}]"))
    elif isinstance(value, str):
        if value.startswith(ABSOLUTE_PATH_PREFIXES):
            errors.append(f"{path} contains a local absolute path")
    return errors


def candidate_hash_for_path(record: dict[str, Any], key: str) -> str | None:
    candidates = [
        f"{key}_sha256",
        f"{key}_hash",
    ]
    if key.endswith("_path"):
        candidates.append(f"{key[:-5]}_sha256")
    if key == "path":
        candidates.append("sha256")
    for candidate in candidates:
        value = record.get(candidate)
        if isinstance(value, str) and HEX_SHA256_RE.fullmatch(value):
            return value
    return None


def iter_source_refs(value: Any, path: str = "$") -> list[tuple[str, str, str | None]]:
    refs: list[tuple[str, str, str | None]] = []
    if isinstance(value, dict):
        for key, item in value.items():
            child = f"{path}.{key}"
            if isinstance(item, str) and is_source_path(item):
                refs.append((child, item, candidate_hash_for_path(value, str(key))))
            refs.extend(iter_source_refs(item, child))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            refs.extend(iter_source_refs(item, f"{path}[{index}]"))
    return refs


def check_assets(manifest: dict[str, Any], repo_root: Path, *, verify_local: bool) -> list[str]:
    errors: list[str] = []
    assets = manifest.get("assets")
    if not isinstance(assets, dict) or not assets:
        return ["manifest assets must be a non-empty object"]
    for name, entry in assets.items():
        if not isinstance(entry, dict):
            errors.append(f"asset {name} is not an object")
            continue
        path_value = entry.get("path")
        if not isinstance(path_value, str) or not path_value.startswith("paper/generated/"):
            errors.append(f"asset {name} must have a paper/generated path")
            continue
        if entry.get("placeholder") is not False:
            errors.append(f"asset {name} is marked as placeholder")
        expected = entry.get("sha256")
        if not isinstance(expected, str) or not HEX_SHA256_RE.fullmatch(expected):
            errors.append(f"asset {name} is missing a valid sha256")
            continue
        local_path = repo_root / path_value
        if not local_path.exists():
            errors.append(f"asset {name} is missing locally: {path_value}")
            continue
        if verify_local and sha256_file(local_path) != expected:
            errors.append(f"asset {name} sha256 mismatch: {path_value}")
    return errors


def check_sources(
    manifest: dict[str, Any],
    repo_root: Path,
    *,
    verify_local: bool,
    require_local: bool,
    require_source_hashes: bool,
) -> list[str]:
    errors: list[str] = []
    sources = manifest.get("sources")
    if not isinstance(sources, dict) or not sources:
        return ["manifest sources must be a non-empty object"]
    for source_name, record in sources.items():
        if not isinstance(record, dict):
            errors.append(f"source {source_name} is not an object")
            continue
        if record.get("status") in (None, "placeholder"):
            errors.append(f"source {source_name} has invalid status: {record.get('status')!r}")
    refs = iter_source_refs(sources, "$.sources")
    if not refs:
        errors.append("manifest sources do not reference any output or paper paths")
    for location, rel_path, expected_hash in refs:
        if ".." in Path(rel_path).parts:
            errors.append(f"{location} contains parent traversal: {rel_path}")
            continue
        local_path = repo_root / rel_path
        if require_source_hashes and rel_path.startswith("outputs/") and expected_hash is None:
            errors.append(f"{location} references {rel_path} without an adjacent sha256 field")
        if require_local and not local_path.exists():
            errors.append(f"{location} missing local source artifact: {rel_path}")
            continue
        if verify_local and local_path.exists() and expected_hash is not None:
            actual = sha256_file(local_path)
            if actual != expected_hash:
                errors.append(f"{location} sha256 mismatch: {rel_path}")
    return errors


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    manifest_path = args.manifest
    if not manifest_path.is_absolute():
        manifest_path = repo_root / manifest_path
    errors: list[str] = []
    try:
        manifest = load_json(manifest_path)
    except Exception as exc:
        print(f"FAIL: cannot load manifest: {exc}", file=sys.stderr)
        return 1
    if manifest.get("artifact") != "tro2026_paper_assets":
        errors.append("manifest artifact must be tro2026_paper_assets")
    if manifest.get("placeholder_mode") is not False:
        errors.append("manifest placeholder_mode must be false")
    source_root = manifest.get("source_root")
    if not isinstance(source_root, str) or not source_root.startswith("outputs/"):
        errors.append("manifest source_root must be a repo-relative outputs/ path")
    errors.extend(collect_hash_field_errors(manifest))
    errors.extend(collect_absolute_path_errors(manifest))
    errors.extend(check_assets(manifest, repo_root, verify_local=True))
    errors.extend(
        check_sources(
            manifest,
            repo_root,
            verify_local=args.verify_local,
            require_local=args.require_local,
            require_source_hashes=args.require_source_hashes,
        )
    )
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print(f"paper result source manifest check passed: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
