#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import gzip
import hashlib
import json
import re
import shutil
import sys
import tarfile
from pathlib import Path
from typing import Any

sys.dont_write_bytecode = True

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from check_cache_artifacts import stable_directory_sha256  # noqa: E402


URL_RE = re.compile(r"^https://[^ \t\r\n]+$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Package local LECT cache directories listed in a cache artifact manifest "
            "and write a filled manifest with archive SHA256 and directory SHA256 values."
        )
    )
    parser.add_argument(
        "manifest",
        type=Path,
        nargs="?",
        default=Path("docs/cache_artifacts.example.json"),
        help="Cache artifact manifest template or filled manifest.",
    )
    parser.add_argument("--repo-root", type=Path, default=Path("."), help="Repository root used to resolve expected_unpack_path.")
    parser.add_argument("--out-dir", type=Path, required=True, help="Directory for cache archives and the filled manifest.")
    parser.add_argument("--artifact-id", action="append", default=[], help="Artifact id to package. Repeatable. Defaults to all artifacts.")
    parser.add_argument("--url-base", default=None, help="Optional base URL used to fill archive.url for each generated archive.")
    parser.add_argument("--manifest-name", default="cache_artifacts.json", help="Output filled manifest filename.")
    parser.add_argument(
        "--gzip-compresslevel",
        type=int,
        default=1,
        choices=range(1, 10),
        metavar="1-9",
        help="Deterministic gzip compression level. Level 1 is fastest and is the default for large LECT caches.",
    )
    parser.add_argument("--force", action="store_true", help="Replace existing archives or output manifest.")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path} root must be a JSON object")
    return data


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def add_tree_to_tar(tar: tarfile.TarFile, source_dir: Path, *, arc_prefix: Path) -> None:
    prefix_info = tarfile.TarInfo(arc_prefix.as_posix())
    prefix_info.type = tarfile.DIRTYPE
    tar.addfile(normalized_tarinfo(prefix_info))
    for path in sorted(source_dir.rglob("*")):
        rel = path.relative_to(source_dir)
        arcname = (arc_prefix / rel).as_posix()
        info = normalized_tarinfo(tar.gettarinfo(str(path), arcname=arcname))
        if path.is_dir():
            tar.addfile(info)
        elif path.is_file():
            with path.open("rb") as stream:
                tar.addfile(info, stream)


def create_tar_gz(source_dir: Path, archive_path: Path, *, arc_prefix: Path, compresslevel: int) -> None:
    with archive_path.open("wb") as raw_stream:
        with gzip.GzipFile(
            filename="",
            mode="wb",
            fileobj=raw_stream,
            mtime=0,
            compresslevel=compresslevel,
        ) as gzip_stream:
            with tarfile.open(fileobj=gzip_stream, mode="w") as tar:
                add_tree_to_tar(tar, source_dir, arc_prefix=arc_prefix)


def artifact_archive_name(artifact: dict[str, Any]) -> str:
    artifact_id = str(artifact["id"])
    return f"{artifact_id}.tar.gz"


def selected_artifacts(manifest: dict[str, Any], ids: set[str]) -> list[dict[str, Any]]:
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        raise ValueError("manifest artifacts must be a non-empty list")
    seen = {str(artifact.get("id")) for artifact in artifacts if isinstance(artifact, dict)}
    missing = sorted(ids - seen)
    if missing:
        raise ValueError(f"unknown artifact ids: {missing}")
    selected: list[dict[str, Any]] = []
    for artifact in artifacts:
        if not isinstance(artifact, dict):
            raise ValueError("all artifacts must be objects")
        if not ids or str(artifact.get("id")) in ids:
            selected.append(copy.deepcopy(artifact))
    return selected


def main() -> int:
    args = parse_args()
    if args.url_base and not URL_RE.fullmatch(args.url_base):
        raise SystemExit("--url-base must be an HTTPS URL")
    repo_root = args.repo_root.resolve()
    manifest_path = args.manifest
    if not manifest_path.is_absolute():
        manifest_path = repo_root / manifest_path
    manifest = load_json(manifest_path)
    ids = set(args.artifact_id)
    artifacts = selected_artifacts(manifest, ids)
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    output_manifest_path = out_dir / args.manifest_name
    if output_manifest_path.exists() and not args.force:
        raise SystemExit(f"output manifest exists: {output_manifest_path} (pass --force)")

    packaged: list[dict[str, Any]] = []
    archive_by_cache_dir: dict[Path, dict[str, Any]] = {}
    for artifact in artifacts:
        artifact_id = str(artifact["id"])
        expected_unpack_path = artifact.get("expected_unpack_path")
        if not isinstance(expected_unpack_path, str) or not expected_unpack_path.startswith("outputs/"):
            raise SystemExit(f"{artifact_id}: expected_unpack_path must be a repo-relative outputs/ path")
        cache_dir = repo_root / expected_unpack_path
        if not cache_dir.is_dir():
            raise SystemExit(f"{artifact_id}: missing local cache directory: {cache_dir}")
        manifest_rel = artifact.get("unpacked", {}).get("manifest_relative_path", "manifest.json")
        snapshot_rel = artifact.get("unpacked", {}).get("snapshot_relative_path", "lect_snapshot")
        if not (cache_dir / manifest_rel).exists():
            raise SystemExit(f"{artifact_id}: missing cache manifest: {cache_dir / manifest_rel}")
        if not (cache_dir / snapshot_rel).exists():
            raise SystemExit(f"{artifact_id}: missing cache snapshot: {cache_dir / snapshot_rel}")

        cache_key = cache_dir.resolve()
        archive_metadata = archive_by_cache_dir.get(cache_key)
        if archive_metadata is None:
            archive_name = artifact_archive_name(artifact)
            archive_path = out_dir / archive_name
            if archive_path.exists():
                if not args.force:
                    raise SystemExit(f"archive exists: {archive_path} (pass --force)")
                archive_path.unlink()
            create_tar_gz(
                cache_dir,
                archive_path,
                arc_prefix=Path(expected_unpack_path),
                compresslevel=int(args.gzip_compresslevel),
            )
            directory_sha = stable_directory_sha256(cache_dir)
            archive_sha = sha256_file(archive_path)
            archive_metadata = {
                "archive_name": archive_name,
                "archive_path": archive_path,
                "archive_sha": archive_sha,
                "directory_sha": directory_sha,
                "size_bytes": archive_path.stat().st_size,
            }
            archive_by_cache_dir[cache_key] = archive_metadata
            print(f"packaged {artifact_id}: {archive_path} sha256={archive_sha}")
        else:
            archive_path = archive_metadata["archive_path"]
            print(f"reused {artifact_id}: {archive_path} sha256={archive_metadata['archive_sha']}")

        archive_name = str(archive_metadata["archive_name"])
        artifact.setdefault("archive", {})
        artifact["archive"]["file_name"] = archive_name
        artifact["archive"]["url"] = (
            f"{args.url_base.rstrip('/')}/{archive_name}" if args.url_base else "TODO-upload-url"
        )
        artifact["archive"]["sha256"] = str(archive_metadata["archive_sha"])
        artifact["archive"]["size_bytes"] = int(archive_metadata["size_bytes"])
        artifact.setdefault("unpacked", {})
        artifact["unpacked"]["directory_sha256"] = str(archive_metadata["directory_sha"])
        packaged.append(artifact)

    output_manifest = {
        "schema_version": int(manifest.get("schema_version", 1)),
        "description": "Filled external LECT cache artifact manifest generated by scripts/package_cache_artifacts.py.",
        "artifacts": packaged,
    }
    output_manifest_path.write_text(json.dumps(output_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {output_manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
