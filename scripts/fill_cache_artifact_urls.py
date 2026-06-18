#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

sys.dont_write_bytecode = True


URL_RE = re.compile(r"^https://[^ \t\r\n]+$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Rewrite archive.url fields in a filled cache artifact manifest if "
            "a separate cache bundle is published outside git. This does not "
            "repackage caches or modify SHA256/size fields."
        )
    )
    parser.add_argument("manifest", type=Path, help="Filled cache artifact manifest to read.")
    parser.add_argument("--url-base", required=True, help="HTTPS base URL containing the cache archives.")
    parser.add_argument("--out", type=Path, required=True, help="Output manifest path.")
    parser.add_argument("--force", action="store_true", help="Replace the output manifest if it already exists.")
    return parser.parse_args()


def load_manifest(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} root must be a JSON object")
    artifacts = data.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        raise ValueError(f"{path} must contain a non-empty artifacts list")
    return data


def main() -> int:
    args = parse_args()
    if not URL_RE.fullmatch(args.url_base):
        raise SystemExit("--url-base must be an HTTPS URL")
    if args.out.exists() and not args.force:
        raise SystemExit(f"output manifest exists: {args.out} (pass --force)")
    manifest = load_manifest(args.manifest)
    url_base = args.url_base.rstrip("/")
    for artifact in manifest["artifacts"]:
        if not isinstance(artifact, dict):
            raise SystemExit("all artifacts must be JSON objects")
        artifact_id = str(artifact.get("id", "<unknown>"))
        archive = artifact.get("archive")
        if not isinstance(archive, dict):
            raise SystemExit(f"{artifact_id}: archive must be an object")
        file_name = archive.get("file_name")
        if not isinstance(file_name, str) or not file_name:
            raise SystemExit(f"{artifact_id}: archive.file_name must be a non-empty string")
        if Path(file_name).name != file_name or Path(file_name).is_absolute() or ".." in Path(file_name).parts:
            raise SystemExit(f"{artifact_id}: archive.file_name must be a plain filename")
        archive["url"] = f"{url_base}/{file_name}"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
