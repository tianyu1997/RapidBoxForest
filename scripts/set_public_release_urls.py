#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

sys.dont_write_bytecode = True


URL_RE = re.compile(r"^https://[^ \t\r\n]+$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Set public repository URLs in release citation metadata.")
    parser.add_argument("--repo-root", type=Path, default=Path("."), help="Repository root.")
    parser.add_argument("--repo-url", required=True, help="Public repository HTTPS URL.")
    parser.add_argument("--doi", default=None, help="Optional software or release DOI to write into CFF metadata.")
    parser.add_argument("--version", default=None, help="Optional version string to write into CFF metadata.")
    parser.add_argument("--release-date", default=None, help="Optional YYYY-MM-DD release date to write into CFF metadata.")
    parser.add_argument("--dry-run", action="store_true", help="Print changed files without writing.")
    return parser.parse_args()


def quote(value: str) -> str:
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def validate_args(args: argparse.Namespace) -> None:
    if not URL_RE.match(args.repo_url):
        raise SystemExit("--repo-url must be an HTTPS URL")
    if any(token in args.repo_url for token in ("TODO", "CHANGE_ME", "<owner>", "example.com", "example-owner")):
        raise SystemExit("--repo-url must be a real public URL, not a placeholder")
    if args.doi and any(token in args.doi for token in ("TODO", "CHANGE_ME", "example.com", "example-owner")):
        raise SystemExit("--doi must not contain placeholders")
    if args.release_date and not re.fullmatch(r"\d{4}-\d{2}-\d{2}", args.release_date):
        raise SystemExit("--release-date must use YYYY-MM-DD")


def set_scalar(lines: list[str], key: str, value: str, *, after: str | None = None) -> list[str]:
    prefix = f"{key}:"
    filtered = [line for line in lines if not line.startswith(prefix)]
    insert_at = len(filtered)
    if after is not None:
        after_prefix = f"{after}:"
        for index, line in enumerate(filtered):
            if line.startswith(after_prefix):
                insert_at = index + 1
                break
    filtered.insert(insert_at, f"{key}: {quote(value)}")
    return filtered


def update_citation(path: Path, *, repo_url: str, doi: str | None, version: str | None, release_date: str | None) -> str:
    lines = path.read_text(encoding="utf-8").splitlines()
    if version:
        lines = set_scalar(lines, "version", version, after="authors")
    if release_date:
        lines = set_scalar(lines, "date-released", release_date, after="version")
    lines = set_scalar(lines, "repository-code", repo_url, after="license")
    lines = set_scalar(lines, "url", repo_url, after="repository-code")
    if doi:
        lines = set_scalar(lines, "doi", doi, after="url")
    return "\n".join(lines).rstrip() + "\n"


def main() -> int:
    args = parse_args()
    validate_args(args)
    root = args.repo_root.resolve()
    paths = [
        root / "CITATION.cff",
        root / "safe_box_forest" / "CITATION.cff",
    ]
    missing = [str(path) for path in paths if not path.exists()]
    if missing:
        raise SystemExit(f"missing citation files: {missing}")
    updates: list[tuple[Path, str]] = []
    for path in paths:
        updated = update_citation(
            path,
            repo_url=args.repo_url,
            doi=args.doi,
            version=args.version,
            release_date=args.release_date,
        )
        updates.append((path, updated))
    for path, updated in updates:
        rel = path.relative_to(root)
        if args.dry_run:
            print(f"would update {rel}")
        else:
            path.write_text(updated, encoding="utf-8")
            print(f"updated {rel}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
