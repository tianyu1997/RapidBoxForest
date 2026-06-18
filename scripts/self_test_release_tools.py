#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import gzip
import hashlib
import io
import json
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

sys.dont_write_bytecode = True


REPO_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run lightweight release-tool regression checks.")
    parser.add_argument("--repo-root", type=Path, default=REPO_ROOT, help="Repository root.")
    parser.add_argument("--keep-tmp", action="store_true", help="Keep the temporary smoke directory for debugging.")
    return parser.parse_args()


def run(command: list[str], *, cwd: Path, expect_success: bool = True) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(command, cwd=str(cwd), text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if expect_success and result.returncode != 0:
        rendered = " ".join(command)
        raise RuntimeError(
            f"command failed unexpectedly ({result.returncode}): {rendered}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    if not expect_success and result.returncode == 0:
        rendered = " ".join(command)
        raise RuntimeError(
            f"command succeeded unexpectedly: {rendered}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    return result


def write_fake_manifest(path: Path) -> None:
    artifact = {
        "id": "fake_cache",
        "role": "smoke-test",
        "required_for": ["release-tool-self-test"],
        "robot": "iiwa",
        "endpoint_source": "AAFK",
        "envelope": "support_hull",
        "depth": 23,
        "depth_semantics": "LECT canonical tree depth",
        "split_schedule": [5, 3, 0, 5],
        "canonical_mode": True,
        "coverage_domain": "smoke",
        "expected_unpack_path": "outputs/fake_cache",
        "build_command": ["echo", "smoke"],
        "archive": {
            "file_name": "TODO-upload-file.tar.gz",
            "url": "TODO-upload-url",
            "sha256": "TODO-upload-sha256",
            "size_bytes": "TODO-upload-size",
        },
        "unpacked": {
            "manifest_relative_path": "manifest.json",
            "snapshot_relative_path": "lect_snapshot",
            "directory_sha256": "TODO-directory-sha256",
        },
    }
    duplicate_artifact = copy.deepcopy(artifact)
    duplicate_artifact["id"] = "fake_cache_alias"
    duplicate_artifact["role"] = "smoke-test-alias"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "artifacts": [artifact, duplicate_artifact],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )


def check_required_file_list_consistency(repo_root: Path) -> None:
    scripts_dir = repo_root / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    import check_public_release
    import check_release_readiness

    release_required = set(check_release_readiness.REQUIRED_RELEASE_FILES)
    public_required = set(check_public_release.REQUIRED_FILES)
    missing_from_public_checker = sorted(release_required - public_required)
    if missing_from_public_checker:
        raise RuntimeError(
            "check_public_release.REQUIRED_FILES is missing release-critical files: "
            + ", ".join(missing_from_public_checker)
        )


def check_staged_scope_policy(repo_root: Path, tmp_root: Path) -> None:
    scripts_dir = repo_root / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    import check_release_readiness

    staged_repo = tmp_root / "staged_scope_repo"
    staged_repo.mkdir(parents=True, exist_ok=True)
    run(["git", "init", "-q"], cwd=staged_repo)
    run(["git", "config", "user.email", "release-tools@example.invalid"], cwd=staged_repo)
    run(["git", "config", "user.name", "Release Tool Self Test"], cwd=staged_repo)
    scratch_file = staged_repo / "CS,CS-"
    scratch_file.write_text("", encoding="utf-8")
    run(["git", "add", "CS,CS-"], cwd=staged_repo)
    run(["git", "commit", "-q", "-m", "seed scratch file"], cwd=staged_repo)

    scratch_file.unlink()
    run(["git", "add", "-u", "CS,CS-"], cwd=staged_repo)
    deletion_errors = check_release_readiness.check_tracking_only_staged_scope(staged_repo)
    if deletion_errors:
        raise RuntimeError(f"non-exported staged deletion should be allowed, got: {deletion_errors}")

    (staged_repo / "private_notes.txt").write_text("private\n", encoding="utf-8")
    run(["git", "add", "private_notes.txt"], cwd=staged_repo)
    add_errors = check_release_readiness.check_tracking_only_staged_scope(staged_repo)
    if not add_errors:
        raise RuntimeError("non-exported staged addition should be rejected")


def write_mutated_manifest(source: Path, destination: Path, *, mutation: str) -> None:
    data = json.loads(source.read_text(encoding="utf-8"))
    artifact = data["artifacts"][0]
    if mutation == "archive_sha":
        artifact["archive"]["sha256"] = "0" * 64
    elif mutation == "archive_size":
        artifact["archive"]["size_bytes"] = int(artifact["archive"]["size_bytes"]) + 1
    elif mutation == "directory_sha":
        artifact["unpacked"]["directory_sha256"] = "0" * 64
    else:
        raise ValueError(f"unknown mutation: {mutation}")
    destination.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def tarinfo(name: str, data: bytes | None = None) -> tarfile.TarInfo:
    info = tarfile.TarInfo(name)
    info.uid = 0
    info.gid = 0
    info.uname = ""
    info.gname = ""
    info.mtime = 0
    if data is None:
        info.type = tarfile.DIRTYPE
        info.mode = 0o755
    else:
        info.size = len(data)
        info.mode = 0o644
    return info


def write_minimal_public_package(
    package_dir: Path,
    cache_manifest: Path,
    *,
    cache_archives_checked: bool,
    release_tools_checked: bool = True,
) -> Path:
    package_dir.mkdir(parents=True, exist_ok=True)
    package_name = "RapidBoxForest-public"
    archive_path = package_dir / f"{package_name}.tar.gz"
    package_manifest_path = package_dir / f"{package_name}.package.json"
    readme_bytes = b"# RapidBoxForest smoke package\n"
    public_manifest = {
        "source_repo": "release tool self-test",
        "file_count": 1,
        "policy": "minimal package for check_public_package regression",
        "files": ["README.md"],
        "file_sha256": {"README.md": hashlib.sha256(readme_bytes).hexdigest()},
    }
    public_manifest_bytes = json.dumps(public_manifest, indent=2, sort_keys=True).encode("utf-8") + b"\n"
    with archive_path.open("wb") as raw_stream:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw_stream, mtime=0) as gzip_stream:
            with tarfile.open(fileobj=gzip_stream, mode="w") as tar:
                tar.addfile(tarinfo(package_name))
                tar.addfile(tarinfo(f"{package_name}/README.md", readme_bytes), fileobj=io.BytesIO(readme_bytes))
                tar.addfile(
                    tarinfo(f"{package_name}/PUBLIC_RELEASE_MANIFEST.json", public_manifest_bytes),
                    fileobj=io.BytesIO(public_manifest_bytes),
                )
    package_manifest = {
        "schema_version": 2,
        "package_name": package_name,
        "archive": archive_path.name,
        "archive_sha256": sha256_file(archive_path),
        "tar_member_count": 3,
        "duplicate_member_count": 0,
        "source_file_count": 1,
        "tree_file_count_including_manifest": 2,
        "public_release_manifest_sha256": hashlib.sha256(public_manifest_bytes).hexdigest(),
        "checks": "passed",
        "release_tools_checked": release_tools_checked,
        "paper_compile_checked": False,
        "repo_url": None,
        "doi": None,
        "version": None,
        "release_date": None,
        "cache_manifest": cache_manifest.name,
        "cache_manifest_sha256": sha256_file(cache_manifest),
        "cache_archives_checked": cache_archives_checked,
    }
    package_manifest_path.write_text(json.dumps(package_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return package_manifest_path


def write_package_without_field(source: Path, destination: Path, field: str) -> None:
    data = json.loads(source.read_text(encoding="utf-8"))
    data.pop(field, None)
    destination.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_package_with_schema_version(source: Path, destination: Path, schema_version: int) -> None:
    data = json.loads(source.read_text(encoding="utf-8"))
    data["schema_version"] = schema_version
    destination.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_self_test(repo_root: Path, tmp_root: Path) -> None:
    check_required_file_list_consistency(repo_root)
    check_staged_scope_policy(repo_root, tmp_root)
    fake_repo = tmp_root / "repo"
    cache_dir = fake_repo / "outputs/fake_cache"
    snapshot_dir = cache_dir / "lect_snapshot"
    out_dir = tmp_root / "out"
    snapshot_dir.mkdir(parents=True)
    out_dir.mkdir(parents=True)
    (cache_dir / "manifest.json").write_text('{"fake":"manifest"}\n', encoding="utf-8")
    (snapshot_dir / "data.bin").write_text("snapshot-bytes\n", encoding="utf-8")

    template_manifest = tmp_root / "cache_artifacts.template.json"
    write_fake_manifest(template_manifest)

    run(
        [
            sys.executable,
            "scripts/package_cache_artifacts.py",
            str(template_manifest),
            "--repo-root",
            str(fake_repo),
            "--out-dir",
            str(out_dir),
            "--url-base",
            "https://artifacts.example.org/rbf",
            "--force",
        ],
        cwd=repo_root,
    )
    filled_manifest = out_dir / "cache_artifacts.json"
    archive_paths = sorted(out_dir.glob("*.tar.gz"))
    if len(archive_paths) != 1:
        raise RuntimeError(f"expected one deduplicated cache archive, got {len(archive_paths)}: {archive_paths}")
    filled_data = json.loads(filled_manifest.read_text(encoding="utf-8"))
    filled_artifacts = filled_data["artifacts"]
    if len(filled_artifacts) != 2:
        raise RuntimeError(f"expected two filled cache artifacts, got {len(filled_artifacts)}")
    archive_names = {artifact["archive"]["file_name"] for artifact in filled_artifacts}
    if len(archive_names) != 1:
        raise RuntimeError(f"duplicate cache artifacts should reuse one archive, got {archive_names}")
    rewritten_manifest = out_dir / "cache_artifacts.rewritten_urls.json"
    run(
        [
            sys.executable,
            "scripts/fill_cache_artifact_urls.py",
            str(filled_manifest),
            "--url-base",
            "https://downloads.example.org/rbf/cache",
            "--out",
            str(rewritten_manifest),
        ],
        cwd=repo_root,
    )
    rewritten_data = json.loads(rewritten_manifest.read_text(encoding="utf-8"))
    rewritten_urls = {artifact["archive"]["url"] for artifact in rewritten_data["artifacts"]}
    if rewritten_urls != {"https://downloads.example.org/rbf/cache/fake_cache.tar.gz"}:
        raise RuntimeError(f"unexpected rewritten cache artifact URLs: {rewritten_urls}")
    run(
        [
            sys.executable,
            "scripts/check_cache_artifacts.py",
            str(rewritten_manifest),
            "--repo-root",
            str(fake_repo),
            "--archive-dir",
            str(out_dir),
            "--verify-local",
        ],
        cwd=repo_root,
    )
    run(
        [
            sys.executable,
            "scripts/check_cache_artifacts.py",
            str(filled_manifest),
            "--repo-root",
            str(fake_repo),
            "--archive-dir",
            str(out_dir),
            "--verify-local",
        ],
        cwd=repo_root,
    )

    if shutil.which("zstd"):
        zstd_out_dir = tmp_root / "out_zstd"
        zstd_out_dir.mkdir(parents=True)
        run(
            [
                sys.executable,
                "scripts/package_cache_artifacts.py",
                str(template_manifest),
                "--repo-root",
                str(fake_repo),
                "--out-dir",
                str(zstd_out_dir),
                "--archive-format",
                "tar.zst",
                "--zstd-level",
                "3",
                "--url-base",
                "https://artifacts.example.org/rbf",
                "--force",
            ],
            cwd=repo_root,
        )
        zstd_manifest = zstd_out_dir / "cache_artifacts.json"
        zstd_archive_paths = sorted(zstd_out_dir.glob("*.tar.zst"))
        if len(zstd_archive_paths) != 1:
            raise RuntimeError(f"expected one deduplicated zstd cache archive, got {len(zstd_archive_paths)}")
        zstd_data = json.loads(zstd_manifest.read_text(encoding="utf-8"))
        zstd_names = {artifact["archive"]["file_name"] for artifact in zstd_data["artifacts"]}
        if zstd_names != {"fake_cache.tar.zst"}:
            raise RuntimeError(f"unexpected zstd archive names: {zstd_names}")
        run(
            [
                sys.executable,
                "scripts/check_cache_artifacts.py",
                str(zstd_manifest),
                "--repo-root",
                str(fake_repo),
                "--archive-dir",
                str(zstd_out_dir),
                "--verify-local",
            ],
            cwd=repo_root,
        )

    for mutation in ("archive_sha", "archive_size", "directory_sha"):
        bad_manifest = out_dir / f"cache_artifacts.bad_{mutation}.json"
        write_mutated_manifest(filled_manifest, bad_manifest, mutation=mutation)
        run(
            [
                sys.executable,
                "scripts/check_cache_artifacts.py",
                str(bad_manifest),
                "--repo-root",
                str(fake_repo),
                "--archive-dir",
                str(out_dir),
                "--verify-local",
            ],
            cwd=repo_root,
            expect_success=False,
        )

    package_dir = tmp_root / "package"
    good_package_manifest = write_minimal_public_package(package_dir, filled_manifest, cache_archives_checked=True)
    run(
        [
            sys.executable,
            "scripts/check_public_package.py",
            str(good_package_manifest),
            "--cache-manifest",
            str(filled_manifest),
            "--require-cache-archives-checked",
            "--require-release-tools-checked",
        ],
        cwd=repo_root,
    )
    for missing_field in ("cache_archives_checked", "release_tools_checked"):
        missing_field_manifest = good_package_manifest.with_name(f"RapidBoxForest-public.package.missing_{missing_field}.json")
        write_package_without_field(good_package_manifest, missing_field_manifest, missing_field)
        run(
            [
                sys.executable,
                "scripts/check_public_package.py",
                str(missing_field_manifest),
                "--cache-manifest",
                str(filled_manifest),
            ],
            cwd=repo_root,
            expect_success=False,
        )
    schema_v1_manifest = good_package_manifest.with_name("RapidBoxForest-public.package.schema_v1.json")
    write_package_with_schema_version(good_package_manifest, schema_v1_manifest, 1)
    run(
        [
            sys.executable,
            "scripts/check_public_package.py",
            str(schema_v1_manifest),
            "--cache-manifest",
            str(filled_manifest),
        ],
        cwd=repo_root,
        expect_success=False,
    )
    bad_package_manifest = write_minimal_public_package(
        tmp_root / "package_bad_cache_check",
        filled_manifest,
        cache_archives_checked=False,
    )
    run(
        [
            sys.executable,
            "scripts/check_public_package.py",
            str(bad_package_manifest),
            "--cache-manifest",
            str(filled_manifest),
            "--require-cache-archives-checked",
            "--require-release-tools-checked",
        ],
        cwd=repo_root,
        expect_success=False,
    )
    bad_release_tools_package_manifest = write_minimal_public_package(
        tmp_root / "package_bad_release_tools_check",
        filled_manifest,
        cache_archives_checked=True,
        release_tools_checked=False,
    )
    run(
        [
            sys.executable,
            "scripts/check_public_package.py",
            str(bad_release_tools_package_manifest),
            "--cache-manifest",
            str(filled_manifest),
            "--require-cache-archives-checked",
            "--require-release-tools-checked",
        ],
        cwd=repo_root,
        expect_success=False,
    )

    run(
        [
            sys.executable,
            "scripts/check_release_readiness.py",
            "--repo-root",
            ".",
            "--cache-archive-dir",
            str(out_dir),
        ],
        cwd=repo_root,
        expect_success=False,
    )
    run(
        [
            sys.executable,
            "scripts/package_public_release.py",
            "--out-dir",
            str(tmp_root / "package_should_fail"),
            "--cache-archive-dir",
            str(out_dir),
            "--force",
        ],
        cwd=repo_root,
        expect_success=False,
    )


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    if args.keep_tmp:
        tmp_root = Path(tempfile.mkdtemp(prefix="rbf-release-tools-self-test-"))
        run_self_test(repo_root, tmp_root)
        print(f"release tool self-test passed: {tmp_root}")
        return 0
    with tempfile.TemporaryDirectory(prefix="rbf-release-tools-self-test-") as tmp:
        run_self_test(repo_root, Path(tmp))
    print("release tool self-test passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
