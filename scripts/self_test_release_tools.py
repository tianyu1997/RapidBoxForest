#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import copy
import gzip
import hashlib
import io
import json
import re
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


def preprocessor_guard_ranges(text: str, guard_marker: str, *, path: Path, guard_name: str) -> list[tuple[int, int]]:
    directive_re = re.compile(r"^[ \t]*#\s*(if|ifdef|ifndef|endif)\b.*$", re.MULTILINE)
    ranges: list[tuple[int, int]] = []
    search_start = 0
    while True:
        guard_start = text.find(guard_marker, search_start)
        if guard_start < 0:
            break

        line_start = text.rfind("\n", 0, guard_start) + 1
        depth = 0
        guard_end = -1
        for match in directive_re.finditer(text, line_start):
            directive = match.group(1)
            if directive in {"if", "ifdef", "ifndef"}:
                depth += 1
            elif directive == "endif":
                depth -= 1
                if depth == 0:
                    guard_end = match.end()
                    break

        if guard_end < 0:
            raise RuntimeError(f"{path} has an unclosed {guard_name} guard")
        ranges.append((guard_start, guard_end))
        search_start = guard_end
    return ranges


def require_ordered_markers(path: Path, text: str, markers: tuple[str, ...]) -> None:
    search_start = 0
    for marker in markers:
        index = text.find(marker, search_start)
        if index < 0:
            raise RuntimeError(f"{path} must contain ordered marker after offset {search_start}: {marker}")
        search_start = index + len(marker)


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


def check_forbidden_sidecar_policy(repo_root: Path, tmp_root: Path) -> None:
    scripts_dir = repo_root / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    import check_public_release
    import check_release_readiness
    import export_public_release

    for dirname in sorted(export_public_release.FORBIDDEN_SOURCE_DIR_NAMES):
        if dirname not in check_public_release.BROKEN_REFERENCE_TERMS:
            raise RuntimeError(f"release reference scanner missing forbidden sidecar: {dirname}")
        if dirname not in check_release_readiness.STALE_REFERENCE_TERMS:
            raise RuntimeError(f"readiness reference scanner missing forbidden sidecar: {dirname}")

    fake_repo = tmp_root / "forbidden_sidecar_repo"
    for dirname in sorted(export_public_release.FORBIDDEN_SOURCE_DIR_NAMES):
        (fake_repo / dirname).mkdir(parents=True)
    sidecar_errors = export_public_release.check_forbidden_source_sidecars(fake_repo)
    if not sidecar_errors:
        raise RuntimeError("forbidden source sidecars should be rejected")
    for dirname in sorted(export_public_release.FORBIDDEN_SOURCE_DIR_NAMES):
        if dirname not in sidecar_errors[0]:
            raise RuntimeError(f"missing forbidden sidecar in error message: {dirname}")
    if "integrate their code into the main modules" not in sidecar_errors[0]:
        raise RuntimeError(f"unexpected forbidden sidecar error: {sidecar_errors}")


def check_core_environment_read_policy(repo_root: Path, tmp_root: Path) -> None:
    scripts_dir = repo_root / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    import check_release_readiness

    allowlist_reasons = check_release_readiness.CORE_GETENV_ALLOWLIST_REASONS
    if set(allowlist_reasons) != check_release_readiness.CORE_GETENV_ALLOWLIST:
        raise RuntimeError("core getenv allowlist must be derived from the documented reason map")
    for rel, reason in sorted(allowlist_reasons.items()):
        if not reason or "TODO" in reason:
            raise RuntimeError(f"core getenv allowlist entry lacks a stable reason: {rel}")
        if not (repo_root / rel).exists():
            raise RuntimeError(f"core getenv allowlist entry does not exist: {rel}")

    fake_repo = tmp_root / "core_env_repo"
    bad_source = fake_repo / "safe_box_forest" / "src" / "planning_forest_env.cpp"
    bad_source.parent.mkdir(parents=True)
    bad_source.write_text(
        '#include <cstdlib>\nint bad_env_switch() { return std::getenv("RBF_BAD_SWITCH") ? 1 : 0; }\n',
        encoding="utf-8",
    )
    errors = check_release_readiness.check_core_environment_reads(fake_repo)
    if not errors or "planning_forest_env.cpp" not in errors[0]:
        raise RuntimeError(f"core getenv policy should reject planner environment reads: {errors}")


def check_public_tree_readiness_policy(repo_root: Path) -> None:
    readiness_text = (repo_root / "scripts" / "check_release_readiness.py").read_text(encoding="utf-8")
    function_start = readiness_text.find("def check_public_tree(")
    if function_start < 0:
        raise RuntimeError("check_release_readiness.py must define check_public_tree")
    function_end = readiness_text.find("\ndef check_public_package(", function_start)
    if function_end < 0:
        raise RuntimeError("check_public_tree must appear before check_public_package")
    function_text = readiness_text[function_start:function_end]
    required_flags = (
        "--check-release-tools",
        "--run-smoke-dry-run",
    )
    for flag in required_flags:
        if flag not in function_text:
            raise RuntimeError(f"public-tree readiness must pass {flag} to check_public_release.py")


def literal_dict_keys_from_assignment(path: Path, assignment_name: str) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(target, ast.Name) and target.id == assignment_name for target in node.targets):
            continue
        if not isinstance(node.value, ast.Dict):
            raise RuntimeError(f"{path}:{assignment_name} must remain a dictionary")
        keys: set[str] = set()
        for key in node.value.keys:
            if not isinstance(key, ast.Constant) or not isinstance(key.value, str):
                raise RuntimeError(f"{path}:{assignment_name} must use literal string keys")
            keys.add(key.value)
        return keys
    raise RuntimeError(f"{path} must define {assignment_name}")


def check_active_experiment_plan_boundary(repo_root: Path) -> None:
    retired_experiments = {"exp07", "exp08"}
    dispatcher_path = repo_root / "experiments" / "run_tro2026.py"
    dispatcher_keys = literal_dict_keys_from_assignment(dispatcher_path, "EXPERIMENTS")
    retired_dispatcher_keys = sorted(dispatcher_keys & retired_experiments)
    if retired_dispatcher_keys:
        raise RuntimeError(
            "retired experiments must not be active dispatcher entries: "
            + ", ".join(retired_dispatcher_keys)
        )

    asset_generator_path = repo_root / "experiments" / "generate_tro2026_paper_assets.py"
    required_tables = literal_dict_keys_from_assignment(asset_generator_path, "REQUIRED_TABLES")
    forbidden_table = "tab_tro_dynamic_update.tex"
    if forbidden_table in required_tables:
        raise RuntimeError(f"{forbidden_table} must not be a required TRO paper table")
    forbidden_generated_table = repo_root / "paper" / "generated" / forbidden_table
    if forbidden_generated_table.exists():
        raise RuntimeError(f"{forbidden_generated_table} must not remain in active generated paper artifacts")

    active_text_files = (
        repo_root / "paper" / "sbf_tro_2026.tex",
        repo_root / "paper" / "generated" / "tro_table_generation_manifest.json",
    )
    forbidden_terms = (
        "tab_tro_dynamic_update",
        "exp07_dynamic_update",
        "update_replan_diagnostic",
    )
    for path in active_text_files:
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8")
        for term in forbidden_terms:
            if term in text:
                raise RuntimeError(f"{path} must not reference retired experiment artifact {term}")


def check_sbf_diagnostic_source_boundary(repo_root: Path) -> None:
    cmake_path = repo_root / "safe_box_forest" / "CMakeLists.txt"
    source_list_path = repo_root / "safe_box_forest" / "cmake" / "SBFSources.cmake"
    cmake_text = cmake_path.read_text(encoding="utf-8")
    source_text = source_list_path.read_text(encoding="utf-8")
    diagnostic_marker = "if(SBF_DIAGNOSTIC_API)"
    diagnostic_start = cmake_text.find(diagnostic_marker)
    if diagnostic_start < 0:
        raise RuntimeError("safe_box_forest CMakeLists.txt must define an SBF_DIAGNOSTIC_API block")
    diagnostic_end = cmake_text.find("endif()", diagnostic_start)
    if diagnostic_end < 0:
        raise RuntimeError("SBF_DIAGNOSTIC_API block must be closed with endif()")
    diagnostic_block = cmake_text[diagnostic_start:diagnostic_end]
    if "SBF_DIAGNOSTIC_SOURCES" not in diagnostic_block:
        raise RuntimeError("SBF_DIAGNOSTIC_API block must add SBF_DIAGNOSTIC_SOURCES")

    def cmake_list_block(name: str) -> str:
        marker = f"set({name}"
        start = source_text.find(marker)
        if start < 0:
            raise RuntimeError(f"{source_list_path} must define {name}")
        end = source_text.find(")", start)
        if end < 0:
            raise RuntimeError(f"{name} list must be closed with ')'")
        return source_text[start:end]

    def cmake_list_items(name: str) -> list[str]:
        items: list[str] = []
        for line in cmake_list_block(name).splitlines()[1:]:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            items.append(stripped)
        return items

    def cmake_cpp_entries(name: str, seen: set[str] | None = None) -> list[str]:
        if seen is None:
            seen = set()
        if name in seen:
            raise RuntimeError(f"CMake source group cycle detected at {name}")
        seen.add(name)
        entries: list[str] = []
        for stripped in cmake_list_items(name):
            if stripped.startswith("${") and stripped.endswith("}"):
                entries.extend(cmake_cpp_entries(stripped[2:-1], seen))
            elif stripped.startswith("src/") and stripped.endswith(".cpp"):
                entries.append(stripped)
            else:
                raise RuntimeError(f"unsupported item in {name}: {stripped}")
        seen.remove(name)
        return entries

    def require_group_refs_only(name: str) -> None:
        for item in cmake_list_items(name):
            if not (item.startswith("${") and item.endswith("}")):
                raise RuntimeError(f"{name} must aggregate child source groups only; found {item}")

    default_source_groups = (
        "SBF_GRAPH_PARTITION_SOURCES",
        "SBF_FFB_GROWER_SOURCES",
        "SBF_CONNECTOR_SOURCES",
        "SBF_PLANNING_BUILD_SOURCES",
        "SBF_OBB_OVERLAY_SOURCES",
        "SBF_QUERY_BRIDGE_SOURCES",
        "SBF_QUERY_RUNTIME_SOURCES",
    )
    core_sources_block = cmake_list_block("SBF_CORE_SOURCES")
    require_group_refs_only("SBF_CORE_SOURCES")
    for group in default_source_groups:
        cmake_list_block(group)
        require_group_refs_only(group)
        if f"${{{group}}}" not in core_sources_block:
            raise RuntimeError(f"SBF_CORE_SOURCES must include {group}")

    graph_partition_source_groups = (
        "SBF_ADAPTIVE_GRID_PARTITION_SOURCES",
        "SBF_BOX_GRAPH_SOURCES",
    )
    graph_partition_sources_block = cmake_list_block("SBF_GRAPH_PARTITION_SOURCES")
    for group in graph_partition_source_groups:
        cmake_list_block(group)
        if f"${{{group}}}" not in graph_partition_sources_block:
            raise RuntimeError(f"SBF_GRAPH_PARTITION_SOURCES must include {group}")

    ffb_grower_source_groups = (
        "SBF_FFB_CORE_SOURCES",
        "SBF_FRONTWAVE_GROWER_SOURCES",
        "SBF_RRT_GROWER_SOURCES",
        "SBF_LEAF_SWEEP_GROWER_SOURCES",
    )
    ffb_grower_sources_block = cmake_list_block("SBF_FFB_GROWER_SOURCES")
    for group in ffb_grower_source_groups:
        cmake_list_block(group)
        if f"${{{group}}}" not in ffb_grower_sources_block:
            raise RuntimeError(f"SBF_FFB_GROWER_SOURCES must include {group}")

    connector_source_groups = (
        "SBF_MERGER_SOURCES",
        "SBF_CONNECTOR_CORE_SOURCES",
        "SBF_CONNECTOR_PAIR_PIPELINE_SOURCES",
        "SBF_CHAIN_PAVE_SOURCES",
    )
    connector_sources_block = cmake_list_block("SBF_CONNECTOR_SOURCES")
    for group in connector_source_groups:
        cmake_list_block(group)
        if f"${{{group}}}" not in connector_sources_block:
            raise RuntimeError(f"SBF_CONNECTOR_SOURCES must include {group}")

    planning_facade_source_groups = (
        "SBF_PLANNING_CORE_SOURCES",
        "SBF_PLANNING_BUILD_PIPELINE_SOURCES",
    )
    planning_facade_sources_block = cmake_list_block("SBF_PLANNING_FACADE_SOURCES")
    for group in planning_facade_source_groups:
        cmake_list_block(group)
        if f"${{{group}}}" not in planning_facade_sources_block:
            raise RuntimeError(f"SBF_PLANNING_FACADE_SOURCES must include {group}")

    planning_build_source_groups = (
        "SBF_PLANNING_FACADE_SOURCES",
        "SBF_PLANNING_ADAPTIVE_SOURCES",
        "SBF_PLANNING_QROOT_SOURCES",
    )
    planning_build_sources_block = cmake_list_block("SBF_PLANNING_BUILD_SOURCES")
    for group in planning_build_source_groups:
        cmake_list_block(group)
        if f"${{{group}}}" not in planning_build_sources_block:
            raise RuntimeError(f"SBF_PLANNING_BUILD_SOURCES must include {group}")

    obb_overlay_source_groups = (
        "SBF_OBB_VALIDATION_SOURCES",
        "SBF_OBB_SAMPLED_BACKEND_SOURCES",
        "SBF_OVERLAY_PORTAL_SOURCES",
    )
    obb_overlay_sources_block = cmake_list_block("SBF_OBB_OVERLAY_SOURCES")
    for group in obb_overlay_source_groups:
        cmake_list_block(group)
        if f"${{{group}}}" not in obb_overlay_sources_block:
            raise RuntimeError(f"SBF_OBB_OVERLAY_SOURCES must include {group}")

    query_bridge_source_groups = (
        "SBF_QUERY_BRIDGE_CORE_SOURCES",
        "SBF_QUERY_BRIDGE_BATCH_SOURCES",
        "SBF_QUERY_BRIDGE_CORRIDOR_SOURCES",
        "SBF_QUERY_BRIDGE_DIRECT_SOURCES",
        "SBF_QUERY_BRIDGE_ENDPOINT_SOURCES",
        "SBF_QUERY_BRIDGE_RRT_SOURCES",
    )
    query_bridge_sources_block = cmake_list_block("SBF_QUERY_BRIDGE_SOURCES")
    for group in query_bridge_source_groups:
        cmake_list_block(group)
        if f"${{{group}}}" not in query_bridge_sources_block:
            raise RuntimeError(f"SBF_QUERY_BRIDGE_SOURCES must include {group}")

    query_runtime_source_groups = (
        "SBF_QUERY_FACADE_SOURCES",
        "SBF_QUERY_UTILITY_SOURCES",
        "SBF_QUERY_SHORTCUT_SOURCES",
        "SBF_QUERY_RESULT_SOURCES",
        "SBF_RUNTIME_INFRA_SOURCES",
    )
    query_runtime_sources_block = cmake_list_block("SBF_QUERY_RUNTIME_SOURCES")
    for group in query_runtime_source_groups:
        cmake_list_block(group)
        if f"${{{group}}}" not in query_runtime_sources_block:
            raise RuntimeError(f"SBF_QUERY_RUNTIME_SOURCES must include {group}")

    diagnostic_list_start = source_text.find("set(SBF_DIAGNOSTIC_SOURCES")
    if diagnostic_list_start < 0:
        raise RuntimeError(f"{source_list_path} must define SBF_DIAGNOSTIC_SOURCES")
    default_sources = source_text[:diagnostic_list_start]
    diagnostic_sources_block = cmake_list_block("SBF_DIAGNOSTIC_SOURCES")
    default_group_sources: dict[str, str] = {}
    for group in default_source_groups:
        for source in cmake_cpp_entries(group):
            previous_group = default_group_sources.get(source)
            if previous_group is not None:
                raise RuntimeError(f"{source} is duplicated in {previous_group} and {group}")
            default_group_sources[source] = group

    diagnostic_group_sources: dict[str, str] = {}
    for source in cmake_cpp_entries("SBF_DIAGNOSTIC_SOURCES"):
        if source in diagnostic_group_sources:
            raise RuntimeError(f"{source} is duplicated in SBF_DIAGNOSTIC_SOURCES")
        diagnostic_group_sources[source] = "SBF_DIAGNOSTIC_SOURCES"

    all_sbf_cpp_sources = {
        path.relative_to(repo_root / "safe_box_forest").as_posix()
        for path in (repo_root / "safe_box_forest" / "src").rglob("*.cpp")
    }
    root_level_cpp_sources = sorted(
        source for source in all_sbf_cpp_sources if Path(source).parent.as_posix() == "src"
    )
    if root_level_cpp_sources:
        raise RuntimeError(
            "safe_box_forest/src root-level .cpp files must move into explicit subdirectories: "
            + ", ".join(root_level_cpp_sources)
        )
    classified_sources = set(default_group_sources) | set(diagnostic_group_sources)
    missing_sources = sorted(all_sbf_cpp_sources - classified_sources)
    extra_sources = sorted(classified_sources - all_sbf_cpp_sources)
    if missing_sources:
        raise RuntimeError(
            "safe_box_forest/src .cpp files missing from SBF source groups: "
            + ", ".join(missing_sources)
        )
    if extra_sources:
        raise RuntimeError(
            "SBF source groups reference missing .cpp files: " + ", ".join(extra_sources)
        )

    graph_partition_group_sources = {
        source
        for group in graph_partition_source_groups
        for source in cmake_cpp_entries(group)
    }
    graph_partition_sources_outside_dir = sorted(
        source for source in graph_partition_group_sources if not source.startswith("src/graph_partition/")
    )
    if graph_partition_sources_outside_dir:
        raise RuntimeError(
            "SBF graph partition source groups must live under src/graph_partition: "
            + ", ".join(graph_partition_sources_outside_dir)
        )
    graph_partition_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if (
            Path(source).name.startswith("adaptive_grid_partition")
            or Path(source).name.startswith("box_graph")
        )
        and not source.startswith("src/graph_partition/")
    )
    if graph_partition_named_sources_outside_dir:
        raise RuntimeError(
            "graph partition implementation files must live under safe_box_forest/src/graph_partition: "
            + ", ".join(graph_partition_named_sources_outside_dir)
        )

    free_box_sources_outside_dir = sorted(
        source for source in cmake_cpp_entries("SBF_FFB_CORE_SOURCES") if not source.startswith("src/free_box/")
    )
    if free_box_sources_outside_dir:
        raise RuntimeError(
            "SBF_FFB_CORE_SOURCES entries must live under src/free_box: "
            + ", ".join(free_box_sources_outside_dir)
        )
    free_box_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if Path(source).name.startswith("find_free_box")
        and not source.startswith("src/free_box/")
    )
    if free_box_named_sources_outside_dir:
        raise RuntimeError(
            "free-box implementation files must live under safe_box_forest/src/free_box: "
            + ", ".join(free_box_named_sources_outside_dir)
        )

    grower_group_sources = {
        source
        for group in ("SBF_FRONTWAVE_GROWER_SOURCES", "SBF_RRT_GROWER_SOURCES")
        for source in cmake_cpp_entries(group)
    }
    grower_sources_outside_dir = sorted(
        source for source in grower_group_sources if not source.startswith("src/grower/")
    )
    if grower_sources_outside_dir:
        raise RuntimeError(
            "SBF grower source groups must live under src/grower: "
            + ", ".join(grower_sources_outside_dir)
        )
    grower_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if (Path(source).name.startswith("grower") or Path(source).name == "frontwave_grower.cpp")
        and not source.startswith("src/grower/")
    )
    if grower_named_sources_outside_dir:
        raise RuntimeError(
            "grower implementation files must live under safe_box_forest/src/grower: "
            + ", ".join(grower_named_sources_outside_dir)
        )

    leaf_sweep_sources_outside_dir = sorted(
        source
        for source in cmake_cpp_entries("SBF_LEAF_SWEEP_GROWER_SOURCES")
        if not source.startswith("src/leaf_sweep_grower/")
    )
    if leaf_sweep_sources_outside_dir:
        raise RuntimeError(
            "SBF_LEAF_SWEEP_GROWER_SOURCES entries must live under src/leaf_sweep_grower: "
            + ", ".join(leaf_sweep_sources_outside_dir)
        )
    leaf_sweep_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if Path(source).name.startswith("leaf_sweep_grower")
        and not source.startswith("src/leaf_sweep_grower/")
    )
    if leaf_sweep_named_sources_outside_dir:
        raise RuntimeError(
            "leaf-sweep grower implementation files must live under safe_box_forest/src/leaf_sweep_grower: "
            + ", ".join(leaf_sweep_named_sources_outside_dir)
        )

    obb_group_sources = {
        source
        for group in ("SBF_OBB_VALIDATION_SOURCES", "SBF_OBB_SAMPLED_BACKEND_SOURCES")
        for source in cmake_cpp_entries(group)
    }
    obb_sources_outside_dir = sorted(
        source for source in obb_group_sources if not source.startswith("src/obb/")
    )
    if obb_sources_outside_dir:
        raise RuntimeError(
            "SBF OBB source groups must live under src/obb: "
            + ", ".join(obb_sources_outside_dir)
        )
    obb_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if Path(source).name.startswith("planning_forest_obb")
        and not source.startswith("src/obb/")
    )
    if obb_named_sources_outside_dir:
        raise RuntimeError(
            "OBB implementation files must live under safe_box_forest/src/obb: "
            + ", ".join(obb_named_sources_outside_dir)
        )

    overlay_sources_outside_dir = sorted(
        source
        for source in cmake_cpp_entries("SBF_OVERLAY_PORTAL_SOURCES")
        if not source.startswith("src/overlay/")
    )
    if overlay_sources_outside_dir:
        raise RuntimeError(
            "SBF_OVERLAY_PORTAL_SOURCES entries must live under src/overlay: "
            + ", ".join(overlay_sources_outside_dir)
        )
    overlay_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if Path(source).name.startswith("planning_forest_overlay")
        and not source.startswith("src/overlay/")
    )
    if overlay_named_sources_outside_dir:
        raise RuntimeError(
            "overlay implementation files must live under safe_box_forest/src/overlay: "
            + ", ".join(overlay_named_sources_outside_dir)
        )

    planning_core_sources_outside_dir = sorted(
        source
        for source in cmake_cpp_entries("SBF_PLANNING_CORE_SOURCES")
        if not source.startswith("src/planning_core/")
    )
    if planning_core_sources_outside_dir:
        raise RuntimeError(
            "SBF_PLANNING_CORE_SOURCES entries must live under src/planning_core: "
            + ", ".join(planning_core_sources_outside_dir)
        )
    planning_core_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if Path(source).name
        in {
            "planning_forest_audit.cpp",
            "planning_forest_core.cpp",
            "planning_forest_database.cpp",
            "planning_forest_diagnostics.cpp",
        }
        and not source.startswith("src/planning_core/")
    )
    if planning_core_named_sources_outside_dir:
        raise RuntimeError(
            "planning core implementation files must live under safe_box_forest/src/planning_core: "
            + ", ".join(planning_core_named_sources_outside_dir)
        )

    planning_build_pipeline_sources_outside_dir = sorted(
        source
        for source in cmake_cpp_entries("SBF_PLANNING_BUILD_PIPELINE_SOURCES")
        if not source.startswith("src/planning_build/")
    )
    if planning_build_pipeline_sources_outside_dir:
        raise RuntimeError(
            "SBF_PLANNING_BUILD_PIPELINE_SOURCES entries must live under src/planning_build: "
            + ", ".join(planning_build_pipeline_sources_outside_dir)
        )
    planning_build_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if (
            Path(source).name == "planning_forest_build.cpp"
            or Path(source).name.startswith("planning_forest_ffb")
            or Path(source).name.startswith("planning_forest_leaf_refine")
            or Path(source).name.startswith("planning_forest_partition")
        )
        and not source.startswith("src/planning_build/")
    )
    if planning_build_named_sources_outside_dir:
        raise RuntimeError(
            "planning build pipeline implementation files must live under safe_box_forest/src/planning_build: "
            + ", ".join(planning_build_named_sources_outside_dir)
        )

    adaptive_sources_outside_dir = sorted(
        source
        for source in cmake_cpp_entries("SBF_PLANNING_ADAPTIVE_SOURCES")
        if not source.startswith("src/planning_adaptive/")
    )
    if adaptive_sources_outside_dir:
        raise RuntimeError(
            "SBF_PLANNING_ADAPTIVE_SOURCES entries must live under src/planning_adaptive: "
            + ", ".join(adaptive_sources_outside_dir)
        )
    adaptive_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if Path(source).name.startswith("planning_forest_adaptive")
        and not source.startswith("src/planning_adaptive/")
    )
    if adaptive_named_sources_outside_dir:
        raise RuntimeError(
            "adaptive planning implementation files must live under safe_box_forest/src/planning_adaptive: "
            + ", ".join(adaptive_named_sources_outside_dir)
        )
    qroot_sources_outside_dir = sorted(
        source
        for source in cmake_cpp_entries("SBF_PLANNING_QROOT_SOURCES")
        if not source.startswith("src/qroot/")
    )
    if qroot_sources_outside_dir:
        raise RuntimeError(
            "SBF_PLANNING_QROOT_SOURCES entries must live under src/qroot: "
            + ", ".join(qroot_sources_outside_dir)
        )
    qroot_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if Path(source).name.startswith("planning_forest_qroot")
        and not source.startswith("src/qroot/")
    )
    if qroot_named_sources_outside_dir:
        raise RuntimeError(
            "query-root implementation files must live under safe_box_forest/src/qroot: "
            + ", ".join(qroot_named_sources_outside_dir)
        )

    connector_group_sources = {
        source
        for group in connector_source_groups
        for source in cmake_cpp_entries(group)
    }
    connector_sources_outside_dir = sorted(
        source for source in connector_group_sources if not source.startswith("src/connector/")
    )
    if connector_sources_outside_dir:
        raise RuntimeError(
            "SBF connector source groups must live under src/connector: "
            + ", ".join(connector_sources_outside_dir)
        )
    connector_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if (Path(source).name.startswith("connector_") or Path(source).name in {"connector.cpp", "merger.cpp"})
        and not source.startswith("src/connector/")
        and source != "src/diagnostic/connector_chain_pave_debug.cpp"
    )
    if connector_named_sources_outside_dir:
        raise RuntimeError(
            "connector implementation files must live under safe_box_forest/src/connector: "
            + ", ".join(connector_named_sources_outside_dir)
        )
    query_bridge_group_sources = {
        source
        for group in query_bridge_source_groups
        for source in cmake_cpp_entries(group)
    }
    query_bridge_sources_outside_dir = sorted(
        source for source in query_bridge_group_sources if not source.startswith("src/query_bridge/")
    )
    if query_bridge_sources_outside_dir:
        raise RuntimeError(
            "SBF query bridge source groups must live under src/query_bridge: "
            + ", ".join(query_bridge_sources_outside_dir)
        )
    query_bridge_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if Path(source).name.startswith("planning_forest_query_bridge")
        and not source.startswith("src/query_bridge/")
    )
    if query_bridge_named_sources_outside_dir:
        raise RuntimeError(
            "query bridge implementation files must live under safe_box_forest/src/query_bridge: "
            + ", ".join(query_bridge_named_sources_outside_dir)
        )
    query_runtime_group_sources = {
        source
        for group in query_runtime_source_groups
        for source in cmake_cpp_entries(group)
    }
    query_runtime_sources_outside_dir = sorted(
        source for source in query_runtime_group_sources if not source.startswith("src/query_runtime/")
    )
    if query_runtime_sources_outside_dir:
        raise RuntimeError(
            "SBF query runtime source groups must live under src/query_runtime: "
            + ", ".join(query_runtime_sources_outside_dir)
        )
    query_runtime_named_sources_outside_dir = sorted(
        source
        for source in all_sbf_cpp_sources
        if (
            Path(source).name == "planning_forest_query.cpp"
            or Path(source).name == "planning_forest_shortcut.cpp"
            or (
                Path(source).name.startswith("planning_forest_query_")
                and not Path(source).name.startswith("planning_forest_query_bridge")
            )
        )
        and not source.startswith("src/query_runtime/")
    )
    if query_runtime_named_sources_outside_dir:
        raise RuntimeError(
            "query runtime implementation files must live under safe_box_forest/src/query_runtime: "
            + ", ".join(query_runtime_named_sources_outside_dir)
        )

    diagnostic_filename_tokens = ("dynamic", "subtractive", "debug", "corridor_refine")
    misplaced_diagnostic_sources = sorted(
        source
        for source in all_sbf_cpp_sources
        if any(token in Path(source).name for token in diagnostic_filename_tokens)
        and not source.startswith("src/diagnostic/")
    )
    if misplaced_diagnostic_sources:
        raise RuntimeError(
            "opt-in diagnostic source files must live under safe_box_forest/src/diagnostic: "
            + ", ".join(misplaced_diagnostic_sources)
        )
    diagnostic_sources_outside_dir = sorted(
        source for source in diagnostic_group_sources if not source.startswith("src/diagnostic/")
    )
    if diagnostic_sources_outside_dir:
        raise RuntimeError(
            "SBF_DIAGNOSTIC_SOURCES entries must live under src/diagnostic: "
            + ", ".join(diagnostic_sources_outside_dir)
        )
    all_sbf_private_headers = {
        path.relative_to(repo_root / "safe_box_forest").as_posix()
        for path in (repo_root / "safe_box_forest" / "src").rglob("*.h")
    }
    root_level_private_headers = sorted(
        header for header in all_sbf_private_headers if Path(header).parent.as_posix() == "src"
    )
    if root_level_private_headers:
        raise RuntimeError(
            "safe_box_forest/src root-level private headers must move into explicit subdirectories: "
            + ", ".join(root_level_private_headers)
        )
    connector_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if Path(header).name.startswith("connector_")
        and not header.startswith("src/connector/")
    )
    if connector_headers_outside_dir:
        raise RuntimeError(
            "connector private headers must live under safe_box_forest/src/connector: "
            + ", ".join(connector_headers_outside_dir)
        )
    graph_partition_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if (
            Path(header).name.startswith("adaptive_grid_partition")
            or Path(header).name.startswith("box_graph")
        )
        and not header.startswith("src/graph_partition/")
    )
    if graph_partition_headers_outside_dir:
        raise RuntimeError(
            "graph partition private headers must live under safe_box_forest/src/graph_partition: "
            + ", ".join(graph_partition_headers_outside_dir)
        )
    free_box_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if (
            Path(header).name.startswith("find_free_box")
            or Path(header).name.startswith("virtual_sparse_ffb")
        )
        and not header.startswith("src/free_box/")
    )
    if free_box_headers_outside_dir:
        raise RuntimeError(
            "free-box private headers must live under safe_box_forest/src/free_box: "
            + ", ".join(free_box_headers_outside_dir)
        )
    grower_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if Path(header).name.startswith("grower")
        and not header.startswith("src/grower/")
    )
    if grower_headers_outside_dir:
        raise RuntimeError(
            "grower private headers must live under safe_box_forest/src/grower: "
            + ", ".join(grower_headers_outside_dir)
        )
    leaf_sweep_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if Path(header).name.startswith("leaf_sweep_grower")
        and not header.startswith("src/leaf_sweep_grower/")
    )
    if leaf_sweep_headers_outside_dir:
        raise RuntimeError(
            "leaf-sweep grower private headers must live under safe_box_forest/src/leaf_sweep_grower: "
            + ", ".join(leaf_sweep_headers_outside_dir)
        )
    obb_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if Path(header).name.startswith("planning_forest_obb")
        and not header.startswith("src/obb/")
    )
    if obb_headers_outside_dir:
        raise RuntimeError(
            "OBB private headers must live under safe_box_forest/src/obb: "
            + ", ".join(obb_headers_outside_dir)
        )
    planning_core_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if Path(header).name in {"planning_forest_audit.h", "planning_forest_diagnostics.h"}
        and not header.startswith("src/planning_core/")
    )
    if planning_core_headers_outside_dir:
        raise RuntimeError(
            "planning core private headers must live under safe_box_forest/src/planning_core: "
            + ", ".join(planning_core_headers_outside_dir)
        )
    planning_build_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if Path(header).name.startswith("planning_forest_ffb")
        and not header.startswith("src/planning_build/")
    )
    if planning_build_headers_outside_dir:
        raise RuntimeError(
            "planning build private headers must live under safe_box_forest/src/planning_build: "
            + ", ".join(planning_build_headers_outside_dir)
        )
    adaptive_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if Path(header).name.startswith("planning_forest_adaptive")
        and not header.startswith("src/planning_adaptive/")
    )
    if adaptive_headers_outside_dir:
        raise RuntimeError(
            "adaptive planning private headers must live under safe_box_forest/src/planning_adaptive: "
            + ", ".join(adaptive_headers_outside_dir)
        )
    qroot_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if Path(header).name.startswith("planning_forest_qroot")
        and not header.startswith("src/qroot/")
    )
    if qroot_headers_outside_dir:
        raise RuntimeError(
            "query-root private headers must live under safe_box_forest/src/qroot: "
            + ", ".join(qroot_headers_outside_dir)
        )
    query_bridge_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if Path(header).name.startswith("planning_forest_query_bridge")
        and not header.startswith("src/query_bridge/")
    )
    if query_bridge_headers_outside_dir:
        raise RuntimeError(
            "query bridge private headers must live under safe_box_forest/src/query_bridge: "
            + ", ".join(query_bridge_headers_outside_dir)
        )
    query_runtime_headers_outside_dir = sorted(
        header
        for header in all_sbf_private_headers
        if (
            Path(header).name.startswith("planning_forest_query_")
            and not Path(header).name.startswith("planning_forest_query_bridge")
        )
        and not header.startswith("src/query_runtime/")
    )
    if query_runtime_headers_outside_dir:
        raise RuntimeError(
            "query runtime private headers must live under safe_box_forest/src/query_runtime: "
            + ", ".join(query_runtime_headers_outside_dir)
        )

    forbidden_private_aggregate_includes = (
        "#include <SBF/safe_box_forest.h>",
        "#include <SBF/planning_config.h>",
        "#include <SBF/planning_result.h>",
        "#include <SBF/build_config.h>",
        "#include <SBF/query.h>",
    )
    private_headers_with_aggregate_includes: list[str] = []
    for header in sorted(all_sbf_private_headers):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        forbidden_includes = [
            include for include in forbidden_private_aggregate_includes if include in header_text
        ]
        if forbidden_includes:
            private_headers_with_aggregate_includes.append(
                f"{header}: {', '.join(forbidden_includes)}"
            )
    if private_headers_with_aggregate_includes:
        raise RuntimeError(
            "safe_box_forest private headers must not include facade/config/result aggregate headers; "
            "include narrow type headers or forward declarations instead: "
            + "; ".join(private_headers_with_aggregate_includes)
        )

    connector_private_headers_with_public_entry_includes: list[str] = []
    for header in sorted(
        header for header in all_sbf_private_headers if header.startswith("src/connector/")
    ):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        if "#include <SBF/connector.h>" in header_text:
            connector_private_headers_with_public_entry_includes.append(header)
    if connector_private_headers_with_public_entry_includes:
        raise RuntimeError(
            "connector private headers must not include the public connector algorithm entry header; "
            "use SBF/connector_types.h plus SBF/runtime_fwd.h or forward declarations instead: "
            + ", ".join(connector_private_headers_with_public_entry_includes)
        )

    grower_private_headers_with_public_entry_includes: list[str] = []
    for header in sorted(
        header for header in all_sbf_private_headers if header.startswith("src/grower/")
    ):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        if "#include <SBF/grower.h>" in header_text:
            grower_private_headers_with_public_entry_includes.append(header)
    if grower_private_headers_with_public_entry_includes:
        raise RuntimeError(
            "grower private headers must not include the public grower algorithm entry header; "
            "use SBF/grower_types.h plus SBF/runtime_fwd.h or forward declarations instead: "
            + ", ".join(grower_private_headers_with_public_entry_includes)
        )

    leaf_sweep_grower_private_headers_with_public_entry_includes: list[str] = []
    for header in sorted(
        header for header in all_sbf_private_headers if header.startswith("src/leaf_sweep_grower/")
    ):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        if "#include <SBF/leaf_sweep_grower.h>" in header_text:
            leaf_sweep_grower_private_headers_with_public_entry_includes.append(header)
    if leaf_sweep_grower_private_headers_with_public_entry_includes:
        raise RuntimeError(
            "leaf-sweep grower private headers must not include the public leaf-sweep grower entry header; "
            "use SBF/leaf_sweep_types.h plus SBF/runtime_fwd.h or narrow oracle declarations as needed: "
            + ", ".join(leaf_sweep_grower_private_headers_with_public_entry_includes)
        )

    ffb_private_headers_with_public_entry_includes: list[str] = []
    for header in sorted(all_sbf_private_headers):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        if "#include <SBF/find_free_box.h>" in header_text:
            ffb_private_headers_with_public_entry_includes.append(header)
    if ffb_private_headers_with_public_entry_includes:
        raise RuntimeError(
            "safe_box_forest private headers must not include the public find-free-box service entry header; "
            "use SBF/find_free_box_types.h plus SBF/runtime_fwd.h or narrow oracle declarations as needed: "
            + ", ".join(ffb_private_headers_with_public_entry_includes)
        )

    graph_private_headers_with_public_entry_includes: list[str] = []
    for header in sorted(all_sbf_private_headers):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        if "#include <SBF/box_graph.h>" in header_text:
            graph_private_headers_with_public_entry_includes.append(header)
    if graph_private_headers_with_public_entry_includes:
        raise RuntimeError(
            "safe_box_forest private headers must not include the full box graph algorithm header; "
            "use SBF/box_adjacency_types.h, SBF/query_graph_types.h, "
            "SBF/query_graph_cache_types.h, SBF/segment_edge_fwd.h, "
            "or local implementation includes instead: "
            + ", ".join(graph_private_headers_with_public_entry_includes)
        )

    private_headers_allowed_full_runtime: set[str] = set()
    private_headers_with_full_runtime = []
    for header in sorted(all_sbf_private_headers):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        if "#include <SBF/runtime.h>" in header_text and header not in private_headers_allowed_full_runtime:
            private_headers_with_full_runtime.append(header)
    if private_headers_with_full_runtime:
        raise RuntimeError(
            "private declaration headers must include SBF/runtime_fwd.h instead of full SBF/runtime.h; "
            "move helpers that call StageContext/StageDiagnostics methods into implementation files where possible: "
            + ", ".join(private_headers_with_full_runtime)
        )
    grower_internal_header = repo_root / "safe_box_forest" / "src" / "grower" / "grower_internal.h"
    grower_internal_text = grower_internal_header.read_text(encoding="utf-8")
    if "#include <SBF/runtime_fwd.h>" not in grower_internal_text:
        raise RuntimeError("grower_internal.h must include SBF/runtime_fwd.h for StageContext declarations")
    if "#include <SBF/runtime.h>" in grower_internal_text:
        raise RuntimeError("grower_internal.h must keep StageContext method calls in grower_internal.cpp")
    leaf_sweep_internal_header = repo_root / "safe_box_forest" / "src" / "leaf_sweep_grower" / "leaf_sweep_grower_internal.h"
    leaf_sweep_internal_text = leaf_sweep_internal_header.read_text(encoding="utf-8")
    if "#include <SBF/runtime_fwd.h>" not in leaf_sweep_internal_text:
        raise RuntimeError("leaf_sweep_grower_internal.h must include SBF/runtime_fwd.h for StageContext declarations")
    if "#include <SBF/runtime.h>" in leaf_sweep_internal_text:
        raise RuntimeError("leaf_sweep_grower_internal.h must keep StageContext method calls in leaf_sweep_grower_internal.cpp")
    ffb_internal_header = repo_root / "safe_box_forest" / "src" / "free_box" / "find_free_box_internal.h"
    ffb_internal_text = ffb_internal_header.read_text(encoding="utf-8")
    if "#include <SBF/runtime_fwd.h>" not in ffb_internal_text:
        raise RuntimeError("find_free_box_internal.h must include SBF/runtime_fwd.h for StageContext declarations")
    if "#include <SBF/runtime.h>" in ffb_internal_text:
        raise RuntimeError("find_free_box_internal.h must keep StageContext method calls in find_free_box_internal.cpp")
    query_bridge_pave_guard_header = (
        repo_root / "safe_box_forest" / "src" / "query_bridge" / "planning_forest_query_bridge_pave_guard.h"
    )
    query_bridge_pave_guard_text = query_bridge_pave_guard_header.read_text(encoding="utf-8")
    if "#include <SBF/runtime_fwd.h>" not in query_bridge_pave_guard_text:
        raise RuntimeError(
            "planning_forest_query_bridge_pave_guard.h must include SBF/runtime_fwd.h for StageContext declarations"
        )
    if "#include <SBF/runtime.h>" in query_bridge_pave_guard_text:
        raise RuntimeError(
            "planning_forest_query_bridge_pave_guard.h must keep StageContext method calls in "
            "planning_forest_query_bridge_pave_guard.cpp"
        )

    private_headers_with_full_scene = []
    for header in sorted(all_sbf_private_headers):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        forbidden_includes = [
            include
            for include in (
                "#include <SBF/scene.h>",
                "#include <LECTDatabase/sbf/scene.h>",
            )
            if include in header_text
        ]
        if forbidden_includes:
            private_headers_with_full_scene.append(f"{header}: {', '.join(forbidden_includes)}")
    if private_headers_with_full_scene:
        raise RuntimeError(
            "private declaration headers must include SBF/scene_types.h or forward declare Scene/CollisionChecker "
            "instead of full scene headers: "
            + "; ".join(private_headers_with_full_scene)
        )

    private_headers_allowed_full_oracle: set[str] = set()
    private_headers_with_full_oracle = []
    for header in sorted(all_sbf_private_headers):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        forbidden_includes = [
            include
            for include in (
                "#include <SBF/oracle.h>",
                "#include <LECTDatabase/sbf/oracle.h>",
            )
            if include in header_text
        ]
        if forbidden_includes and header not in private_headers_allowed_full_oracle:
            private_headers_with_full_oracle.append(f"{header}: {', '.join(forbidden_includes)}")
    if private_headers_with_full_oracle:
        raise RuntimeError(
            "private declaration headers must include LECTDatabase/sbf/oracle_types.h or forward declarations "
            "instead of full oracle headers; move helpers that call oracle methods into implementation files: "
            + "; ".join(private_headers_with_full_oracle)
        )
    if "#include <LECTDatabase/sbf/oracle_types.h>" not in grower_internal_text:
        raise RuntimeError("grower_internal.h must include oracle type declarations")
    if "#include <SBF/oracle.h>" in grower_internal_text:
        raise RuntimeError("grower_internal.h must keep BoxOracle method calls in grower_internal.cpp")
    if "#include <LECTDatabase/sbf/oracle_types.h>" not in leaf_sweep_internal_text:
        raise RuntimeError("leaf_sweep_grower_internal.h must include oracle type declarations")
    if "#include <SBF/oracle.h>" in leaf_sweep_internal_text:
        raise RuntimeError(
            "leaf_sweep_grower_internal.h must keep DatabaseBoxOracle method calls in leaf_sweep_grower_internal.cpp"
        )
    virtual_sparse_header = repo_root / "safe_box_forest" / "src" / "free_box" / "virtual_sparse_ffb.h"
    virtual_sparse_text = virtual_sparse_header.read_text(encoding="utf-8")
    if "#include <LECTDatabase/sbf/oracle_types.h>" not in virtual_sparse_text:
        raise RuntimeError("virtual_sparse_ffb.h must include oracle type declarations")
    if "#include <SBF/oracle.h>" in virtual_sparse_text:
        raise RuntimeError("virtual_sparse_ffb.h must keep BoxOracle method calls in virtual_sparse_ffb.cpp")
    cmake_text = (repo_root / "safe_box_forest" / "cmake" / "SBFSources.cmake").read_text(encoding="utf-8")
    if "src/free_box/find_free_box_internal.cpp" not in cmake_text:
        raise RuntimeError("SBF_FFB_CORE_SOURCES must include find_free_box_internal.cpp")
    if "src/grower/grower_internal.cpp" not in cmake_text:
        raise RuntimeError("SBF_RRT_GROWER_SOURCES must include grower_internal.cpp")
    if "src/grower/grower_entry.cpp" not in cmake_text:
        raise RuntimeError("SBF_RRT_GROWER_SOURCES must include grower_entry.cpp")
    if "src/grower/grower_sampling.cpp" not in cmake_text:
        raise RuntimeError("SBF_RRT_GROWER_SOURCES must include grower_sampling.cpp")
    if "src/graph_partition/adaptive_grid_partition_overlay_components.cpp" not in cmake_text:
        raise RuntimeError(
            "SBF_ADAPTIVE_GRID_PARTITION_SOURCES must include adaptive_grid_partition_overlay_components.cpp"
        )
    if "src/planning_adaptive/planning_forest_adaptive_validation.cpp" not in cmake_text:
        raise RuntimeError(
            "SBF_PLANNING_ADAPTIVE_SOURCES must include planning_forest_adaptive_validation.cpp"
        )
    if "src/leaf_sweep_grower/leaf_sweep_grower_internal.cpp" not in cmake_text:
        raise RuntimeError("SBF_LEAF_SWEEP_GROWER_SOURCES must include leaf_sweep_grower_internal.cpp")
    if "src/query_bridge/planning_forest_query_bridge_pave_guard.cpp" not in cmake_text:
        raise RuntimeError("SBF_QUERY_BRIDGE_CORE_SOURCES must include planning_forest_query_bridge_pave_guard.cpp")
    if "src/connector/connector_chain_pave_commit.cpp" not in cmake_text:
        raise RuntimeError("SBF_CHAIN_PAVE_SOURCES must include connector_chain_pave_commit.cpp")
    if "src/free_box/virtual_sparse_ffb.cpp" not in cmake_text:
        raise RuntimeError("SBF_FFB_CORE_SOURCES must include virtual_sparse_ffb.cpp")

    adaptive_partition_private_headers_with_public_entry_includes: list[str] = []
    for header in sorted(all_sbf_private_headers):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        if "#include <SBF/adaptive_grid_partition.h>" in header_text:
            adaptive_partition_private_headers_with_public_entry_includes.append(header)
    if adaptive_partition_private_headers_with_public_entry_includes:
        raise RuntimeError(
            "safe_box_forest private headers must not include the full adaptive partition class header; "
            "use SBF/adaptive_grid_partition_types.h or forward declarations instead: "
            + ", ".join(adaptive_partition_private_headers_with_public_entry_includes)
        )

    narrow_private_helper_headers = {
        "src/graph_partition/adaptive_grid_partition_keys.h": {
            "forbidden": (
                "#include <SBF/adaptive_grid_partition.h>",
            ),
            "required": (
                "#include <SBF/adaptive_grid_partition_types.h>",
            ),
        },
        "src/graph_partition/adaptive_grid_partition_geometry.h": {
            "forbidden": (
                "#include <SBF/adaptive_grid_partition.h>",
            ),
            "required": (
                "#include <SBF/adaptive_grid_partition_types.h>",
            ),
        },
        "src/diagnostic/planning_forest_dynamic_segment_fallback_helpers.h": {
            "forbidden": (
                "#include <SBF/adaptive_grid_partition.h>",
            ),
            "required": (
                "#include <SBF/adaptive_grid_partition_types.h>",
                "const AdaptiveGridPartitionStats& partition_stats",
            ),
        },
        "src/planning_adaptive/planning_forest_adaptive_cover_utils.h": {
            "forbidden": (
                "#include <SBF/safe_box_forest.h>",
                "#include <SBF/leaf_sweep_grower.h>",
                "#include <SBF/planning_result.h>",
            ),
            "required": (
                "#include <SBF/adaptive_leaf_sweep_config.h>",
                "#include <SBF/leaf_sweep_types.h>",
                "#include <LECTDatabase/sbf/oracle_types.h>",
                "#include <rbf/lect_database/split_policy.h>",
                "struct AdaptiveLeafSweepResult;",
            ),
        },
        "src/planning_adaptive/planning_forest_adaptive_diagnostics.h": {
            "forbidden": (
                '#include "planning_forest_adaptive_merge.h"',
                "#include <SBF/box_graph.h>",
                "#include <SBF/planning_result.h>",
            ),
            "required": (
                "#include <SBF/adaptive_leaf_sweep_config.h>",
                "struct AdaptiveLeafSweepResult;",
                "struct AdjacencyBuildStats;",
                "struct BudgetedMergeStats;",
            ),
        },
        "src/obb/planning_forest_obb.h": {
            "forbidden": (
                "#include <SBF/safe_box_forest.h>",
                "#include <SBF/scene.h>",
            ),
            "required": (
                "#include <SBF/adaptive_leaf_sweep_config.h>",
                "#include <SBF/scene_types.h>",
            ),
        },
        "src/query_runtime/planning_forest_query_repair.h": {
            "forbidden": (
                "#include <SBF/planning_config.h>",
                "#include <LECTDatabase/sbf/scene.h>",
            ),
            "required": (
                "#include <SBF/query_config.h>",
                "#include <SBF/query_result.h>",
                "#include <SBF/query_runtime_config.h>",
                "#include <SBF/scene_types.h>",
                "struct RBFPlanningConfig;",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_options.h": {
            "forbidden": (
                "#include <SBF/planning_config.h>",
                "#include <SBF/api.h>",
                "#include <SBF/runtime.h>",
            ),
            "required": (
                "#include <SBF/query_result.h>",
                "#include <SBF/runtime_fwd.h>",
                "struct RBFPlanningConfig;",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_rrt_utils.h": {
            "forbidden": (
                "#include <SBF/planning_config.h>",
                "#include <SBF/runtime.h>",
                "#include <LECTDatabase/sbf/scene.h>",
            ),
            "required": (
                "#include <SBF/connector_types.h>",
                "#include <SBF/runtime_fwd.h>",
                "#include <SBF/scene_types.h>",
                "struct RBFPlanningConfig;",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_attempt_paths.h": {
            "forbidden": (
                "#include <SBF/planning_config.h>",
                "#include <SBF/runtime.h>",
                "#include <LECTDatabase/sbf/scene.h>",
            ),
            "required": (
                "#include <SBF/runtime_fwd.h>",
                "#include <SBF/scene_types.h>",
                "struct RBFPlanningConfig;",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_task.h": {
            "forbidden": (
                "#include <SBF/connector.h>",
            ),
            "required": (
                "#include <SBF/connector_types.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_endpoint_runtime.h": {
            "forbidden": (
                "#include <SBF/adaptive_grid_partition.h>",
            ),
            "required": (
                "class AdaptiveGridPartition;",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_endpoint_targets.h": {
            "forbidden": (
                "#include <SBF/adaptive_grid_partition.h>",
            ),
            "required": (
                "class AdaptiveGridPartition;",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_corridor_graph.h": {
            "forbidden": (
                "#include <SBF/adaptive_grid_partition.h>",
            ),
            "required": (
                "class AdaptiveGridPartition;",
            ),
        },
    }
    for header, include_sets in narrow_private_helper_headers.items():
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        forbidden_includes = [
            include for include in include_sets["forbidden"] if include in header_text
        ]
        if forbidden_includes:
            raise RuntimeError(
                f"{header} must include narrow type headers instead of broad implementation/facade headers: "
                + ", ".join(forbidden_includes)
            )
        missing_includes = [
            include for include in include_sets["required"] if include not in header_text
        ]
        if missing_includes:
            raise RuntimeError(
                f"{header} is missing explicit narrow type includes: "
                + ", ".join(missing_includes)
            )

    narrow_private_implementation_sources = {
        "src/query_bridge/planning_forest_query_bridge_corridor_options.cpp": {
            "forbidden": (
                "#include <SBF/safe_box_forest.h>",
            ),
            "required": (
                "#include <SBF/planning_config.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_repair_options.cpp": {
            "forbidden": (
                "#include <SBF/safe_box_forest.h>",
            ),
            "required": (
                "#include <SBF/planning_config.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_hipac_utils.cpp": {
            "forbidden": (
                "#include <SBF/safe_box_forest.h>",
            ),
            "required": (
                "#include <SBF/adaptive_leaf_sweep_config.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_hipac.cpp": {
            "forbidden": (),
            "required": (
                "#include <SBF/safe_box_forest.h>",
                "#include <SBF/adaptive_grid_partition.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_direct_corridor.cpp": {
            "forbidden": (),
            "required": (
                "#include <SBF/safe_box_forest.h>",
                "#include <SBF/adaptive_grid_partition.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_endpoint.cpp": {
            "forbidden": (),
            "required": (
                "#include <SBF/safe_box_forest.h>",
                "#include <SBF/adaptive_grid_partition.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_corridor_graph.cpp": {
            "forbidden": (
                "#include <SBF/safe_box_forest.h>",
            ),
            "required": (
                '#include "planning_forest_query_bridge_corridor_graph.h"',
                "#include <SBF/adaptive_grid_partition.h>",
                "#include <SBF/runtime.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_corridor_commit.cpp": {
            "forbidden": (
                "#include <SBF/safe_box_forest.h>",
            ),
            "required": (
                '#include "planning_forest_query_bridge_corridor_graph.h"',
                "#include <SBF/adaptive_grid_partition.h>",
                "#include <SBF/runtime.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_endpoint_runtime.cpp": {
            "forbidden": (),
            "required": (
                '#include "planning_forest_query_bridge_endpoint_runtime.h"',
                "#include <SBF/adaptive_grid_partition.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_endpoint_targets.cpp": {
            "forbidden": (),
            "required": (
                '#include "planning_forest_query_bridge_endpoint_targets.h"',
                "#include <SBF/adaptive_grid_partition.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_batch_policy.cpp": {
            "forbidden": (
                "#include <SBF/safe_box_forest.h>",
            ),
            "required": (
                '#include "planning_forest_query_bridge_policy.h"',
            ),
        },
        "src/connector/connector_pair_commit.cpp": {
            "forbidden": (),
            "required": (
                '#include "connector_pair_commit.h"',
                "#include <SBF/connector.h>",
            ),
        },
        "src/connector/connector_chain_pave.cpp": {
            "forbidden": (),
            "required": (
                '#include "connector_chain_pave_internal.h"',
                "#include <SBF/connector.h>",
            ),
        },
        "src/connector/connector_chain_pave_commit.cpp": {
            "forbidden": (),
            "required": (
                '#include "connector_chain_pave_internal.h"',
                "#include <SBF/find_free_box.h>",
                "#include <SBF/oracle.h>",
            ),
        },
        "src/connector/connector_pair_tasks.cpp": {
            "forbidden": (),
            "required": (
                '#include "connector_pair_tasks.h"',
                "#include <SBF/connector.h>",
            ),
        },
        "src/query_bridge/planning_forest_query_bridge_rrt_utils.cpp": {
            "forbidden": (),
            "required": (
                '#include "planning_forest_query_bridge_rrt_utils.h"',
                "#include <SBF/connector.h>",
            ),
        },
    }
    for source, include_sets in narrow_private_implementation_sources.items():
        source_text = (repo_root / "safe_box_forest" / source).read_text(encoding="utf-8")
        forbidden_includes = [
            include for include in include_sets["forbidden"] if include in source_text
        ]
        if forbidden_includes:
            raise RuntimeError(
                f"{source} must include narrow type headers instead of the planner facade: "
                + ", ".join(forbidden_includes)
            )
        missing_includes = [
            include for include in include_sets["required"] if include not in source_text
        ]
        if missing_includes:
            raise RuntimeError(
                f"{source} is missing explicit narrow implementation includes: "
                + ", ".join(missing_includes)
            )

    misplaced_diagnostic_headers = sorted(
        header
        for header in all_sbf_private_headers
        if any(token in Path(header).name for token in diagnostic_filename_tokens)
        and not header.startswith("src/diagnostic/")
    )
    if misplaced_diagnostic_headers:
        raise RuntimeError(
            "opt-in diagnostic private headers must live under safe_box_forest/src/diagnostic: "
            + ", ".join(misplaced_diagnostic_headers)
        )

    diagnostic_header_names = {
        path.name
        for path in (repo_root / "safe_box_forest" / "src").rglob("*.h")
        if any(token in path.name for token in diagnostic_filename_tokens)
    }
    diagnostic_include_re = re.compile(r'^[ \t]*#\s*include\s+"([^"]+)"', re.MULTILINE)
    public_diagnostic_include_re = re.compile(
        r'^[ \t]*#\s*include\s+<SBF/(debug|diagnostic_result|subtractive_build_config|dynamic_update_config|diagnostic_build_config)\.h>',
        re.MULTILINE,
    )
    allowed_default_diagnostic_guard_sources = {
        "src/connector/connector_chain_pave_hooks.cpp",
        "src/planning_core/planning_forest_diagnostic_hooks.cpp",
    }
    allowed_default_dynamic_cache_source = "src/planning_core/planning_forest_diagnostic_hooks.cpp"
    allowed_default_chain_pave_debug_hook_source = "src/connector/connector_chain_pave_hooks.cpp"
    for source in sorted(default_group_sources):
        source_path = repo_root / "safe_box_forest" / source
        source_text = source_path.read_text(encoding="utf-8")
        for match in diagnostic_include_re.finditer(source_text):
            included_header = Path(match.group(1)).name
            if included_header not in diagnostic_header_names:
                continue
            raise RuntimeError(
                f"{source} must not include diagnostic helper header {included_header}; "
                "route default code through guarded facade helpers instead"
            )
        guard_ranges = preprocessor_guard_ranges(
            source_text,
            "#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API",
            path=source_path,
            guard_name="SBF_DIAGNOSTIC_API",
        )
        for match in public_diagnostic_include_re.finditer(source_text):
            if any(start <= match.start() < end for start, end in guard_ranges):
                continue
            raise RuntimeError(
                f"{source} must guard diagnostic public include {match.group(0).strip()} "
                "behind SBF_DIAGNOSTIC_API"
            )
        if "RebuildProfile" in source_text:
            raise RuntimeError(
                f"{source} must not implement dynamic-update RebuildProfile helpers; "
                "place them under SBF_DIAGNOSTIC_SOURCES"
            )
        if "DebugBoundaryFfbFailure" in source_text:
            raise RuntimeError(
                f"{source} must not assemble debug boundary failure payloads; "
                "place them under SBF_DIAGNOSTIC_SOURCES"
            )
        if "SBF_DIAGNOSTIC_API" in source_text and source not in allowed_default_diagnostic_guard_sources:
            raise RuntimeError(
                f"{source} must not contain SBF_DIAGNOSTIC_API branches; "
                "route diagnostic lifecycle work through explicit hook files"
            )
        direct_dynamic_cache_terms = (
            "initialize_dynamic_collision_cache(",
            "clear_dynamic_collision_cache(",
            "populate_dynamic_collision_cache(",
            "dynamic_collision_cache_box_count(",
        )
        if source != allowed_default_dynamic_cache_source:
            for term in direct_dynamic_cache_terms:
                if term in source_text:
                    raise RuntimeError(
                        f"{source} must not call {term} directly; "
                        "use the diagnostic lifecycle hook wrappers"
                    )
        if source != allowed_default_chain_pave_debug_hook_source:
            if "record_chain_pave_boundary_debug_failure(" in source_text:
                raise RuntimeError(
                    f"{source} must not call record_chain_pave_boundary_debug_failure() directly; "
                    "use the chain-pave diagnostic hook wrapper"
                )

    diagnostic_sources = (
        "src/diagnostic/connector_chain_pave_debug.cpp",
        "src/diagnostic/planning_forest_corridor_refine.cpp",
        "src/diagnostic/planning_forest_debug.cpp",
        "src/diagnostic/planning_forest_dynamic_cache.cpp",
        "src/diagnostic/planning_forest_dynamic_collision_cache.cpp",
        "src/diagnostic/planning_forest_dynamic_helpers.cpp",
        "src/diagnostic/planning_forest_dynamic_partition.cpp",
        "src/diagnostic/planning_forest_dynamic_remove.cpp",
        "src/diagnostic/planning_forest_dynamic_refill.cpp",
        "src/diagnostic/planning_forest_dynamic_segment_endpoint.cpp",
        "src/diagnostic/planning_forest_dynamic_segment_fallback.cpp",
        "src/diagnostic/planning_forest_subtractive.cpp",
        "src/diagnostic/planning_forest_subtractive_seeds.cpp",
    )
    for source in diagnostic_sources:
        if source not in diagnostic_sources_block:
            raise RuntimeError(f"{source} must be listed in SBF_DIAGNOSTIC_SOURCES")
        if source in default_sources or source in default_group_sources:
            raise RuntimeError(f"{source} must not be in the default sbf_core source list")


def check_sbf_header_diagnostic_boundary(repo_root: Path) -> None:
    guard_marker = "#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API"

    def require_terms_guarded(header_path: Path, diagnostic_terms: tuple[str, ...]) -> None:
        header_text = header_path.read_text(encoding="utf-8")
        guard_ranges = preprocessor_guard_ranges(
            header_text,
            guard_marker,
            path=header_path,
            guard_name="SBF_DIAGNOSTIC_API",
        )

        if not guard_ranges:
            raise RuntimeError(f"{header_path} must contain SBF_DIAGNOSTIC_API guarded declarations")

        def in_diagnostic_guard(index: int) -> bool:
            return any(start <= index < end for start, end in guard_ranges)

        for term in diagnostic_terms:
            index = header_text.find(term)
            if index < 0:
                raise RuntimeError(f"{header_path} is missing expected diagnostic declaration term {term}")
            while index >= 0:
                if not in_diagnostic_guard(index):
                    raise RuntimeError(f"{term} in {header_path} must be inside an SBF_DIAGNOSTIC_API guard")
                index = header_text.find(term, index + len(term))

    include_dir = repo_root / "safe_box_forest" / "include" / "SBF"
    detail_dir = include_dir / "detail"
    forest_header = include_dir / "safe_box_forest.h"
    forest_header_text = forest_header.read_text(encoding="utf-8")
    diagnostic_public_fragment = detail_dir / "planning_forest_diagnostic_public_methods.inc"
    diagnostic_partition_fragment = detail_dir / "planning_forest_diagnostic_partition_methods.inc"
    diagnostic_cache_fragment = detail_dir / "planning_forest_diagnostic_cache_methods.inc"
    diagnostic_state_fragment = detail_dir / "planning_forest_diagnostic_state.inc"
    private_entry_fragment = detail_dir / "planning_forest_private_entry_methods.inc"
    private_query_bridge_direct_fragment = detail_dir / "planning_forest_private_query_bridge_direct_methods.inc"
    private_adaptive_build_fragment = detail_dir / "planning_forest_private_adaptive_build_methods.inc"
    private_query_bridge_batch_fragment = detail_dir / "planning_forest_private_query_bridge_batch_methods.inc"
    private_topology_fragment = detail_dir / "planning_forest_private_topology_methods.inc"
    private_overlay_fragment = detail_dir / "planning_forest_private_overlay_methods.inc"
    private_cache_fragment = detail_dir / "planning_forest_private_cache_methods.inc"
    diagnostic_fragments = (
        diagnostic_public_fragment,
        diagnostic_partition_fragment,
        diagnostic_cache_fragment,
        diagnostic_state_fragment,
    )
    private_fragments = (
        private_entry_fragment,
        private_query_bridge_direct_fragment,
        private_adaptive_build_fragment,
        private_query_bridge_batch_fragment,
        private_topology_fragment,
        private_overlay_fragment,
        private_cache_fragment,
    )
    class_body_fragment_texts = {
        path: path.read_text(encoding="utf-8") for path in diagnostic_fragments + private_fragments
    }
    diagnostic_fragment_texts = {
        path: class_body_fragment_texts[path] for path in diagnostic_fragments
    }
    private_fragment_texts = {
        path: class_body_fragment_texts[path] for path in private_fragments
    }
    for path, text in class_body_fragment_texts.items():
        for forbidden in (
            "#include",
            "#pragma once",
            "namespace rbf",
            "class RBFPlanningForest",
            "SBF_DIAGNOSTIC_API",
        ):
            if forbidden in text:
                raise RuntimeError(
                    f"{path} must remain a class-body declaration fragment without {forbidden}"
                )
    if "#include <SBF/debug.h>" in forest_header_text:
        raise RuntimeError(
            f"{forest_header} must forward declare debug payload return types instead of including SBF/debug.h"
        )
    for include in (
        "#include <SBF/diagnostic_result.h>",
        "#include <SBF/subtractive_build_config.h>",
    ):
        if include in forest_header_text:
            raise RuntimeError(
                f"{forest_header} must forward declare diagnostic payload/config types instead of including {include}"
            )
    if "const SubtractiveBuildOptions& options = {}" in forest_header_text:
        raise RuntimeError(
            f"{forest_header} must not use a default SubtractiveBuildOptions argument that forces a full config include"
        )
    diagnostic_public_text = diagnostic_fragment_texts[diagnostic_public_fragment]
    if "const SubtractiveBuildOptions& options = {}" in diagnostic_public_text:
        raise RuntimeError(
            f"{diagnostic_public_fragment} must not use a default SubtractiveBuildOptions argument"
        )
    if diagnostic_public_text.count("BuildProfile build_subtractive") != 2:
        raise RuntimeError(
            f"{diagnostic_public_fragment} must expose explicit build_subtractive overloads"
        )
    require_ordered_markers(
        forest_header,
        forest_header_text,
        (
            "class RBFPlanningForest {",
            "public:",
            "// Build and coverage construction.",
            "// Query, endpoint repair, and bridge construction.",
            "void clear_forest();",
            "// Diagnostic-only facade entry points.",
            "#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API",
            "#include <SBF/detail/planning_forest_diagnostic_public_methods.inc>",
            "#endif",
            "// State accessors and cache handles.",
            "private:",
            "// FFB and query entry helpers.",
            "#include <SBF/detail/planning_forest_private_entry_methods.inc>",
            "// Query bridge point-location and direct-corridor helpers.",
            "#include <SBF/detail/planning_forest_private_query_bridge_direct_methods.inc>",
            "// Adaptive cover build orchestration.",
            "#include <SBF/detail/planning_forest_private_adaptive_build_methods.inc>",
            "// Query bridge paving, HiPaC, and batch helpers.",
            "#include <SBF/detail/planning_forest_private_query_bridge_batch_methods.inc>",
            "// Core forest topology and adaptive partition helpers.",
            "#include <SBF/detail/planning_forest_private_topology_methods.inc>",
            "#include <SBF/detail/planning_forest_diagnostic_partition_methods.inc>",
            "// OBB and portal corridor helpers.",
            "#include <SBF/detail/planning_forest_private_overlay_methods.inc>",
            "// Query cache helpers.",
            "#include <SBF/detail/planning_forest_private_cache_methods.inc>",
            "#include <SBF/detail/planning_forest_diagnostic_cache_methods.inc>",
            "// Core state.",
        ),
    )
    require_terms_guarded(
        forest_header,
        (
            "struct DebugChainPaveResult;",
            "struct RebuildProfile;",
            "struct SubtractiveBuildOptions;",
            "struct SubtractiveObstacleGroup;",
            "DynamicCollisionCacheState",
            "#include <SBF/detail/planning_forest_diagnostic_public_methods.inc>",
            "#include <SBF/detail/planning_forest_diagnostic_partition_methods.inc>",
            "#include <SBF/detail/planning_forest_diagnostic_cache_methods.inc>",
            "#include <SBF/detail/planning_forest_diagnostic_state.inc>",
        ),
    )
    for term in (
        "build_subtractive",
        "debug_chain_pave(",
        "debug_chain_pave_waypoints(",
        "refine_query_corridor(",
        "connect_update_segment_fallback",
        "connect_update_endpoint_segment_fallback",
        "add_obstacle_and_rebuild",
        "add_obstacles_and_rebuild",
        "remove_obstacle_and_regrow",
        "remove_obstacle_suffix_and_regrow",
        "refresh_dynamic_partition_after",
        "populate_dynamic_collision_cache",
        "clear_dynamic_collision_cache",
        "rebuild_dynamic_collision_cache_index",
        "add_dynamic_collision_cache_box",
        "promote_unblocked_collision_cache",
        "refill_removed_box_with_leaf_sweep",
        "dynamic_collision_cache_",
    ):
        if term in forest_header_text:
            raise RuntimeError(
                f"{forest_header} must route diagnostic declaration term {term} through detail fragments"
            )
    require_ordered_markers(
        diagnostic_public_fragment,
        diagnostic_public_text,
        (
            "const OracleCounters* oracle_counters() const;",
            "BuildProfile build_subtractive",
            "DebugChainPaveResult debug_chain_pave(",
            "DebugChainPaveResult debug_chain_pave_waypoints(",
            "int refine_query_corridor(",
            "RebuildProfile connect_update_segment_fallback();",
            "RebuildProfile connect_update_endpoint_segment_fallback(",
            "RebuildProfile add_obstacle_and_rebuild(",
            "RebuildProfile add_obstacles_and_rebuild(",
            "RebuildProfile remove_obstacle_and_regrow(",
            "RebuildProfile remove_obstacle_suffix_and_regrow(",
        ),
    )
    require_ordered_markers(
        diagnostic_partition_fragment,
        diagnostic_fragment_texts[diagnostic_partition_fragment],
        (
            "void refresh_adaptive_partition_diagnostics(RebuildProfile& profile) const;",
            "void refresh_dynamic_partition_after_update(",
            "void refresh_dynamic_partition_after_append(",
            "void refresh_dynamic_partition_after_remove_append(",
        ),
    )
    require_ordered_markers(
        diagnostic_cache_fragment,
        diagnostic_fragment_texts[diagnostic_cache_fragment],
        (
            "void initialize_dynamic_collision_cache();",
            "int dynamic_collision_cache_box_count() const;",
            "void populate_dynamic_collision_cache(",
            "void clear_dynamic_collision_cache();",
            "void rebuild_dynamic_collision_cache_index();",
            "void add_dynamic_collision_cache_box(",
            "int promote_unblocked_collision_cache(",
            "int refill_removed_box_with_leaf_sweep(",
        ),
    )
    if "dynamic_collision_cache_" not in diagnostic_fragment_texts[diagnostic_state_fragment]:
        raise RuntimeError(f"{diagnostic_state_fragment} must own the diagnostic cache state member")
    private_fragment_markers = {
        private_entry_fragment: (
            "FindFreeBoxResult find_free_box_in_domain(",
            "QueryResult run_query_internal(",
            "int anchor_query_endpoint_box(",
            "int locate_box_partition_first(",
        ),
        private_query_bridge_direct_fragment: (
            "int locate_query_bridge_box(",
            "int try_query_bridge_direct_ffb_corridor(",
            "void run_query_bridge_direct_corridor_residual_segments(",
            "int finish_query_bridge_direct_corridor_attempt(",
            "int try_add_query_direct_corridor_full_residual_edge(",
        ),
        private_adaptive_build_fragment: (
            "AdaptiveLeafSweepResult build_fixed_virtual_leaf_sweep_cover(",
            "AdaptiveLeafSweepResult build_adaptive_fast_virtual_checkpoint_cover(",
            "void finalize_adaptive_deep_leaf_sweep_cover_result(",
            "AdaptiveDepthSnapshot evaluate_adaptive_depth_snapshot(",
        ),
        private_query_bridge_batch_fragment: (
            "std::pair<int, int> locate_query_bridge_boxes(",
            "int run_query_bridge_chain_pave(",
            "std::vector<QueryBridgeSearchTask> prepare_query_bridge_batch_tasks(",
            "int try_promote_query_repair_to_hipac(",
        ),
        private_topology_fragment: (
            "void reset_oracle(",
            "void rebuild_adaptive_partition(",
            "int add_segment_edge_partition_first(",
            "bool try_add_endpoint_main_residual_segment_edge(",
        ),
        private_overlay_fragment: (
            "int try_add_clearance_retry_obb_edge(",
            "bool build_cell_native_portal_corridor_chain(",
            "bool build_ffb_portal_corridor_chain(",
        ),
        private_cache_fragment: (
            "void invalidate_query_cache() const;",
            "void initialize_optional_collision_cache();",
            "void populate_optional_collision_cache_from_leaf_sweep(",
            "void record_optional_collision_cache_box_count(",
        ),
    }
    for path, markers in private_fragment_markers.items():
        require_ordered_markers(path, private_fragment_texts[path], markers)
        for marker in markers:
            if marker in forest_header_text:
                raise RuntimeError(f"{forest_header} must route private declaration {marker} through {path.name}")
    explicit_debug_payload_consumers = (
        repo_root / "safe_box_forest" / "src" / "diagnostic" / "connector_chain_pave_debug.cpp",
        repo_root / "safe_box_forest" / "src" / "diagnostic" / "planning_forest_debug.cpp",
        repo_root / "safe_box_forest" / "python" / "diagnostic" / "binding_debug_chain_pave_result.h",
        repo_root / "safe_box_forest" / "tests" / "test_facade_surface.cpp",
    )
    for path in explicit_debug_payload_consumers:
        text = path.read_text(encoding="utf-8")
        if "#include <SBF/debug.h>" not in text:
            raise RuntimeError(f"{path} must explicitly include SBF/debug.h when inspecting debug payloads")
    planning_core_text = (
        repo_root / "safe_box_forest" / "src" / "planning_core" / "planning_forest_core.cpp"
    ).read_text(encoding="utf-8")
    planning_debug_text = (
        repo_root / "safe_box_forest" / "src" / "diagnostic" / "planning_forest_debug.cpp"
    ).read_text(encoding="utf-8")
    if "RBFPlanningForest::oracle_counters" in planning_core_text:
        raise RuntimeError("diagnostic oracle_counters facade implementation must not live in planning_forest_core.cpp")
    if "RBFPlanningForest::oracle_counters" not in planning_debug_text:
        raise RuntimeError("diagnostic oracle_counters facade implementation must live in planning_forest_debug.cpp")
    explicit_diagnostic_result_consumers = (
        repo_root / "safe_box_forest" / "src" / "diagnostic" / "planning_forest_dynamic_cache.cpp",
        repo_root / "safe_box_forest" / "src" / "diagnostic" / "planning_forest_dynamic_collision_cache.cpp",
        repo_root / "safe_box_forest" / "src" / "diagnostic" / "planning_forest_dynamic_refill.cpp",
        repo_root / "safe_box_forest" / "src" / "diagnostic" / "planning_forest_dynamic_remove.cpp",
        repo_root / "safe_box_forest" / "src" / "diagnostic" / "planning_forest_dynamic_segment_endpoint.cpp",
        repo_root / "safe_box_forest" / "src" / "diagnostic" / "planning_forest_dynamic_segment_fallback.cpp",
        repo_root / "safe_box_forest" / "python" / "diagnostic" / "binding_planning_forest_diagnostic_facade_methods.h",
    )
    for path in explicit_diagnostic_result_consumers:
        text = path.read_text(encoding="utf-8")
        if "#include <SBF/diagnostic_result.h>" not in text:
            raise RuntimeError(f"{path} must explicitly include SBF/diagnostic_result.h when inspecting RebuildProfile")
    dynamic_partition_source = repo_root / "safe_box_forest" / "src" / "diagnostic" / "planning_forest_dynamic_partition.cpp"
    dynamic_partition_text = dynamic_partition_source.read_text(encoding="utf-8")
    if "#include <SBF/diagnostic_result.h>" not in dynamic_partition_text:
        raise RuntimeError(f"{dynamic_partition_source} must explicitly include SBF/diagnostic_result.h")
    for term in (
        "refresh_adaptive_partition_diagnostics(RebuildProfile&",
        "refresh_dynamic_partition_after_update",
        "refresh_dynamic_partition_after_append",
        "refresh_dynamic_partition_after_remove_append",
    ):
        if term not in dynamic_partition_text:
            raise RuntimeError(f"{dynamic_partition_source} must own diagnostic partition helper {term}")
    explicit_subtractive_config_consumers = (
        repo_root / "safe_box_forest" / "src" / "diagnostic" / "planning_forest_subtractive.cpp",
        repo_root / "safe_box_forest" / "python" / "diagnostic" / "binding_diagnostic_types.h",
        repo_root / "safe_box_forest" / "python" / "diagnostic" / "binding_planning_forest_diagnostic_facade_methods.h",
    )
    for path in explicit_subtractive_config_consumers:
        text = path.read_text(encoding="utf-8")
        if "#include <SBF/subtractive_build_config.h>" not in text:
            raise RuntimeError(
                f"{path} must explicitly include SBF/subtractive_build_config.h when using subtractive payloads"
            )
    forbidden_facade_includes = (
        "#include <SBF/box_graph.h>",
        "#include <SBF/grower_types.h>",
        "#include <SBF/build_config.h>",
        "#include <SBF/query.h>",
        "#include <SBF/runtime.h>",
        "#include <LECTDatabase/sbf/oracle_types.h>",
        "#include <LECTDatabase/online_cache.h>",
        "#include <rbf/lect_database.h>",
    )
    for include in forbidden_facade_includes:
        if include in forest_header_text:
            raise RuntimeError(f"{forest_header}: facade must not directly include {include}")
    required_facade_includes = (
        "#include <SBF/adaptive_leaf_sweep_config.h>",
        "#include <SBF/box_adjacency_types.h>",
        "#include <SBF/connector_types.h>",
        "#include <SBF/find_free_box_types.h>",
        "#include <SBF/leaf_sweep_types.h>",
        "#include <SBF/planning_config.h>",
        "#include <SBF/planning_result.h>",
        "#include <SBF/query_bridge_config.h>",
        "#include <SBF/query_graph_cache_types.h>",
        "#include <SBF/query_result.h>",
        "#include <SBF/query_runtime_config.h>",
        "#include <SBF/runtime_fwd.h>",
        "#include <SBF/scene.h>",
        "#include <SBF/segment_edge_types.h>",
    )
    for include in required_facade_includes:
        if include not in forest_header_text:
            raise RuntimeError(f"{forest_header}: facade must directly include signature type header {include}")
    if "#include <SBF/diagnostic_build_config.h>" in forest_header_text:
        raise RuntimeError(f"{forest_header}: facade must include diagnostic split headers, not diagnostic_build_config.h")
    if "namespace rbf::lect_database" not in forest_header_text:
        raise RuntimeError(f"{forest_header}: facade LECT forward declarations must use rbf::lect_database")
    if "namespace lect_database {" in forest_header_text:
        raise RuntimeError(f"{forest_header}: facade must not forward declare LECT types in the global namespace")
    if "class SharedEndpointEvidenceCache;" not in forest_header_text:
        raise RuntimeError(f"{forest_header}: facade must forward declare SharedEndpointEvidenceCache")
    scene_types_header = include_dir / "scene_types.h"
    scene_types_text = scene_types_header.read_text(encoding="utf-8")
    for include in (
        "#include <link_interval_envelope/robot.h>",
        "#include <link_interval_envelope/types.h>",
    ):
        if include not in scene_types_text:
            raise RuntimeError(f"{scene_types_header}: scene type facade must include {include}")
    for declaration in (
        "class CollisionChecker;",
        "class Scene;",
    ):
        if declaration not in scene_types_text:
            raise RuntimeError(f"{scene_types_header}: scene type facade must forward declare {declaration}")
    if "#include <SBF/scene.h>" in scene_types_text:
        raise RuntimeError(f"{scene_types_header}: scene type facade must not include full SBF/scene.h")
    build_config_text = (include_dir / "build_config.h").read_text(encoding="utf-8")
    for include in (
        "#include <SBF/adaptive_leaf_sweep_config.h>",
        "#include <SBF/database_runtime_config.h>",
    ):
        if include not in build_config_text:
            raise RuntimeError(f"SBF/build_config.h must remain a compatibility aggregate including {include}")
    for include in (
        "#include <SBF/box_graph.h>",
        "#include <SBF/scene.h>",
        "#include <LECTDatabase/online_cache.h>",
        "#include <rbf/lect_database.h>",
    ):
        if include in build_config_text:
            raise RuntimeError(f"SBF/build_config.h must not directly include heavy dependency {include}")
    for term in (
        "struct LeafSweepRefineConfig",
        "struct AdaptiveLeafSweepConfig",
        "struct LectDatabaseRuntimeConfig",
    ):
        if term in build_config_text:
            raise RuntimeError(f"SBF/build_config.h must not define {term}; use the narrow type header")
    adaptive_leaf_config_header = include_dir / "adaptive_leaf_sweep_config.h"
    adaptive_leaf_config_text = adaptive_leaf_config_header.read_text(encoding="utf-8")
    for term in (
        "struct LeafSweepRefineConfig",
        "struct AdaptiveLeafSweepConfig",
    ):
        if term not in adaptive_leaf_config_text:
            raise RuntimeError(f"{adaptive_leaf_config_header} must define {term}")
    grower_header_text = (include_dir / "grower.h").read_text(encoding="utf-8")
    if "#include <SBF/find_free_box_types.h>" not in grower_header_text:
        raise RuntimeError("SBF/grower.h must include SBF/find_free_box_types.h for FFB signature records")
    if "#include <SBF/find_free_box.h>" in grower_header_text:
        raise RuntimeError("SBF/grower.h must not include the full find-free-box service entry header")
    if "class FindFreeBoxService;" not in grower_header_text:
        raise RuntimeError("SBF/grower.h must forward declare FindFreeBoxService instead of including the service header")
    ffb_config_header = include_dir / "find_free_box_config.h"
    ffb_config_text = ffb_config_header.read_text(encoding="utf-8")
    ffb_types_text = (include_dir / "find_free_box_types.h").read_text(encoding="utf-8")
    for term in (
        "enum class FindFreeBoxSearchMode",
        "struct FindFreeBoxOptions",
    ):
        if term not in ffb_config_text:
            raise RuntimeError(f"SBF/find_free_box_config.h must define {term}")
        if term in ffb_types_text:
            raise RuntimeError(f"SBF/find_free_box_types.h must not define {term}; use SBF/find_free_box_config.h")
    if "#include <SBF/find_free_box_config.h>" not in ffb_types_text:
        raise RuntimeError("SBF/find_free_box_types.h must include SBF/find_free_box_config.h")
    if "struct FindFreeBoxResult" not in ffb_types_text:
        raise RuntimeError("SBF/find_free_box_types.h must define FindFreeBoxResult")
    connector_types_text = (include_dir / "connector_types.h").read_text(encoding="utf-8")
    if "#include <SBF/find_free_box_config.h>" not in connector_types_text:
        raise RuntimeError("SBF/connector_types.h must include SBF/find_free_box_config.h for ChainPaveConfig")
    if "#include <SBF/find_free_box_types.h>" in connector_types_text:
        raise RuntimeError("SBF/connector_types.h must not include FFB result payloads")
    if "#include <SBF/box_graph_types.h>" in connector_types_text:
        raise RuntimeError("SBF/connector_types.h must not include graph/cache/segment-edge payloads")
    connector_internal_text = (
        repo_root / "safe_box_forest" / "src" / "connector" / "connector_internal.h"
    ).read_text(encoding="utf-8")
    if "#include <SBF/find_free_box_types.h>" not in connector_internal_text:
        raise RuntimeError("connector_internal.h must include SBF/find_free_box_types.h for FFB result helpers")
    if "#include <SBF/box_adjacency_types.h>" not in connector_internal_text:
        raise RuntimeError("connector_internal.h must include SBF/box_adjacency_types.h for graph helper signatures")
    if "#include <LECTDatabase/sbf/oracle_types.h>" not in connector_internal_text:
        raise RuntimeError("connector_internal.h must include LECTDatabase/sbf/oracle_types.h for BoxOracle signatures")
    if "#include <SBF/oracle.h>" in connector_internal_text:
        raise RuntimeError("connector_internal.h must include oracle type declarations instead of full SBF/oracle.h")
    if "#include <SBF/box_graph_types.h>" in connector_internal_text:
        raise RuntimeError("connector_internal.h must include narrow graph types instead of SBF/box_graph_types.h")
    connector_private_graph_headers = (
        "connector_frontier_bridge.h",
        "connector_pair_commit.h",
    )
    for header_name in connector_private_graph_headers:
        header_text = (
            repo_root / "safe_box_forest" / "src" / "connector" / header_name
        ).read_text(encoding="utf-8")
        if "#include <SBF/box_adjacency_types.h>" not in header_text:
            raise RuntimeError(f"{header_name} must include SBF/box_adjacency_types.h for graph helper signatures")
        if "#include <SBF/segment_edge_fwd.h>" not in header_text:
            raise RuntimeError(f"{header_name} must include SBF/segment_edge_fwd.h for segment-edge signatures")
        if "#include <SBF/segment_edge_types.h>" in header_text:
            raise RuntimeError(f"{header_name} must include segment-edge forward declarations instead of payloads")
        if "#include <SBF/box_graph_types.h>" in header_text:
            raise RuntimeError(f"{header_name} must include narrow graph types instead of SBF/box_graph_types.h")
    connector_private_scene_headers = (
        "connector_birrt.h",
        "connector_pair_candidates.h",
        "connector_pair_tasks.h",
    )
    for header_name in connector_private_scene_headers:
        header_text = (
            repo_root / "safe_box_forest" / "src" / "connector" / header_name
        ).read_text(encoding="utf-8")
        if "#include <SBF/scene_types.h>" not in header_text:
            raise RuntimeError(f"{header_name} must include SBF/scene_types.h for Robot/BoxNode records")
    public_headers_requiring_oracle_types = (
        "connector.h",
        "grower.h",
        "leaf_sweep_grower.h",
        "merger.h",
    )
    for header_name in public_headers_requiring_oracle_types:
        header_text = (include_dir / header_name).read_text(encoding="utf-8")
        if "#include <LECTDatabase/sbf/oracle_types.h>" not in header_text:
            raise RuntimeError(f"SBF/{header_name} must include LECTDatabase/sbf/oracle_types.h for oracle signatures")
        if "#include <SBF/oracle.h>" in header_text:
            raise RuntimeError(f"SBF/{header_name} must not include the full oracle class header")
    runtime_fwd_header = include_dir / "runtime_fwd.h"
    runtime_fwd_text = runtime_fwd_header.read_text(encoding="utf-8")
    if "class StageContext;" not in runtime_fwd_text:
        raise RuntimeError("SBF/runtime_fwd.h must forward declare StageContext")
    public_headers_requiring_runtime_fwd = (
        "connector.h",
        "find_free_box.h",
        "grower.h",
        "leaf_sweep_grower.h",
        "merger.h",
        "safe_box_forest.h",
    )
    for header_name in public_headers_requiring_runtime_fwd:
        header_text = (include_dir / header_name).read_text(encoding="utf-8")
        if "#include <SBF/runtime_fwd.h>" not in header_text:
            raise RuntimeError(f"SBF/{header_name} must include SBF/runtime_fwd.h for StageContext signatures")
        if "#include <SBF/runtime.h>" in header_text:
            raise RuntimeError(f"SBF/{header_name} must not include the full runtime execution header")
    box_graph_header = include_dir / "box_graph.h"
    box_adjacency_types_header = include_dir / "box_adjacency_types.h"
    query_graph_types_header = include_dir / "query_graph_types.h"
    query_graph_cache_types_header = include_dir / "query_graph_cache_types.h"
    box_graph_types_header = include_dir / "box_graph_types.h"
    for header in (
        box_adjacency_types_header,
        query_graph_types_header,
        query_graph_cache_types_header,
        box_graph_types_header,
    ):
        if not header.is_file():
            raise RuntimeError(f"SBF/{header.name} is required for graph/cache/path type records")
    box_graph_text = box_graph_header.read_text(encoding="utf-8")
    box_adjacency_types_text = box_adjacency_types_header.read_text(encoding="utf-8")
    query_graph_types_text = query_graph_types_header.read_text(encoding="utf-8")
    query_graph_cache_types_text = query_graph_cache_types_header.read_text(encoding="utf-8")
    box_graph_types_text = box_graph_types_header.read_text(encoding="utf-8")
    if "#include <SBF/box_graph_types.h>" in box_graph_text:
        raise RuntimeError("SBF/box_graph.h must include narrow graph headers instead of SBF/box_graph_types.h")
    for include in (
        "#include <SBF/box_adjacency_types.h>",
        "#include <SBF/query_graph_cache_types.h>",
        "#include <SBF/query_graph_types.h>",
        "#include <SBF/segment_edge_fwd.h>",
    ):
        if include not in box_graph_text:
            raise RuntimeError(f"SBF/box_graph.h must include narrow graph dependency {include}")
    if "#include <SBF/box_graph.h>" in box_graph_types_text:
        raise RuntimeError("SBF/box_graph_types.h must not include the full graph algorithm header")
    for include in (
        "#include <SBF/box_adjacency_types.h>",
        "#include <SBF/query_graph_cache_types.h>",
        "#include <SBF/query_graph_types.h>",
    ):
        if include not in box_graph_types_text:
            raise RuntimeError(f"SBF/box_graph_types.h must remain a compatibility aggregate including {include}")
    for term in (
        "using AdjacencyGraph",
        "struct AdjacencyBuildStats",
    ):
        if term not in box_adjacency_types_text:
            raise RuntimeError(f"SBF/box_adjacency_types.h must define {term}")
        if term in box_graph_types_text:
            raise RuntimeError(f"SBF/box_graph_types.h must not define {term}; use SBF/box_adjacency_types.h")
        if term in box_graph_text:
            raise RuntimeError(f"SBF/box_graph.h must not define {term}; use SBF/box_adjacency_types.h")
    if "struct QueryGraphCache" not in query_graph_cache_types_text:
        raise RuntimeError("SBF/query_graph_cache_types.h must define QueryGraphCache")
    if "struct QueryGraphCache" in box_graph_types_text:
        raise RuntimeError("SBF/box_graph_types.h must not define QueryGraphCache; use SBF/query_graph_cache_types.h")
    if "#include <SBF/box_adjacency_types.h>" not in query_graph_cache_types_text:
        raise RuntimeError("SBF/query_graph_cache_types.h must include SBF/box_adjacency_types.h")
    if "#include <SBF/segment_edge_fwd.h>" not in query_graph_cache_types_text:
        raise RuntimeError("SBF/query_graph_cache_types.h must include SBF/segment_edge_fwd.h for SegmentEdgeList")
    for term in (
        "struct DijkstraResult",
        "struct QueryGraphCostOptions",
        "struct QueryShortcutCostOptions",
    ):
        if term not in query_graph_types_text:
            raise RuntimeError(f"SBF/query_graph_types.h must define {term}")
        if term in box_graph_types_text:
            raise RuntimeError(f"SBF/box_graph_types.h must not define {term}; use SBF/query_graph_types.h")
        if term in box_graph_text:
            raise RuntimeError(f"SBF/box_graph.h must not define {term}; use SBF/query_graph_types.h")
    public_headers_requiring_adjacency_types = (
        "connector.h",
        "grower_types.h",
        "safe_box_forest.h",
    )
    for header_name in public_headers_requiring_adjacency_types:
        header_text = (include_dir / header_name).read_text(encoding="utf-8")
        if "#include <SBF/box_adjacency_types.h>" not in header_text:
            raise RuntimeError(f"SBF/{header_name} must include SBF/box_adjacency_types.h for adjacency records")
        if "#include <SBF/box_graph.h>" in header_text:
            raise RuntimeError(f"SBF/{header_name} must not directly include the graph algorithm entry header")
        if "#include <SBF/box_graph_types.h>" in header_text:
            raise RuntimeError(f"SBF/{header_name} must include the narrow graph type header instead of SBF/box_graph_types.h")
    query_header_text = (include_dir / "query.h").read_text(encoding="utf-8")
    for include in (
        "#include <SBF/box_adjacency_types.h>",
        "#include <SBF/query_graph_cache_types.h>",
        "#include <SBF/query_graph_types.h>",
        "#include <SBF/segment_edge_fwd.h>",
    ):
        if include not in query_header_text:
            raise RuntimeError(f"SBF/query.h must include narrow query graph dependency {include}")
    adaptive_partition_types_text = (include_dir / "adaptive_grid_partition_types.h").read_text(encoding="utf-8")
    if "#include <SBF/query_graph_types.h>" not in adaptive_partition_types_text:
        raise RuntimeError("SBF/adaptive_grid_partition_types.h must include SBF/query_graph_types.h")
    if "#include <SBF/box_graph_types.h>" in adaptive_partition_types_text:
        raise RuntimeError("SBF/adaptive_grid_partition_types.h must not include graph cache/segment-edge payloads")
    if "#include <rbf/core.h>" not in adaptive_partition_types_text:
        raise RuntimeError("SBF/adaptive_grid_partition_types.h must include rbf/core.h directly for Interval records")
    merger_header_text = (include_dir / "merger.h").read_text(encoding="utf-8")
    if "#include <SBF/box_graph_types.h>" in merger_header_text:
        raise RuntimeError("SBF/merger.h must not include graph/cache/segment-edge payloads")
    if "#include <rbf/core.h>" not in merger_header_text:
        raise RuntimeError("SBF/merger.h must include rbf/core.h directly for BoxNode/Interval signatures")
    leaf_sweep_types_text = (include_dir / "leaf_sweep_types.h").read_text(encoding="utf-8")
    if "#include <SBF/box_graph_types.h>" in leaf_sweep_types_text:
        raise RuntimeError("SBF/leaf_sweep_types.h must not include graph/cache/segment-edge payloads")
    if "#include <rbf/core.h>" not in leaf_sweep_types_text:
        raise RuntimeError("SBF/leaf_sweep_types.h must include rbf/core.h directly for BoxNode records")
    for private_header in (repo_root / "safe_box_forest" / "src").rglob("*.h"):
        private_header_text = private_header.read_text(encoding="utf-8")
        if "#include <SBF/box_graph_types.h>" in private_header_text:
            rel_header = private_header.relative_to(repo_root / "safe_box_forest")
            raise RuntimeError(
                f"{rel_header} must include narrow graph/core headers instead of SBF/box_graph_types.h"
            )
    private_graph_header_requirements = {
        "src/planning_adaptive/planning_forest_adaptive_cover_utils.h": (
            "#include <SBF/box_adjacency_types.h>",
            "#include <rbf/core.h>",
        ),
        "src/planning_adaptive/planning_forest_adaptive_merge.h": (
            "#include <rbf/core.h>",
        ),
        "src/planning_adaptive/planning_forest_adaptive_merge_grid.h": (
            "#include <rbf/core.h>",
        ),
        "src/qroot/planning_forest_qroot_growers.h": (
            "#include <SBF/box_adjacency_types.h>",
            "#include <rbf/core.h>",
        ),
        "src/qroot/planning_forest_qroot_helpers.h": (
            "#include <SBF/box_adjacency_types.h>",
            "#include <rbf/core.h>",
        ),
        "src/diagnostic/planning_forest_dynamic_collision_cache_state.h": (
            "#include <rbf/core.h>",
        ),
        "src/diagnostic/planning_forest_dynamic_helpers.h": (
            "#include <SBF/box_adjacency_types.h>",
            "#include <rbf/core.h>",
        ),
        "src/diagnostic/planning_forest_dynamic_segment_fallback_helpers.h": (
            "#include <rbf/core.h>",
        ),
        "src/diagnostic/planning_forest_subtractive_seeds.h": (
            "#include <SBF/box_adjacency_types.h>",
            "#include <rbf/core.h>",
        ),
        "src/planning_build/planning_forest_ffb_helpers.h": (
            "#include <rbf/core.h>",
        ),
        "src/query_bridge/planning_forest_query_bridge_corridor_graph.h": (
            "#include <rbf/core.h>",
        ),
        "src/query_bridge/planning_forest_query_bridge_endpoint_targets.h": (
            "#include <rbf/core.h>",
        ),
        "src/query_bridge/planning_forest_query_bridge_endpoint_runtime.h": (
            "#include <SBF/box_adjacency_types.h>",
            "#include <rbf/core.h>",
        ),
        "src/query_bridge/planning_forest_query_bridge_endpoint_index.h": (
            "#include <rbf/core.h>",
        ),
    }
    for rel_header, required_includes in private_graph_header_requirements.items():
        header_text = (repo_root / "safe_box_forest" / rel_header).read_text(encoding="utf-8")
        for include in required_includes:
            if include not in header_text:
                raise RuntimeError(f"{rel_header} must include {include} for its declared graph/core records")
    private_scene_header_requirements = {
        "src/diagnostic/planning_forest_dynamic_helpers.h": (
            "#include <SBF/scene_types.h>",
        ),
        "src/obb/planning_forest_obb.h": (
            "#include <SBF/scene_types.h>",
        ),
        "src/planning_core/planning_forest_audit.h": (
            "#include <SBF/scene_types.h>",
        ),
        "src/query_bridge/planning_forest_query_bridge_attempt_paths.h": (
            "#include <SBF/scene_types.h>",
        ),
        "src/query_bridge/planning_forest_query_bridge_path_utils.h": (
            "#include <SBF/scene_types.h>",
        ),
        "src/query_bridge/planning_forest_query_bridge_rrt_utils.h": (
            "#include <SBF/scene_types.h>",
        ),
        "src/query_runtime/planning_forest_query_repair.h": (
            "#include <SBF/scene_types.h>",
        ),
        "src/query_runtime/planning_forest_query_utils.h": (
            "#include <SBF/scene_types.h>",
        ),
    }
    for rel_header, required_includes in private_scene_header_requirements.items():
        header_text = (repo_root / "safe_box_forest" / rel_header).read_text(encoding="utf-8")
        for include in required_includes:
            if include not in header_text:
                raise RuntimeError(f"{rel_header} must include {include} for scene-facing signatures")
    api_header = include_dir / "api.h"
    api_text = api_header.read_text(encoding="utf-8")
    api_aggregate_includes = (
        "#include <SBF/build_profile.h>",
        "#include <SBF/query_result.h>",
        "#include <SBF/runtime_config.h>",
        "#include <SBF/segment_edge_types.h>",
    )
    for include in api_aggregate_includes:
        if include not in api_text:
            raise RuntimeError(f"SBF/api.h must remain a compatibility aggregate including {include}")
    moved_api_terms = {
        "build_profile.h": ("struct BuildProfile",),
        "query_result.h": ("enum class PathAuditStatus", "struct QueryResult"),
        "runtime_config.h": ("enum class ExecutionMode", "struct RuntimeConfig"),
        "segment_edge_fwd.h": (
            "enum class SegmentEdgeType",
            "enum class SegmentEdgeValidation",
            "using SegmentEdgeList",
        ),
        "segment_edge_types.h": (
            "#include <SBF/segment_edge_fwd.h>",
            "struct SegmentEdge",
        ),
    }
    for header_name, terms in moved_api_terms.items():
        header_text = (include_dir / header_name).read_text(encoding="utf-8")
        if "#include <SBF/api.h>" in header_text:
            raise RuntimeError(f"SBF/{header_name} must not include the compatibility aggregate SBF/api.h")
        for term in terms:
            if term not in header_text:
                raise RuntimeError(f"SBF/{header_name} must define {term}")
            if term in api_text:
                raise RuntimeError(f"SBF/api.h must not define {term}; use SBF/{header_name}")
    public_or_private_headers_with_api_includes: list[str] = []
    for path in sorted((repo_root / "safe_box_forest" / "include" / "SBF").glob("*.h")):
        if path.name == "api.h":
            continue
        header_text = path.read_text(encoding="utf-8")
        if "#include <SBF/api.h>" in header_text:
            public_or_private_headers_with_api_includes.append(f"include/SBF/{path.name}")
    all_sbf_private_headers = {
        path.relative_to(repo_root / "safe_box_forest").as_posix()
        for path in (repo_root / "safe_box_forest" / "src").rglob("*.h")
    }
    for header in sorted(all_sbf_private_headers):
        header_text = (repo_root / "safe_box_forest" / header).read_text(encoding="utf-8")
        if "#include <SBF/api.h>" in header_text:
            public_or_private_headers_with_api_includes.append(header)
    if public_or_private_headers_with_api_includes:
        raise RuntimeError(
            "SBF headers must include narrow type headers instead of the compatibility aggregate SBF/api.h: "
            + ", ".join(public_or_private_headers_with_api_includes)
        )
    package_entry_header = include_dir / "sbf.h"
    package_entry_text = package_entry_header.read_text(encoding="utf-8")
    if "#include <SBF/safe_box_forest.h>" not in package_entry_text:
        raise RuntimeError("SBF/sbf.h must remain the public package entry for SBF/safe_box_forest.h")
    package_entry_include_users: list[str] = []
    for root in (
        repo_root / "safe_box_forest" / "src",
        repo_root / "safe_box_forest" / "python",
        repo_root / "safe_box_forest" / "tests",
    ):
        for path in sorted(root.rglob("*")):
            if path.suffix not in {".h", ".cpp", ".py"}:
                continue
            text = path.read_text(encoding="utf-8")
            if "#include <SBF/sbf.h>" in text or '#include "SBF/sbf.h"' in text:
                package_entry_include_users.append(path.relative_to(repo_root).as_posix())
    if package_entry_include_users:
        raise RuntimeError(
            "production, binding, and test code must include SBF/safe_box_forest.h or narrower headers "
            "instead of the package entry SBF/sbf.h: "
            + ", ".join(package_entry_include_users)
        )
    planning_forest_header = include_dir / "planning_forest.h"
    planning_forest_text = planning_forest_header.read_text(encoding="utf-8")
    if "#include <SBF/safe_box_forest.h>" not in planning_forest_text:
        raise RuntimeError("SBF/planning_forest.h must remain a compatibility alias for SBF/safe_box_forest.h")
    planning_forest_alias_users: list[str] = []
    for root in (
        repo_root / "safe_box_forest" / "include" / "SBF",
        repo_root / "safe_box_forest" / "src",
        repo_root / "safe_box_forest" / "python",
        repo_root / "safe_box_forest" / "tests",
    ):
        for path in sorted(root.rglob("*")):
            if path == planning_forest_header or path.suffix not in {".h", ".cpp", ".py"}:
                continue
            text = path.read_text(encoding="utf-8")
            if "#include <SBF/planning_forest.h>" in text or '#include "SBF/planning_forest.h"' in text:
                planning_forest_alias_users.append(path.relative_to(repo_root).as_posix())
    if planning_forest_alias_users:
        raise RuntimeError(
            "production, binding, and test code must include SBF/safe_box_forest.h directly "
            "instead of the legacy SBF/planning_forest.h alias: "
            + ", ".join(planning_forest_alias_users)
        )
    detail_header = include_dir / "detail.h"
    detail_text = detail_header.read_text(encoding="utf-8")
    if "#define RBF_PLANNING_DETAIL_INCLUDED 1" not in detail_text:
        raise RuntimeError("SBF/detail.h must remain an explicit legacy detail aggregate")
    for include in (
        "#include <SBF/runtime.h>",
        "#include <SBF/scene.h>",
        "#include <SBF/box_graph.h>",
        "#include <SBF/oracle.h>",
        "#include <SBF/find_free_box.h>",
        "#include <SBF/query.h>",
        "#include <SBF/merger.h>",
        "#include <SBF/connector.h>",
        "#include <SBF/grower.h>",
        "#include <SBF/safe_box_forest.h>",
    ):
        if include not in detail_text:
            raise RuntimeError(f"SBF/detail.h compatibility aggregate must include {include}")
    detail_include_users: list[str] = []
    for root in (
        repo_root / "safe_box_forest" / "include" / "SBF",
        repo_root / "safe_box_forest" / "src",
        repo_root / "safe_box_forest" / "python",
        repo_root / "safe_box_forest" / "tests",
    ):
        for path in sorted(root.rglob("*")):
            if path == detail_header or path.suffix not in {".h", ".cpp", ".py"}:
                continue
            text = path.read_text(encoding="utf-8")
            if "#include <SBF/detail.h>" in text or '#include "SBF/detail.h"' in text:
                detail_include_users.append(path.relative_to(repo_root).as_posix())
    if detail_include_users:
        raise RuntimeError(
            "production, binding, and test code must include narrow headers instead of legacy SBF/detail.h: "
            + ", ".join(detail_include_users)
        )
    runtime_header_text = (include_dir / "runtime.h").read_text(encoding="utf-8")
    if "#include <SBF/runtime_config.h>" not in runtime_header_text:
        raise RuntimeError("SBF/runtime.h must include SBF/runtime_config.h for RuntimeConfig")
    if "#include <SBF/api.h>" in runtime_header_text:
        raise RuntimeError("SBF/runtime.h must not include the compatibility aggregate SBF/api.h")
    query_graph_cache_types_text_for_api = (include_dir / "query_graph_cache_types.h").read_text(encoding="utf-8")
    if "#include <SBF/segment_edge_fwd.h>" not in query_graph_cache_types_text_for_api:
        raise RuntimeError("SBF/query_graph_cache_types.h must include SBF/segment_edge_fwd.h for SegmentEdgeList")
    adaptive_partition_header = include_dir / "adaptive_grid_partition.h"
    adaptive_partition_types_header = include_dir / "adaptive_grid_partition_types.h"
    if not adaptive_partition_types_header.is_file():
        raise RuntimeError("SBF/adaptive_grid_partition_types.h is required for adaptive partition records")
    adaptive_partition_text = adaptive_partition_header.read_text(encoding="utf-8")
    adaptive_partition_types_text = adaptive_partition_types_header.read_text(encoding="utf-8")
    if "#include <SBF/adaptive_grid_partition_types.h>" not in adaptive_partition_text:
        raise RuntimeError("SBF/adaptive_grid_partition.h must include SBF/adaptive_grid_partition_types.h")
    if "#include <SBF/segment_edge_fwd.h>" not in adaptive_partition_text:
        raise RuntimeError("SBF/adaptive_grid_partition.h must include SBF/segment_edge_fwd.h for overlay edge APIs")
    if "#include <SBF/segment_edge_types.h>" in adaptive_partition_text:
        raise RuntimeError("SBF/adaptive_grid_partition.h must include segment-edge forward declarations instead of payloads")
    if "#include <SBF/box_graph_types.h>" in adaptive_partition_text:
        raise RuntimeError("SBF/adaptive_grid_partition.h must not include compatibility graph aggregate SBF/box_graph_types.h")
    for include in (
        "#include <SBF/api.h>",
        "#include <SBF/box_graph.h>",
    ):
        if include in adaptive_partition_text:
            raise RuntimeError(f"SBF/adaptive_grid_partition.h must use the type header instead of {include}")
    if "#include <SBF/adaptive_grid_partition.h>" in adaptive_partition_types_text:
        raise RuntimeError("SBF/adaptive_grid_partition_types.h must not include the full partition class header")
    for term in (
        "enum class PartitionCellState",
        "struct CellPath",
        "struct GridRange",
        "struct PartitionCell",
        "struct AdaptiveGridPartitionSparseCellRecord",
        "struct AdaptiveGridPartitionStats",
        "struct AdaptiveGridPartitionMergeOptions",
        "struct AdaptiveGridPartitionMergeResult",
        "struct AdaptiveGridPartitionDeltaResult",
        "struct AdaptiveGridPartitionQueryOptions",
        "struct AdaptiveGridPartitionQueryResult",
        "struct AdaptiveGridPartitionNearestBox",
        "struct AdaptiveGridPartitionComponentPair",
        "struct AdaptiveGridPartitionConnectivityDominance",
        "struct AdaptiveGridPartitionLandmark",
    ):
        if term not in adaptive_partition_types_text:
            raise RuntimeError(f"SBF/adaptive_grid_partition_types.h must define {term}")
        if term in adaptive_partition_text:
            raise RuntimeError(f"SBF/adaptive_grid_partition.h must not define {term}; use the type header")
    database_runtime_config_header = include_dir / "database_runtime_config.h"
    database_runtime_config_text = database_runtime_config_header.read_text(encoding="utf-8")
    if "struct LectDatabaseRuntimeConfig" not in database_runtime_config_text:
        raise RuntimeError(f"{database_runtime_config_header} must define LectDatabaseRuntimeConfig")
    if "#include <LECTDatabase/online_cache/config.h>" not in database_runtime_config_text:
        raise RuntimeError(
            f"{database_runtime_config_header} must include the narrow online-cache config header"
        )
    if "#include <LECTDatabase/online_cache.h>" in database_runtime_config_text:
        raise RuntimeError(
            f"{database_runtime_config_header} must not include the full online-cache tree facade"
        )
    online_cache_config_header = repo_root / "lect_database" / "include" / "LECTDatabase" / "online_cache" / "config.h"
    online_cache_cache_tree_header = repo_root / "lect_database" / "include" / "LECTDatabase" / "online_cache" / "cache_tree.h"
    online_cache_config_text = online_cache_config_header.read_text(encoding="utf-8")
    online_cache_cache_tree_text = online_cache_cache_tree_header.read_text(encoding="utf-8")
    if "struct OnlineEnvelopeCacheConfig" not in online_cache_config_text:
        raise RuntimeError(f"{online_cache_config_header} must define OnlineEnvelopeCacheConfig")
    if "#include <LECTDatabase/online_cache/config.h>" not in online_cache_cache_tree_text:
        raise RuntimeError(f"{online_cache_cache_tree_header} must include the narrow config header")
    if "struct OnlineEnvelopeCacheConfig" in online_cache_cache_tree_text:
        raise RuntimeError(
            f"{online_cache_cache_tree_header} must not define OnlineEnvelopeCacheConfig; use config.h"
        )
    dynamic_update_config_header = include_dir / "dynamic_update_config.h"
    subtractive_build_config_header = include_dir / "subtractive_build_config.h"
    diagnostic_build_config_header = include_dir / "diagnostic_build_config.h"
    require_terms_guarded(
        dynamic_update_config_header,
        (
            "DynamicUpdateConfig",
            "enable_spatial_dirty_region",
            "warm_rebuild_dirty_box_threshold",
        ),
    )
    require_terms_guarded(
        subtractive_build_config_header,
        (
            "#include <SBF/scene_types.h>",
            "SubtractiveObstacleGroup",
            "SubtractiveBuildOptions",
        ),
    )
    diagnostic_build_config_text = diagnostic_build_config_header.read_text(encoding="utf-8")
    for include in (
        "#include <SBF/dynamic_update_config.h>",
        "#include <SBF/subtractive_build_config.h>",
    ):
        if include not in diagnostic_build_config_text:
            raise RuntimeError(f"{diagnostic_build_config_header}: compatibility aggregate must include {include}")
    for term in (
        "struct DynamicUpdateConfig",
        "struct SubtractiveObstacleGroup",
        "struct SubtractiveBuildOptions",
    ):
        if term in diagnostic_build_config_text:
            raise RuntimeError(f"{diagnostic_build_config_header}: must not define {term}; use split config headers")
    require_terms_guarded(
        include_dir / "planning_config.h",
        (
            "DynamicUpdateConfig dynamic_update",
        ),
    )
    planning_config_text = (include_dir / "planning_config.h").read_text(encoding="utf-8")
    leaf_sweep_types_text = (include_dir / "leaf_sweep_types.h").read_text(encoding="utf-8")
    for header_name, header_text in (
        ("leaf_sweep_types.h", leaf_sweep_types_text),
        ("subtractive_build_config.h", subtractive_build_config_header.read_text(encoding="utf-8")),
    ):
        if "#include <SBF/scene_types.h>" not in header_text:
            raise RuntimeError(f"SBF/{header_name} must include SBF/scene_types.h for Obstacle records")
        if "#include <SBF/scene.h>" in header_text:
            raise RuntimeError(f"SBF/{header_name} must not include the full scene/collision-checker header")
    dynamic_update_config_text = dynamic_update_config_header.read_text(encoding="utf-8")
    if "#include <SBF/scene_types.h>" in dynamic_update_config_text:
        raise RuntimeError("SBF/dynamic_update_config.h must not include scene records")
    if "#include <SBF/diagnostic_build_config.h>" not in build_config_text:
        raise RuntimeError("SBF/build_config.h must keep diagnostic_build_config.h as a compatibility aggregate")
    if "#include <SBF/dynamic_update_config.h>" not in planning_config_text:
        raise RuntimeError("SBF/planning_config.h must include SBF/dynamic_update_config.h for dynamic_update")
    if "#include <SBF/diagnostic_build_config.h>" in planning_config_text:
        raise RuntimeError("SBF/planning_config.h must not include the diagnostic build aggregate")
    for header_name in ("connector.h", "leaf_sweep_grower.h"):
        header_text = (include_dir / header_name).read_text(encoding="utf-8")
        if "#include <SBF/scene_types.h>" not in header_text:
            raise RuntimeError(f"SBF/{header_name} must include SBF/scene_types.h for Robot/Obstacle signatures")
    if "#include <SBF/build_config.h>" in planning_config_text:
        raise RuntimeError("SBF/planning_config.h must include narrow config headers, not SBF/build_config.h")
    if "#include <SBF/database_runtime_config.h>" not in planning_config_text:
        raise RuntimeError("SBF/planning_config.h must include SBF/database_runtime_config.h")
    if "#include <SBF/query.h>" in planning_config_text:
        raise RuntimeError("SBF/planning_config.h must not pull the CorridorQuery algorithm header")
    if "#include <SBF/query_bridge_config.h>" in planning_config_text:
        raise RuntimeError("SBF/planning_config.h must not pull query bridge batch option records")
    if "#include <SBF/query_config.h>" not in planning_config_text:
        raise RuntimeError("SBF/planning_config.h must include SBF/query_config.h for QueryConfig")
    if "#include <SBF/query_runtime_config.h>" not in planning_config_text:
        raise RuntimeError(
            "SBF/planning_config.h must include SBF/query_runtime_config.h for query runtime options"
        )
    if "#include <SBF/runtime.h>" in planning_config_text:
        raise RuntimeError("SBF/planning_config.h must not pull runtime execution helpers")
    if "#include <SBF/runtime_config.h>" not in planning_config_text:
        raise RuntimeError("SBF/planning_config.h must include SBF/runtime_config.h directly for RuntimeConfig")
    if "#include <SBF/api.h>" in planning_config_text:
        raise RuntimeError("SBF/planning_config.h must not include the compatibility aggregate SBF/api.h")
    query_config_header = include_dir / "query_config.h"
    if not query_config_header.is_file():
        raise RuntimeError("SBF/query_config.h is required for planner query option records")
    query_config_text = query_config_header.read_text(encoding="utf-8")
    if "struct QueryConfig" not in query_config_text:
        raise RuntimeError("SBF/query_config.h must define QueryConfig")
    query_header_text = (include_dir / "query.h").read_text(encoding="utf-8")
    if "#include <SBF/query_config.h>" not in query_header_text:
        raise RuntimeError("SBF/query.h must include SBF/query_config.h")
    if "#include <SBF/query_result.h>" not in query_header_text:
        raise RuntimeError("SBF/query.h must include SBF/query_result.h for QueryResult")
    if "#include <SBF/api.h>" in query_header_text:
        raise RuntimeError("SBF/query.h must not include the compatibility aggregate SBF/api.h")
    if "struct QueryConfig" in query_header_text:
        raise RuntimeError("SBF/query.h must keep QueryConfig in SBF/query_config.h")
    query_runtime_config_header = include_dir / "query_runtime_config.h"
    if not query_runtime_config_header.is_file():
        raise RuntimeError("SBF/query_runtime_config.h is required for query runtime option records")
    query_runtime_config_text = query_runtime_config_header.read_text(encoding="utf-8")
    for term in (
        "enum class PortalMembershipPolicy",
        "struct RBFQueryRuntimeOptions",
        "struct EndpointMainBoxCorridorConfig",
    ):
        if term not in query_runtime_config_text:
            raise RuntimeError(f"SBF/query_runtime_config.h must define {term}")
    query_bridge_config_text = (include_dir / "query_bridge_config.h").read_text(encoding="utf-8")
    for term in (
        "PortalMembershipPolicy",
        "RBFQueryRuntimeOptions",
        "EndpointMainBoxCorridorConfig",
    ):
        if term in query_bridge_config_text:
            raise RuntimeError(f"SBF/query_bridge_config.h must not define query runtime term {term}")
    require_terms_guarded(
        include_dir / "query_bridge_config.h",
        (
            "CorridorRefineMode",
        ),
    )
    diagnostic_result_header = include_dir / "diagnostic_result.h"
    require_terms_guarded(
        diagnostic_result_header,
        (
            "RebuildProfile",
            "dirty_boxes",
            "collision_cache_candidates",
            "used_spatial_dirty_region",
            "used_warm_rebuild",
            "warm_rebuild_ms",
        ),
    )
    diagnostic_result_text = diagnostic_result_header.read_text(encoding="utf-8")
    diagnostic_result_guard_index = diagnostic_result_text.find(guard_marker)
    if diagnostic_result_guard_index < 0:
        raise RuntimeError(f"{diagnostic_result_header}: diagnostic results must be behind SBF_DIAGNOSTIC_API")
    diagnostic_result_preamble = diagnostic_result_text[:diagnostic_result_guard_index]
    if "#include " in diagnostic_result_preamble or "struct " in diagnostic_result_preamble or "namespace " in diagnostic_result_preamble:
        raise RuntimeError(f"{diagnostic_result_header}: default include path must not pull diagnostic dependencies")
    planning_result_text = (include_dir / "planning_result.h").read_text(encoding="utf-8")
    if "struct RebuildProfile {" in planning_result_text:
        raise RuntimeError("SBF/planning_result.h must keep RebuildProfile in SBF/diagnostic_result.h")
    if "#include <SBF/diagnostic_result.h>" in planning_result_text:
        raise RuntimeError("SBF/planning_result.h must not pull diagnostic result payloads")
    if "#include <SBF/runtime.h>" in planning_result_text:
        raise RuntimeError("SBF/planning_result.h must not pull runtime execution helpers")
    if "#include <SBF/build_profile.h>" not in planning_result_text:
        raise RuntimeError("SBF/planning_result.h must include SBF/build_profile.h directly for BuildProfile")
    if "#include <SBF/api.h>" in planning_result_text:
        raise RuntimeError("SBF/planning_result.h must not include the compatibility aggregate SBF/api.h")
    connector_types_header = include_dir / "connector_types.h"
    connector_types_text = connector_types_header.read_text(encoding="utf-8")
    if "struct DebugBoundaryFfbFailure {" in connector_types_text:
        raise RuntimeError(f"{connector_types_header}: DebugBoundaryFfbFailure definition must live in SBF/debug.h")
    require_terms_guarded(
        connector_types_header,
        (
            "DebugBoundaryFfbFailure",
            "debug_boundary_failures",
        ),
    )
    debug_header = include_dir / "debug.h"
    debug_header_text = debug_header.read_text(encoding="utf-8")
    debug_guard_index = debug_header_text.find(guard_marker)
    if debug_guard_index < 0:
        raise RuntimeError(f"{debug_header}: debug payloads must be behind SBF_DIAGNOSTIC_API")
    debug_preamble = debug_header_text[:debug_guard_index]
    if "#include " in debug_preamble or "struct " in debug_preamble or "namespace " in debug_preamble:
        raise RuntimeError(f"{debug_header}: default include path must not pull diagnostic dependencies")
    if "struct DebugBoundaryFfbFailure {" not in debug_header_text:
        raise RuntimeError(f"{debug_header}: DebugBoundaryFfbFailure definition is missing")
    if "#include <SBF/find_free_box_types.h>" in debug_header_text:
        raise RuntimeError(f"{debug_header}: debug payloads must not include full FFB result/config header")
    if "#include <LECTDatabase/sbf/oracle_types.h>" not in debug_header_text:
        raise RuntimeError(f"{debug_header}: debug payloads must include oracle type records directly")
    require_terms_guarded(
        debug_header,
        (
            "DebugBoundaryFfbFailure",
            "DebugChainPaveResult",
            "boundary_failures",
        ),
    )


def check_sbf_python_binding_boundary(repo_root: Path) -> None:
    python_dir = repo_root / "safe_box_forest" / "python"
    module_path = python_dir / "bindings.cpp"
    module_text = module_path.read_text(encoding="utf-8")
    aggregator_path = python_dir / "binding_planning_forest_methods.h"
    aggregator_text = aggregator_path.read_text(encoding="utf-8")

    baseline_aggregator_include = '#include "baseline/binding_baseline_planner_functions.h"'
    if baseline_aggregator_include not in module_text:
        raise RuntimeError(f"{module_path} must include baseline planner bindings via python/baseline")

    baseline_binding_headers = (
        python_dir / "baseline" / "binding_baseline_planner_functions.h",
        python_dir / "baseline" / "binding_baseline_bitstar_functions.h",
        python_dir / "baseline" / "binding_baseline_cspace_functions.h",
        python_dir / "baseline" / "binding_baseline_prm_functions.h",
        python_dir / "baseline" / "binding_baseline_rrt_functions.h",
        python_dir / "baseline" / "binding_baseline_utility_functions.h",
    )
    for path in baseline_binding_headers:
        if not path.is_file():
            raise RuntimeError(f"missing baseline Python binding header: {path}")

    misplaced_baseline_binding_headers = sorted(path.name for path in python_dir.glob("binding_baseline_*.h"))
    if misplaced_baseline_binding_headers:
        raise RuntimeError(
            "baseline Python binding headers must live under safe_box_forest/python/baseline: "
            + ", ".join(misplaced_baseline_binding_headers)
        )

    baseline_planner_text = baseline_binding_headers[0].read_text(encoding="utf-8")
    for header in (
        "binding_baseline_bitstar_functions.h",
        "binding_baseline_cspace_functions.h",
        "binding_baseline_prm_functions.h",
        "binding_baseline_rrt_functions.h",
        "binding_baseline_utility_functions.h",
    ):
        if f'#include "{header}"' not in baseline_planner_text:
            raise RuntimeError(f"{baseline_binding_headers[0]} must include {header}")
    for path in baseline_binding_headers[1:]:
        text = path.read_text(encoding="utf-8")
        if "#include <SBF/sbf.h>" in text:
            raise RuntimeError(f"{path} must include narrow baseline dependencies, not SBF/sbf.h")
        for header in ("../binding_utils.h", "../ompl_binding_utils.h"):
            if f'#include "{header}"' not in text:
                raise RuntimeError(f"{path} must include shared baseline utility header {header}")
    ompl_utils_path = python_dir / "ompl_binding_utils.h"
    ompl_utils_text = ompl_utils_path.read_text(encoding="utf-8")
    if "#include <SBF/sbf.h>" in ompl_utils_text:
        raise RuntimeError(f"{ompl_utils_path} must include SBF/scene.h directly, not SBF/sbf.h")
    if "#include <SBF/scene.h>" not in ompl_utils_text:
        raise RuntimeError(f"{ompl_utils_path} must include SBF/scene.h for Robot/Scene/CollisionChecker")
    baseline_rrt_text = (python_dir / "baseline" / "binding_baseline_rrt_functions.h").read_text(
        encoding="utf-8"
    )
    if "#include <SBF/box_graph.h>" not in baseline_rrt_text:
        raise RuntimeError("baseline RRT Python binding must include SBF/box_graph.h for path_length")
    if "#include <SBF/connector.h>" not in baseline_rrt_text:
        raise RuntimeError("baseline RRT Python binding must include SBF/connector.h for rrt_connect")
    for path in (
        python_dir / "baseline" / "binding_baseline_bitstar_functions.h",
        python_dir / "baseline" / "binding_baseline_prm_functions.h",
        python_dir / "baseline" / "binding_baseline_rrt_functions.h",
        python_dir / "baseline" / "binding_baseline_utility_functions.h",
    ):
        text = path.read_text(encoding="utf-8")
        if "#include <SBF/scene.h>" not in text:
            raise RuntimeError(f"{path} must include SBF/scene.h for Robot/Scene/CollisionChecker")

    required_headers = (
        "binding_planning_forest_build_methods.h",
        "binding_planning_forest_database_methods.h",
        "binding_planning_forest_query_methods.h",
    )
    required_calls = (
        "register_planning_forest_build_methods(forest_class);",
        "register_planning_forest_query_methods(forest_class);",
        "register_planning_forest_database_methods(forest_class);",
    )
    for header in required_headers:
        if f'#include "{header}"' not in aggregator_text:
            raise RuntimeError(f"{aggregator_path} must include {header}")
    for call in required_calls:
        if call not in aggregator_text:
            raise RuntimeError(f"{aggregator_path} must call {call}")
    production_forest_method_headers = (
        aggregator_path,
        python_dir / "binding_planning_forest_build_methods.h",
        python_dir / "binding_planning_forest_database_methods.h",
        python_dir / "binding_planning_forest_query_methods.h",
    )
    for path in production_forest_method_headers:
        text = path.read_text(encoding="utf-8")
        if "#include <SBF/sbf.h>" in text:
            raise RuntimeError(f"{path} must include SBF/safe_box_forest.h directly, not SBF/sbf.h")
        if "#include <SBF/safe_box_forest.h>" not in text:
            raise RuntimeError(f"{path} must include SBF/safe_box_forest.h for RBFPlanningForest")

    def assert_all_guarded(path: Path, needle: str) -> None:
        text = path.read_text(encoding="utf-8")
        guard_marker = (
            "#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API &&"
        )
        guard_ranges = preprocessor_guard_ranges(
            text,
            guard_marker,
            path=path,
            guard_name="Python diagnostic",
        )
        for guard_start, guard_end in guard_ranges:
            guard_context = text[guard_start:guard_end]
            if "SBF_PYTHON_DEBUG_METHODS" not in guard_context:
                raise RuntimeError(
                    f"diagnostic guard in {path} must require SBF_PYTHON_DEBUG_METHODS"
                )

        def in_python_diagnostic_guard(index: int) -> bool:
            return any(start <= index < end for start, end in guard_ranges)

        index = text.find(needle)
        if index < 0:
            raise RuntimeError(f"{path} must contain {needle}")
        while index >= 0:
            if not in_python_diagnostic_guard(index):
                raise RuntimeError(
                    f"{needle} in {path} must require both SBF_DIAGNOSTIC_API "
                    "and SBF_PYTHON_DEBUG_METHODS"
                )
            index = text.find(needle, index + len(needle))

    diagnostic_module_needles = (
        '#include "diagnostic/binding_diagnostic_types.h"',
        "register_diagnostic_types(module);",
    )
    for needle in diagnostic_module_needles:
        assert_all_guarded(module_path, needle)

    diagnostic_option_needles = (
        '#include "diagnostic/binding_diagnostic_types.h"',
        "register_planning_config_diagnostic_fields(planning_config_class);",
    )
    for needle in diagnostic_option_needles:
        assert_all_guarded(python_dir / "binding_planning_option_types.h", needle)

    diagnostic_forest_needles = (
        '#include "diagnostic/binding_planning_forest_debug_methods.h"',
        '#include "diagnostic/binding_planning_forest_diagnostic_facade_methods.h"',
        "register_planning_forest_diagnostic_facade_methods(forest_class);",
        "register_planning_forest_debug_methods(forest_class);",
    )
    for needle in diagnostic_forest_needles:
        assert_all_guarded(aggregator_path, needle)

    diagnostic_binding_headers = (
        python_dir / "binding_oracle_utils.h",
        python_dir / "diagnostic" / "binding_diagnostic_types.h",
        python_dir / "diagnostic" / "binding_planning_forest_debug_methods.h",
        python_dir / "diagnostic" / "binding_planning_forest_diagnostic_facade_methods.h",
    )
    for path in diagnostic_binding_headers:
        if not path.is_file():
            raise RuntimeError(f"missing diagnostic Python binding header: {path}")

    diagnostic_facade_binding_header = python_dir / "diagnostic" / "binding_planning_forest_diagnostic_facade_methods.h"
    diagnostic_facade_binding_text = diagnostic_facade_binding_header.read_text(encoding="utf-8")
    for include in (
        '#include "../binding_utils.h"',
        '#include "../binding_oracle_utils.h"',
        "#include <SBF/safe_box_forest.h>",
    ):
        if include not in diagnostic_facade_binding_text:
            raise RuntimeError(f"{diagnostic_facade_binding_header} must include required dependency {include}")
    if "#include <SBF/sbf.h>" in diagnostic_facade_binding_text:
        raise RuntimeError(
            f"{diagnostic_facade_binding_header} must include SBF/safe_box_forest.h directly, not SBF/sbf.h"
        )
    debug_method_text = (python_dir / "diagnostic" / "binding_planning_forest_debug_methods.h").read_text(
        encoding="utf-8"
    )
    if "#include <SBF/sbf.h>" in debug_method_text:
        raise RuntimeError("diagnostic debug Python binding aggregator must not include SBF/sbf.h")
    if "#include <SBF/safe_box_forest.h>" not in debug_method_text:
        raise RuntimeError("diagnostic debug Python binding aggregator must include SBF/safe_box_forest.h")
    for forbidden in (
        '#include "../binding_utils.h"',
        '#include "../binding_oracle_utils.h"',
        "#include <SBF/box_graph.h>",
        "#include <SBF/find_free_box.h>",
        "#include <SBF/oracle.h>",
        "#include <SBF/runtime.h>",
        "#include <rbf/lect_database/read_snapshot.h>",
    ):
        if forbidden in debug_method_text:
            raise RuntimeError(f"diagnostic debug Python binding aggregator must not include implementation dependency {forbidden}")
    chain_pave_result_header = python_dir / "diagnostic" / "binding_debug_chain_pave_result.h"
    chain_pave_methods_header = python_dir / "diagnostic" / "binding_debug_chain_pave_methods.h"
    endpoint_oracle_methods_header = python_dir / "diagnostic" / "binding_debug_endpoint_oracle_methods.h"
    find_free_box_methods_header = python_dir / "diagnostic" / "binding_debug_find_free_box_methods.h"
    path_cover_methods_header = python_dir / "diagnostic" / "binding_debug_path_cover_methods.h"
    chain_pave_result_text = chain_pave_result_header.read_text(encoding="utf-8")
    chain_pave_methods_text = chain_pave_methods_header.read_text(encoding="utf-8")
    endpoint_oracle_methods_text = endpoint_oracle_methods_header.read_text(encoding="utf-8")
    find_free_box_methods_text = find_free_box_methods_header.read_text(encoding="utf-8")
    path_cover_methods_text = path_cover_methods_header.read_text(encoding="utf-8")
    for include in (
        "#include <SBF/debug.h>",
        '#include "../binding_utils.h"',
        '#include "../binding_oracle_utils.h"',
    ):
        if include not in chain_pave_result_text:
            raise RuntimeError(f"{chain_pave_result_header} must include required dependency {include}")
    if '#include "binding_debug_chain_pave_methods.h"' not in debug_method_text:
        raise RuntimeError("diagnostic debug Python binding must include binding_debug_chain_pave_methods.h")
    if "register_debug_chain_pave_methods(forest_class);" not in debug_method_text:
        raise RuntimeError("diagnostic debug Python binding must delegate chain-pave method registration")
    if '#include "binding_debug_endpoint_oracle_methods.h"' not in debug_method_text:
        raise RuntimeError("diagnostic debug Python binding must include binding_debug_endpoint_oracle_methods.h")
    if "register_debug_endpoint_oracle_methods(forest_class);" not in debug_method_text:
        raise RuntimeError("diagnostic debug Python binding must delegate endpoint/oracle method registration")
    if '#include "binding_debug_find_free_box_methods.h"' not in debug_method_text:
        raise RuntimeError("diagnostic debug Python binding must include binding_debug_find_free_box_methods.h")
    if "register_debug_find_free_box_methods(forest_class);" not in debug_method_text:
        raise RuntimeError("diagnostic debug Python binding must delegate FFB trace method registration")
    if '#include "binding_debug_path_cover_methods.h"' not in debug_method_text:
        raise RuntimeError("diagnostic debug Python binding must include binding_debug_path_cover_methods.h")
    if "register_debug_path_cover_methods(forest_class);" not in debug_method_text:
        raise RuntimeError("diagnostic debug Python binding must delegate path-cover method registration")
    for include in (
        "#include <SBF/safe_box_forest.h>",
        '#include "binding_debug_chain_pave_result.h"',
        '#include "../binding_utils.h"',
    ):
        if include not in chain_pave_methods_text:
            raise RuntimeError(f"{chain_pave_methods_header} must include required dependency {include}")
    for term in (
        'def("debug_chain_pave"',
        'def("debug_chain_pave_waypoints"',
        "debug_chain_pave_result_to_python(res)",
    ):
        if term not in chain_pave_methods_text:
            raise RuntimeError(f"{chain_pave_methods_header} must own chain-pave binding term {term}")
        if term in debug_method_text:
            raise RuntimeError(f"{term} must stay in binding_debug_chain_pave_methods.h")
    if "debug_chain_pave_result_to_python" not in chain_pave_result_text:
        raise RuntimeError(f"{chain_pave_result_header} must own debug_chain_pave_result_to_python")
    if chain_pave_methods_text.count("debug_chain_pave_result_to_python(res)") != 2:
        raise RuntimeError("debug_chain_pave bindings must share debug_chain_pave_result_to_python")
    if 'result["boundary_failures"]' in debug_method_text:
        raise RuntimeError("chain-pave boundary failure Python conversion must stay in binding_debug_chain_pave_result.h")
    if 'result["boundary_failures"]' in chain_pave_methods_text:
        raise RuntimeError("chain-pave boundary failure Python conversion must stay in binding_debug_chain_pave_result.h")
    for include in (
        "#include <SBF/oracle.h>",
        "#include <SBF/safe_box_forest.h>",
        "#include <link_interval_envelope/batch.h>",
        "#include <rbf/lect_database/read_snapshot.h>",
        "#include <sbf/envelope/ifk_aa_source.h>",
        '#include "../binding_utils.h"',
        '#include "../binding_oracle_utils.h"',
    ):
        if include not in endpoint_oracle_methods_text:
            raise RuntimeError(f"{endpoint_oracle_methods_header} must include required dependency {include}")
    for term in (
        'def("debug_external_endpoint_lookup"',
        'def("debug_validate_intervals"',
        'def("debug_compute_envelope_summary"',
        "link_interval_envelope::compute_envelope_batch",
        "rbf::source_channel",
        "rbf::compute_link_envelope",
        "rbf::collide_envelope_aabbs",
    ):
        if term not in endpoint_oracle_methods_text:
            raise RuntimeError(f"{endpoint_oracle_methods_header} must own endpoint/oracle binding term {term}")
        if term in debug_method_text:
            raise RuntimeError(f"{term} must stay in binding_debug_endpoint_oracle_methods.h")
    for include in (
        "#include <SBF/find_free_box.h>",
        "#include <SBF/oracle.h>",
        "#include <SBF/safe_box_forest.h>",
        "#include <rbf/lect_database/read_snapshot.h>",
        '#include "../binding_utils.h"',
        '#include "../binding_oracle_utils.h"',
    ):
        if include not in find_free_box_methods_text:
            raise RuntimeError(f"{find_free_box_methods_header} must include required dependency {include}")
    for term in (
        'def("debug_find_free_box"',
        "linear_trace_not_production_ffb",
    ):
        if term not in find_free_box_methods_text:
            raise RuntimeError(f"{find_free_box_methods_header} must own FFB trace binding term {term}")
        if term in debug_method_text:
            raise RuntimeError(f"{term} must stay in binding_debug_find_free_box_methods.h")
    for include in (
        "#include <SBF/box_graph.h>",
        "#include <SBF/find_free_box.h>",
        "#include <SBF/oracle.h>",
        "#include <SBF/runtime.h>",
        "#include <SBF/safe_box_forest.h>",
        "#include <rbf/lect_database/read_snapshot.h>",
        '#include "../binding_utils.h"',
        '#include "../binding_oracle_utils.h"',
        "#include <atomic>",
        "#include <thread>",
    ):
        if include not in path_cover_methods_text:
            raise RuntimeError(f"{path_cover_methods_header} must include required dependency {include}")
    for term in (
        'def("debug_cover_path_with_ffb"',
        "densify_path_pybind",
        "interval_boxes_connected_pybind",
        "repair_corridor_adjacency",
    ):
        if term not in path_cover_methods_text:
            raise RuntimeError(f"{path_cover_methods_header} must own path-cover binding term {term}")
        if term in debug_method_text:
            raise RuntimeError(f"{term} must stay in binding_debug_path_cover_methods.h")

    diagnostic_binding_header_names = {
        "binding_debug_chain_pave_methods.h",
        "binding_debug_endpoint_oracle_methods.h",
        "binding_debug_find_free_box_methods.h",
        "binding_debug_path_cover_methods.h",
        "binding_debug_chain_pave_result.h",
        "binding_diagnostic_types.h",
        "binding_planning_forest_debug_methods.h",
        "binding_planning_forest_diagnostic_facade_methods.h",
    }
    misplaced_diagnostic_binding_headers = sorted(
        path.name
        for path in python_dir.glob("binding*.h")
        if path.name in diagnostic_binding_header_names
    )
    if misplaced_diagnostic_binding_headers:
        raise RuntimeError(
            "diagnostic Python binding headers must live under safe_box_forest/python/diagnostic: "
            + ", ".join(misplaced_diagnostic_binding_headers)
        )

    diagnostic_type_header = python_dir / "diagnostic" / "binding_diagnostic_types.h"
    diagnostic_type_text = diagnostic_type_header.read_text(encoding="utf-8")
    if "#include <SBF/sbf.h>" in diagnostic_type_text:
        raise RuntimeError(f"{diagnostic_type_header} must include diagnostic type headers, not SBF/sbf.h")
    for include in (
        "#include <SBF/diagnostic_result.h>",
        "#include <SBF/dynamic_update_config.h>",
        "#include <SBF/planning_config.h>",
        "#include <SBF/subtractive_build_config.h>",
    ):
        if include not in diagnostic_type_text:
            raise RuntimeError(f"{diagnostic_type_header} must include required diagnostic dependency {include}")
    diagnostic_type_needles = (
        "py::class_<rbf::DynamicUpdateConfig>",
        "rbf::DynamicUpdateConfig::enable_spatial_dirty_region",
        "rbf::DynamicUpdateConfig::warm_rebuild_dirty_box_threshold",
        "py::class_<rbf::SubtractiveObstacleGroup>",
        "rbf::SubtractiveObstacleGroup::validation_obstacles",
        "py::class_<rbf::SubtractiveBuildOptions>",
        "register_planning_config_diagnostic_fields",
        '.def_readwrite("dynamic_update"',
        "py::class_<rbf::RebuildProfile>",
        "rbf::RebuildProfile::dirty_boxes",
        "rbf::RebuildProfile::collision_cache_candidates",
        "rbf::RebuildProfile::used_spatial_dirty_region",
        "rbf::RebuildProfile::used_warm_rebuild",
        "rbf::RebuildProfile::warm_rebuild_ms",
    )
    for needle in diagnostic_type_needles:
        assert_all_guarded(diagnostic_type_header, needle)

    production_type_headers = (
        python_dir / "binding_planner_core_types.h",
        python_dir / "binding_planning_option_types.h",
    )
    narrow_python_binding_headers = {
        python_dir / "binding_utils.h": (
            "#include <SBF/scene_types.h>",
        ),
        python_dir / "binding_oracle_utils.h": (
            "#include <LECTDatabase/sbf/oracle_types.h>",
        ),
        python_dir / "binding_basic_types.h": (
            "#include <SBF/query_result.h>",
            "#include <SBF/scene_types.h>",
            "#include <rbf/lect_database/canonicalization.h>",
            "#include <sbf/envelope/endpoint_source.h>",
            "#include <sbf/envelope/envelope_type.h>",
        ),
        python_dir / "binding_planner_core_types.h": (
            "#include <SBF/grower_types.h>",
            "#include <SBF/leaf_sweep_types.h>",
            "#include <SBF/runtime_config.h>",
            "#include <SBF/segment_edge_types.h>",
        ),
        python_dir / "binding_adaptive_types.h": (
            "#include <SBF/adaptive_leaf_sweep_config.h>",
            "#include <SBF/planning_result.h>",
            "#include <SBF/query_runtime_config.h>",
        ),
        python_dir / "binding_planning_option_types.h": (
            "#include <SBF/build_profile.h>",
            "#include <SBF/planning_config.h>",
            "#include <SBF/query_result.h>",
            "#include <SBF/query_runtime_config.h>",
            "#include <LECTDatabase/online_cache/config.h>",
            "#include <LECTDatabase/sbf/oracle_types.h>",
            "#include <rbf/lect_database/split_policy.h>",
        ),
    }
    for path, required_includes in narrow_python_binding_headers.items():
        text = path.read_text(encoding="utf-8")
        if "#include <SBF/sbf.h>" in text:
            raise RuntimeError(f"{path} must include narrow public type headers, not the SBF/sbf.h facade")
        for include in required_includes:
            if include not in text:
                raise RuntimeError(f"{path} must include required narrow dependency {include}")
    binding_utils_text = (python_dir / "binding_utils.h").read_text(encoding="utf-8")
    if "#include <LECTDatabase/sbf/oracle_types.h>" in binding_utils_text:
        raise RuntimeError("binding_utils.h must not include oracle records; use binding_oracle_utils.h")
    for path in baseline_binding_headers:
        text = path.read_text(encoding="utf-8")
        if "binding_oracle_utils.h" in text:
            raise RuntimeError(f"{path} must not depend on diagnostic oracle binding utilities")

    forbidden_diagnostic_type_terms = (
        "py::class_<rbf::DynamicUpdateConfig>",
        "py::class_<rbf::SubtractiveObstacleGroup>",
        "py::class_<rbf::SubtractiveBuildOptions>",
        "py::class_<rbf::RebuildProfile>",
        '.def_readwrite("dynamic_update"',
    )
    for path in production_type_headers:
        text = path.read_text(encoding="utf-8")
        for term in forbidden_diagnostic_type_terms:
            if term in text:
                raise RuntimeError(f"{path} must not directly register diagnostic Python type term {term}")

    production_method_headers = (
        python_dir / "binding_planning_forest_build_methods.h",
        python_dir / "binding_planning_forest_database_methods.h",
        python_dir / "binding_planning_forest_query_methods.h",
    )
    forbidden_terms = (
        "SBF_DIAGNOSTIC_API",
        "SBF_PYTHON_DEBUG_METHODS",
        "build_subtractive",
        "debug_chain_pave",
        "refine_query_corridor",
        "add_obstacle_and_rebuild",
        "remove_obstacle_and_regrow",
        "connect_update_segment_fallback",
    )
    for path in production_method_headers:
        text = path.read_text(encoding="utf-8")
        for term in forbidden_terms:
            if term in text:
                raise RuntimeError(f"{path} must not contain diagnostic binding term {term}")

    init_path = python_dir / "sbf" / "__init__.py"
    diagnostic_init_path = python_dir / "sbf" / "diagnostic_exports.py"
    init_tree = ast.parse(init_path.read_text(encoding="utf-8"), filename=str(init_path))
    diagnostic_init_tree = ast.parse(
        diagnostic_init_path.read_text(encoding="utf-8"),
        filename=str(diagnostic_init_path),
    )
    root_cpp_import_aliases: dict[str, str | None] = {}
    root_alias_assignments: dict[str, str] = {}
    imports_diagnostic_exports = False
    imports_diagnostic_star = False
    for node in init_tree.body:
        if isinstance(node, ast.ImportFrom) and node.module == "_sbf_cpp":
            for alias in node.names:
                root_cpp_import_aliases[alias.name] = alias.asname
        if isinstance(node, ast.ImportFrom) and node.module == "diagnostic_exports":
            for alias in node.names:
                if alias.name == "_diagnostic_exports":
                    imports_diagnostic_exports = True
                if alias.name == "*":
                    imports_diagnostic_star = True
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if isinstance(target, ast.Name) and isinstance(node.value, ast.Name):
                root_alias_assignments[target.id] = node.value.id

    expected_root_alias_imports = {
        "RBFPlanningConfig": "_RBFPlanningConfig",
        "RBFPlanningForest": "_RBFPlanningForest",
    }
    for imported_name, alias_name in expected_root_alias_imports.items():
        if root_cpp_import_aliases.get(imported_name) != alias_name:
            raise RuntimeError(f"{init_path}: {imported_name} must be imported as {alias_name}")
    expected_root_alias_assignments = {
        "SBFConfig": "_RBFPlanningConfig",
        "SafeBoxForest": "_RBFPlanningForest",
    }
    for public_name, private_name in expected_root_alias_assignments.items():
        if root_alias_assignments.get(public_name) != private_name:
            raise RuntimeError(f"{init_path}: {public_name} must alias {private_name}")
    if not imports_diagnostic_exports or not imports_diagnostic_star:
        raise RuntimeError(
            f"{init_path}: diagnostic symbols must be imported only through sbf.diagnostic_exports"
        )

    diagnostic_export_names = {
        "DynamicUpdateConfig",
        "RebuildProfile",
        "SubtractiveBuildOptions",
        "SubtractiveObstacleGroup",
    }
    imported_from_cpp: set[str] = set()
    diagnostic_extend_names: set[str] = set()
    diagnostic_all_from_list = False
    all_has_diagnostic_splice = False
    all_literal_diagnostics: set[str] = set()

    for node in diagnostic_init_tree.body:
        if isinstance(node, ast.Try):
            for item in node.body:
                if isinstance(item, ast.ImportFrom) and item.module == "_sbf_cpp":
                    imported_from_cpp.update(alias.name for alias in item.names)
            for item in node.orelse:
                if not isinstance(item, ast.Expr) or not isinstance(item.value, ast.Call):
                    continue
                call = item.value
                if not (
                    isinstance(call.func, ast.Attribute)
                    and call.func.attr == "extend"
                    and isinstance(call.func.value, ast.Name)
                    and call.func.value.id == "_diagnostic_exports"
                ):
                    continue
                if len(call.args) != 1 or not isinstance(call.args[0], ast.List):
                    raise RuntimeError(
                        f"{diagnostic_init_path}: _diagnostic_exports.extend must use one list literal"
                    )
                for elt in call.args[0].elts:
                    if not isinstance(elt, ast.Constant) or not isinstance(elt.value, str):
                        raise RuntimeError(f"{diagnostic_init_path}: _diagnostic_exports entries must be string literals")
                    diagnostic_extend_names.add(elt.value)
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "__all__" for target in node.targets
        ):
            if (
                isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Name)
                and node.value.func.id == "list"
                and len(node.value.args) == 1
                and isinstance(node.value.args[0], ast.Name)
                and node.value.args[0].id == "_diagnostic_exports"
            ):
                diagnostic_all_from_list = True

    for node in init_tree.body:
        if isinstance(node, ast.Try):
            raise RuntimeError(f"{init_path}: diagnostic try-import logic must live in sbf.diagnostic_exports")
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "__all__" for target in node.targets
        ):
            if not isinstance(node.value, ast.List):
                raise RuntimeError(f"{init_path}: __all__ must remain a list literal")
            for elt in node.value.elts:
                if isinstance(elt, ast.Starred) and isinstance(elt.value, ast.Name):
                    if elt.value.id == "_diagnostic_exports":
                        all_has_diagnostic_splice = True
                    continue
                if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                    if elt.value in diagnostic_export_names:
                        all_literal_diagnostics.add(elt.value)
                    continue
                raise RuntimeError(f"{init_path}: unsupported __all__ entry")

    if imported_from_cpp != diagnostic_export_names:
        raise RuntimeError(f"{diagnostic_init_path}: diagnostic try-import set changed: {sorted(imported_from_cpp)}")
    if diagnostic_extend_names != diagnostic_export_names:
        raise RuntimeError(
            f"{diagnostic_init_path}: _diagnostic_exports list changed: {sorted(diagnostic_extend_names)}"
        )
    if not diagnostic_all_from_list:
        raise RuntimeError(f"{diagnostic_init_path}: __all__ must be list(_diagnostic_exports)")
    if not all_has_diagnostic_splice:
        raise RuntimeError(f"{init_path}: __all__ must include *_diagnostic_exports")
    if all_literal_diagnostics:
        raise RuntimeError(
            f"{init_path}: diagnostic names must not be literal __all__ entries: "
            + ", ".join(sorted(all_literal_diagnostics))
        )


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
    check_forbidden_sidecar_policy(repo_root, tmp_root)
    check_core_environment_read_policy(repo_root, tmp_root)
    check_public_tree_readiness_policy(repo_root)
    check_active_experiment_plan_boundary(repo_root)
    check_sbf_diagnostic_source_boundary(repo_root)
    check_sbf_header_diagnostic_boundary(repo_root)
    check_sbf_python_binding_boundary(repo_root)
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
