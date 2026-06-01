from __future__ import annotations

import argparse
import json
import math
import os
import sys
from argparse import ArgumentParser, Namespace
from pathlib import Path
from typing import Any, Iterable


def bootstrap_imports() -> Path:
    root = Path(__file__).resolve().parents[1]
    build_dir = os.environ.get("SBF_BUILD_DIR")
    candidates = []
    if build_dir:
        candidates.append(Path(build_dir) / "python")
    candidates.extend((root / "build_py310" / "python", root / "build" / "python", root / "python"))
    for candidate in reversed(candidates):
        text = str(candidate)
        if text in sys.path:
            sys.path.remove(text)
        if candidate.exists():
            sys.path.insert(0, text)
    return root


ROOT = bootstrap_imports()
REPO_ROOT = ROOT.parents[1]

import sbf  # noqa: E402


RBF_LIFELONG_PRESET = "rbf_ifk_aa_aafkvol_d40_s15_canonical_lifelong"
RBF_ONLY_OUTPUT_ROOT = ROOT / "outputs" / "paper" / "rbf_only"


def mean(values: Iterable[float]) -> float | None:
    data = list(values)
    return sum(data) / len(data) if data else None


def median(values: Iterable[float]) -> float | None:
    data = sorted(values)
    if not data:
        return None
    mid = len(data) // 2
    if len(data) % 2:
        return data[mid]
    return 0.5 * (data[mid - 1] + data[mid])


def segment_resolution_from_step(edge_step: float, segment_step: float) -> int:
    if segment_step <= 0.0:
        return 1
    return max(1, int(math.ceil(max(0.0, float(edge_step)) / float(segment_step))))


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def box_volume_sum(boxes: list[Any]) -> float:
    return sum(float(box.volume) for box in boxes)


def count_status(boxes: list[Any], status: Any) -> int:
    return sum(1 for box in boxes if box.safety_status == status)


def set_if_available(obj: Any, name: str, value: Any) -> bool:
    try:
        setattr(obj, name, value)
        return True
    except AttributeError:
        return False


def set_path_if_available(obj: Any, path: str, value: Any) -> bool:
    current = obj
    parts = path.split(".")
    for part in parts[:-1]:
        try:
            current = getattr(current, part)
        except AttributeError:
            return False
    return set_if_available(current, parts[-1], value)


def serialize_depth_dimensions(depth_dimensions: Iterable[int]) -> str:
    return ",".join(str(int(dim)) for dim in depth_dimensions)


def compute_inert_dim_mask(robot: Any, root_intervals: Iterable[Any]) -> list[int]:
    """Detect kinematically inert split dims for a robot over a root domain.

    Uses the aafk_volume_min depth schedule (greedy min-endpoint-measure split
    selection): a dim never chosen by the schedule contributes nothing to the
    endpoint envelope and is safe to mask. Returns a 0/1 mask (0 = mask the dim).
    Per-robot, one-shot; no hot-path cost.
    """
    root = list(root_intervals)
    ndim = len(root)
    if ndim == 0:
        return []
    counts = list(sbf.aafk_volume_min_depth_schedule(robot, root, max(8 * ndim, 48), 8))
    hist = [0] * ndim
    for d in counts:
        idx = int(d)
        if 0 <= idx < ndim:
            hist[idx] += 1
    return [0 if hist[d] == 0 else 1 for d in range(ndim)]


def apply_dim_mask(cfg: Any, mask: Iterable[int]) -> None:
    """Apply a per-dim split mask to grower + connector BestTighten configs."""
    mask_list = [int(v) for v in mask]
    for prefix in ("grower.find_free_box", "connector.pave.find_free_box"):
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.dim_mask", list(mask_list))


def _schedule_root_intervals(root_intervals: Iterable[Any] | None) -> list[Any] | None:
    if root_intervals is None:
        return None
    return [interval for interval in root_intervals]


def _aafk_sample_nodes_per_depth(sample_nodes_per_depth: int | None) -> int:
    if sample_nodes_per_depth is None:
        return 8
    return max(1, int(sample_nodes_per_depth))


def aafk_volume_min_dim6_schedule(
    robot: Any,
    max_depth: int,
    root_intervals: Iterable[Any] | None = None,
    sample_nodes_per_depth: int | None = None,
) -> list[int]:
    """Volume-min greedy schedule with guaranteed coverage of starved DOFs.

    The pure AAFKVolumeMin greedy heuristic never splits joints that barely change
    the end-effector AAFK volume (notably the wrist roll, dim_6), starving them and
    leaving LECT leaf boxes full-width along those axes. This hybrid keeps the
    volume-min picks but reserves every ``n_joints``-th slot to round-robin the
    dimensions that are entirely absent from the greedy schedule, so every DOF
    receives coverage (like round-robin) while preserving volume-min priority for
    the remaining 6/7 of the splits.
    """
    schedule_root = _schedule_root_intervals(root_intervals)
    sample_budget = _aafk_sample_nodes_per_depth(sample_nodes_per_depth)
    if schedule_root is None:
        base = list(sbf.aafk_volume_min_depth_schedule(robot, int(max_depth), sample_budget))
    else:
        base = list(sbf.aafk_volume_min_depth_schedule(robot, schedule_root, int(max_depth), sample_budget))
    n_joints = int(robot.n_joints())
    present = set(int(d) for d in base)
    missing = [d for d in range(n_joints) if d not in present]
    if not missing:
        return [int(d) for d in base]
    schedule = [int(d) for d in base]
    cursor = 0
    for i in range(len(schedule)):
        if (i + 1) % n_joints == 0:
            schedule[i] = missing[cursor % len(missing)]
            cursor += 1
    return schedule


def make_aafk_volume_min_dim6_split_policy(
    robot: Any,
    max_depth: int,
    root_intervals: Iterable[Any] | None = None,
    sample_nodes_per_depth: int | None = None,
) -> Any:
    schedule = aafk_volume_min_dim6_schedule(
        robot,
        int(max_depth),
        root_intervals=root_intervals,
        sample_nodes_per_depth=sample_nodes_per_depth,
    )
    if len(schedule) < int(max_depth):
        raise RuntimeError(
            f"AAFKVolumeMinDim6 schedule has {len(schedule)} entries, expected at least {int(max_depth)}"
        )
    descriptor = sbf.SplitPolicyDescriptor()
    descriptor.strategy = sbf.SplitStrategy.AAFKVolumeMin
    descriptor.min_width = 0.0
    descriptor.midpoint = True
    descriptor.deterministic_tie_break = True
    descriptor.depth_dimensions = schedule
    descriptor.dimension_schedule_hash = str(sbf.stable_hash(serialize_depth_dimensions(schedule)))
    return descriptor


def make_aafk_volume_min_split_policy(
    robot: Any,
    max_depth: int,
    root_intervals: Iterable[Any] | None = None,
    sample_nodes_per_depth: int | None = None,
) -> Any:
    schedule_root = _schedule_root_intervals(root_intervals)
    sample_budget = _aafk_sample_nodes_per_depth(sample_nodes_per_depth)
    if schedule_root is None:
        schedule = list(sbf.aafk_volume_min_depth_schedule(robot, int(max_depth), sample_budget))
    else:
        schedule = list(sbf.aafk_volume_min_depth_schedule(robot, schedule_root, int(max_depth), sample_budget))
    if len(schedule) < int(max_depth):
        raise RuntimeError(
            f"AAFKVolumeMin schedule has {len(schedule)} entries, expected at least {int(max_depth)}"
        )
    descriptor = sbf.SplitPolicyDescriptor()
    descriptor.strategy = sbf.SplitStrategy.AAFKVolumeMin
    descriptor.min_width = 0.0
    descriptor.midpoint = True
    descriptor.deterministic_tie_break = True
    descriptor.depth_dimensions = schedule
    descriptor.dimension_schedule_hash = str(sbf.stable_hash(serialize_depth_dimensions(schedule)))
    return descriptor


def make_support_hull_volume_min_split_policy(
    robot: Any,
    max_depth: int,
    root_intervals: Iterable[Any] | None = None,
    sample_nodes_per_depth: int | None = None,
) -> Any:
    """Seed/scene-independent split policy minimising the SupportHull envelope.

    Identical machinery to :func:`make_aafk_volume_min_split_policy` (the runtime
    replays ``depth_dimensions`` under the AAFKVolumeMin strategy) but the schedule
    is computed against the SupportHull swept-envelope volume — the envelope used
    at certification time — instead of the looser endpoint-AABB sum. The schedule
    remains a pure function of (robot, canonical root intervals).
    """
    schedule_root = _schedule_root_intervals(root_intervals)
    sample_budget = _aafk_sample_nodes_per_depth(sample_nodes_per_depth)
    if schedule_root is None:
        schedule = list(sbf.support_hull_volume_min_depth_schedule(robot, int(max_depth), sample_budget))
    else:
        schedule = list(
            sbf.support_hull_volume_min_depth_schedule(robot, schedule_root, int(max_depth), sample_budget)
        )
    if len(schedule) < int(max_depth):
        raise RuntimeError(
            f"SupportHullVolumeMin schedule has {len(schedule)} entries, expected at least {int(max_depth)}"
        )
    descriptor = sbf.SplitPolicyDescriptor()
    descriptor.strategy = sbf.SplitStrategy.AAFKVolumeMin
    descriptor.min_width = 0.0
    descriptor.midpoint = True
    descriptor.deterministic_tie_break = True
    descriptor.depth_dimensions = schedule
    descriptor.dimension_schedule_hash = str(sbf.stable_hash(serialize_depth_dimensions(schedule)))
    return descriptor


def set_rbf_envelope(cfg: Any, envelope: str, args: Namespace) -> None:
    if envelope == "link":
        cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB
    elif envelope == "kdop26":
        cfg.envelope_type.type = sbf.EnvelopeType.KDOP
        cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
        cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
    elif envelope == "support_hull":
        cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
        cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
        cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        cfg.envelope_type.support_hull_config.safety_epsilon = float(args.support_hull_safety_epsilon)
    else:
        raise ValueError(f"unknown RBF envelope {envelope!r}")


def apply_rbf_lifelong_defaults(cfg: Any, args: Namespace, robot: Any, seed: int, preset: str) -> None:
    if robot is None:
        raise ValueError(f"preset {preset!r} requires a robot to generate the AAFKVolumeMin depth schedule")
    max_depth = int(args.rbf_max_depth)
    skip_depth = int(args.rbf_ffb_start_depth)
    if skip_depth > max_depth:
        raise ValueError(f"rbf ffb start depth {skip_depth} cannot exceed max depth {max_depth}")

    split_policy = make_aafk_volume_min_split_policy(
        robot,
        max_depth,
        sample_nodes_per_depth=int(getattr(args, "aafk_sample_nodes_per_depth", 8)),
    )
    cfg.database.split_policy = split_policy
    cfg.database.canonical_mode = bool(args.rbf_canonical_cache)
    if bool(cfg.database.canonical_mode):
        set_if_available(cfg.database, "symmetry_descriptor", "joint_symmetry_native_v1")
    cfg.database.create_if_missing = True
    cfg.database.read_only = False
    cfg.database.verify_identity = True
    cfg.database.replay_journal = True
    cfg.database.max_tree_depth = int(args.rbf_max_tree_depth)
    set_if_available(cfg.database, "propagate_parent_hulls", True)
    set_if_available(cfg.database, "defer_parent_hull_writes", True)
    cfg.database.checkpoint_after_build = True
    cfg.database.online_cache.max_nodes = int(args.rbf_online_cache_max_nodes)
    cfg.database.online_cache.max_payload_bytes = int(args.rbf_online_cache_max_payload_bytes)
    cfg.database.online_cache.allow_database_backfill = True
    cache_label = str(args.rbf_cache_label or f"{preset}_seed{int(seed)}")
    cfg.database.path = str(Path(args.rbf_cache_root) / cache_label)
    set_if_available(
        cfg.database,
        "auto_publish_snapshot_after_checkpoint",
        bool(getattr(args, "rbf_auto_publish_snapshot", True)),
    )
    set_if_available(
        cfg.database,
        "auto_publish_snapshot_async",
        bool(getattr(args, "rbf_auto_publish_snapshot_async", True)),
    )
    if getattr(args, "rbf_snapshot_path", None):
        set_if_available(cfg.database, "auto_publish_snapshot_path", str(Path(args.rbf_snapshot_path)))

    cfg.endpoint_source.source = sbf.EndpointSource.IFK
    set_rbf_envelope(cfg, str(args.rbf_envelope), args)
    cfg.validation.mode = sbf.OracleValidationMode.StrictCertificate
    cfg.validation.accept_unsafe_free = False
    cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
    cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
    cfg.grower.find_free_box.max_depth = max_depth
    cfg.grower.find_free_box.skip_to_depth = skip_depth
    cfg.connector.pave.find_free_box.max_depth = max_depth
    cfg.connector.pave.find_free_box.skip_to_depth = skip_depth


def set_online_cache_backfill(cfg: Any, allow_database_backfill: bool) -> None:
    cfg.database.online_cache.allow_database_backfill = bool(allow_database_backfill)


def default_external_evidence_snapshot_path(source_path: Path) -> Path:
    return Path(source_path) / "lect_snapshot"


def configure_external_evidence_reuse(
    cfg: Any,
    source_path: Path,
    args: Namespace | None = None,
    *,
    materialization: bool = True,
    scoring: bool = True,
    backfill_active: bool = False,
) -> dict[str, Any]:
    external_path = Path(source_path)
    mode = str(getattr(args, "external_evidence_mode", "snapshot")) if args is not None else "snapshot"
    if mode not in {"snapshot", "legacy"}:
        raise ValueError(f"unsupported external evidence reuse mode {mode!r}")
    use_snapshot = mode == "snapshot"
    snapshot_path = default_external_evidence_snapshot_path(external_path)

    cfg.database.external_evidence_path = str(external_path)
    set_if_available(cfg.database, "external_evidence_use_snapshot", use_snapshot)
    set_if_available(
        cfg.database,
        "external_evidence_auto_build_snapshot",
        bool(getattr(args, "external_evidence_auto_build_snapshot", True)) if args is not None else True,
    )
    set_if_available(cfg.database, "external_evidence_snapshot_path", str(snapshot_path) if use_snapshot else "")
    cfg.validation.external_evidence_materialization = bool(materialization)
    cfg.validation.external_evidence_scoring = bool(scoring)
    cfg.validation.external_evidence_backfill_active = bool(backfill_active)
    return {
        "mode": mode,
        "path": str(external_path),
        "snapshot_path": str(snapshot_path) if use_snapshot else None,
        "backfill_active": bool(backfill_active),
    }


def rbf_lifelong_config_metadata(cfg: Any, args: Namespace | None = None) -> dict[str, Any]:
    split_policy = cfg.database.split_policy
    depth_dimensions = list(split_policy.depth_dimensions)
    rbf_envelope_arg = str(getattr(args, "rbf_envelope", "support_hull")) if args is not None else None
    endpoint_source_raw = str(cfg.endpoint_source.source).split(".")[-1]
    endpoint_channel = "safe" if endpoint_source_raw in {"IFK", "HIFK"} else "rapid"
    split_policy_name = str(getattr(args, "lect_split_policy", "aafk_volume_min")) if args is not None else "configured"
    return {
        "preset": RBF_LIFELONG_PRESET,
        "endpoint_source": endpoint_source_raw,
        "endpoint_channel": endpoint_channel,
        "split_policy": split_policy_name,
        "aafk_sample_nodes_per_depth": int(getattr(args, "aafk_sample_nodes_per_depth", 8)) if args is not None else 8,
        "split_policy_descriptor": sbf.split_policy_descriptor(split_policy),
        "split_policy_hash": int(sbf.split_policy_hash(split_policy)),
        "dimension_schedule_hash": str(split_policy.dimension_schedule_hash),
        "depth_dimensions": depth_dimensions,
        "schedule_depth": len(depth_dimensions),
        "max_depth": int(cfg.grower.find_free_box.max_depth),
        "ffb_start_depth": int(cfg.grower.find_free_box.skip_to_depth),
        "envelope": rbf_envelope_arg,
        "envelope_type_raw": str(cfg.envelope_type.type).split(".")[-1],
        "canonical_mode": bool(cfg.database.canonical_mode),
        "symmetry_descriptor": str(getattr(cfg.database, "symmetry_descriptor", "")),
        "database_path": str(cfg.database.path),
        "database_snapshot_path": str(
            getattr(cfg.database, "auto_publish_snapshot_path", "") or (Path(str(cfg.database.path)) / "lect_snapshot")
        ) if str(getattr(cfg.database, "path", "")) else "",
        "auto_publish_snapshot_after_checkpoint": bool(
            getattr(cfg.database, "auto_publish_snapshot_after_checkpoint", False)
        ),
        "auto_publish_snapshot_async": bool(getattr(cfg.database, "auto_publish_snapshot_async", True)),
        "external_evidence_path": str(getattr(cfg.database, "external_evidence_path", "")),
        "external_evidence_snapshot_path": str(getattr(cfg.database, "external_evidence_snapshot_path", "")),
        "external_evidence_use_snapshot": bool(getattr(cfg.database, "external_evidence_use_snapshot", False)),
        "external_evidence_auto_build_snapshot": bool(getattr(cfg.database, "external_evidence_auto_build_snapshot", True)),
        "external_evidence_mode": (
            "snapshot" if bool(getattr(cfg.database, "external_evidence_use_snapshot", False)) else "legacy"
        ) if str(getattr(cfg.database, "external_evidence_path", "")) else "off",
        "propagate_parent_hulls": bool(getattr(cfg.database, "propagate_parent_hulls", True)),
        "defer_parent_hull_writes": bool(getattr(cfg.database, "defer_parent_hull_writes", False)),
        "checkpoint_after_build": bool(getattr(cfg.database, "checkpoint_after_build", False)),
        "online_cache_allow_database_backfill": bool(getattr(cfg.database.online_cache, "allow_database_backfill", True)),
        "external_evidence_materialization": bool(getattr(cfg.validation, "external_evidence_materialization", True)),
        "external_evidence_backfill_active": bool(getattr(cfg.validation, "external_evidence_backfill_active", True)),
        "prewarm_depth": int(getattr(args, "rbf_prewarm_depth", 18)) if args is not None else 18,
    }


def add_common_sbf_args(parser: ArgumentParser) -> None:
    parser.add_argument(
        "--preset",
        choices=["ifk_strict", "crit_link_coverage", "kdop26_coverage", "support_hull_coverage", "coverage_hybrid", RBF_LIFELONG_PRESET],
        default="support_hull_coverage",
    )
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--seed-base", type=int, default=20260504)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--task-batch-size", type=int, default=8)
    parser.add_argument("--worker-local-ffb", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-boxes", type=int, default=5000)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--ffb-depth", type=int, default=120)
    parser.add_argument("--max-consecutive-miss", type=int, default=2000)
    parser.add_argument("--grid-delta", type=float, default=0.04)
    parser.add_argument("--envelope-subdivisions", type=int, default=4)
    parser.add_argument("--kdop-safety-epsilon", type=float, default=1e-9)
    parser.add_argument("--support-hull-safety-epsilon", type=float, default=1e-9)
    parser.add_argument("--split-policy", choices=["widest-first", "best-tighten"], default="best-tighten")
    parser.add_argument("--best-tighten-depth-synchronous", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-prefer-sector-boundary", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-use-minimax", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-shape-balancing", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--best-tighten-recent-dim-cooling", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--ffb-min-split-width", type=float, default=0.0)
    # Seed-independent split policy: the canonical LECT split depends only on
    # (robot, domain). No query-seed coupling knob is exposed.
    parser.add_argument("--ffb-auto-mask-inert", action=argparse.BooleanOptionalAction, default=True,
                        help="Auto-detect kinematically inert split dims (per-robot, via aafk schedule) and mask them by default.")
    parser.add_argument("--endpoint-cache-min-effective-width", type=float, default=0.0)
    parser.add_argument("--enable-merger", action="store_true", default=False)
    parser.add_argument("--enable-connector", action="store_true", default=True)
    parser.add_argument("--no-enable-connector", dest="enable_connector", action="store_false")
    parser.add_argument("--rrt-goal-bias", type=float, default=0.2)
    parser.add_argument("--intertree-goal-bias", type=float, default=0.25)
    parser.add_argument("--unexplored-prob", type=float, default=0.45)
    parser.add_argument(
        "--sample-categorical-allocation",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Draw the per-sample target category from a single explicit categorical split instead of sequential Bernoulli gates.",
    )
    parser.add_argument(
        "--sample-uniform-prob",
        type=float,
        default=0.0,
        help="Explicit pure-uniform probability for categorical allocation; any leftover mass is also assigned to uniform.",
    )
    parser.add_argument("--step-ratio", type=float, default=0.08)
    parser.add_argument("--component-connect-prob", type=float, default=0.45)
    parser.add_argument("--component-connect-candidate-limit", type=int, default=4)
    parser.add_argument("--component-connect-stage-normalized-linf", type=float, default=0.35)
    parser.add_argument("--component-connect-ffb-depth-increment", type=int, default=40)
    parser.add_argument("--component-connect-ffb-max-depth", type=int, default=160)
    parser.add_argument("--stop-after-connect", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--post-connect-extra-boxes", type=int, default=0)
    parser.add_argument("--quality-min-connected-boxes", type=int, default=64)
    parser.add_argument("--post-connect-time-budget-ms", type=float, default=450.0)
    parser.add_argument("--coverage-first-stop-loss", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--hard-frontier-failure-threshold", type=int, default=1)
    parser.add_argument("--hard-frontier-box-horizon", type=int, default=300)
    parser.add_argument("--strict-path-audit", action="store_true", default=True)
    parser.add_argument("--no-strict-path-audit", dest="strict_path_audit", action="store_false")
    parser.add_argument("--audit-resolution", type=int, default=32)
    parser.add_argument("--audit-segment-step", type=float, default=0.01)
    parser.add_argument("--repair-on-audit-failure", action="store_true", default=True)
    parser.add_argument("--no-repair-on-audit-failure", dest="repair_on_audit_failure", action="store_false")
    parser.add_argument("--repair-max-attempts", type=int, default=6)
    parser.add_argument("--repair-rrt-max-iters", type=int, default=20000)
    parser.add_argument("--repair-timeout-ms", type=float, default=750.0)
    parser.add_argument("--repair-local-sampling-radius", type=float, default=0.4)
    parser.add_argument("--repair-local-sampling-growth", type=float, default=2.0)
    parser.add_argument("--validation-cache", action="store_true", default=True)
    parser.add_argument("--no-validation-cache", dest="validation_cache", action="store_false")
    parser.add_argument("--validation-cache-max-entries", type=int, default=200000)
    parser.add_argument("--endpoint-evidence-cache", action="store_true", default=True)
    parser.add_argument("--no-endpoint-evidence-cache", dest="endpoint_evidence_cache", action="store_false")
    parser.add_argument("--incremental-materialization", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--incremental-materialization-bind-online-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--worker-shared-endpoint-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--collision-shortcut", action="store_true", default=True)
    parser.add_argument("--no-collision-shortcut", dest="collision_shortcut", action="store_false")
    parser.add_argument("--collision-shortcut-resolution", type=int, default=24)
    parser.add_argument("--frontier-bridge", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--frontier-bridge-adaptive-ffb", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--frontier-bridge-ffb-max-depth", type=int, default=0)
    parser.add_argument("--frontier-bridge-ffb-depth-increment", type=int, default=2)
    parser.add_argument("--frontier-bridge-gap-stall-iterations", type=int, default=4)
    parser.add_argument("--frontier-bridge-candidate-limit", type=int, default=8)
    parser.add_argument("--connector-point-gap-tolerance", type=float, default=0.0)
    parser.add_argument("--connector-point-gap-resolution", type=int, default=16)
    parser.add_argument("--connector-bridge-boxes", type=int, default=0)
    parser.add_argument("--segment-edges", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--connector-pair-batch-size", type=int, default=1)
    parser.add_argument("--connector-pair-timeout-ms", type=float, default=250.0)
    parser.add_argument("--connector-max-pairs-per-gap", type=int, default=8)
    parser.add_argument("--connector-birrt", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--connector-rrt-iters", type=int, default=50000)
    parser.add_argument("--connector-rrt-timeout-ms", type=float, default=2000.0)
    parser.add_argument("--connector-rrt-step-size", type=float, default=0.25)
    parser.add_argument("--connector-rrt-goal-bias", type=float, default=0.4)
    parser.add_argument("--connector-segment-resolution", type=int, default=16)
    parser.add_argument("--sbf-bridge-segment-step", type=float, default=0.01)
    parser.add_argument("--connector-pave-max-chain", type=int, default=0)
    parser.add_argument("--connector-pave-steps", type=int, default=12)
    parser.add_argument("--connector-pave-depth", type=int, default=120)
    parser.add_argument("--rbf-max-depth", type=int, default=40)
    parser.add_argument("--rbf-max-tree-depth", type=int, default=64)
    parser.add_argument("--rbf-ffb-start-depth", type=int, default=15)
    parser.add_argument("--rbf-prewarm-depth", type=int, default=18)
    parser.add_argument("--rbf-envelope", choices=["link", "kdop26", "support_hull"], default="support_hull")
    parser.add_argument("--rbf-canonical-cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rbf-cache-root", type=Path, default=RBF_ONLY_OUTPUT_ROOT / "cache")
    parser.add_argument("--rbf-cache-label", default="")
    parser.add_argument("--rbf-snapshot-path", type=Path, default=None)
    parser.add_argument("--rbf-auto-publish-snapshot", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rbf-auto-publish-snapshot-async", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rbf-online-cache-max-nodes", type=int, default=200000)
    parser.add_argument("--rbf-online-cache-max-payload-bytes", type=int, default=512 * 1024 * 1024)
    parser.add_argument(
        "--aafk-sample-nodes-per-depth",
        type=int,
        default=8,
        help="Number of sampled nodes per depth when building the AAFKVolumeMin dimension schedule.",
    )
    parser.add_argument("--external-evidence-mode", choices=["snapshot", "legacy"], default="snapshot")
    parser.add_argument("--external-evidence-auto-build-snapshot", action=argparse.BooleanOptionalAction, default=True)


def configure_standalone_sbf(args: Namespace, seed: int, preset: str | None = None, robot: Any | None = None) -> sbf.SBFConfig:
    chosen = preset or args.preset
    cfg = sbf.SBFConfig()
    cfg.database.canonical_mode = bool(args.rbf_canonical_cache)
    if bool(cfg.database.canonical_mode):
        set_if_available(cfg.database, "symmetry_descriptor", "joint_symmetry_native_v1")
    else:
        set_if_available(cfg.database, "symmetry_descriptor", "")
    bridge_segment_resolution = segment_resolution_from_step(
        float(args.connector_rrt_step_size),
        float(getattr(args, "sbf_bridge_segment_step", 0.01) or 0.0),
    )
    cfg.enable_merger = bool(args.enable_merger)
    cfg.enable_connector = bool(args.enable_connector)
    cfg.runtime.mode = sbf.ExecutionMode.Parallel if args.threads > 1 else sbf.ExecutionMode.Inline
    cfg.runtime.n_threads = max(1, int(args.threads))
    cfg.runtime.batch_size = max(1, int(args.task_batch_size))
    cfg.runtime.parallel_threshold = 1

    if chosen == RBF_LIFELONG_PRESET:
        cfg.endpoint_source.source = sbf.EndpointSource.IFK
        set_rbf_envelope(cfg, str(args.rbf_envelope), args)
        cfg.validation.mode = sbf.OracleValidationMode.StrictCertificate
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
    elif chosen == "ifk_strict":
        cfg.endpoint_source.source = sbf.EndpointSource.IFK
        cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB
        cfg.validation.mode = sbf.OracleValidationMode.StrictCertificate
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitCertifiedOnly
    elif chosen in {"crit_link_coverage", "kdop26_coverage", "support_hull_coverage"}:
        cfg.endpoint_source.source = sbf.EndpointSource.CritSample
        if chosen == "kdop26_coverage":
            cfg.envelope_type.type = sbf.EnvelopeType.KDOP
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        elif chosen == "support_hull_coverage":
            cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
            cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
            cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
            cfg.envelope_type.support_hull_config.safety_epsilon = float(args.support_hull_safety_epsilon)
        else:
            cfg.envelope_type.type = sbf.EnvelopeType.LinkIAABB
        cfg.validation.mode = sbf.OracleValidationMode.CoverageHeuristic
        cfg.validation.accept_unsafe_free = True
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
    else:
        cfg.endpoint_source.source = sbf.EndpointSource.CritSample
        cfg.envelope_type.type = sbf.EnvelopeType.SupportHull
        cfg.envelope_type.kdop_config.direction_set = sbf.KdopDirectionSet.DOP26
        cfg.envelope_type.kdop_config.safety_epsilon = float(args.kdop_safety_epsilon)
        cfg.envelope_type.support_hull_config.safety_epsilon = float(args.support_hull_safety_epsilon)
        cfg.validation.mode = sbf.OracleValidationMode.CoverageHeuristic
        cfg.validation.accept_unsafe_free = True
        cfg.grower.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed
        cfg.connector.pave.commit_policy = sbf.BoxCommitPolicy.CommitProvisionalAllowed

    cfg.envelope_type.n_subdivisions = int(args.envelope_subdivisions)
    use_best_tighten = getattr(args, "split_policy", "best-tighten") == "best-tighten"
    set_path_if_available(cfg, "grower.find_free_box.split.use_best_tighten", use_best_tighten)
    set_path_if_available(cfg, "connector.pave.find_free_box.split.use_best_tighten", use_best_tighten)
    for prefix in ("grower.find_free_box", "connector.pave.find_free_box"):
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.depth_synchronous", bool(args.best_tighten_depth_synchronous))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.prefer_sector_boundary", bool(args.best_tighten_prefer_sector_boundary))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.use_minimax", bool(args.best_tighten_use_minimax))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.shape_balancing", bool(args.best_tighten_shape_balancing))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.recent_dim_cooling", bool(args.best_tighten_recent_dim_cooling))
        set_path_if_available(cfg, f"{prefix}.split.best_tighten.min_candidate_width", float(args.ffb_min_split_width))
    cfg.validation.enable_validation_cache = bool(args.validation_cache)
    cfg.validation.validation_cache_max_entries = int(args.validation_cache_max_entries)
    set_if_available(cfg.validation, "enable_endpoint_evidence_cache", bool(args.endpoint_evidence_cache))
    set_if_available(cfg.validation, "endpoint_cache_min_effective_width", float(args.endpoint_cache_min_effective_width))
    incremental_materialization = bool(args.incremental_materialization)
    if bool(args.incremental_materialization_bind_online_cache) and not bool(args.endpoint_evidence_cache):
        incremental_materialization = False
    force_stateless_materialization = (
        cfg.endpoint_source.source == sbf.EndpointSource.CritSample
        and cfg.validation.mode == sbf.OracleValidationMode.CoverageHeuristic
    )
    set_if_available(
        cfg.validation,
        "stateless_materialization_context",
        force_stateless_materialization or not incremental_materialization,
    )
    set_if_available(cfg.validation, "enable_worker_shared_endpoint_cache", bool(args.worker_shared_endpoint_cache) and bool(args.endpoint_evidence_cache))

    cfg.grower.mode = sbf.GrowerMode.RRT
    cfg.grower.rng_seed = int(args.seed_base) + int(seed)
    cfg.grower.max_boxes = int(args.max_boxes)
    cfg.grower.timeout_ms = float(args.timeout_ms)
    cfg.grower.max_consecutive_miss = int(args.max_consecutive_miss)
    cfg.grower.n_threads = max(1, int(args.threads))
    cfg.grower.task_batch_size = max(1, int(args.task_batch_size))
    cfg.grower.parallel_threshold = 1
    cfg.grower.worker_local_ffb = bool(args.worker_local_ffb) and args.threads > 1
    cfg.grower.find_free_box.max_depth = int(args.ffb_depth)
    cfg.grower.find_free_box.skip_to_depth = int(args.rbf_ffb_start_depth)
    cfg.grower.find_free_box.split_reserved_leaf = True
    cfg.grower.find_free_box.split_unknown_leaf = True
    cfg.grower.find_free_box.reject_seed_collision = False
    set_if_available(cfg.grower, "rrt_goal_bias", float(args.rrt_goal_bias))
    set_if_available(cfg.grower, "intertree_goal_bias", float(args.intertree_goal_bias))
    set_if_available(cfg.grower, "sustained_goal_bias_cap", min(0.25, float(args.intertree_goal_bias)))
    set_if_available(cfg.grower, "rrt_step_ratio", float(args.step_ratio))
    set_if_available(cfg.grower, "unexplored_sample_prob", float(args.unexplored_prob))
    if bool(args.sample_categorical_allocation):
        total_target_prob = (
            max(0.0, float(args.component_connect_prob))
            + max(0.0, float(args.intertree_goal_bias))
            + max(0.0, float(args.rrt_goal_bias))
            + max(0.0, float(args.unexplored_prob))
            + max(0.0, float(args.sample_uniform_prob))
        )
        if total_target_prob > 1.0 + 1e-9:
            raise ValueError(
                "categorical grower probabilities must sum to <= 1.0: "
                f"component_connect + intertree + rrt + unexplored + uniform = {total_target_prob:.6f}"
            )
    set_if_available(cfg.grower, "sample_categorical_allocation", bool(args.sample_categorical_allocation))
    set_if_available(cfg.grower, "sample_uniform_prob", float(args.sample_uniform_prob))
    cfg.grower.connect_mode = True
    cfg.grower.expand_all_roots_per_sample = True
    set_if_available(cfg.grower, "component_connect_prob", float(args.component_connect_prob))
    set_if_available(cfg.grower, "component_connect_candidate_limit", int(args.component_connect_candidate_limit))
    cfg.grower.component_connect_island_aware = True
    cfg.grower.component_connect_frontier_cache = True
    cfg.grower.component_connect_staged_growth = True
    set_if_available(cfg.grower, "component_connect_stage_normalized_linf", float(args.component_connect_stage_normalized_linf))
    set_if_available(cfg.grower, "component_connect_adaptive_ffb", True)
    set_if_available(cfg.grower, "component_connect_ffb_depth_increment", int(args.component_connect_ffb_depth_increment))
    set_if_available(cfg.grower, "component_connect_ffb_max_depth", int(args.component_connect_ffb_max_depth))
    set_if_available(cfg.grower, "stop_after_connect", bool(args.stop_after_connect))
    set_if_available(cfg.grower, "post_connect_extra_boxes", int(args.post_connect_extra_boxes))
    set_if_available(cfg.grower, "quality_min_connected_boxes", int(args.quality_min_connected_boxes))
    set_if_available(cfg.grower, "post_connect_time_budget_ms", float(args.post_connect_time_budget_ms))
    set_if_available(cfg.grower, "coverage_first_stop_loss", bool(args.coverage_first_stop_loss))
    set_if_available(cfg.grower, "hard_frontier_failure_threshold", int(args.hard_frontier_failure_threshold))
    set_if_available(cfg.grower, "hard_frontier_box_horizon", int(args.hard_frontier_box_horizon))

    cfg.query.nearest_if_outside = False
    cfg.query.shortcut_boxes = True
    cfg.query.collision_shortcut = bool(args.collision_shortcut)
    cfg.query.collision_shortcut_resolution = int(args.collision_shortcut_resolution)
    cfg.query.strict_path_audit = bool(args.strict_path_audit)
    cfg.query.audit_resolution = max(int(args.audit_resolution), bridge_segment_resolution)
    cfg.query.audit_segment_step = float(args.audit_segment_step)
    cfg.query.repair_on_audit_failure = bool(args.repair_on_audit_failure)
    cfg.query.repair_max_attempts = int(args.repair_max_attempts)
    cfg.query.repair_rrt_max_iters = int(args.repair_rrt_max_iters)
    cfg.query.repair_timeout_ms = float(args.repair_timeout_ms)
    set_if_available(cfg.query, "repair_local_sampling_radius", float(args.repair_local_sampling_radius))
    set_if_available(cfg.query, "repair_local_sampling_growth", float(args.repair_local_sampling_growth))

    set_if_available(cfg.connector, "frontier_bridge", bool(args.frontier_bridge))
    set_if_available(cfg.connector, "frontier_bridge_adaptive_ffb", bool(args.frontier_bridge_adaptive_ffb))
    set_if_available(cfg.connector, "frontier_bridge_ffb_max_depth", int(args.frontier_bridge_ffb_max_depth))
    set_if_available(cfg.connector, "frontier_bridge_ffb_depth_increment", int(args.frontier_bridge_ffb_depth_increment))
    set_if_available(cfg.connector, "frontier_bridge_gap_stall_iterations", int(args.frontier_bridge_gap_stall_iterations))
    set_if_available(cfg.connector, "frontier_bridge_candidate_limit", int(args.frontier_bridge_candidate_limit))
    set_if_available(cfg.connector, "point_validated_gap_tolerance", float(args.connector_point_gap_tolerance))
    set_if_available(cfg.connector, "point_validated_gap_resolution", int(args.connector_point_gap_resolution))
    cfg.connector.max_total_bridge_boxes = int(args.connector_bridge_boxes)
    cfg.connector.segment_edges_enabled = bool(args.segment_edges)
    cfg.connector.rrt_segment_edges = bool(args.segment_edges)
    cfg.connector.point_gap_segment_edges = bool(args.segment_edges)
    cfg.connector.n_threads = max(1, int(args.threads))
    set_if_available(cfg.connector, "pair_batch_size", max(1, int(args.connector_pair_batch_size)))
    cfg.connector.parallel_threshold = 1
    set_if_available(cfg.connector, "per_pair_timeout_ms", float(args.connector_pair_timeout_ms))
    set_if_available(cfg.connector, "max_pairs_per_gap", int(args.connector_max_pairs_per_gap))
    set_if_available(cfg.connector, "enable_birrt", bool(args.connector_birrt))
    cfg.connector.rrt.max_iters = int(args.connector_rrt_iters)
    cfg.connector.rrt.timeout_ms = float(args.connector_rrt_timeout_ms)
    cfg.connector.rrt.step_size = float(args.connector_rrt_step_size)
    cfg.connector.rrt.goal_bias = float(args.connector_rrt_goal_bias)
    cfg.connector.rrt.segment_resolution = max(int(args.connector_segment_resolution), bridge_segment_resolution)
    cfg.connector.pave.max_chain = int(args.connector_pave_max_chain)
    set_if_available(cfg.connector.pave, "max_steps_per_waypoint", int(args.connector_pave_steps))
    cfg.connector.pave.find_free_box.max_depth = int(args.connector_pave_depth)
    cfg.connector.pave.find_free_box.skip_to_depth = int(args.rbf_ffb_start_depth)
    cfg.connector.pave.find_free_box.split_reserved_leaf = True
    cfg.connector.pave.find_free_box.split_unknown_leaf = True
    cfg.connector.pave.find_free_box.reject_seed_collision = False
    if chosen == RBF_LIFELONG_PRESET:
        apply_rbf_lifelong_defaults(cfg, args, robot, seed, chosen)
    return cfg


def query_result_payload(label: str, result: sbf.QueryResult, wall_s: float) -> dict[str, Any]:
    return {
        "name": label,
        "ok": bool(result.success),
        "t_s": float(wall_s),
        "length": float(result.path_length) if result.success else 0.0,
        "audit_status": str(result.audit_status).split(".")[-1],
        "audit_passed": bool(result.audit_passed),
        "audit_time_ms": float(result.audit_time_ms),
        "repair_time_ms": float(result.repair_time_ms),
        "repair_count": int(result.repair_count),
        "segment_edges_used": int(result.segment_edges_used),
        "remaining_unsafe_assumptions": int(result.remaining_unsafe_assumptions),
    }