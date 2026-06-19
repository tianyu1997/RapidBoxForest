from __future__ import annotations

import math


def diagnostic_number(diagnostics: dict[str, float], key: str, default: float = 0.0) -> float:
    value = diagnostics.get(key, default)
    if "_ms" in key or key.endswith(".ms"):
        profile_total_key = f"profile.{key}.total_ms"
        profile_value = diagnostics.get(profile_total_key)
        if profile_value is not None:
            try:
                if key not in diagnostics or abs(float(value)) <= 1e-15:
                    value = profile_value
            except (TypeError, ValueError):
                value = profile_value
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def diagnostic_max(diagnostics: dict[str, float], keys: list[str]) -> float:
    value = 0.0
    for key in keys:
        value = max(value, diagnostic_number(diagnostics, key, 0.0))
    return value


def diagnostic_sum_suffix(diagnostics: dict[str, float], suffix: str) -> float:
    return float(sum(
        float(value)
        for key, value in diagnostics.items()
        if str(key).endswith(suffix)
    ))


def diagnostic_max_suffix(diagnostics: dict[str, float], suffix: str) -> float:
    value = 0.0
    for key, item in diagnostics.items():
        if str(key).endswith(suffix):
            try:
                value = max(value, float(item))
            except (TypeError, ValueError):
                continue
    return value


def diagnostic_first_suffix(
    diagnostics: dict[str, float],
    suffix: str,
    default: float = math.nan,
) -> float:
    for key in sorted(diagnostics):
        if str(key).endswith(suffix):
            try:
                return float(diagnostics[key])
            except (TypeError, ValueError):
                return float(default)
    return float(default)


def external_evidence_diagnostic_fields(diagnostics: dict[str, float]) -> dict[str, float]:
    reused_keys = [
        "oracle.materialization_reused_external_evidence",
        "adaptive.oracle.materialization_reused_external_evidence",
        "adaptive.external_reused_hits_normalized",
        "leaf_sweep.worker_oracle.materialization_reused_external_evidence",
        "grower.worker_oracle.materialization_reused_external_evidence",
    ]
    exact_hit_keys = [
        "oracle.materialization_external_exact_hits",
        "adaptive.oracle.materialization_external_exact_hits",
        "adaptive.external_exact_hits_normalized",
        "leaf_sweep.worker_oracle.materialization_external_exact_hits",
        "grower.worker_oracle.materialization_external_exact_hits",
    ]
    exact_miss_keys = [
        "oracle.materialization_external_exact_misses",
        "adaptive.oracle.materialization_external_exact_misses",
        "adaptive.external_exact_misses_normalized",
        "leaf_sweep.worker_oracle.materialization_external_exact_misses",
        "grower.worker_oracle.materialization_external_exact_misses",
    ]
    replay_check_keys = [
        "oracle.interval_replay_compatibility_checks",
        "adaptive.oracle.interval_replay_compatibility_checks",
        "adaptive.interval_replay_compatibility_checks_normalized",
        "leaf_sweep.worker_oracle.interval_replay_compatibility_checks",
        "grower.worker_oracle.interval_replay_compatibility_checks",
    ]
    replay_compatible_keys = [
        "oracle.interval_replay_compatible",
        "adaptive.oracle.interval_replay_compatible",
        "adaptive.interval_replay_compatible_normalized",
        "leaf_sweep.worker_oracle.interval_replay_compatible",
        "grower.worker_oracle.interval_replay_compatible",
    ]
    replay_incompatible_keys = [
        "oracle.interval_replay_incompatible",
        "adaptive.oracle.interval_replay_incompatible",
        "adaptive.interval_replay_incompatible_normalized",
        "leaf_sweep.worker_oracle.interval_replay_incompatible",
        "grower.worker_oracle.interval_replay_incompatible",
    ]
    replay_direct_hit_keys = [
        "oracle.interval_replay_direct_exact_hits",
        "adaptive.oracle.interval_replay_direct_exact_hits",
        "adaptive.interval_replay_direct_exact_hits_normalized",
        "leaf_sweep.worker_oracle.interval_replay_direct_exact_hits",
        "grower.worker_oracle.interval_replay_direct_exact_hits",
    ]
    replay_key_only_blocked_keys = [
        "oracle.interval_replay_key_only_blocked",
        "adaptive.oracle.interval_replay_key_only_blocked",
        "adaptive.interval_replay_key_only_blocked_normalized",
        "leaf_sweep.worker_oracle.interval_replay_key_only_blocked",
        "grower.worker_oracle.interval_replay_key_only_blocked",
    ]
    reused_hits = diagnostic_max(diagnostics, reused_keys)
    exact_hits = diagnostic_max(diagnostics, exact_hit_keys)
    exact_misses = diagnostic_max(diagnostics, exact_miss_keys)
    return {
        "external_hits": reused_hits,
        "external_reused_hits": reused_hits,
        "external_exact_hits": exact_hits,
        "external_exact_misses": exact_misses,
        "interval_replay_compatibility_checks": diagnostic_max(diagnostics, replay_check_keys),
        "interval_replay_compatible": diagnostic_max(diagnostics, replay_compatible_keys),
        "interval_replay_incompatible": diagnostic_max(diagnostics, replay_incompatible_keys),
        "interval_replay_direct_exact_hits": diagnostic_max(diagnostics, replay_direct_hit_keys),
        "interval_replay_key_only_blocked": diagnostic_max(diagnostics, replay_key_only_blocked_keys),
    }


def query_bridge_diagnostic_fields(
    diagnostics: dict[str, float],
    query_count: int,
    max_task_index: int = 16,
) -> dict[str, float | int]:
    fields: dict[str, float | int] = {}
    scalar_keys = [
        "query_bridge.batch_total_ms",
        "query_bridge.batch_rrt_ms_total",
        "query_bridge.batch_probe_ms_total",
        "query_bridge.batch_pave_ms_total",
        "query_bridge.batch_no_path_retry_ms_total",
        "query_bridge.batch_no_path_retry_adaptive_ms_total",
        "query_bridge.batch_no_path_retry_adaptive_attempts",
        "query_bridge.batch_no_path_retry_adaptive_successes",
        "query_bridge.batch_tasks_initial",
        "query_bridge.batch_tasks_attempted",
        "query_bridge.batch_tasks_skipped",
        "query_bridge.batch_tasks_skipped_after_rrt",
        "query_bridge.batch_tasks_no_path",
        "query_bridge.parallel_task_rrt_jobs",
        "query_bridge.rrt_fixed_iters",
        "query_bridge.local_radius_schedule_size",
        "query_bridge.hybridize_attempt_paths_tasks",
        "query_bridge.hybridize_attempt_paths_candidates",
        "query_bridge.hybridize_attempt_paths_accepts",
        "query_bridge.hybridize_attempt_paths_delta",
        "query_bridge.hybridize_attempt_paths_audit_rejects",
        "query_bridge.no_path_retry_budget_stages",
        "query_bridge.oracle_node_validations",
        "query_bridge.oracle_validation_cache_hits",
        "query_bridge.oracle_validation_cache_misses",
        "query_bridge.oracle_materializations",
        "query_bridge.oracle_external_exact_hits",
        "query_bridge.oracle_external_exact_misses",
        "query_bridge.oracle_interval_replay_compatibility_checks",
        "query_bridge.oracle_interval_replay_compatible",
        "query_bridge.oracle_interval_replay_incompatible",
        "query_bridge.oracle_interval_replay_direct_exact_hits",
        "query_bridge.oracle_interval_replay_key_only_blocked",
        "query_bridge.oracle_shared_endpoint_cache_hits",
        "query_bridge.oracle_endpoint_path_ms",
        "query_bridge.oracle_classify_ms",
        "query_bridge.oracle_validate_total_ms",
        "query_bridge.oracle_materialization_endpoint_ms",
        "query_bridge.oracle_materialization_envelope_ms",
        "query_bridge.oracle_envelope_collision_queries",
        "query_bridge.oracle_envelope_gjk_tests",
        "query_bridge.direct_segment_after_rrt",
        "query_bridge.direct_segment_after_rrt_edges",
        "query_bridge.direct_segment_after_rrt_audit_rejects",
        "query_bridge.direct_segment_after_rrt_add_fail",
        "query_bridge.direct_corridor_ms",
        "query_bridge.direct_corridor_ms_total",
        "query_bridge.direct_corridor_samples",
        "query_bridge.direct_corridor_samples_total",
        "query_bridge.direct_corridor_ffb_calls",
        "query_bridge.direct_corridor_ffb_calls_total",
        "query_bridge.direct_corridor_all_ffb_calls",
        "query_bridge.direct_corridor_all_ffb_calls_total",
        "query_bridge.direct_corridor_ffb_start_depth",
        "query_bridge.direct_corridor_direct_ffb_ms",
        "query_bridge.direct_corridor_repair_ffb_ms",
        "query_bridge.direct_corridor_adaptive_repair_ffb_ms",
        "query_bridge.direct_corridor_adaptive_initial_bad_fraction",
        "query_bridge.direct_corridor_adaptive_final_bad_fraction",
        "query_bridge.direct_corridor_segment_audit_ms",
        "query_bridge.direct_corridor_mark_initial_ms",
        "query_bridge.direct_corridor_mark_incremental_ms",
        "query_bridge.direct_corridor_initialize_dsu_ms",
        "query_bridge.direct_corridor_assimilate_ms",
        "query_bridge.direct_corridor_partition_neighbor_candidates",
        "query_bridge.direct_corridor_added",
        "query_bridge.direct_corridor_added_total",
        "query_bridge.direct_corridor_repair_calls",
        "query_bridge.direct_corridor_repair_calls_total",
        "query_bridge.direct_corridor_repair_added",
        "query_bridge.direct_corridor_repair_added_total",
        "query_bridge.direct_corridor_adaptive_repair_calls",
        "query_bridge.direct_corridor_adaptive_repair_calls_total",
        "query_bridge.direct_corridor_adaptive_repair_added",
        "query_bridge.direct_corridor_adaptive_repair_added_total",
        "query_bridge.direct_corridor_bad_initial",
        "query_bridge.direct_corridor_bad_initial_total",
        "query_bridge.direct_corridor_bad_final",
        "query_bridge.direct_corridor_bad_final_total",
        "query_bridge.direct_corridor_assimilate_covered_samples",
        "query_bridge.direct_corridor_assimilate_coverage_span_max",
        "query_bridge.direct_corridor_assimilate_coverage_span_mean",
        "query_bridge.direct_corridor_segment_edges",
        "query_bridge.direct_corridor_segment_edges_total",
        "query_bridge.direct_corridor_local_connected",
        "query_bridge.direct_corridor_local_residual_overlay_connected",
        "query_bridge.direct_corridor_full_residual_without_local_overlay",
        "query_bridge.direct_corridor_full_residual_audit_rejects",
        "query_bridge.direct_corridor_full_residual_edges",
        "query_bridge.direct_corridor_full_residual_edges_without_local_overlay",
        "query_bridge.direct_corridor_full_adjacency_scans_avoided",
        "query_bridge.partition_graph_dense_chain_pave_skipped",
        "query_bridge.partition_graph_forward_chain_pave_skipped",
        "query_bridge.partition_graph_reverse_chain_pave_skipped",
        "query_bridge.partition_graph_gap_connector_skipped",
        "query_bridge.full_adjacency_rebuilds_avoided",
        "query_bridge.segment_edge_audit_rejects",
        "query_bridge.segment_edge_blocked_no_max_depth_ffb_failure",
    ]
    integer_suffixes = (
        "_tasks",
        "_jobs",
        "_calls",
        "_samples",
        "_added",
        "_edges",
        "_skipped",
        "_attempted",
        "_rejects",
        "_avoided",
        "_connected",
        "_initial",
        "_final",
        "_failures",
        "_successes",
    )
    for key in scalar_keys:
        out_key = "diag_" + key.replace(".", "_")
        value = diagnostic_number(diagnostics, key, 0.0)
        if out_key.endswith(integer_suffixes):
            fields[out_key] = int(value)
        else:
            fields[out_key] = value

    batch_ms = float(fields.get("diag_query_bridge_batch_total_ms", 0.0))
    fields["diag_query_bridge_batch_per_query_ms"] = batch_ms / max(1, int(query_count))
    for key, value in diagnostics.items():
        prefix = "query_bridge.batch_task."
        if not str(key).startswith(prefix):
            continue
        rest = str(key)[len(prefix):]
        parts = rest.split(".", 1)
        if len(parts) != 2:
            continue
        try:
            task_index = int(parts[0])
        except ValueError:
            continue
        if task_index >= max_task_index:
            continue
        metric = parts[1]
        if metric not in {
            "rrt_ms",
            "pave_ms",
            "total_ms",
            "waypoint_length",
            "skip_reason_code",
            "direct_corridor_ms",
            "direct_corridor_ffb_start_depth",
            "direct_corridor_added",
            "direct_corridor_repair_added",
            "direct_corridor_adaptive_repair_calls",
            "direct_corridor_adaptive_repair_added",
            "direct_corridor_bad_initial",
            "direct_corridor_bad_final",
            "direct_corridor_partition_append_calls",
            "direct_corridor_partition_append_boxes",
            "direct_corridor_assimilate_local_hits",
            "direct_corridor_assimilate_full_scan_fallbacks",
            "direct_corridor_assimilate_local_sample_tests",
            "direct_corridor_segment_edges",
            "direct_corridor_local_connected",
            "direct_corridor_full_residual_edge",
        }:
            continue
        out_key = f"diag_query_bridge_task{task_index}_{metric}"
        numeric = float(value)
        if not metric.endswith("_ms"):
            fields[out_key] = int(numeric)
        else:
            fields[out_key] = numeric
    return fields
