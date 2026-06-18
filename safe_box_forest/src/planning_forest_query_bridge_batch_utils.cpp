#include "planning_forest_query_bridge_batch_utils.h"

#include <SBF/box_graph.h>

#include "env_config.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace rbf {

QueryBridgeAcceptanceThresholds query_bridge_acceptance_thresholds_from_env() {
    QueryBridgeAcceptanceThresholds thresholds;
    thresholds.max_segment_fraction = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_ACCEPT_SEGMENT_FRACTION", 0.25));
    thresholds.path_ratio =
        std::max(0.0, detail::env_double_or_default("RBF_QUERY_BRIDGE_ACCEPT_PATH_RATIO", 1.50));
    thresholds.path_additive = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_ACCEPT_PATH_ADDITIVE", 0.75));
    thresholds.max_path_length = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_ADAPTIVE_MAX_PATH_LENGTH", 4.5));
    return thresholds;
}

QueryBridgePartitionPathFirstOptions query_bridge_partition_path_first_options_from_env(
    bool partition_native_mode) {
    QueryBridgePartitionPathFirstOptions options;
    options.enabled =
        partition_native_mode &&
        detail::env_int_or_default("RBF_QUERY_BRIDGE_PARTITION_PATH_FIRST", 0) != 0;
    options.allow_long =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_PARTITION_PATH_FIRST_ALLOW_LONG", 0) != 0;
    options.max_segment_fraction = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_PARTITION_PATH_FIRST_MAX_SEGMENT_FRACTION",
                                      0.95));
    return options;
}

QueryBridgeRetryOptions query_bridge_retry_options_from_env() {
    QueryBridgeRetryOptions options;
    options.skip_deferred_short_edges =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_SKIP_DEFERRED_SHORT", 1) != 0;
    options.segment_only_retry_attempts =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_SEGMENT_ONLY_RETRY_ATTEMPTS", 1));
    options.no_path_retry_attempts =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_NO_PATH_RETRY_ATTEMPTS", 1));
    options.no_path_retry_stop_on_first_success =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_NO_PATH_RETRY_STOP_ON_FIRST_SUCCESS", 0) != 0;
    options.forced_attempts =
        std::max(1, detail::env_int_or_default("RBF_QUERY_BRIDGE_FORCED_ATTEMPTS", 1));
    options.attempt_offset =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_ATTEMPT_OFFSET", 0));
    options.rrt_fixed_iters =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_RRT_FIXED_ITERS", 0));
    options.rrt_fixed_timeout_ms = std::max(
        0.0,
        detail::env_double_or_default("RBF_QUERY_BRIDGE_RRT_FIXED_TIMEOUT_MS", 0.0));
    options.rrt_clearance =
        std::max(0.0, detail::env_double_or_default("RBF_QUERY_BRIDGE_RRT_CLEARANCE", 0.0));
    options.local_radius_schedule =
        detail::env_double_list_or_empty("RBF_QUERY_BRIDGE_LOCAL_RADIUS_SCHEDULE");
    options.local_radius_append_unrestricted_attempt =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_LOCAL_RADIUS_APPEND_UNRESTRICTED_ATTEMPT",
                                   1) != 0;
    options.rrt_optimize_after_first_iters = std::max(
        0,
        detail::env_int_or_default("RBF_QUERY_BRIDGE_RRT_OPTIMIZE_AFTER_FIRST_ITERS", 0));
    options.attempt_fallback_paths =
        std::max(0, detail::env_int_or_default("RBF_QUERY_BRIDGE_ATTEMPT_FALLBACK_PATHS", 0));
    options.no_path_retry_budget_iters =
        detail::env_int_list_or_empty("RBF_QUERY_BRIDGE_NO_PATH_RETRY_BUDGET_ITERS");
    options.no_path_retry_budget_attempts =
        detail::env_int_list_or_empty("RBF_QUERY_BRIDGE_NO_PATH_RETRY_BUDGET_ATTEMPTS");
    options.no_path_retry_budget_stages =
        std::min(options.no_path_retry_budget_iters.size(),
                 options.no_path_retry_budget_attempts.size());
    options.post_rrt_skip_forced =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_POST_RRT_SKIP_FORCED", 0) != 0;
    return options;
}

void record_query_bridge_retry_diagnostics(StageContext& context,
                                           const QueryBridgeRetryOptions& options) {
    context.diagnostics().set_value("query_bridge.skip_deferred_short_edges",
                                    options.skip_deferred_short_edges ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.segment_only_retry_attempts_default",
                                    static_cast<double>(options.segment_only_retry_attempts));
    context.diagnostics().set_value("query_bridge.no_path_retry_attempts_default",
                                    static_cast<double>(options.no_path_retry_attempts));
    context.diagnostics().set_value("query_bridge.no_path_retry_stop_on_first_success",
                                    options.no_path_retry_stop_on_first_success ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.rrt_fixed_iters",
                                    static_cast<double>(options.rrt_fixed_iters));
    context.diagnostics().set_value("query_bridge.rrt_fixed_timeout_ms",
                                    options.rrt_fixed_timeout_ms);
    context.diagnostics().set_value("query_bridge.rrt_clearance",
                                    options.rrt_clearance);
    context.diagnostics().set_value("query_bridge.local_radius_schedule_size",
                                    static_cast<double>(options.local_radius_schedule.size()));
    context.diagnostics().set_value("query_bridge.local_radius_append_unrestricted_attempt",
                                    options.local_radius_append_unrestricted_attempt ? 1.0 : 0.0);
    context.diagnostics().set_value("query_bridge.rrt_optimize_after_first_iters",
                                    static_cast<double>(options.rrt_optimize_after_first_iters));
    context.diagnostics().set_value("query_bridge.attempt_fallback_paths",
                                    static_cast<double>(options.attempt_fallback_paths));
    context.diagnostics().set_value("query_bridge.no_path_retry_budget_stages",
                                    static_cast<double>(options.no_path_retry_budget_stages));
    for (std::size_t stage = 0; stage < options.no_path_retry_budget_stages; ++stage) {
        const std::string prefix =
            "query_bridge.no_path_retry_budget_stage." + std::to_string(stage) + ".";
        context.diagnostics().set_value(prefix + "iters",
                                        static_cast<double>(options.no_path_retry_budget_iters[stage]));
        context.diagnostics().set_value(
            prefix + "attempts",
            static_cast<double>(options.no_path_retry_budget_attempts[stage]));
    }
    context.diagnostics().set_value("query_bridge.post_rrt_skip_forced",
                                    options.post_rrt_skip_forced ? 1.0 : 0.0);
}

QueryBridgeParallelRrtOptions query_bridge_parallel_rrt_options_from_env() {
    QueryBridgeParallelRrtOptions options;
    options.early_stop =
        detail::env_int_or_default("RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP", 0) != 0;
    options.early_stop_min_successes =
        std::max(1,
                 detail::env_int_or_default(
                     "RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_MIN_SUCCESSES",
                     1));
    options.early_stop_ratio =
        std::max(1.0,
                 detail::env_double_or_default(
                     "RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_RATIO",
                     1.75));
    options.early_stop_additive =
        std::max(0.0,
                 detail::env_double_or_default(
                     "RBF_QUERY_BRIDGE_PARALLEL_RRT_EARLY_STOP_ADDITIVE",
                     0.75));
    return options;
}

void record_query_bridge_parallel_rrt_diagnostics(
    StageContext& context,
    const QueryBridgeParallelRrtOptions& options) {
    context.diagnostics().set_value("query_bridge.parallel_rrt_early_stop_enabled",
                                    options.early_stop ? 1.0 : 0.0);
}

bool query_bridge_parallel_rrt_path_good_enough(
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const std::vector<Eigen::VectorXd>& path,
    const QueryBridgeParallelRrtOptions& options) {
    if (path.empty()) {
        return false;
    }
    const double direct = (goal - start).norm();
    if (direct <= 1e-9) {
        return true;
    }
    const double length = path_length(path);
    return length <= std::max(direct * options.early_stop_ratio,
                              direct + options.early_stop_additive);
}

void add_query_bridge_oracle_counter_delta(BuildProfile& profile,
                                           const OracleCounters& before,
                                           const OracleCounters& after) {
    auto add_counter_delta = [&](const std::string& key, auto after_value, auto before_value) {
        profile.diagnostics[key] += static_cast<double>(after_value - before_value);
    };
    add_counter_delta("query_bridge.oracle_node_validations",
                      after.node_validations,
                      before.node_validations);
    add_counter_delta("query_bridge.oracle_validation_cache_hits",
                      after.validation_cache_hits,
                      before.validation_cache_hits);
    add_counter_delta("query_bridge.oracle_validation_cache_misses",
                      after.validation_cache_misses,
                      before.validation_cache_misses);
    add_counter_delta("query_bridge.oracle_materializations",
                      after.materializations,
                      before.materializations);
    add_counter_delta("query_bridge.oracle_external_exact_hits",
                      after.materialization_external_exact_hits,
                      before.materialization_external_exact_hits);
    add_counter_delta("query_bridge.oracle_external_exact_misses",
                      after.materialization_external_exact_misses,
                      before.materialization_external_exact_misses);
    add_counter_delta("query_bridge.oracle_interval_replay_compatibility_checks",
                      after.interval_replay_compatibility_checks,
                      before.interval_replay_compatibility_checks);
    add_counter_delta("query_bridge.oracle_interval_replay_compatible",
                      after.interval_replay_compatible,
                      before.interval_replay_compatible);
    add_counter_delta("query_bridge.oracle_interval_replay_incompatible",
                      after.interval_replay_incompatible,
                      before.interval_replay_incompatible);
    add_counter_delta("query_bridge.oracle_interval_replay_direct_exact_hits",
                      after.interval_replay_direct_exact_hits,
                      before.interval_replay_direct_exact_hits);
    add_counter_delta("query_bridge.oracle_interval_replay_key_only_blocked",
                      after.interval_replay_key_only_blocked,
                      before.interval_replay_key_only_blocked);
    add_counter_delta("query_bridge.oracle_shared_endpoint_cache_hits",
                      after.materialization_reused_shared_endpoint_cache,
                      before.materialization_reused_shared_endpoint_cache);
    add_counter_delta("query_bridge.oracle_endpoint_path_ms",
                      after.validate_node_endpoint_path_time_us * 1.0e-3,
                      before.validate_node_endpoint_path_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_classify_ms",
                      after.validate_node_classify_time_us * 1.0e-3,
                      before.validate_node_classify_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_validate_total_ms",
                      after.validate_node_total_time_us * 1.0e-3,
                      before.validate_node_total_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_materialization_endpoint_ms",
                      after.materialization_endpoint_time_us * 1.0e-3,
                      before.materialization_endpoint_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_materialization_envelope_ms",
                      after.materialization_envelope_time_us * 1.0e-3,
                      before.materialization_envelope_time_us * 1.0e-3);
    add_counter_delta("query_bridge.oracle_envelope_collision_queries",
                      after.envelope_collision_queries,
                      before.envelope_collision_queries);
    add_counter_delta("query_bridge.oracle_envelope_gjk_tests",
                      after.envelope_collision_gjk_tests,
                      before.envelope_collision_gjk_tests);
}

std::string query_bridge_task_key(std::size_t index, const std::string& suffix) {
    return "query_bridge.batch_task." + std::to_string(index) + "." + suffix;
}

double query_bridge_point_segment_distance_sq(const Eigen::VectorXd& point,
                                              const Eigen::VectorXd& a,
                                              const Eigen::VectorXd& b) {
    if (point.size() != a.size() || point.size() != b.size()) {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::VectorXd ab = b - a;
    const double denom = ab.squaredNorm();
    if (denom <= 1e-18) {
        return (point - a).squaredNorm();
    }
    const double t = std::clamp((point - a).dot(ab) / denom, 0.0, 1.0);
    return (point - (a + t * ab)).squaredNorm();
}

double query_bridge_point_polyline_distance_sq(
    const Eigen::VectorXd& point,
    const std::vector<Eigen::VectorXd>& path) {
    if (path.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index < path.size(); ++index) {
        best = std::min(best,
                        query_bridge_point_segment_distance_sq(point,
                                                               path[index - 1],
                                                               path[index]));
    }
    if (path.size() == 1) {
        best = (point - path.front()).squaredNorm();
    }
    return best;
}

bool query_bridge_result_acceptable(const QueryResult& current,
                                    const Eigen::VectorXd& start,
                                    const Eigen::VectorXd& goal,
                                    const QueryBridgeAcceptanceThresholds& thresholds) {
    if (!current.success || !current.audit_passed) {
        return false;
    }
    const double raw_length =
        current.raw_path_length > 1e-12 ? current.raw_path_length : current.path_length;
    const double segment_fraction =
        raw_length > 1e-12 ? current.segment_edge_length / raw_length
                           : std::numeric_limits<double>::infinity();
    if (!(segment_fraction <= thresholds.max_segment_fraction)) {
        return false;
    }
    const double direct = (goal - start).norm();
    return direct <= 1e-9 ||
           current.path_length <= std::max(direct * thresholds.path_ratio,
                                            direct + thresholds.path_additive) ||
           current.path_length <= thresholds.max_path_length;
}

void accumulate_query_bridge_direct_corridor_totals(const BuildProfile& profile,
                                                    StageContext& context,
                                                    std::size_t task_index) {
    auto add = [&](const std::string& suffix, const std::string& total_key) {
        const auto it = profile.diagnostics.find(query_bridge_task_key(task_index, suffix));
        if (it != profile.diagnostics.end()) {
            context.diagnostics().add_counter(total_key, it->second);
        }
    };
    add("direct_corridor_ms", "query_bridge.direct_corridor_ms_total");
    add("direct_corridor_samples", "query_bridge.direct_corridor_samples_total");
    add("direct_corridor_ffb_calls", "query_bridge.direct_corridor_ffb_calls_total");
    add("direct_corridor_all_ffb_calls", "query_bridge.direct_corridor_all_ffb_calls_total");
    add("direct_corridor_direct_ffb_ms", "query_bridge.direct_corridor_direct_ffb_ms");
    add("direct_corridor_repair_ffb_ms", "query_bridge.direct_corridor_repair_ffb_ms");
    add("direct_corridor_adaptive_repair_ffb_ms",
        "query_bridge.direct_corridor_adaptive_repair_ffb_ms");
    add("direct_corridor_lateral_repair_ffb_ms",
        "query_bridge.direct_corridor_lateral_repair_ffb_ms");
    add("direct_corridor_segment_audit_ms",
        "query_bridge.direct_corridor_segment_audit_ms");
    add("direct_corridor_added", "query_bridge.direct_corridor_added_total");
    add("direct_corridor_repair_calls", "query_bridge.direct_corridor_repair_calls_total");
    add("direct_corridor_repair_added", "query_bridge.direct_corridor_repair_added_total");
    add("direct_corridor_adaptive_repair_calls",
        "query_bridge.direct_corridor_adaptive_repair_calls_total");
    add("direct_corridor_adaptive_repair_added",
        "query_bridge.direct_corridor_adaptive_repair_added_total");
    add("direct_corridor_lateral_repair_calls",
        "query_bridge.direct_corridor_lateral_repair_calls_total");
    add("direct_corridor_lateral_repair_added",
        "query_bridge.direct_corridor_lateral_repair_added_total");
    add("direct_corridor_bad_initial", "query_bridge.direct_corridor_bad_initial_total");
    add("direct_corridor_bad_final", "query_bridge.direct_corridor_bad_final_total");
    add("direct_corridor_segment_edges", "query_bridge.direct_corridor_segment_edges_total");
    add("direct_corridor_ffb_find_calls", "query_bridge.direct_corridor_ffb_find_calls_total");
    add("direct_corridor_ffb_binary_requested",
        "query_bridge.direct_corridor_ffb_binary_requested_total");
    add("direct_corridor_ffb_virtual_sparse_binary_attempts",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_attempts_total");
    add("direct_corridor_ffb_virtual_sparse_binary_successes",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_successes_total");
    add("direct_corridor_ffb_virtual_sparse_binary_probes",
        "query_bridge.direct_corridor_ffb_virtual_sparse_binary_probes_total");
    add("direct_corridor_ffb_binary_materialized_fallback_calls",
        "query_bridge.direct_corridor_ffb_binary_materialized_fallback_calls_total");
    add("direct_corridor_ffb_binary_blocked_adaptive_depths",
        "query_bridge.direct_corridor_ffb_binary_blocked_adaptive_depths_total");
    add("direct_corridor_ffb_binary_virtual_unsupported",
        "query_bridge.direct_corridor_ffb_binary_virtual_unsupported_total");
    add("direct_corridor_ffb_linear_descent_calls",
        "query_bridge.direct_corridor_ffb_linear_descent_calls_total");
    add("direct_corridor_transition_connected_ms",
        "query_bridge.direct_corridor_transition_connected_ms");
    add("direct_corridor_transition_connected_calls",
        "query_bridge.direct_corridor_transition_connected_calls");
    add("direct_corridor_bad_transitions_ms",
        "query_bridge.direct_corridor_bad_transitions_ms");
    add("direct_corridor_bad_transitions_calls",
        "query_bridge.direct_corridor_bad_transitions_calls");
    add("direct_corridor_current_cover_ms", "query_bridge.direct_corridor_current_cover_ms");
    add("direct_corridor_current_cover_calls", "query_bridge.direct_corridor_current_cover_calls");
    add("direct_corridor_current_cover_partition_ms",
        "query_bridge.direct_corridor_current_cover_partition_ms");
    add("direct_corridor_current_cover_corridor_scan_ms",
        "query_bridge.direct_corridor_current_cover_corridor_scan_ms");
    add("direct_corridor_current_cover_direct_index_ms",
        "query_bridge.direct_corridor_current_cover_direct_index_ms");
    add("direct_corridor_duplicate_lookup_ms", "query_bridge.direct_corridor_duplicate_lookup_ms");
    add("direct_corridor_duplicate_lookup_calls",
        "query_bridge.direct_corridor_duplicate_lookup_calls");
    add("direct_corridor_commit_total_ms", "query_bridge.direct_corridor_commit_total_ms");
    add("direct_corridor_commit_calls", "query_bridge.direct_corridor_commit_calls");
    add("direct_corridor_commit_dynamic_policy_ms",
        "query_bridge.direct_corridor_commit_dynamic_policy_ms");
    add("direct_corridor_commit_partition_append_ms",
        "query_bridge.direct_corridor_commit_partition_append_ms");
    add("direct_corridor_partition_append_calls",
        "query_bridge.direct_corridor_partition_append_calls");
    add("direct_corridor_partition_append_boxes",
        "query_bridge.direct_corridor_partition_append_boxes");
    add("direct_corridor_assimilate_calls", "query_bridge.direct_corridor_assimilate_calls");
    add("direct_corridor_assimilate_sample_scan_ms",
        "query_bridge.direct_corridor_assimilate_sample_scan_ms");
    add("direct_corridor_assimilate_local_hits",
        "query_bridge.direct_corridor_assimilate_local_hits");
    add("direct_corridor_assimilate_full_scan_fallbacks",
        "query_bridge.direct_corridor_assimilate_full_scan_fallbacks");
    add("direct_corridor_assimilate_local_sample_tests",
        "query_bridge.direct_corridor_assimilate_local_sample_tests");
    add("direct_corridor_assimilate_candidate_build_ms",
        "query_bridge.direct_corridor_assimilate_candidate_build_ms");
    add("direct_corridor_assimilate_adjacency_ms",
        "query_bridge.direct_corridor_assimilate_adjacency_ms");
    add("direct_corridor_segment_insert_ms", "query_bridge.direct_corridor_segment_insert_ms");
    add("direct_corridor_segment_insert_calls",
        "query_bridge.direct_corridor_segment_insert_calls");
    add("direct_corridor_direct_task_build_ms",
        "query_bridge.direct_corridor_direct_task_build_ms");
    add("direct_corridor_direct_loop_ms", "query_bridge.direct_corridor_direct_loop_ms");
    add("direct_corridor_repair_loop_ms", "query_bridge.direct_corridor_repair_loop_ms");
    add("direct_corridor_adaptive_loop_ms", "query_bridge.direct_corridor_adaptive_loop_ms");
    add("direct_corridor_lateral_loop_ms", "query_bridge.direct_corridor_lateral_loop_ms");
    add("direct_corridor_residual_segment_loop_ms",
        "query_bridge.direct_corridor_residual_segment_loop_ms");
}

}  // namespace rbf
