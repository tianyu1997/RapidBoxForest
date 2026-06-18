#include "planning_forest_query_bridge_batch_utils.h"

#include <string>

namespace rbf {

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

}  // namespace rbf
