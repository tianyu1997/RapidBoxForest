#include "grower_options.h"

#include <SBF/runtime.h>

#include "grower_internal.h"

#include <algorithm>
#include <string>

namespace rbf {

int select_depth_stage_index(const GrowerConfig& config, int box_count) {
    if (config.depth_stages.empty()) {
        return -1;
    }
    for (int index = 0; index < static_cast<int>(config.depth_stages.size()); ++index) {
        const auto& stage = config.depth_stages[static_cast<std::size_t>(index)];
        if (stage.box_limit <= 0 || box_count < stage.box_limit) {
            return index;
        }
    }
    return static_cast<int>(config.depth_stages.size()) - 1;
}

const GrowerConfig::DepthStage* depth_stage_or_null(const GrowerConfig& config,
                                                    int stage_index) {
    if (stage_index < 0 || stage_index >= static_cast<int>(config.depth_stages.size())) {
        return nullptr;
    }
    return &config.depth_stages[static_cast<std::size_t>(stage_index)];
}

FindFreeBoxOptions staged_ffb_options(const GrowerConfig& config, int stage_index) {
    FindFreeBoxOptions options = config.find_free_box;
    const auto* stage = depth_stage_or_null(config, stage_index);
    if (stage != nullptr && stage->ffb_depth > 0) {
        options.max_depth = stage->ffb_depth;
    }
    return options;
}

FindFreeBoxOptions component_connect_ffb_options(const GrowerConfig& config,
                                                 StageContext& context,
                                                 const FindFreeBoxOptions& base_options,
                                                 int stage_index,
                                                 int pair_unknown_failures) {
    FindFreeBoxOptions options = base_options;
    if (!config.component_connect_adaptive_ffb) {
        return options;
    }
    if (config.component_connect_depth_after_unknown_only && pair_unknown_failures <= 0) {
        context.diagnostics().add_counter("grower.component_connect_base_depth_tasks");
        return options;
    }
    const auto* stage = depth_stage_or_null(config, stage_index);
    const int depth_increment = stage != nullptr && stage->component_connect_ffb_depth_increment >= 0
        ? stage->component_connect_ffb_depth_increment
        : config.component_connect_ffb_depth_increment;
    const int configured_max_depth = stage != nullptr && stage->component_connect_ffb_max_depth > 0
        ? stage->component_connect_ffb_max_depth
        : config.component_connect_ffb_max_depth;
    const int base_depth = std::max(0, options.max_depth);
    const int max_depth = std::max(base_depth, configured_max_depth);
    options.max_depth = std::min(max_depth,
                                 base_depth + std::max(0, depth_increment) *
                                                  std::max(1, pair_unknown_failures));
    context.diagnostics().add_counter("grower.component_connect_adaptive_ffb_tasks");
    if (pair_unknown_failures > 0) {
        context.diagnostics().add_counter("grower.component_connect_unknown_depth_retry_tasks");
        set_grower_max_diagnostic(context,
                           "grower.component_connect_pair_unknown_failures_max",
                           static_cast<double>(pair_unknown_failures));
    }
    set_grower_max_diagnostic(context,
                       "grower.component_connect_adaptive_ffb_depth_max",
                       static_cast<double>(options.max_depth));
    return options;
}

void record_grower_ffb_failure(StageContext& context,
                               const FindFreeBoxResult& result) {
    context.diagnostics().add_counter("grower.ffb_failures");
    context.diagnostics().add_counter("grower.ffb_fail_code." + std::to_string(result.fail_code));
    if (result.seed_collision) {
        context.diagnostics().add_counter("grower.ffb_seed_collision");
    }
    if (result.hit_unknown_depth_cap) {
        context.diagnostics().add_counter("grower.ffb_unknown_depth_cap");
    }
    if (result.hit_reserved_depth_cap) {
        context.diagnostics().add_counter("grower.ffb_reserved_depth_cap");
    }
    if (result.deadline_reached) {
        context.diagnostics().add_counter("grower.ffb_deadline_reached");
    }
}

void record_worker_oracle_counters(StageContext& context,
                                   const OracleCounters& counters) {
    context.diagnostics().add_counter("grower.worker_oracle.node_validations", counters.node_validations);
    context.diagnostics().add_counter("grower.worker_oracle.certified_free", counters.certified_free);
    context.diagnostics().add_counter("grower.worker_oracle.certified_occupied", counters.certified_occupied);
    context.diagnostics().add_counter("grower.worker_oracle.materializations", counters.materializations);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_source_incremental_state",
                                      counters.materialization_source_incremental_state);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_reused_endpoint_cache",
                                      counters.materialization_reused_endpoint_cache);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_reused_external_evidence",
                                      counters.materialization_reused_external_evidence);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_external_exact_hits",
                                      counters.materialization_external_exact_hits);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_external_exact_misses",
                                      counters.materialization_external_exact_misses);
    context.diagnostics().add_counter("grower.worker_oracle.interval_replay_compatibility_checks",
                                      counters.interval_replay_compatibility_checks);
    context.diagnostics().add_counter("grower.worker_oracle.interval_replay_compatible",
                                      counters.interval_replay_compatible);
    context.diagnostics().add_counter("grower.worker_oracle.interval_replay_incompatible",
                                      counters.interval_replay_incompatible);
    context.diagnostics().add_counter("grower.worker_oracle.interval_replay_direct_exact_hits",
                                      counters.interval_replay_direct_exact_hits);
    context.diagnostics().add_counter("grower.worker_oracle.interval_replay_key_only_blocked",
                                      counters.interval_replay_key_only_blocked);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_reused_shared_endpoint_cache",
                                      counters.materialization_reused_shared_endpoint_cache);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_stored_shared_endpoint_cache",
                                      counters.materialization_stored_shared_endpoint_cache);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_reused_cached_envelope",
                                      counters.materialization_reused_cached_envelope);
    context.diagnostics().add_counter("grower.worker_oracle.canonical_frame_invalid",
                                      counters.canonical_frame_invalid);
    context.diagnostics().add_counter("grower.worker_oracle.canonical_reflected_seed_misses",
                                      counters.canonical_reflected_seed_misses);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_endpoint_time_us",
                                      counters.materialization_endpoint_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_endpoint_wall_time_us",
                                      counters.materialization_endpoint_wall_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.validate_node_total_time_us",
                                      counters.validate_node_total_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.validate_node_preamble_time_us",
                                      counters.validate_node_preamble_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.validate_node_endpoint_path_time_us",
                                      counters.validate_node_endpoint_path_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.validate_node_classify_time_us",
                                      counters.validate_node_classify_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.validate_node_overhead_time_us",
                                      counters.validate_node_overhead_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_envelope_time_us",
                                      counters.materialization_envelope_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_cache_lookup_time_us",
                                      counters.materialization_cache_lookup_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_cache_read_time_us",
                                      counters.materialization_cache_read_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_external_lookup_time_us",
                                      counters.materialization_external_lookup_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_external_read_time_us",
                                      counters.materialization_external_read_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_envelope_compute_time_us",
                                      counters.materialization_envelope_compute_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_envelope_read_time_us",
                                      counters.materialization_envelope_read_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_envelope_collision_time_us",
                                      counters.materialization_envelope_collision_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_candidate_dirty_count",
                                      counters.materialization_candidate_dirty_count);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_predh_rebuild_count",
                                      counters.materialization_predh_rebuild_count);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_evaluations", counters.scoring_evaluations);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_changed_dim_inferred",
                                      counters.scoring_changed_dim_inferred);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_source_incremental_state",
                                      counters.scoring_source_incremental_state);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_reused_endpoint_cache",
                                      counters.scoring_reused_endpoint_cache);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_reused_external_evidence",
                                      counters.scoring_reused_external_evidence);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_endpoint_time_us",
                                      counters.scoring_endpoint_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_envelope_time_us",
                                      counters.scoring_envelope_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_candidate_dirty_count",
                                      counters.scoring_candidate_dirty_count);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_predh_rebuild_count",
                                      counters.scoring_predh_rebuild_count);
}

}  // namespace rbf
