#include "planning_forest_diagnostics.h"

#include <algorithm>

namespace rbf {

double boundary_max_depth_failure_count_local(const StageContext& context) {
    const auto& diagnostics = context.diagnostics();
    return diagnostics.value("connector.chain_pave_boundary_fail_depth_cap", 0.0) +
           diagnostics.value("connector.chain_pave_boundary_fail_unknown_depth_cap", 0.0) +
           diagnostics.value("connector.chain_pave_boundary_fail_reserved_depth_cap", 0.0);
}

double diagnostic_map_value(const std::unordered_map<std::string, double>& diagnostics,
                            const std::string& key) {
    const auto it = diagnostics.find(key);
    return it == diagnostics.end() ? 0.0 : it->second;
}

void set_diagnostic_max(std::unordered_map<std::string, double>& diagnostics,
                        const std::string& key,
                        double value) {
    auto it = diagnostics.find(key);
    if (it == diagnostics.end() || value > it->second) {
        diagnostics[key] = value;
    }
}

double portal_membership_policy_code(PortalMembershipPolicy policy) {
    switch (policy) {
    case PortalMembershipPolicy::GlobalForestOnly:
        return 0.0;
    case PortalMembershipPolicy::PortalInteriorIndex:
        return 1.0;
    }
    return -1.0;
}

void record_portal_membership_policy(std::unordered_map<std::string, double>& diagnostics,
                                     PortalMembershipPolicy policy,
                                     const std::string& prefix) {
    diagnostics[prefix + "policy"] = portal_membership_policy_code(policy);
    diagnostics[prefix + "global_forest_only"] =
        policy == PortalMembershipPolicy::GlobalForestOnly ? 1.0 : 0.0;
    diagnostics[prefix + "portal_interior_index"] =
        policy == PortalMembershipPolicy::PortalInteriorIndex ? 1.0 : 0.0;
    if (policy == PortalMembershipPolicy::PortalInteriorIndex) {
        diagnostics[prefix + "portal_interior_index_requested"] += 1.0;
        diagnostics[prefix + "portal_interior_index_unavailable"] += 1.0;
        diagnostics[prefix + "global_forest_only_fallback"] += 1.0;
    }
}

void record_portal_membership_policy(StageDiagnostics& diagnostics,
                                     PortalMembershipPolicy policy,
                                     const std::string& prefix) {
    diagnostics.set_value(prefix + "policy", portal_membership_policy_code(policy));
    diagnostics.set_value(prefix + "global_forest_only",
                          policy == PortalMembershipPolicy::GlobalForestOnly ? 1.0 : 0.0);
    diagnostics.set_value(prefix + "portal_interior_index",
                          policy == PortalMembershipPolicy::PortalInteriorIndex ? 1.0 : 0.0);
    if (policy == PortalMembershipPolicy::PortalInteriorIndex) {
        diagnostics.add_counter(prefix + "portal_interior_index_requested");
        diagnostics.add_counter(prefix + "portal_interior_index_unavailable");
        diagnostics.add_counter(prefix + "global_forest_only_fallback");
    }
}

bool is_latched_diagnostic_key(const std::string& key) {
    return key == "portal_membership.policy" ||
           key == "portal_membership.global_forest_only" ||
           key == "portal_membership.portal_interior_index" ||
           key == "ffb.free_ancestor_depth_max" ||
           key == "ffb.virtual_sparse_binary_probe_depth_max";
}

void merge_diagnostic_snapshot(std::unordered_map<std::string, double>& diagnostics,
                               const std::unordered_map<std::string, double>& snapshot) {
    for (const auto& [key, value] : snapshot) {
        if (is_latched_diagnostic_key(key)) {
            set_diagnostic_max(diagnostics, key, value);
        } else {
            diagnostics[key] += value;
        }
    }
}

void merge_diagnostic_snapshot(StageDiagnostics& diagnostics,
                               const std::unordered_map<std::string, double>& snapshot) {
    for (const auto& [key, value] : snapshot) {
        if (is_latched_diagnostic_key(key)) {
            diagnostics.set_value(key, std::max(diagnostics.value(key), value));
        } else {
            diagnostics.add_counter(key, value);
        }
    }
}

void record_oracle_cache_counter_snapshot(std::unordered_map<std::string, double>& diagnostics,
                                          const OracleCounters& counters,
                                          const std::string& prefix) {
    diagnostics[prefix + "node_validations"] =
        static_cast<double>(counters.node_validations);
    diagnostics[prefix + "interval_validations"] =
        static_cast<double>(counters.interval_validations);
    diagnostics[prefix + "certified_free"] =
        static_cast<double>(counters.certified_free);
    diagnostics[prefix + "certified_occupied"] =
        static_cast<double>(counters.certified_occupied);
    diagnostics[prefix + "materializations"] =
        static_cast<double>(counters.materializations);
    diagnostics[prefix + "materialization_reused_external_evidence"] =
        static_cast<double>(counters.materialization_reused_external_evidence);
    diagnostics[prefix + "materialization_external_exact_hits"] =
        static_cast<double>(counters.materialization_external_exact_hits);
    diagnostics[prefix + "materialization_external_exact_misses"] =
        static_cast<double>(counters.materialization_external_exact_misses);
    diagnostics[prefix + "materialization_external_live_fallbacks"] =
        static_cast<double>(counters.materialization_external_live_fallbacks);
    diagnostics[prefix + "materialization_external_maybe_live_retries"] =
        static_cast<double>(counters.materialization_external_maybe_live_retries);
    diagnostics[prefix + "materialization_external_maybe_live_retry_free"] =
        static_cast<double>(counters.materialization_external_maybe_live_retry_free);
    diagnostics[prefix + "interval_replay_compatibility_checks"] =
        static_cast<double>(counters.interval_replay_compatibility_checks);
    diagnostics[prefix + "interval_replay_compatible"] =
        static_cast<double>(counters.interval_replay_compatible);
    diagnostics[prefix + "interval_replay_incompatible"] =
        static_cast<double>(counters.interval_replay_incompatible);
    diagnostics[prefix + "interval_replay_direct_exact_hits"] =
        static_cast<double>(counters.interval_replay_direct_exact_hits);
    diagnostics[prefix + "interval_replay_key_only_blocked"] =
        static_cast<double>(counters.interval_replay_key_only_blocked);
    diagnostics[prefix + "canonical_frame_invalid"] =
        static_cast<double>(counters.canonical_frame_invalid);
    diagnostics[prefix + "canonical_reflected_seed_misses"] =
        static_cast<double>(counters.canonical_reflected_seed_misses);
    diagnostics[prefix + "scoring_reused_external_evidence"] =
        static_cast<double>(counters.scoring_reused_external_evidence);
}

void record_depth_semantics_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                        const std::string& prefix,
                                        int sweep_start_depth,
                                        int sweep_max_depth,
                                        int target_max_depth,
                                        const FindFreeBoxOptions& seed_ffb_options,
                                        int deep_ffb_depth) {
    diagnostics[prefix + "sweep_start_depth"] =
        static_cast<double>(sweep_start_depth);
    diagnostics[prefix + "sweep_max_depth"] =
        static_cast<double>(sweep_max_depth);
    diagnostics[prefix + "target_max_depth"] =
        static_cast<double>(target_max_depth);
    diagnostics[prefix + "seed_ffb_start_depth"] =
        static_cast<double>(seed_ffb_options.start_depth);
    diagnostics[prefix + "seed_ffb_skip_to_depth"] =
        static_cast<double>(seed_ffb_options.skip_to_depth);
    diagnostics[prefix + "seed_ffb_max_depth"] =
        static_cast<double>(seed_ffb_options.max_depth);
    diagnostics[prefix + "deep_ffb_depth"] =
        static_cast<double>(deep_ffb_depth);
    diagnostics[prefix + "sweep_seed_ffb_depths_independent"] = 1.0;
}

void normalize_external_evidence_diagnostics(
    std::unordered_map<std::string, double>& diagnostics,
    const OracleCounters* active_oracle_counters) {
    if (active_oracle_counters != nullptr) {
        record_oracle_cache_counter_snapshot(diagnostics,
                                             *active_oracle_counters,
                                             "adaptive.oracle.");
    }

    auto max_of = [&](std::initializer_list<const char*> keys) {
        double value = 0.0;
        for (const char* key : keys) {
            value = std::max(value, diagnostic_map_value(diagnostics, key));
        }
        return value;
    };

    const double reused_hits = max_of({
        "oracle.materialization_reused_external_evidence",
        "adaptive.oracle.materialization_reused_external_evidence",
        "leaf_sweep.worker_oracle.materialization_reused_external_evidence",
        "grower.worker_oracle.materialization_reused_external_evidence",
    });
    const double exact_hits = max_of({
        "oracle.materialization_external_exact_hits",
        "adaptive.oracle.materialization_external_exact_hits",
        "leaf_sweep.worker_oracle.materialization_external_exact_hits",
        "grower.worker_oracle.materialization_external_exact_hits",
    });
    const double exact_misses = max_of({
        "oracle.materialization_external_exact_misses",
        "adaptive.oracle.materialization_external_exact_misses",
        "leaf_sweep.worker_oracle.materialization_external_exact_misses",
        "grower.worker_oracle.materialization_external_exact_misses",
    });
    const double replay_checks = max_of({
        "oracle.interval_replay_compatibility_checks",
        "adaptive.oracle.interval_replay_compatibility_checks",
        "leaf_sweep.worker_oracle.interval_replay_compatibility_checks",
        "grower.worker_oracle.interval_replay_compatibility_checks",
    });
    const double replay_compatible = max_of({
        "oracle.interval_replay_compatible",
        "adaptive.oracle.interval_replay_compatible",
        "leaf_sweep.worker_oracle.interval_replay_compatible",
        "grower.worker_oracle.interval_replay_compatible",
    });
    const double replay_incompatible = max_of({
        "oracle.interval_replay_incompatible",
        "adaptive.oracle.interval_replay_incompatible",
        "leaf_sweep.worker_oracle.interval_replay_incompatible",
        "grower.worker_oracle.interval_replay_incompatible",
    });
    const double replay_direct_hits = max_of({
        "oracle.interval_replay_direct_exact_hits",
        "adaptive.oracle.interval_replay_direct_exact_hits",
        "leaf_sweep.worker_oracle.interval_replay_direct_exact_hits",
        "grower.worker_oracle.interval_replay_direct_exact_hits",
    });
    const double replay_key_only_blocked = max_of({
        "oracle.interval_replay_key_only_blocked",
        "adaptive.oracle.interval_replay_key_only_blocked",
        "leaf_sweep.worker_oracle.interval_replay_key_only_blocked",
        "grower.worker_oracle.interval_replay_key_only_blocked",
    });

    set_diagnostic_max(diagnostics,
                       "oracle.materialization_reused_external_evidence",
                       reused_hits);
    set_diagnostic_max(diagnostics,
                       "oracle.materialization_external_exact_hits",
                       exact_hits);
    set_diagnostic_max(diagnostics,
                       "oracle.materialization_external_exact_misses",
                       exact_misses);
    diagnostics["adaptive.external_reused_hits_normalized"] = reused_hits;
    diagnostics["adaptive.external_exact_hits_normalized"] = exact_hits;
    diagnostics["adaptive.external_exact_misses_normalized"] = exact_misses;
    diagnostics["adaptive.interval_replay_compatibility_checks_normalized"] = replay_checks;
    diagnostics["adaptive.interval_replay_compatible_normalized"] = replay_compatible;
    diagnostics["adaptive.interval_replay_incompatible_normalized"] = replay_incompatible;
    diagnostics["adaptive.interval_replay_direct_exact_hits_normalized"] = replay_direct_hits;
    diagnostics["adaptive.interval_replay_key_only_blocked_normalized"] = replay_key_only_blocked;
}


} // namespace rbf
