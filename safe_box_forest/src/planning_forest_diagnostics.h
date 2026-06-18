#pragma once

#include <SBF/safe_box_forest.h>

#include <string>
#include <unordered_map>

namespace rbf {

bool legacy_query_boxcorridor_enabled();

double boundary_max_depth_failure_count_local(const StageContext& context);

double diagnostic_map_value(const std::unordered_map<std::string, double>& diagnostics,
                            const std::string& key);

void set_diagnostic_max(std::unordered_map<std::string, double>& diagnostics,
                        const std::string& key,
                        double value);

void record_portal_membership_policy(std::unordered_map<std::string, double>& diagnostics,
                                     PortalMembershipPolicy policy,
                                     const std::string& prefix = "portal_membership.");

void record_portal_membership_policy(StageDiagnostics& diagnostics,
                                     PortalMembershipPolicy policy,
                                     const std::string& prefix = "portal_membership.");

void merge_diagnostic_snapshot(std::unordered_map<std::string, double>& diagnostics,
                               const std::unordered_map<std::string, double>& snapshot);

void merge_diagnostic_snapshot(StageDiagnostics& diagnostics,
                               const std::unordered_map<std::string, double>& snapshot);

void record_oracle_cache_counter_snapshot(std::unordered_map<std::string, double>& diagnostics,
                                          const OracleCounters& counters,
                                          const std::string& prefix);

void record_depth_semantics_diagnostics(std::unordered_map<std::string, double>& diagnostics,
                                        const std::string& prefix,
                                        int sweep_start_depth,
                                        int sweep_max_depth,
                                        int target_max_depth,
                                        const FindFreeBoxOptions& seed_ffb_options,
                                        int deep_ffb_depth);

void normalize_external_evidence_diagnostics(
    std::unordered_map<std::string, double>& diagnostics,
    const OracleCounters* active_oracle_counters = nullptr);

} // namespace rbf
