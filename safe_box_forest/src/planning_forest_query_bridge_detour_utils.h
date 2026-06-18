#pragma once

#include <SBF/safe_box_forest.h>

#include <Eigen/Core>

#include <vector>

namespace rbf {

struct QueryBridgeSearchTask;

struct QueryBridgeDetourOptions {
    bool enabled = false;
    bool candidate = false;
    double replace_factor = 1.0;
    int dims = 4;
    int rounds = 2;
    int max_candidates = 32;
    bool multi_axis = false;
    int random_candidates = 0;
    double offset = 0.35;
    double two_bend_alpha = 0.35;
};

QueryBridgeDetourOptions query_bridge_detour_options_from_env();

void record_query_bridge_detour_diagnostics(StageContext& context,
                                            const QueryBridgeDetourOptions& options);

std::vector<Eigen::VectorXd> query_bridge_deterministic_detour_fallback_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const std::vector<Interval>& planning_domain,
    const QueryBridgeDetourOptions& options,
    int rng_seed_base,
    StageContext& context);

bool query_bridge_maybe_apply_detour_path(
    const QueryBridgeSearchTask& task,
    const Robot& audit_robot,
    const Scene& scene,
    const QueryConfig& query_config,
    const std::vector<Interval>& planning_domain,
    const QueryBridgeDetourOptions& options,
    int rng_seed_base,
    StageContext& context,
    double& best_length,
    std::vector<Eigen::VectorXd>& waypoint_path);

}  // namespace rbf
