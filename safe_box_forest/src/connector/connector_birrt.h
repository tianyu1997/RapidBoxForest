#pragma once

#include <SBF/connector_types.h>
#include <SBF/runtime_fwd.h>
#include <SBF/scene_types.h>

#include <Eigen/Core>

#include <atomic>
#include <memory>
#include <vector>

namespace rbf {

class CollisionChecker;

struct RRTConnectStats {
    bool success = false;
    bool direct = false;
    bool cancelled = false;
    bool timed_out = false;
    int iterations = 0;
    int merge_iteration = -1;
    int forward_tree_size = 0;
    int backward_tree_size = 0;
    int raw_waypoints = 0;
    int shortcut_waypoints = 0;
    double elapsed_ms = 0.0;
};

struct RRTConnectOutcome {
    std::vector<Eigen::VectorXd> path;
    RRTConnectStats stats;
};

RRTConnectOutcome birrt_connect_impl(const Eigen::Ref<const Eigen::VectorXd>& start,
                                     const Eigen::Ref<const Eigen::VectorXd>& goal,
                                     const CollisionChecker& checker,
                                     const Robot& robot,
                                     const RRTConnectConfig& config,
                                     int seed,
                                     std::shared_ptr<std::atomic<bool>> cancel);

void record_birrt_stats(StageDiagnostics& diagnostics, const RRTConnectStats& stats);

}  // namespace rbf
