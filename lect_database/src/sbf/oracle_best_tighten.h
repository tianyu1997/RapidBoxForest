#pragma once

#include <LECTDatabase/sbf/oracle.h>

#include <sbf/core/joint_symmetry.h>

#include <functional>
#include <optional>
#include <vector>

namespace rbf {

using BestTightenScoreFn = std::function<double(const std::vector<Interval>&)>;

struct BestTightenCandidate {
    bool valid = false;
    int dim = -1;
    double split_val = 0.0;
    double parent_score = 0.0;
    double left_score = 0.0;
    double right_score = 0.0;
    double reduction = 0.0;
    double minimax_score = 0.0;
    double balanced_score = 0.0;
    double normalized_reduction = 0.0;
    double score_imbalance = 0.0;
    double parent_aspect = 1.0;
    double child_aspect = 1.0;
    double split_width_fraction = 1.0;
    double shape_penalty = 0.0;
    double recent_dim_fraction = 0.0;
    double recent_dim_penalty = 0.0;
    bool sector_aligned = false;
    bool shape_healthy = true;
};

std::vector<double> link_aabb_volumes(const std::vector<float>& link_aabbs);

double normalized_link_aabb_volume_score(
    const std::vector<float>& link_aabbs,
    const std::vector<double>& reference_volumes);

BestTightenCandidate choose_best_tighten_split(
    const std::vector<Interval>& intervals,
    int depth,
    const BestTightenScoreFn& scorer,
    const std::optional<JointSymmetry>& symmetry,
    const BestTightenOptions& options,
    std::vector<int>& depth_dims,
    std::vector<int>& recent_dims);

BestTightenOptions with_fk_effective_split_filter(
    BestTightenOptions options,
    const Robot& robot);

}  // namespace rbf
