// RBFPlanningForest v6 — Endpoint Source unified dispatch (Phase B5)
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/crit_source.h>
#include <sbf/envelope/analytical_source.h>
#include <sbf/envelope/gcpc_source.h>
#include <sbf/envelope/mc_source.h>
#include <sbf/envelope/ifk_aa_source.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr double kHifkIfkWidthThresh = 0.05;
constexpr double kHifkWideMaxWidthThresh = 0.35;
constexpr double kHifkWideRmsWidthThresh = 0.20;
constexpr double kHifkDepth5GainThresh = 0.18;
constexpr double kHifkDepth5MeasureThresh = 0.10;
constexpr double kPlanarExtentEpsilon = 1e-12;

struct HifkWidthStats {
    double max_width = 0.0;
    double rms_width = 0.0;
};

struct HifkSplitEstimate {
    double parent_measure = 0.0;
    double best_gain = 0.0;
    int best_dim = -1;
};

HifkWidthStats hifk_width_stats(const std::vector<rbf::Interval>& intervals) {
    HifkWidthStats stats;
    if (intervals.empty()) {
        return stats;
    }

    double sum_sq_width = 0.0;
    for (const rbf::Interval& interval : intervals) {
        const double width = std::max(0.0, interval.width());
        stats.max_width = std::max(stats.max_width, width);
        sum_sq_width += width * width;
    }
    stats.rms_width = std::sqrt(sum_sq_width / static_cast<double>(intervals.size()));
    return stats;
}

double endpoint_split_measure(const std::vector<float>& iaabbs, int n_active_links) {
    bool planar_xy = true;
    for (int endpoint = 0; endpoint < n_active_links * 2; ++endpoint) {
        const float* box = iaabbs.data() + endpoint * 6;
        if (std::abs(static_cast<double>(box[5] - box[2])) > kPlanarExtentEpsilon) {
            planar_xy = false;
            break;
        }
    }

    double measure = 0.0;
    for (int endpoint = 0; endpoint < n_active_links * 2; ++endpoint) {
        const float* box = iaabbs.data() + endpoint * 6;
        const double dx = std::max(0.0, static_cast<double>(box[3] - box[0]));
        const double dy = std::max(0.0, static_cast<double>(box[4] - box[1]));
        const double dz = std::max(0.0, static_cast<double>(box[5] - box[2]));
        measure += planar_xy ? dx * dy : dx * dy * dz;
    }
    return measure;
}

HifkSplitEstimate estimate_best_aafk_split_gain(
    const rbf::Robot& robot,
    const std::vector<rbf::Interval>& intervals)
{
    HifkSplitEstimate estimate;
    if (intervals.empty()) {
        return estimate;
    }

    const rbf::EndpointIAABBResult parent = rbf::compute_endpoint_iaabb_ifk_aa(robot, intervals);
    estimate.parent_measure = endpoint_split_measure(parent.endpoint_iaabbs, parent.n_active_links);
    if (estimate.parent_measure <= 1e-15) {
        return estimate;
    }

    double best_hull_measure = std::numeric_limits<double>::infinity();
    for (int joint = 0; joint < static_cast<int>(intervals.size()); ++joint) {
        const double width = intervals[static_cast<std::size_t>(joint)].width();
        if (width <= 0.0) {
            continue;
        }

        const double mid =
            (intervals[static_cast<std::size_t>(joint)].lo + intervals[static_cast<std::size_t>(joint)].hi) * 0.5;
        std::vector<rbf::Interval> left(intervals);
        std::vector<rbf::Interval> right(intervals);
        left[static_cast<std::size_t>(joint)].hi = mid;
        right[static_cast<std::size_t>(joint)].lo = mid;

        const rbf::EndpointIAABBResult left_result = rbf::compute_endpoint_iaabb_ifk_aa(robot, left);
        const rbf::EndpointIAABBResult right_result = rbf::compute_endpoint_iaabb_ifk_aa(robot, right);
        std::vector<float> hull = left_result.endpoint_iaabbs;
        rbf::hull_endpoint_iaabbs(
            hull.data(),
            right_result.endpoint_iaabbs.data(),
            left_result.n_active_links * 2);
        const double hull_measure = endpoint_split_measure(hull, left_result.n_active_links);
        const double gain =
            (estimate.parent_measure - hull_measure) / estimate.parent_measure;
        if (gain > estimate.best_gain + 1e-12 ||
            (std::abs(gain - estimate.best_gain) <= 1e-12 && hull_measure < best_hull_measure)) {
            estimate.best_gain = gain;
            estimate.best_dim = joint;
            best_hull_measure = hull_measure;
        }
    }

    return estimate;
}

}  // namespace

namespace rbf {

int recommend_hifk_depth(
    const std::vector<Interval>& intervals,
    int max_depth_cap)
{
    if (max_depth_cap <= 0 || intervals.empty()) {
        return 0;
    }

    const HifkWidthStats stats = hifk_width_stats(intervals);
    if (stats.max_width <= kHifkIfkWidthThresh) {
        return 0;
    }

    const int recommended =
        (stats.max_width <= kHifkWideMaxWidthThresh || stats.rms_width <= kHifkWideRmsWidthThresh)
            ? 3
            : 5;
    return std::min(max_depth_cap, recommended);
}

int recommend_hifk_depth(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int max_depth_cap)
{
    if (max_depth_cap <= 0 || intervals.empty()) {
        return 0;
    }

    const HifkWidthStats stats = hifk_width_stats(intervals);
    if (stats.max_width <= kHifkIfkWidthThresh) {
        return 0;
    }

    const HifkSplitEstimate estimate = estimate_best_aafk_split_gain(robot, intervals);
    if (estimate.best_dim < 0) {
        return recommend_hifk_depth(intervals, max_depth_cap);
    }

    if (stats.max_width > kHifkWideMaxWidthThresh &&
        stats.rms_width > kHifkWideRmsWidthThresh &&
        estimate.parent_measure >= kHifkDepth5MeasureThresh &&
        estimate.best_gain >= kHifkDepth5GainThresh) {
        return std::min(max_depth_cap, 5);
    }

    return std::min(max_depth_cap, 3);
}

EndpointIAABBResult compute_endpoint_iaabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config,
    FKState* fk,
    int changed_dim)
{
    switch (config.source) {
    case EndpointSource::IFK:
        return compute_endpoint_iaabb_ifk_aa(robot, intervals);

    case EndpointSource::CritSample:
        return compute_endpoint_iaabb_crit(robot, intervals, config.n_samples_crit,
                                           42, changed_dim, fk,
                                           config.n_threads,
                                           config.parallel_min_combos);

    case EndpointSource::Analytical:
        return compute_endpoint_iaabb_analytical(robot, intervals,
                                                 config.max_phase_analytical,
                                                 config.bypass_narrow_skip);

    case EndpointSource::GCPC: {
        assert(config.gcpc_cache != nullptr);
        return compute_endpoint_iaabb_gcpc(
            robot,
            intervals,
            *config.gcpc_cache,
            config.max_phase_analytical,
            config.gcpc_match_analytical,
            config.bypass_narrow_skip);
    }

    case EndpointSource::MC:
        return compute_endpoint_iaabb_mc(robot, intervals,
                                         config.n_samples_crit,
                                         42, changed_dim);

    case EndpointSource::HIFK: {
        const int effective_hifk_max_depth =
            config.hifk_max_depth < 0
                ? recommend_hifk_depth(robot, intervals)
                : config.hifk_max_depth;
        return compute_endpoint_iaabb_hifk_aa(robot, intervals,
                                              effective_hifk_max_depth,
                                              config.hifk_n_threads,
                                              config.hifk_vol_ratio_thresh);
    }

    default:
        // Fallback to IFK
        return compute_endpoint_iaabb_ifk_aa(robot, intervals);
    }
}

void hull_endpoint_iaabbs(float* dst, const float* src, int n_endpoints) {
    for (int i = 0; i < n_endpoints; ++i) {
        const float* s = src + i * 6;
        float*       d = dst + i * 6;
        // lo = min(dst_lo, src_lo)
        for (int k = 0; k < 3; ++k)
            if (s[k] < d[k]) d[k] = s[k];
        // hi = max(dst_hi, src_hi)
        for (int k = 3; k < 6; ++k)
            if (s[k] > d[k]) d[k] = s[k];
    }
}

}  // namespace rbf
