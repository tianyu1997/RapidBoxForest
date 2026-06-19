#include "oracle_best_tighten.h"

#include "oracle_options.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace rbf {
namespace {

constexpr double kBestTightenScoreEpsilon = 1e-12;

double interval_width(const Interval& interval) {
    return std::max(0.0, interval.width());
}

double max_interval_width(const std::vector<Interval>& intervals) {
    double width = 0.0;
    for (const auto& interval : intervals) {
        width = std::max(width, interval_width(interval));
    }
    return width;
}

double min_positive_interval_width(const std::vector<Interval>& intervals) {
    double width = std::numeric_limits<double>::infinity();
    for (const auto& interval : intervals) {
        const double item_width = interval_width(interval);
        if (item_width > 0.0) {
            width = std::min(width, item_width);
        }
    }
    return std::isfinite(width) ? width : 0.0;
}

double aspect_ratio(const std::vector<Interval>& intervals) {
    const double min_width = min_positive_interval_width(intervals);
    if (min_width <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return max_interval_width(intervals) / min_width;
}

double score_scale(double parent_score,
                   double left_score,
                   double right_score,
                   double minimax_score) {
    return std::max({1.0,
                     std::abs(parent_score),
                     std::abs(left_score),
                     std::abs(right_score),
                     std::abs(minimax_score)});
}

bool nearly_less(double lhs, double rhs) {
    const double scale = std::max({1.0, std::abs(lhs), std::abs(rhs)});
    return lhs < rhs - kBestTightenScoreEpsilon * scale;
}

double positive_ratio_penalty(double value, double limit) {
    if (limit <= 0.0 || value <= limit) {
        return 0.0;
    }
    return std::log1p(value / limit - 1.0);
}

double recent_dim_fraction(const std::vector<int>& recent_dims,
                           int dim,
                           const BestTightenOptions& options) {
    if (!options.recent_dim_cooling || options.recent_dim_window <= 0 || recent_dims.empty()) {
        return 0.0;
    }
    const int window = std::min(options.recent_dim_window, static_cast<int>(recent_dims.size()));
    int count = 0;
    for (int index = static_cast<int>(recent_dims.size()) - window;
         index < static_cast<int>(recent_dims.size());
         ++index) {
        if (recent_dims[static_cast<std::size_t>(index)] == dim) {
            count += 1;
        }
    }
    return static_cast<double>(count) / static_cast<double>(window);
}

bool candidate_dim_allowed(const std::vector<Interval>& intervals,
                           int dim,
                           const BestTightenOptions& options) {
    if (dim < 0 || dim >= static_cast<int>(intervals.size())) {
        return false;
    }
    if (options.max_candidate_dim >= 0 && dim > options.max_candidate_dim) {
        return false;
    }
    if (!options.dim_mask.empty() &&
        dim < static_cast<int>(options.dim_mask.size()) &&
        options.dim_mask[static_cast<std::size_t>(dim)] == 0) {
        return false;
    }
    return interval_width(intervals[static_cast<std::size_t>(dim)]) > options.min_candidate_width;
}

bool has_depth_dim(const std::vector<int>& depth_dims, int depth) {
    return depth >= 0 && depth < static_cast<int>(depth_dims.size()) &&
           depth_dims[static_cast<std::size_t>(depth)] >= 0;
}

std::optional<int> depth_dim(const std::vector<int>& depth_dims, int depth) {
    if (!has_depth_dim(depth_dims, depth)) {
        return std::nullopt;
    }
    return depth_dims[static_cast<std::size_t>(depth)];
}

void observe_best_tighten_choice(int depth,
                                 int dim,
                                 const BestTightenOptions& options,
                                 std::vector<int>& depth_dims,
                                 std::vector<int>& recent_dims) {
    if (depth < 0 || dim < 0) {
        return;
    }
    if (options.depth_synchronous) {
        if (depth >= static_cast<int>(depth_dims.size())) {
            depth_dims.resize(static_cast<std::size_t>(depth + 1), -1);
        }
        depth_dims[static_cast<std::size_t>(depth)] = dim;
    }
    recent_dims.push_back(dim);
    if (options.recent_dim_window > 0 &&
        static_cast<int>(recent_dims.size()) >
            std::max(options.recent_dim_window * 4, options.recent_dim_window + 8)) {
        recent_dims.erase(recent_dims.begin(),
                          recent_dims.begin() + static_cast<std::ptrdiff_t>(recent_dims.size() -
                                                                           options.recent_dim_window));
    }
}

std::optional<double> sector_boundary_split_value(const std::vector<Interval>& intervals,
                                                  int dim,
                                                  const std::optional<JointSymmetry>& symmetry) {
    if (!symmetry || symmetry->type == JointSymmetryType::NONE || symmetry->period <= 0.0) {
        return std::nullopt;
    }
    if (symmetry->joint_index != dim) {
        return std::nullopt;
    }
    const auto& interval = intervals[static_cast<std::size_t>(dim)];
    double best = 0.0;
    double best_dist = std::numeric_limits<double>::infinity();
    const double mid = 0.5 * (interval.lo + interval.hi);
    const int k_min = static_cast<int>(std::floor((interval.lo - symmetry->canonical_lo) / symmetry->period)) - 1;
    const int k_max = static_cast<int>(std::ceil((interval.hi - symmetry->canonical_lo) / symmetry->period)) + 1;
    for (int k = k_min; k <= k_max; ++k) {
        const double candidate = symmetry->canonical_lo + static_cast<double>(k) * symmetry->period;
        if (candidate <= interval.lo || candidate >= interval.hi) {
            continue;
        }
        const double dist = std::abs(candidate - mid);
        if (dist < best_dist) {
            best = candidate;
            best_dist = dist;
        }
    }
    if (!std::isfinite(best_dist)) {
        return std::nullopt;
    }
    return best;
}

double midpoint_split_value(const Interval& interval) {
    return 0.5 * (interval.lo + interval.hi);
}

BestTightenCandidate evaluate_best_tighten_dim(const std::vector<Interval>& intervals,
                                               int dim,
                                               double split_val,
                                               double parent_score,
                                               const BestTightenScoreFn& scorer,
                                               bool sector_aligned,
                                               double recent_dim_fraction_value,
                                               const BestTightenOptions& options) {
    BestTightenCandidate candidate;
    candidate.dim = dim;
    candidate.split_val = split_val;
    candidate.sector_aligned = sector_aligned;
    const auto& interval = intervals[static_cast<std::size_t>(dim)];
    if (split_val <= interval.lo || split_val >= interval.hi) {
        return candidate;
    }
    auto left = intervals;
    auto right = intervals;
    left[static_cast<std::size_t>(dim)].hi = split_val;
    right[static_cast<std::size_t>(dim)].lo = split_val;
    candidate.parent_score = parent_score;
    candidate.left_score = scorer(left);
    candidate.right_score = scorer(right);
    candidate.minimax_score = std::max(candidate.left_score, candidate.right_score) +
        options.width_penalty * interval.width();
    candidate.valid = std::isfinite(candidate.parent_score) && std::isfinite(candidate.left_score) &&
                      std::isfinite(candidate.right_score);
    if (!candidate.valid) {
        return candidate;
    }

    const double representative_child_score = options.use_minimax
        ? std::max(candidate.left_score, candidate.right_score)
        : 0.5 * (candidate.left_score + candidate.right_score);
    candidate.reduction = candidate.parent_score - representative_child_score;

    const double parent_width_max = max_interval_width(intervals);
    const double split_width = interval_width(interval);
    candidate.parent_aspect = aspect_ratio(intervals);
    candidate.child_aspect = std::max(aspect_ratio(left), aspect_ratio(right));
    candidate.split_width_fraction = parent_width_max > 0.0 ? split_width / parent_width_max : 1.0;

    const double parent_scale = std::max(1.0, std::abs(candidate.parent_score));
    candidate.normalized_reduction = candidate.reduction / parent_scale;
    const double child_sum_scale = std::max(1.0, std::abs(candidate.left_score) + std::abs(candidate.right_score));
    candidate.score_imbalance = std::abs(candidate.left_score - candidate.right_score) / child_sum_scale;

    const double aspect_penalty = positive_ratio_penalty(candidate.child_aspect, options.max_child_aspect);
    double thin_penalty = 0.0;
    if (options.min_split_width_fraction > 0.0 &&
        candidate.split_width_fraction < options.min_split_width_fraction) {
        thin_penalty = std::log1p(options.min_split_width_fraction /
                                  std::max(candidate.split_width_fraction, 1e-300) - 1.0);
    }
    candidate.shape_penalty = aspect_penalty + thin_penalty;
    candidate.shape_healthy = candidate.shape_penalty <= 0.0;
    candidate.recent_dim_fraction = recent_dim_fraction_value;
    const double recent_shape_pressure = positive_ratio_penalty(candidate.child_aspect,
                                                               options.recent_dim_shape_aspect_trigger);
    candidate.recent_dim_penalty = candidate.recent_dim_fraction * recent_shape_pressure;

    const double envelope_score = options.use_minimax
        ? std::max(candidate.left_score, candidate.right_score)
        : -(candidate.reduction);
    const double scale = score_scale(candidate.parent_score,
                                     candidate.left_score,
                                     candidate.right_score,
                                     envelope_score);
    const double thin_width_penalty = options.width_penalty * thin_penalty * scale;
    candidate.balanced_score = envelope_score +
        scale * (options.shape_weight * candidate.shape_penalty +
                 options.recent_dim_weight * candidate.recent_dim_penalty +
                 options.balance_weight * candidate.score_imbalance -
                 options.relative_gain_weight * candidate.normalized_reduction -
                 options.widest_tiebreak_weight * candidate.split_width_fraction) +
        thin_width_penalty;
    if (options.dim_priority_weight != 0.0 &&
        dim >= 0 && dim < static_cast<int>(options.dim_priority_weights.size())) {
        candidate.balanced_score -=
            scale * options.dim_priority_weight *
            options.dim_priority_weights[static_cast<std::size_t>(dim)];
    }
    return candidate;
}

bool better_best_tighten_candidate(const BestTightenCandidate& lhs,
                                   const BestTightenCandidate& rhs,
                                   const BestTightenOptions& options) {
    if (!lhs.valid) {
        return false;
    }
    if (!rhs.valid) {
        return true;
    }
    if (options.shape_balancing) {
        if (lhs.shape_healthy != rhs.shape_healthy) {
            return lhs.shape_healthy;
        }
        if (nearly_less(lhs.balanced_score, rhs.balanced_score)) {
            return true;
        }
        if (nearly_less(rhs.balanced_score, lhs.balanced_score)) {
            return false;
        }
        if (std::abs(lhs.split_width_fraction - rhs.split_width_fraction) > 1e-12) {
            return lhs.split_width_fraction > rhs.split_width_fraction;
        }
        if (std::abs(lhs.child_aspect - rhs.child_aspect) > 1e-12) {
            return lhs.child_aspect < rhs.child_aspect;
        }
        if (std::abs(lhs.normalized_reduction - rhs.normalized_reduction) > 1e-12) {
            return lhs.normalized_reduction > rhs.normalized_reduction;
        }
        if (lhs.sector_aligned != rhs.sector_aligned) {
            return lhs.sector_aligned;
        }
        return lhs.dim < rhs.dim;
    }
    if (options.use_minimax && lhs.minimax_score != rhs.minimax_score) {
        return lhs.minimax_score < rhs.minimax_score;
    }
    if (lhs.reduction != rhs.reduction) {
        return lhs.reduction > rhs.reduction;
    }
    if (lhs.sector_aligned != rhs.sector_aligned) {
        return lhs.sector_aligned;
    }
    return lhs.dim < rhs.dim;
}

}  // namespace

std::vector<double> link_aabb_volumes(const std::vector<float>& link_aabbs) {
    const std::size_t count = link_aabbs.size() / 6U;
    std::vector<double> volumes(count, 0.0);
    for (std::size_t index = 0; index < count; ++index) {
        const std::size_t base = index * 6U;
        const double dx = std::max(0.0, static_cast<double>(link_aabbs[base + 3U] - link_aabbs[base + 0U]));
        const double dy = std::max(0.0, static_cast<double>(link_aabbs[base + 4U] - link_aabbs[base + 1U]));
        const double dz = std::max(0.0, static_cast<double>(link_aabbs[base + 5U] - link_aabbs[base + 2U]));
        volumes[index] = dx * dy * dz;
    }
    return volumes;
}

double normalized_link_aabb_volume_score(const std::vector<float>& link_aabbs,
                                         const std::vector<double>& reference_volumes) {
    const std::size_t count = std::min(link_aabbs.size() / 6U, reference_volumes.size());
    if (count == 0U) {
        return 0.0;
    }
    double score = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const std::size_t base = index * 6U;
        const double dx = std::max(0.0, static_cast<double>(link_aabbs[base + 3U] - link_aabbs[base + 0U]));
        const double dy = std::max(0.0, static_cast<double>(link_aabbs[base + 4U] - link_aabbs[base + 1U]));
        const double dz = std::max(0.0, static_cast<double>(link_aabbs[base + 5U] - link_aabbs[base + 2U]));
        const double volume = dx * dy * dz;
        const double reference = std::max(reference_volumes[index], 1e-300);
        score += std::log1p(volume / reference);
    }
    return score / static_cast<double>(count);
}

BestTightenCandidate choose_best_tighten_split(const std::vector<Interval>& intervals,
                                               int depth,
                                               const BestTightenScoreFn& scorer,
                                               const std::optional<JointSymmetry>& symmetry,
                                               const BestTightenOptions& options,
                                               std::vector<int>& depth_dims,
                                               std::vector<int>& recent_dims) {
    if (!scorer) {
        throw std::invalid_argument("DatabaseBoxOracle best-tighten split requires an envelope score function");
    }
    if (options.depth_synchronous && has_depth_dim(depth_dims, depth)) {
        const int replay_dim = *depth_dim(depth_dims, depth);
        if (candidate_dim_allowed(intervals, replay_dim, options)) {
            const auto boundary = options.prefer_sector_boundary
                ? sector_boundary_split_value(intervals, replay_dim, symmetry)
                : std::nullopt;
            const double split_val = boundary.value_or(
                midpoint_split_value(intervals[static_cast<std::size_t>(replay_dim)]));
            const auto& interval = intervals[static_cast<std::size_t>(replay_dim)];
            if (split_val > interval.lo && split_val < interval.hi) {
                BestTightenCandidate replayed;
                replayed.valid = true;
                replayed.dim = replay_dim;
                replayed.split_val = split_val;
                replayed.sector_aligned = boundary.has_value();
                observe_best_tighten_choice(depth, replay_dim, options, depth_dims, recent_dims);
                return replayed;
            }
        }
    }

    const double parent_score = scorer(intervals);
    if (!std::isfinite(parent_score)) {
        throw std::invalid_argument("DatabaseBoxOracle best-tighten parent score is not finite");
    }

    BestTightenCandidate best;
    for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
        if (!candidate_dim_allowed(intervals, dim, options)) {
            continue;
        }
        const auto boundary = options.prefer_sector_boundary
            ? sector_boundary_split_value(intervals, dim, symmetry)
            : std::nullopt;
        const double split_val = boundary.value_or(
            midpoint_split_value(intervals[static_cast<std::size_t>(dim)]));
        const auto candidate = evaluate_best_tighten_dim(intervals,
                                                         dim,
                                                         split_val,
                                                         parent_score,
                                                         scorer,
                                                         boundary.has_value(),
                                                         recent_dim_fraction(recent_dims, dim, options),
                                                         options);
        if (oracle_best_tighten_debug_enabled() && depth <= 2) {
            std::fprintf(stderr,
                         "[BT_DIM] depth=%d dim=%d val=%.5f sect=%d parent=%.6f L=%.6f R=%.6f minimax=%.6f valid=%d\n",
                         depth, dim, split_val, (int)boundary.has_value(),
                         candidate.parent_score, candidate.left_score, candidate.right_score,
                         candidate.minimax_score, (int)candidate.valid);
        }
        if (better_best_tighten_candidate(candidate, best, options)) {
            best = candidate;
        }
    }
    if (oracle_best_tighten_debug_enabled() && depth <= 2) {
        std::fprintf(stderr, "[BT_PICK] depth=%d -> dim=%d val=%.5f minimax=%.6f\n",
                     depth, best.dim, best.split_val, best.minimax_score);
    }

    if (!best.valid) {
        throw std::invalid_argument("DatabaseBoxOracle best-tighten could not find a valid split");
    }
    observe_best_tighten_choice(depth, best.dim, options, depth_dims, recent_dims);
    return best;
}

int fk_effective_max_split_dim(const Robot& robot) {
    const int last_active_frame = robot.last_active_frame();
    if (last_active_frame < 0) {
        return -1;
    }
    return std::max(0, last_active_frame - 1);
}

BestTightenOptions with_fk_effective_split_filter(BestTightenOptions options, const Robot& robot) {
    const int fk_max_dim = fk_effective_max_split_dim(robot);
    if (fk_max_dim < 0) {
        return options;
    }
    options.max_candidate_dim = options.max_candidate_dim >= 0
        ? std::min(options.max_candidate_dim, fk_max_dim)
        : fk_max_dim;
    return options;
}

}  // namespace rbf
