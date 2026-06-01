#include <LECTDatabase/sbf/oracle.h>

#include <sbf/core/joint_symmetry.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/ifk_aa_source.h>

#include <link_interval_envelope/incremental_context.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <system_error>

namespace rbf {
namespace {

using lect_database::NodeId;
using Clock = std::chrono::steady_clock;

double elapsed_us(Clock::time_point start) {
    return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
}

double interval_volume(const std::vector<Interval>& intervals) {
    double volume = 1.0;
    for (const auto& interval : intervals) {
        volume *= std::max(0.0, interval.width());
    }
    return volume;
}

std::uint64_t make_envelope_cache_key(lect_database::NodeId node_id, int sector) {
    std::uint64_t key = static_cast<std::uint64_t>(node_id);
    const std::uint64_t sector_bits = static_cast<std::uint64_t>(static_cast<std::uint32_t>(sector));
    key ^= sector_bits + 0x9e3779b97f4a7c15ull + (key << 6) + (key >> 2);
    return key;
}

std::uint64_t next_session_id() {
    static std::atomic<std::uint64_t> counter{0};
    return counter.fetch_add(1, std::memory_order_relaxed);
}

void record_envelope_collision(OracleCounters& counters, const EnvelopeCollisionStats& stats) {
    counters.envelope_collision_envelope_aabb_tests += stats.envelope_aabb_tests;
    counters.envelope_collision_envelope_aabb_rejects += stats.envelope_aabb_rejects;
    counters.envelope_collision_link_union_aabb_tests += stats.link_union_aabb_tests;
    counters.envelope_collision_link_union_aabb_rejects += stats.link_union_aabb_rejects;
    counters.envelope_collision_link_aabb_tests += stats.link_aabb_tests;
    counters.envelope_collision_link_aabb_rejects += stats.link_aabb_rejects;
    counters.envelope_collision_kdop_tests += stats.kdop_tests;
    counters.envelope_collision_kdop_rejects += stats.kdop_rejects;
    counters.envelope_collision_kdop_axes_tested += stats.kdop_axes_tested;
    counters.envelope_collision_gjk_tests += stats.gjk_tests;
    counters.envelope_collision_gjk_rejects += stats.gjk_rejects;
    counters.envelope_collision_gjk_iterations += stats.gjk_iterations;
}

lect_database::EvidenceChannel database_channel_for_endpoint(EndpointSource source) {
    return source_channel(source) == 0 ? lect_database::EvidenceChannel::Safe
                                       : lect_database::EvidenceChannel::Rapid;
}

EndpointSourceConfig hifk_config_for_materialization(const DatabaseBoxOracle& oracle,
                                                     OracleNodeId node,
                                                     EndpointSourceConfig config) {
    if (config.source != EndpointSource::HIFK) {
        return config;
    }

    const auto& split_policy = oracle.database().split_policy_descriptor();
    config.hifk_depth_offset = oracle.depth(node);
    config.hifk_min_split_width = split_policy.min_width;
    config.hifk_depth_dimensions.clear();
    config.hifk_root_intervals.clear();

    switch (split_policy.strategy) {
    case lect_database::SplitStrategy::RoundRobin:
        config.hifk_split_strategy = HifkSplitStrategy::RoundRobin;
        break;
    case lect_database::SplitStrategy::WidestRoot:
        config.hifk_split_strategy = HifkSplitStrategy::WidestRoot;
        config.hifk_root_intervals = oracle.root_intervals();
        break;
    case lect_database::SplitStrategy::AAFKVolumeMin:
        config.hifk_split_strategy = HifkSplitStrategy::FixedDepthSchedule;
        config.hifk_depth_dimensions = split_policy.depth_dimensions;
        break;
    }

    return config;
}

rbf::lect_database::LectDatabaseConfig make_worker_database_config(const DatabaseBoxOracle& master,
                                                                   const std::filesystem::path& path,
                                                                   const std::vector<Interval>& root_intervals,
                                                                   int root_depth) {
    lect_database::LectDatabaseConfig config;
    config.path = path;
    config.root_intervals = root_intervals;
    config.split_policy = master.database().split_policy_descriptor();
    config.identity = master.database().identity();
    config.identity.root_domain_fingerprint = lect_database::fingerprint_intervals(root_intervals);
    config.open.read_only = false;
    config.open.create_if_missing = true;
    config.open.verify_identity = true;
    config.open.replay_journal = true;
    config.root_depth = root_depth;
    return config;
}

lect_database::NodeId to_database_node(OracleNodeId node) {
    if (node < 0 || static_cast<std::uint64_t>(node) > std::numeric_limits<lect_database::NodeId>::max()) {
        return lect_database::kInvalidNodeId;
    }
    return static_cast<lect_database::NodeId>(node);
}

OracleNodeId from_database_node(lect_database::NodeId node) {
    return lect_database::valid_node_id(node) ? static_cast<OracleNodeId>(node) : kInvalidOracleNodeId;
}

OracleNodeId remap_lookup(const std::unordered_map<OracleNodeId, OracleNodeId>& node_remap,
                          OracleNodeId worker_node) {
    const auto it = node_remap.find(worker_node);
    return it == node_remap.end() ? kInvalidOracleNodeId : it->second;
}

bool database_uses_canonical_symmetry(const lect_database::LectDatabase& database) {
    const auto& identity = database.identity();
    return identity.canonical_mode && lect_database::uses_joint_symmetry_native(identity.symmetry_descriptor);
}

std::optional<JointSymmetry> primary_database_symmetry(const Robot& robot,
                                                       const lect_database::LectDatabase& database) {
    if (!database_uses_canonical_symmetry(database)) {
        return std::nullopt;
    }
    auto symmetries = detect_joint_symmetries(robot);
    if (symmetries.empty()) {
        return std::nullopt;
    }
    JointSymmetry symmetry = symmetries.front();
    if (symmetry.type == JointSymmetryType::NONE || symmetry.period <= 0.0 || symmetry.joint_index < 0) {
        return std::nullopt;
    }
    return symmetry;
}

int normalize_sector(lect_database::SectorId sector) {
    return ((static_cast<int>(sector) % 4) + 4) % 4;
}

double normalized_joint_value(double value, double origin) {
    double normalized = std::fmod(value - origin, TWO_PI);
    if (normalized < 0.0) {
        normalized += TWO_PI;
    }
    return normalized + origin;
}

double canonical_value_in_sector(const JointSymmetry& symmetry,
                                 double value,
                                 lect_database::SectorId sector) {
    return normalized_joint_value(value, symmetry.canonical_lo) -
        static_cast<double>(normalize_sector(sector)) * symmetry.period;
}

Interval canonical_interval_for_sector(const JointSymmetry& symmetry,
                                       const Interval& interval,
                                       lect_database::SectorId sector) {
    const double lo = canonical_value_in_sector(symmetry, interval.lo, sector);
    const double hi = canonical_value_in_sector(symmetry, interval.hi, sector);
    Interval canonical{std::min(lo, hi), std::max(lo, hi)};
    canonical.lo = std::max(canonical.lo, symmetry.canonical_lo);
    canonical.hi = std::min(canonical.hi, symmetry.canonical_hi);
    return canonical;
}

Interval map_canonical_interval_to_sector(const JointSymmetry& symmetry,
                                          const Interval& canonical,
                                          lect_database::SectorId sector,
                                          const Interval& limit,
                                          double reference_value) {
    double lo = 0.0;
    double hi = 0.0;
    symmetry.map_interval(canonical.lo, canonical.hi, normalize_sector(sector), lo, hi);
    while (hi > limit.hi + 1e-12) {
        lo -= TWO_PI;
        hi -= TWO_PI;
    }
    while (lo < limit.lo - 1e-12) {
        lo += TWO_PI;
        hi += TWO_PI;
    }
    const double center = 0.5 * (lo + hi);
    const double lower_shift_center = center - TWO_PI;
    const double upper_shift_center = center + TWO_PI;
    if (std::abs(reference_value - lower_shift_center) < std::abs(reference_value - center) &&
        lo - TWO_PI >= limit.lo - 1e-12) {
        lo -= TWO_PI;
        hi -= TWO_PI;
    } else if (std::abs(reference_value - upper_shift_center) < std::abs(reference_value - center) &&
               hi + TWO_PI <= limit.hi + 1e-12) {
        lo += TWO_PI;
        hi += TWO_PI;
    }
    return {std::max(lo, limit.lo), std::min(hi, limit.hi)};
}

using EnvelopeScoreFn = std::function<double(const std::vector<Interval>&)>;

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

BestTightenCandidate evaluate_best_tighten_dim(const std::vector<Interval>& intervals,
                                               int dim,
                                               double split_val,
                                               double parent_score,
                                               const EnvelopeScoreFn& scorer,
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
    // L2 soft dim-priority: gently prefer high-sensitivity dims (smaller score
    // is better, so subtract). Inert dims with ~0 weight get no boost.
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

BestTightenCandidate choose_best_tighten_split(const std::vector<Interval>& intervals,
                                               int depth,
                                               const EnvelopeScoreFn& scorer,
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
        if (std::getenv("SBF_BT_DEBUG") && depth <= 2) {
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
    if (std::getenv("SBF_BT_DEBUG") && depth <= 2) {
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

struct CanonicalEvidenceFrame {
    std::vector<Interval> lookup_intervals;
    lect_database::SectorId sector = lect_database::kPrimarySector;
    std::optional<JointSymmetry> symmetry;
};

CanonicalEvidenceFrame canonical_evidence_frame_for_intervals(const Robot& robot,
                                                              const lect_database::LectDatabase& database,
                                                              const std::vector<Interval>& query_intervals) {
    CanonicalEvidenceFrame frame;
    frame.lookup_intervals = query_intervals;
    frame.symmetry = primary_database_symmetry(robot, database);
    if (!frame.symmetry || query_intervals.empty()) {
        return frame;
    }
    const JointSymmetry& symmetry = *frame.symmetry;
    const std::size_t joint_index = static_cast<std::size_t>(symmetry.joint_index);
    if (joint_index >= query_intervals.size()) {
        frame.symmetry.reset();
        return frame;
    }
    double canonical_center = query_intervals[joint_index].center();
    frame.sector = symmetry.canonicalize(canonical_center, canonical_center);
    frame.lookup_intervals[joint_index] = canonical_interval_for_sector(
        symmetry,
        query_intervals[joint_index],
        frame.sector);
    return frame;
}

// A query box whose symmetry-joint interval spans a sector boundary cannot be
// represented faithfully by a single canonical sector (the canonical cache is
// keyed per sector). Detect this so we can fall back to computing the envelope
// directly on the native query intervals.
bool intervals_straddle_sector_boundary(const JointSymmetry& symmetry,
                                        const std::vector<Interval>& intervals) {
    const std::size_t joint_index = static_cast<std::size_t>(symmetry.joint_index);
    if (joint_index >= intervals.size()) {
        return false;
    }
    double dummy = 0.0;
    const int sector_lo = symmetry.canonicalize(intervals[joint_index].lo, dummy);
    const int sector_hi = symmetry.canonicalize(intervals[joint_index].hi, dummy);
    return normalize_sector(sector_lo) != normalize_sector(sector_hi);
}

}  // namespace

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                                     lect_database::LectDatabase& database,
                                     Scene scene,
                                     EndpointSourceConfig endpoint_config,
                                     EnvelopeTypeConfig envelope_config,
                         OracleValidationConfig validation_config,
                                                                                                                                                 const lect_database::LectExternalEvidenceSource* external_evidence_source,
                                                                                                                                                 const lect_database::LectDatabase* direct_external_evidence_database)
    : robot_(std::move(robot)),
      database_(database),
        external_evidence_source_(external_evidence_source),
        direct_external_evidence_database_(direct_external_evidence_database),
      endpoint_config_(std::move(endpoint_config)),
      envelope_config_(std::move(envelope_config)),
      validation_config_(std::move(validation_config)),
      scene_(std::move(scene)),
      checker_(robot_, scene_) {
    seed_best_tighten_schedule_from_policy();
}

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                         lect_database::OnlineEnvelopeCacheTree& online_cache,
                         Scene scene,
                         EndpointSourceConfig endpoint_config,
                         EnvelopeTypeConfig envelope_config,
                         OracleValidationConfig validation_config,
                                                                         const lect_database::LectExternalEvidenceSource* external_evidence_source,
                                                                         const lect_database::LectDatabase* direct_external_evidence_database)
    : robot_(std::move(robot)),
    database_(online_cache.database()),
    online_cache_(&online_cache),
    external_evidence_source_(external_evidence_source),
    direct_external_evidence_database_(direct_external_evidence_database),
    endpoint_config_(std::move(endpoint_config)),
    envelope_config_(std::move(envelope_config)),
    validation_config_(std::move(validation_config)),
    scene_(std::move(scene)),
    checker_(robot_, scene_) {
    seed_best_tighten_schedule_from_policy();
}

void DatabaseBoxOracle::seed_best_tighten_schedule_from_policy() {
    // Pre-seed the per-depth best-tighten split dimensions from the canonical
    // FixedDepthSchedule so the grower replays a (robot, domain)-determined
    // dimension sequence instead of lazily fixing it from the first query that
    // reaches each depth. This keeps node paths canonical (and therefore the
    // external-evidence keys stable / reusable) regardless of seed or query
    // order. Only applies to the AAFKVolumeMin FixedDepthSchedule strategy.
    const auto& policy = database_.split_policy_descriptor();
    if (policy.strategy != lect_database::SplitStrategy::AAFKVolumeMin) {
        return;
    }
    if (policy.depth_dimensions.empty()) {
        return;
    }
    best_tighten_depth_dims_ = policy.depth_dimensions;
}

int DatabaseBoxOracle::n_dims() const {
    return static_cast<int>(database_.root_intervals().size());
}

int DatabaseBoxOracle::max_tree_depth() const {
    return database_.max_tree_depth();
}

const std::vector<Interval>& DatabaseBoxOracle::root_intervals() const {
    return database_.root_intervals();
}

std::vector<Interval> DatabaseBoxOracle::node_intervals(OracleNodeId node) const {
    if (online_cache_ != nullptr) {
        auto box = online_cache_->node_intervals(to_database_node(node));
        return box ? std::move(*box) : database_.root_intervals();
    }
    auto box = database_.node_box(to_database_node(node));
    return box ? std::move(*box) : database_.root_intervals();
}

Eigen::VectorXd DatabaseBoxOracle::tree_configuration_for_query(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    Eigen::VectorXd tree_q = q;
    if (database_uses_canonical_symmetry(database_) && tree_q.size() > 0) {
        lect_database::canonicalize_configuration_for_robot(
            robot_,
            true,
            database_.identity().symmetry_descriptor,
            std::span<double>(tree_q.data(), static_cast<std::size_t>(tree_q.size())));
    }
    return tree_q;
}

std::vector<Interval> DatabaseBoxOracle::query_intervals_for_node(OracleNodeId node,
                                                                  const std::vector<Interval>& tree_intervals,
                                                                  const Eigen::Ref<const Eigen::VectorXd>& q) const {
    (void)node;
    auto symmetry = primary_database_symmetry(robot_, database_);
    if (!symmetry || tree_intervals.empty() || q.size() <= symmetry->joint_index) {
        return tree_intervals;
    }
    const auto& limits = robot_.joint_limits().limits;
    const std::size_t joint_index = static_cast<std::size_t>(symmetry->joint_index);
    if (joint_index >= tree_intervals.size() || joint_index >= limits.size()) {
        return tree_intervals;
    }
    double canonical_value = q[static_cast<int>(joint_index)];
    const lect_database::SectorId sector = symmetry->canonicalize(canonical_value, canonical_value);
    std::vector<Interval> query_intervals = tree_intervals;
    query_intervals[joint_index] = map_canonical_interval_to_sector(
        *symmetry,
        tree_intervals[joint_index],
        sector,
        limits[joint_index],
        q[static_cast<int>(joint_index)]);
    return query_intervals;
}

bool DatabaseBoxOracle::contains_point(OracleNodeId node, const Eigen::Ref<const Eigen::VectorXd>& q) const {
    const Eigen::VectorXd tree_q = tree_configuration_for_query(q);
    const auto box = database_.node_box(to_database_node(node));
    if (!box || tree_q.size() != static_cast<int>(box->size())) {
        return false;
    }
    for (int dim = 0; dim < tree_q.size(); ++dim) {
        if (!(*box)[static_cast<std::size_t>(dim)].contains(tree_q[dim])) {
            return false;
        }
    }
    return true;
}

bool DatabaseBoxOracle::is_leaf(OracleNodeId node) const {
    if (online_cache_ != nullptr) {
        return online_cache_->is_leaf(to_database_node(node));
    }
    return database_.topology(to_database_node(node)).leaf;
}

int DatabaseBoxOracle::depth(OracleNodeId node) const {
    if (online_cache_ != nullptr) {
        return online_cache_->depth(to_database_node(node));
    }
    return database_.topology(to_database_node(node)).depth;
}

int DatabaseBoxOracle::split_dim(OracleNodeId node) const {
    return database_.topology(to_database_node(node)).split_dim;
}

double DatabaseBoxOracle::split_value(OracleNodeId node) const {
    return database_.topology(to_database_node(node)).split_value;
}

OracleNodeId DatabaseBoxOracle::left_child(OracleNodeId node) const {
    const auto id = database_.topology(to_database_node(node)).left;
    return from_database_node(id);
}

OracleNodeId DatabaseBoxOracle::right_child(OracleNodeId node) const {
    const auto id = database_.topology(to_database_node(node)).right;
    return from_database_node(id);
}

SplitNodeResult DatabaseBoxOracle::split_node(OracleNodeId node,
                                              const std::vector<Interval>& intervals,
                                              int changed_dim,
                                              const OracleSplitOptions& options) {
    SplitNodeResult result;
    std::pair<lect_database::NodeId, lect_database::NodeId> children{
        lect_database::kInvalidNodeId,
        lect_database::kInvalidNodeId,
    };

    if (options.use_best_tighten && !intervals.empty()) {
        auto best_tighten_options = with_fk_effective_split_filter(options.best_tighten, robot_);
        try {
            const auto scoring_endpoint_config = hifk_config_for_materialization(*this, node, endpoint_config_);
            link_interval_envelope::IncrementalEnvelopeContext probe(
                robot_, scoring_endpoint_config, envelope_config_);
            int next_changed_dim_hint = changed_dim;
            if (best_tighten_reference_volumes_.empty()) {
                link_interval_envelope::IncrementalEnvelopeContext reference_probe(
                    robot_, endpoint_config_, envelope_config_);
                const auto reference = reference_probe.compute(robot_.joint_limits().limits, -1);
                best_tighten_reference_volumes_ = link_aabb_volumes(reference.envelope.link_iaabbs);
            }
            const auto symmetry = best_tighten_options.prefer_sector_boundary
                ? primary_database_symmetry(robot_, database_)
                : std::nullopt;
            const auto scorer = [this, &probe, &next_changed_dim_hint](const std::vector<Interval>& candidate_intervals) {
                next_changed_dim_hint = -1;
                // Best-tighten scores arbitrary candidate child intervals that jump
                // around the tree (parent -> left -> right -> next-dim left ...). The
                // incremental endpoint state (notably the heuristic CritSample source)
                // is only valid for single-dim diffs branching from the *same* parent,
                // not for these arbitrary jumps, which corrupts the scored envelope and
                // makes children appear larger than their parent. Force a full,
                // stateless recompute per candidate so the minimax scores are correct.
                probe.reset();
                const auto scored = probe.compute(candidate_intervals, -1);
                counters_.scoring_evaluations += 1;
                counters_.scoring_endpoint_time_us += scored.endpoint_time_us;
                counters_.scoring_envelope_time_us += scored.envelope_time_us;
                counters_.scoring_candidate_dirty_count += scored.endpoint.candidate_dirty_count;
                counters_.scoring_predh_rebuild_count += scored.endpoint.predh_rebuild_count;
                if (scored.changed_dim >= 0) {
                    counters_.scoring_changed_dim_inferred += 1;
                }
                if (scored.used_incremental_fk) {
                    counters_.scoring_incremental_fk += 1;
                }
                if (scored.used_source_incremental_state) {
                    counters_.scoring_source_incremental_state += 1;
                }
                if (scored.reused_fk) {
                    counters_.scoring_reused_fk += 1;
                }
                if (scored.reused_endpoint_cache) {
                    counters_.scoring_reused_endpoint_cache += 1;
                }
                return normalized_link_aabb_volume_score(scored.envelope.link_iaabbs,
                                                         best_tighten_reference_volumes_);
            };

            const auto candidate = choose_best_tighten_split(intervals,
                                                             depth(node),
                                                             scorer,
                                                             symmetry,
                                                             best_tighten_options,
                                                             best_tighten_depth_dims_,
                                                             best_tighten_recent_dims_);
            children = database_.split_leaf(to_database_node(node), candidate.dim, candidate.split_val);
            if (std::getenv("SBF_BT_DEBUG")) {
                static std::atomic<long> ok{0};
                long n = ++ok;
                if (n <= 20 || n % 200 == 0) {
                    std::fprintf(stderr,
                                 "[BT_OK] n=%ld dim=%d val=%.5f valid_children=%d\n",
                                 n, candidate.dim, candidate.split_val,
                                 (int)(lect_database::valid_node_id(children.first) &&
                                       lect_database::valid_node_id(children.second)));
                }
            }
        } catch (const std::exception& e) {
            if (std::getenv("SBF_BT_DEBUG")) {
                static std::atomic<long> bad{0};
                long n = ++bad;
                if (n <= 20 || n % 200 == 0) {
                    std::fprintf(stderr, "[BT_EXC] n=%ld what=%s\n", n, e.what());
                }
            }
        }
    }

    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        children = online_cache_ != nullptr
            ? online_cache_->split_leaf(to_database_node(node))
            : database_.split_leaf(to_database_node(node));
    }
    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        return result;
    }
    const auto topology = database_.topology(to_database_node(node));
    result.split = true;
    result.node = node;
    result.left = from_database_node(children.first);
    result.right = from_database_node(children.second);
    result.split_dim = topology.split_dim;
    result.split_value = topology.split_value;
    return result;
}

SplitNodeResult DatabaseBoxOracle::split_node_at(OracleNodeId node, int split_dim, double split_value) {
    SplitNodeResult result;
    const auto children = database_.split_leaf(to_database_node(node), split_dim, split_value);
    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        return result;
    }
    const auto topology = database_.topology(to_database_node(node));
    result.split = true;
    result.node = node;
    result.left = from_database_node(children.first);
    result.right = from_database_node(children.second);
    result.split_dim = topology.split_dim;
    result.split_value = topology.split_value;
    return result;
}

bool DatabaseBoxOracle::point_in_collision(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    return checker_.check_config(q);
}

lect_database::EvidenceKey DatabaseBoxOracle::endpoint_key(OracleNodeId node) const {
    lect_database::EvidenceKey key;
    key.node_id = to_database_node(node);
    const auto topology = database_.topology(key.node_id);
    key.node_path = topology.path;
    key.node_path_valid = lect_database::valid_node_id(topology.id);
    key.sector = lect_database::kPrimarySector;
    key.channel = database_channel_for_endpoint(endpoint_config_.source);
    key.endpoint_source = endpoint_config_.source;
    key.payload_kind = lect_database::EvidencePayloadKind::EndpointEnvelope;
    return key;
}

std::optional<DatabaseBoxOracle::EndpointPayload> DatabaseBoxOracle::endpoint_payload_for_node(
    OracleNodeId node,
    const std::vector<Interval>& intervals,
    int changed_dim) {
    const auto key = endpoint_key(node);
    const auto evidence_frame = canonical_evidence_frame_for_intervals(robot_, database_, intervals);
    const int normalized_sector = normalize_sector(evidence_frame.sector);
    const std::uint64_t envelope_cache_key = make_envelope_cache_key(key.node_id, normalized_sector);
    if (evidence_frame.symmetry &&
        intervals_straddle_sector_boundary(*evidence_frame.symmetry, intervals)) {
        // The query box spans a sector boundary; a single canonical sector cannot
        // represent it. Compute the envelope directly on the native query box so
        // validation never has to remember a sector for boundary boxes.
        const auto endpoint_start = Clock::now();
        EndpointSourceConfig materialization_config =
            hifk_config_for_materialization(*this, node, endpoint_config_);
        EndpointIAABBResult endpoint =
            compute_endpoint_iaabb(robot_, intervals, materialization_config, nullptr, changed_dim);
        counters_.materialization_endpoint_wall_time_us += elapsed_us(endpoint_start);
        if (endpoint.endpoint_iaabbs.empty()) {
            return std::nullopt;
        }
        EndpointPayload payload;
        payload.owned_payload = std::move(endpoint.endpoint_iaabbs);
        payload.payload = payload.owned_payload;
        return payload;
    }
    auto reflect_payload = [&](EndpointPayload payload) -> EndpointPayload {
        if (!evidence_frame.symmetry || normalize_sector(evidence_frame.sector) == 0 || payload.payload.empty()) {
            return payload;
        }
        const int n_endpoint_boxes = static_cast<int>(payload.payload.size() / 6u);
        if (n_endpoint_boxes <= 0 || payload.payload.size() != static_cast<std::size_t>(n_endpoint_boxes) * 6u) {
            return payload;
        }
        EndpointPayload reflected;
        reflected.owned_payload.resize(payload.payload.size());
        evidence_frame.symmetry->transform_all_endpoint_iaabbs(
            payload.payload.data(),
            n_endpoint_boxes,
            normalize_sector(evidence_frame.sector),
            reflected.owned_payload.data());
        reflected.payload = reflected.owned_payload;
        reflected.envelope_cache_key = payload.envelope_cache_key;
        reflected.envelope_cacheable = payload.envelope_cacheable;
        return reflected;
    };
    const bool use_endpoint_evidence_cache = validation_config_.enable_endpoint_evidence_cache;
    const bool store_endpoint_evidence_cache =
        use_endpoint_evidence_cache && validation_config_.store_endpoint_evidence_cache;
    const bool read_local_endpoint_cache =
        use_endpoint_evidence_cache && (online_cache_ != nullptr || store_endpoint_evidence_cache);
    auto lookup_external_payload = [&]() -> std::optional<EndpointPayload> {
        if (!validation_config_.external_evidence_materialization || external_evidence_source_ == nullptr) {
            return std::nullopt;
        }
        const auto external_lookup_start = Clock::now();
        std::optional<lect_database::EvidenceRecordView> cached;
        const bool direct_external_keys_match =
            direct_external_evidence_database_ != nullptr &&
            database_.identity().root_domain_fingerprint == direct_external_evidence_database_->identity().root_domain_fingerprint &&
            database_.identity().split_policy_hash == direct_external_evidence_database_->identity().split_policy_hash;
        if (direct_external_keys_match) {
            cached = direct_external_evidence_database_->evidence(key);
        }
        if (!cached) {
            cached = external_evidence_source_->endpoint_for_box_exact(evidence_frame.lookup_intervals, key);
        }
        counters_.materialization_external_lookup_time_us += elapsed_us(external_lookup_start);
        if (!cached) {
            return std::nullopt;
        }
        counters_.materialization_reused_external_evidence += 1;
        last_validation_detail_.reused_external_evidence = true;
        if (validation_config_.external_evidence_backfill_active && store_endpoint_evidence_cache) {
            lect_database::EvidenceRecord backfill;
            backfill.key = key;
            backfill.child_hull = cached->child_hull;
            backfill.unavailable = cached->unavailable;
            backfill.payload.assign(cached->payload.begin(), cached->payload.end());
            if (online_cache_ != nullptr) {
                online_cache_->put_evidence(backfill);
            } else {
                database_.put_evidence(std::move(backfill));
            }
        }
        const auto read_start = Clock::now();
        EndpointPayload payload;
        payload.record_storage = cached->storage;
        payload.storage_owner = cached->storage_owner;
        payload.payload = cached->payload;
        payload.envelope_cache_key = envelope_cache_key;
        payload.envelope_cacheable = true;
        counters_.materialization_external_read_time_us += elapsed_us(read_start);
        return reflect_payload(std::move(payload));
    };
    const bool prefer_external_first =
        validation_config_.external_evidence_materialization && external_evidence_source_ != nullptr &&
        !validation_config_.external_evidence_backfill_active;
    if (prefer_external_first) {
        if (auto payload = lookup_external_payload()) {
            return payload;
        }
    }
    if (read_local_endpoint_cache) {
        if (online_cache_ != nullptr) {
            const auto lookup_start = Clock::now();
            auto cached = online_cache_->evidence(key);
            counters_.materialization_cache_lookup_time_us += elapsed_us(lookup_start);
            if (cached) {
                const auto read_start = Clock::now();
                EndpointPayload payload;
                payload.owned_payload = std::move(cached->payload);
                payload.payload = payload.owned_payload;
                payload.envelope_cache_key = envelope_cache_key;
                payload.envelope_cacheable = true;
                counters_.materialization_cache_read_time_us += elapsed_us(read_start);
                counters_.materialization_reused_endpoint_cache += 1;
                return reflect_payload(std::move(payload));
            }
        }
        const auto database_lookup_start = Clock::now();
        auto local_cached = database_.evidence(key);
        counters_.materialization_cache_lookup_time_us += elapsed_us(database_lookup_start);
        if (local_cached) {
            const auto read_start = Clock::now();
            EndpointPayload payload;
            payload.record_storage = local_cached->storage;
            payload.storage_owner = local_cached->storage_owner;
            payload.payload = local_cached->payload;
            payload.envelope_cache_key = envelope_cache_key;
            payload.envelope_cacheable = true;
            counters_.materialization_cache_read_time_us += elapsed_us(read_start);
            counters_.materialization_reused_endpoint_cache += 1;
            return reflect_payload(std::move(payload));
        }
    } else {
        counters_.materialization_skipped_endpoint_cache += 1;
    }
    if (!prefer_external_first) {
        if (auto payload = lookup_external_payload()) {
            return payload;
        }
    }

    // Cross-task shared endpoint cache (interval-keyed, thread-safe). Lets a
    // worker reuse an endpoint another worker (or an earlier batch) already
    // computed for the identical canonical box, the way a persistent oracle
    // reuses evidence across queries.
    const bool use_shared_endpoint_cache =
        shared_endpoint_cache_ != nullptr &&
        validation_config_.enable_endpoint_evidence_cache &&
        validation_config_.enable_worker_shared_endpoint_cache;
    if (use_shared_endpoint_cache) {
        const auto shared_lookup_start = Clock::now();
        auto shared_cached =
            shared_endpoint_cache_->endpoint_for_box_exact(evidence_frame.lookup_intervals, key);
        counters_.materialization_cache_lookup_time_us += elapsed_us(shared_lookup_start);
        if (shared_cached) {
            counters_.materialization_reused_shared_endpoint_cache += 1;
            last_validation_detail_.reused_endpoint_cache = true;
            EndpointPayload payload;
            payload.record_storage = shared_cached->storage;
            payload.storage_owner = shared_cached->storage_owner;
            payload.payload = shared_cached->payload;
            payload.envelope_cache_key = envelope_cache_key;
            payload.envelope_cacheable = true;
            return reflect_payload(std::move(payload));
        }
    }

    const auto endpoint_start = Clock::now();
    EndpointSourceConfig materialization_config = hifk_config_for_materialization(*this, node, endpoint_config_);
    bool used_source_incremental_state = false;
    EndpointIAABBResult endpoint;
    if (materialization_config.source == EndpointSource::IFK) {
        endpoint = compute_endpoint_iaabb_ifk_aa_stateful(
            robot_, evidence_frame.lookup_intervals, aa_fk_prefix_state_, &used_source_incremental_state);
    } else if (materialization_config.source == EndpointSource::HIFK) {
        endpoint = compute_endpoint_iaabb_hifk_aa_stateful(
            robot_, evidence_frame.lookup_intervals, materialization_config, hifk_aa_state_, &used_source_incremental_state);
    } else {
        endpoint = compute_endpoint_iaabb(
            robot_, evidence_frame.lookup_intervals, materialization_config, nullptr, changed_dim);
    }
    const bool used_incremental_fk =
        used_source_incremental_state &&
        (materialization_config.source == EndpointSource::IFK || materialization_config.source == EndpointSource::HIFK);
    last_validation_detail_.changed_dim = changed_dim;
    last_validation_detail_.used_incremental_fk = used_incremental_fk;
    last_validation_detail_.used_source_incremental_state = used_source_incremental_state;
    if (used_incremental_fk) {
        counters_.materialization_incremental_fk += 1;
    }
    if (used_source_incremental_state) {
        counters_.materialization_source_incremental_state += 1;
    }
    counters_.materialization_endpoint_wall_time_us += elapsed_us(endpoint_start);
    if (endpoint.endpoint_iaabbs.empty()) {
        return std::nullopt;
    }
    EndpointPayload payload;
    payload.owned_payload = std::move(endpoint.endpoint_iaabbs);
    payload.payload = payload.owned_payload;
    payload.envelope_cache_key = envelope_cache_key;
    payload.envelope_cacheable = true;
    lect_database::EvidenceRecord record;
    record.key = key;
    record.payload = payload.owned_payload;
    record.child_hull = false;
    if (store_endpoint_evidence_cache) {
        if (online_cache_ != nullptr) {
            online_cache_->put_evidence(record);
        } else {
            database_.put_evidence(record);
        }
    }
    if (use_shared_endpoint_cache) {
        shared_endpoint_cache_->put(evidence_frame.lookup_intervals, key,
                                    payload.owned_payload, /*child_hull=*/false,
                                    /*unavailable=*/false);
        counters_.materialization_stored_shared_endpoint_cache += 1;
    }
    counters_.materializations += 1;
    if (store_endpoint_evidence_cache) {
        counters_.materialization_stored_endpoint += 1;
    }
    counters_.materialization_endpoint_time_us += endpoint.enumerate_time_us;
    return reflect_payload(std::move(payload));
}

BoxValidation DatabaseBoxOracle::classify_payload(OracleNodeId node,
                                                  const std::vector<Interval>& intervals,
                                                  const EndpointPayload& endpoint_payload) {
    (void)intervals;
    const auto envelope_read_start = Clock::now();
    const LinkEnvelope* envelope = nullptr;
    const bool use_envelope_cache = endpoint_payload.envelope_cacheable &&
        !database_.bulk_prewarm_mode_enabled() &&
        !database_.streaming_prewarm_mode_enabled();
    if (use_envelope_cache) {
        const auto cache_it = envelope_cache_.find(endpoint_payload.envelope_cache_key);
        counters_.materialization_envelope_read_time_us += elapsed_us(envelope_read_start);
        if (cache_it != envelope_cache_.end()) {
            counters_.materialization_reused_cached_envelope += 1;
            envelope = &cache_it->second;
        }
    } else {
        counters_.materialization_envelope_read_time_us += elapsed_us(envelope_read_start);
    }

    LinkEnvelope computed_envelope;
    double envelope_compute_us = 0.0;
    if (envelope == nullptr) {
        const auto envelope_start = Clock::now();
        computed_envelope = compute_link_envelope(endpoint_payload.payload.data(),
                                                  robot_.n_active_links(),
                                                  robot_.active_link_radii(),
                                                  envelope_config_);
        envelope_compute_us = elapsed_us(envelope_start);
        counters_.materialization_envelope_compute_time_us += envelope_compute_us;
        if (use_envelope_cache) {
            auto [cache_it, inserted] = envelope_cache_.try_emplace(endpoint_payload.envelope_cache_key,
                                                                    std::move(computed_envelope));
            if (!inserted) {
                cache_it->second = std::move(computed_envelope);
            }
            envelope = &cache_it->second;
        } else {
            envelope = &computed_envelope;
        }
    }
    EnvelopeCollisionStats collision_stats;
    EnvelopeCollisionOptions collision_options;
    collision_options.safety_epsilon = std::max(envelope_config_.kdop_config.safety_epsilon,
                                                envelope_config_.support_hull_config.safety_epsilon);
    if (const char* dbg = std::getenv("RBF_ENV_DEBUG"); dbg != nullptr && dbg[0] == '1') {
        std::fprintf(stderr,
            "[ENVDBG] cfg.type=%d n_sub=%d keep_kdop=%d || env.type=%d env.n_sub=%d env.n_active=%d link_iaabbs=%zu support_hulls=%zu kdop_intervals=%zu kdop_n_axes=%d\n",
            static_cast<int>(envelope_config_.type), envelope_config_.n_subdivisions,
            static_cast<int>(envelope_config_.support_hull_config.keep_kdop),
            static_cast<int>(envelope->type), envelope->n_subdivisions, envelope->n_active_links,
            envelope->link_iaabbs.size(), envelope->support_hulls.size(),
            envelope->kdop_intervals.size(), envelope->kdop_n_axes);
    }
    const auto collision_start = Clock::now();
    const CollisionResultKind collision = collide_envelope_aabbs(*envelope,
                                                                 scene_.obstacles().data(),
                                                                 scene_.n_obstacles(),
                                                                 collision_options,
                                                                 &collision_stats);
    const double collision_us = elapsed_us(collision_start);
    counters_.materialization_envelope_collision_time_us += collision_us;
    counters_.materialization_envelope_time_us += envelope_compute_us + collision_us;
    record_envelope_collision(counters_, collision_stats);
    counters_.envelope_collision_queries += 1;
    if (const char* dbg = std::getenv("RBF_ENV_DEBUG"); dbg != nullptr && dbg[0] == '1') {
        EnvelopeCollisionStats all_stats;
        EnvelopeCollisionOptions all_opts = collision_options;
        all_opts.count_all_pairs = true;
        const CollisionResultKind all = collide_envelope_aabbs(*envelope,
            scene_.obstacles().data(), scene_.n_obstacles(), all_opts, &all_stats);
        std::fprintf(stderr,
            "[ENVDBG] result=%d(0=Free,1=Maybe) all=%d || aabb_tests=%lld aabb_rej=%lld link_tests=%lld link_rej=%lld kdop_tests=%lld kdop_rej=%lld gjk_tests=%lld gjk_rej=%lld maybe_pairs=%lld\n",
            static_cast<int>(collision), static_cast<int>(all),
            (long long)all_stats.envelope_aabb_tests, (long long)all_stats.envelope_aabb_rejects,
            (long long)all_stats.link_aabb_tests, (long long)all_stats.link_aabb_rejects,
            (long long)all_stats.kdop_tests, (long long)all_stats.kdop_rejects,
            (long long)all_stats.gjk_tests, (long long)all_stats.gjk_rejects,
            (long long)all_stats.maybe_pairs);
    }
    last_validation_detail_.node = node;
    last_validation_detail_.depth = depth(node);
    last_validation_detail_.mode = validation_config_.mode;
    last_validation_detail_.endpoint_source = endpoint_config_.source;
    last_validation_detail_.endpoint_safety_level = endpoint_source_default_safety(endpoint_config_.source);
    last_validation_detail_.endpoint_is_safe = endpoint_safety_is_certified(last_validation_detail_.endpoint_safety_level);
    last_validation_detail_.materialized = true;
    last_validation_detail_.aabb_overlap = collision != CollisionResultKind::DefinitelyFree;
    if (collision == CollisionResultKind::DefinitelyFree) {
        counters_.envelope_collision_free += 1;
        const bool endpoint_certified = endpoint_safety_is_certified(last_validation_detail_.endpoint_safety_level);
        const bool accept_heuristic = validation_config_.accept_unsafe_free ||
            validation_config_.mode == OracleValidationMode::CoverageHeuristic;
        if (endpoint_certified || accept_heuristic) {
            last_validation_detail_.validation = BoxValidation::Free;
            if (endpoint_certified) {
                counters_.certified_free += 1;
                last_validation_detail_.safety_status = BoxSafetyStatus::CertifiedFree;
            } else {
                counters_.provisional_free += 1;
                last_validation_detail_.safety_status = BoxSafetyStatus::ProvisionalFree;
                last_validation_detail_.strict_audit_required = true;
            }
            last_validation_detail_.collision_possible = false;
            return BoxValidation::Free;
        }
        counters_.unsafe_free_rejected += 1;
    }
    counters_.collision_possible += 1;
    counters_.envelope_collision_maybe += 1;
    last_validation_detail_.validation = BoxValidation::Unknown;
    last_validation_detail_.safety_status = BoxSafetyStatus::Unknown;
    last_validation_detail_.strict_audit_required = collision == CollisionResultKind::DefinitelyFree;
    last_validation_detail_.collision_possible = true;
    return BoxValidation::Unknown;
}

BoxValidation DatabaseBoxOracle::validate_node(OracleNodeId node,
                                               const std::vector<Interval>& intervals,
                                               int changed_dim) {
    const auto total_start = Clock::now();
    counters_.node_validations += 1;
    last_validation_detail_ = {};
    last_validation_detail_.node = node;
    last_validation_detail_.depth = depth(node);
    last_validation_detail_.mode = validation_config_.mode;
    const double preamble_us = elapsed_us(total_start);
    counters_.validate_node_preamble_time_us += preamble_us;
    if (scene_.empty()) {
        counters_.certified_free += 1;
        last_validation_detail_.validation = BoxValidation::Free;
        last_validation_detail_.safety_status = BoxSafetyStatus::CertifiedFree;
        last_validation_detail_.collision_possible = false;
        const double total_us = elapsed_us(total_start);
        counters_.validate_node_total_time_us += total_us;
        counters_.validate_node_overhead_time_us += std::max(0.0, total_us - preamble_us);
        return BoxValidation::Free;
    }
    const auto endpoint_path_start = Clock::now();
    auto payload = endpoint_payload_for_node(node, intervals, changed_dim);
    const double endpoint_path_us = elapsed_us(endpoint_path_start);
    counters_.validate_node_endpoint_path_time_us += endpoint_path_us;
    if (!payload) {
        counters_.collision_possible += 1;
        last_validation_detail_.validation = BoxValidation::Unknown;
        last_validation_detail_.safety_status = BoxSafetyStatus::Unknown;
        last_validation_detail_.collision_possible = true;
        const double total_us = elapsed_us(total_start);
        counters_.validate_node_total_time_us += total_us;
        counters_.validate_node_overhead_time_us += std::max(0.0, total_us - preamble_us - endpoint_path_us);
        return BoxValidation::Unknown;
    }
    const auto classify_start = Clock::now();
    const BoxValidation result = classify_payload(node, intervals, *payload);
    const double classify_us = elapsed_us(classify_start);
    counters_.validate_node_classify_time_us += classify_us;
    const double total_us = elapsed_us(total_start);
    counters_.validate_node_total_time_us += total_us;
    counters_.validate_node_overhead_time_us +=
        std::max(0.0, total_us - preamble_us - endpoint_path_us - classify_us);
    return result;
}

bool DatabaseBoxOracle::validate_intervals(const std::vector<Interval>& intervals) {
    counters_.interval_validations += 1;
    return !checker_.check_box(intervals);
}

bool DatabaseBoxOracle::is_reserved(OracleNodeId node) const {
    return node_to_box_.find(node) != node_to_box_.end();
}

std::optional<int> DatabaseBoxOracle::reservation_owner(OracleNodeId node) const {
    const auto it = node_to_box_.find(node);
    return it == node_to_box_.end() ? std::nullopt : std::optional<int>(it->second);
}

void DatabaseBoxOracle::reserve_node(OracleNodeId node, int box_id) {
    node_to_box_[node] = box_id;
    box_to_node_[box_id] = node;
}

void DatabaseBoxOracle::release_node(OracleNodeId node) {
    const auto it = node_to_box_.find(node);
    if (it == node_to_box_.end()) {
        return;
    }
    box_to_node_.erase(it->second);
    node_to_box_.erase(it);
}

void DatabaseBoxOracle::release_box(int box_id) {
    const auto it = box_to_node_.find(box_id);
    if (it == box_to_node_.end()) {
        return;
    }
    node_to_box_.erase(it->second);
    box_to_node_.erase(it);
}

void DatabaseBoxOracle::clear_reservations() {
    node_to_box_.clear();
    box_to_node_.clear();
}

void DatabaseBoxOracle::record_visit(OracleNodeId node) {
    if (node >= 0) {
        ++visit_counts_[node];
    }
}

OracleNodeId DatabaseBoxOracle::select_unexplored_node() const {
    OracleNodeId best_node = kInvalidOracleNodeId;
    double best_weight = -1.0;
    for (lect_database::NodeId node_id : database_.node_ids()) {
        const auto topology = database_.topology(node_id);
        const OracleNodeId node = from_database_node(node_id);
        if (!topology.leaf || is_reserved(node)) {
            continue;
        }
        const double volume = interval_volume(node_intervals(node));
        const auto visit_it = visit_counts_.find(node);
        const double visit_count = visit_it == visit_counts_.end()
            ? 0.0
            : static_cast<double>(visit_it->second);
        const double weight = volume / (visit_count + 1.0);
        if (weight > best_weight) {
            best_weight = weight;
            best_node = node;
        }
    }
    if (best_node >= 0) {
        ++visit_counts_[best_node];
    }
    return best_node;
}

int DatabaseBoxOracle::common_ancestor_depth(OracleNodeId lhs_node, OracleNodeId rhs_node) const {
    const auto lhs = to_database_node(lhs_node);
    const auto rhs = to_database_node(rhs_node);
    if (lhs_node < 0 || rhs_node < 0 || !database_.node(lhs) || !database_.node(rhs)) {
        return -1;
    }
    const auto ancestor = database_.lca(lhs, rhs);
    if (!lect_database::valid_node_id(ancestor)) {
        return -1;
    }
    return database_.topology(ancestor).depth;
}

std::unique_ptr<BoxOracleSession> DatabaseBoxOracle::make_session(const OracleSessionConfig& config) {
    return std::make_unique<DatabaseBoxOracleSession>(*this, config);
}

std::shared_ptr<lect_database::SharedEndpointEvidenceCache> DatabaseBoxOracle::shared_endpoint_cache() {
    if (!shared_endpoint_cache_) {
        shared_endpoint_cache_ = std::make_shared<lect_database::SharedEndpointEvidenceCache>(
            validation_config_.shared_endpoint_cache_max_entries,
            validation_config_.shared_endpoint_cache_max_bytes);
    }
    return shared_endpoint_cache_;
}

void DatabaseBoxOracle::set_scene(Scene scene) {
    scene_ = std::move(scene);
    checker_.set_scene(scene_);
}

DatabaseBoxOracleSession::DatabaseBoxOracleSession(DatabaseBoxOracle& master,
                                                   const OracleSessionConfig& config)
    : master_(master),
      master_domain_root_(config.domain_root >= 0 ? config.domain_root : master.root_node()),
      read_only_(config.read_only),
      temp_dir_(make_temp_dir()) {
    if (master_domain_root_ < 0 || !lect_database::valid_node_id(to_database_node(master_domain_root_)) ||
        !master.database().node(to_database_node(master_domain_root_))) {
        throw std::out_of_range("LECTDatabase oracle session domain root is out of range");
    }
    const auto worker_root = master.node_intervals(master_domain_root_);
    if (worker_root.empty()) {
        throw std::runtime_error("LECTDatabase oracle session domain root has no intervals");
    }
    std::error_code ec;
    std::filesystem::create_directories(temp_dir_, ec);
    if (ec) {
        throw std::runtime_error("LECTDatabase oracle session failed to create temp directory");
    }

    std::string reason;
    worker_database_.emplace();
    auto worker_config = make_worker_database_config(master_,
                                                     temp_dir_,
                                                     worker_root,
                                                     master_.depth(master_domain_root_));
    if (!worker_database_->open(std::move(worker_config), &reason)) {
        throw std::runtime_error("LECTDatabase oracle session failed to open worker database: " + reason);
    }
    auto worker_validation_config = master_.validation_config();
    worker_validation_config.store_endpoint_evidence_cache = false;
    worker_validation_config.external_evidence_backfill_active = false;
    worker_oracle_ = std::make_unique<DatabaseBoxOracle>(master_.robot(),
                                                         *worker_database_,
                                                         master_.scene(),
                                                         master_.endpoint_config(),
                                                         master_.envelope_config(),
                                                         worker_validation_config,
                                                         master_.external_evidence_source(),
                                                         master_.direct_external_evidence_database());
    if (worker_validation_config.enable_worker_shared_endpoint_cache) {
        worker_oracle_->set_shared_endpoint_cache(master_.shared_endpoint_cache());
    }
    worker_oracle_->best_tighten_depth_dims_ = master_.best_tighten_depth_dims_;
    worker_oracle_->best_tighten_recent_dims_ = master_.best_tighten_recent_dims_;
    worker_oracle_->best_tighten_reference_volumes_ = master_.best_tighten_reference_volumes_;
    node_remap_.emplace(worker_oracle_->root_node(), master_domain_root_);
}

DatabaseBoxOracleSession::~DatabaseBoxOracleSession() {
    worker_oracle_.reset();
    worker_database_.reset();
    std::error_code ec;
    std::filesystem::remove_all(temp_dir_, ec);
}

bool DatabaseBoxOracleSession::commit() {
    if (read_only_) {
        committed_ = true;
        return true;
    }
    if (committed_) {
        return true;
    }
    if (!replay_structure(worker_oracle_->root_node(), master_domain_root_)) {
        return false;
    }
    if (!copy_worker_leaf_evidence()) {
        return false;
    }
    for (const auto& [worker_node, visit_count] : worker_oracle_->visit_counts_) {
        if (visit_count == 0) {
            continue;
        }
        const OracleNodeId master_node = remap_lookup(node_remap_, worker_node);
        if (master_node >= 0) {
            master_.visit_counts_[master_node] += visit_count;
        }
    }
    committed_ = true;
    return true;
}

OracleNodeId DatabaseBoxOracleSession::map_node_to_master(OracleNodeId worker_node) const {
    return remap_lookup(node_remap_, worker_node);
}

bool DatabaseBoxOracleSession::replay_structure(OracleNodeId worker_node, OracleNodeId master_node) {
    node_remap_[worker_node] = master_node;
    const auto worker_topology = worker_database_->topology(to_database_node(worker_node));
    if (worker_topology.leaf) {
        return true;
    }

    auto master_topology = master_.database().topology(to_database_node(master_node));
    if (master_topology.leaf) {
        const auto children = master_.database().split_leaf(to_database_node(master_node),
                                                            worker_topology.split_dim,
                                                            worker_topology.split_value);
        if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
            return false;
        }
        master_topology = master_.database().topology(to_database_node(master_node));
    } else if (master_topology.split_dim != worker_topology.split_dim ||
               std::abs(master_topology.split_value - worker_topology.split_value) > 1e-12) {
        return false;
    }

    const OracleNodeId master_left = from_database_node(master_topology.left);
    const OracleNodeId master_right = from_database_node(master_topology.right);
    if (master_left < 0 || master_right < 0) {
        return false;
    }
    node_remap_[from_database_node(worker_topology.left)] = master_left;
    node_remap_[from_database_node(worker_topology.right)] = master_right;
    return replay_structure(from_database_node(worker_topology.left), master_left) &&
           replay_structure(from_database_node(worker_topology.right), master_right);
}

bool DatabaseBoxOracleSession::copy_worker_leaf_evidence() {
    for (const auto& record : worker_database_->evidence_records()) {
        const auto topology = worker_database_->topology(record.key.node_id);
        if (!topology.leaf) {
            continue;
        }
        const OracleNodeId mapped_node = remap_lookup(node_remap_, from_database_node(record.key.node_id));
        if (mapped_node < 0) {
            return false;
        }
        auto replay = record;
        replay.key.node_id = to_database_node(mapped_node);
        const auto mapped_topology = master_.database().topology(replay.key.node_id);
        replay.key.node_path = mapped_topology.path;
        replay.key.node_path_valid = lect_database::valid_node_id(mapped_topology.id);
        if (!master_.database().put_evidence(std::move(replay))) {
            return false;
        }
    }
    return true;
}

std::filesystem::path DatabaseBoxOracleSession::make_temp_dir() {
    return std::filesystem::temp_directory_path() /
        ("lectdb_sbf_session_" + std::to_string(next_session_id()));
}

std::unique_ptr<BoxOracleSession> DatabaseBoxOracleFactory::make_session(const OracleSessionConfig& config) {
    return master_.make_session(config);
}

}  // namespace rbf
