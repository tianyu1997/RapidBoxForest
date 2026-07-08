#include "connector_chain_pave_internal.h"

#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace rbf {
namespace {

double chain_pave_segment_exit_param(const BoxNode& box,
                                     const Eigen::VectorXd& a,
                                     const Eigen::VectorXd& b,
                                     double u0) {
    double u_hi = 1.0;
    const Eigen::VectorXd v = b - a;
    for (int d = 0; d < a.size(); ++d) {
        const double lo = box.joint_intervals[d].lo;
        const double hi = box.joint_intervals[d].hi;
        if (std::abs(v[d]) < 1e-15) {
            continue;
        }
        const double t1 = (lo - a[d]) / v[d];
        const double t2 = (hi - a[d]) / v[d];
        u_hi = std::min(u_hi, std::max(t1, t2));
    }
    return std::max(u0, std::min(1.0, u_hi));
}

Eigen::VectorXd chain_pave_boundary_seed_from_box(const BoxNode& box,
                                                  const Eigen::VectorXd& from,
                                                  const Eigen::VectorXd& target,
                                                  double adjacency_tolerance,
                                                  double gap_fill_min_step) {
    const double seg_len = (target - from).norm();
    if (seg_len < 1e-12) {
        return target;
    }
    const double u_exit = chain_pave_segment_exit_param(box, from, target, 0.0);
    // The FFB seed should step just outside the current box face; otherwise FFB
    // can certify a free box beyond a tiny gap and return a free but non-adjacent
    // result.
    const double face_epsilon =
        std::max(16.0 * std::max(0.0, adjacency_tolerance),
                 std::min(1e-6, std::max(gap_fill_min_step, 1e-12)));
    const double u_step =
        std::max(1e-12, face_epsilon / std::max(seg_len, 1e-12));
    const double u_seed = std::min(1.0, u_exit + u_step);
    return (from + u_seed * (target - from)).eval();
}

std::uint64_t chain_pave_mix_u64(std::uint64_t value) {
    value ^= value >> 33;
    value *= 0xff51afd7ed558ccdULL;
    value ^= value >> 33;
    value *= 0xc4ceb9fe1a85ec53ULL;
    value ^= value >> 33;
    return value;
}

}  // namespace

void record_chain_pave_boundary_ffb_failure(const FindFreeBoxResult& result,
                                            const Eigen::VectorXd& seed,
                                            BoxOracle& oracle,
                                            const ChainPaveConfig& config,
                                            StageContext& context) {
    context.diagnostics().add_counter(
        "connector.chain_pave_boundary_fail_code." + std::to_string(result.fail_code));
    if (result.seed_collision || result.fail_code == 1) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_seed_collision");
    }
    if (result.hit_unknown_depth_cap || result.hit_reserved_depth_cap ||
        result.fail_code == 2) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_depth_cap");
    }
    if (result.hit_unknown_depth_cap) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_unknown_depth_cap");
    }
    if (result.hit_reserved_depth_cap) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_reserved_depth_cap");
    }
    if (result.fail_code == 3) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_occupied");
    }
    if (result.deadline_reached || result.fail_code == 4) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_deadline");
    }
    if (result.fail_code == 5) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_out_of_domain");
    }
    if (result.fail_code == 6) {
        context.diagnostics().add_counter("connector.chain_pave_boundary_fail_split");
    }
    record_optional_chain_pave_boundary_failure_payload(result, seed, oracle, config);
}

std::vector<Eigen::VectorXd> chain_pave_boundary_seed_candidates(const BoxNode& box,
                                                                 const Eigen::VectorXd& from,
                                                                 const Eigen::VectorXd& target,
                                                                 double requested_step,
                                                                 double adjacency_tolerance,
                                                                 double gap_fill_min_step) {
    std::vector<Eigen::VectorXd> seeds;
    const Eigen::VectorXd forward_seed =
        chain_pave_boundary_seed_from_box(box,
                                          from,
                                          target,
                                          adjacency_tolerance,
                                          gap_fill_min_step);
    seeds.push_back(forward_seed);
    if (from.size() != target.size() || box.n_dims() != from.size()) {
        return seeds;
    }
    const Eigen::VectorXd delta = target - from;
    const double distance = delta.norm();
    if (distance < 1e-12) {
        return seeds;
    }

    struct LateralDim {
        int dim = -1;
        double score = 0.0;
        double width = 0.0;
    };
    std::vector<LateralDim> dims;
    dims.reserve(static_cast<std::size_t>(from.size()));
    for (int dim = 0; dim < from.size(); ++dim) {
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
        const double width = interval.width();
        if (width <= 2.0 * adjacency_tolerance) {
            continue;
        }
        const double alignment = std::abs(delta[dim]) / distance;
        dims.push_back({dim, width * (1.0 - alignment), width});
    }
    std::sort(dims.begin(), dims.end(), [](const LateralDim& lhs,
                                           const LateralDim& rhs) {
        return lhs.score > rhs.score;
    });

    const double base_radius =
        std::max(gap_fill_min_step,
                 0.35 * std::max(requested_step, gap_fill_min_step));
    const int max_lateral_dims = std::min<int>(2, static_cast<int>(dims.size()));
    for (int rank = 0; rank < max_lateral_dims; ++rank) {
        const int dim = dims[static_cast<std::size_t>(rank)].dim;
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
        const double radius =
            std::min(base_radius,
                     std::max(gap_fill_min_step,
                              0.45 * dims[static_cast<std::size_t>(rank)].width));
        for (const double sign : {1.0, -1.0}) {
            Eigen::VectorXd candidate = seeds.front();
            candidate[dim] = std::clamp(candidate[dim] + sign * radius,
                                        interval.lo + adjacency_tolerance,
                                        interval.hi - adjacency_tolerance);
            if ((candidate - from).norm() >=
                std::max(gap_fill_min_step, 1e-6) * 0.25 &&
                (candidate - seeds.front()).norm() > 1e-12) {
                seeds.push_back(std::move(candidate));
            }
        }
    }
    return seeds;
}

Eigen::VectorXd chain_pave_closest_point_in_box(const BoxNode& box,
                                                const Eigen::VectorXd& point) {
    Eigen::VectorXd out(point.size());
    for (int dim = 0; dim < point.size(); ++dim) {
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(dim)];
        out[dim] = std::clamp(point[dim], interval.lo, interval.hi);
    }
    return out;
}

std::uint64_t chain_pave_boundary_seed_key(int parent_id,
                                           std::size_t segment_index,
                                           const BoxNode& parent_box,
                                           const Eigen::VectorXd& cursor,
                                           const Eigen::VectorXd& seed,
                                           double adjacency_tolerance,
                                           double gap_fill_min_step) {
    int face_dim = 0;
    int side = 1;
    double best_gap = -1.0;
    for (int dim = 0; dim < seed.size(); ++dim) {
        const auto& interval = parent_box.joint_intervals[static_cast<std::size_t>(dim)];
        double gap = 0.0;
        int candidate_side = seed[dim] >= cursor[dim] ? 1 : 0;
        if (seed[dim] > interval.hi + adjacency_tolerance) {
            gap = seed[dim] - interval.hi;
            candidate_side = 1;
        } else if (seed[dim] < interval.lo - adjacency_tolerance) {
            gap = interval.lo - seed[dim];
            candidate_side = 0;
        } else {
            gap = std::abs(seed[dim] - cursor[dim]) * 1e-3;
        }
        if (gap > best_gap) {
            best_gap = gap;
            face_dim = dim;
            side = candidate_side;
        }
    }
    std::uint64_t key = chain_pave_mix_u64(
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(parent_id)));
    key ^= chain_pave_mix_u64(static_cast<std::uint64_t>(
        segment_index + 0x9e3779b97f4a7c15ULL));
    key ^= chain_pave_mix_u64(static_cast<std::uint64_t>(
        (face_dim & 0xff) | ((side & 0x1) << 8)));
    const double bucket =
        std::max(1e-6, std::max(gap_fill_min_step, 1e-9) * 0.25);
    for (int dim = 0; dim < seed.size(); ++dim) {
        const auto quantized =
            static_cast<std::int64_t>(std::llround(seed[dim] / bucket));
        key ^= chain_pave_mix_u64(static_cast<std::uint64_t>(
            quantized + 0x9e3779b97f4a7c15LL +
            static_cast<std::int64_t>(dim) * 0x100000001b3LL));
    }
    return key;
}

void record_chain_pave_connected_stats(StageContext& context,
                                       int added,
                                       int max_chain,
                                       const ChainPaveConnectedStats& stats) {
    context.diagnostics().set_value("connector.chain_pave_connected_added",
                                    static_cast<double>(added));
    context.diagnostics().set_value("connector.chain_pave_connected_segments",
                                    static_cast<double>(stats.segments));
    context.diagnostics().set_value("connector.chain_pave_connected_steps",
                                    static_cast<double>(stats.steps));
    context.diagnostics().set_value("connector.chain_pave_connected_reach_failures",
                                    static_cast<double>(stats.reach_failures));
    context.diagnostics().set_value("connector.chain_pave_connected_target_hits",
                                    static_cast<double>(stats.target_hits));
    context.diagnostics().set_value("connector.chain_pave_boundary_target_hits",
                                    static_cast<double>(stats.target_hits));
    if (added >= max_chain) {
        context.diagnostics().add_counter("connector.chain_pave_connected_max_chain_hits");
    }
}

}  // namespace rbf
