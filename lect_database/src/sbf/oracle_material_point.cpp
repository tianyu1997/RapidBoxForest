#include "oracle_material_point.h"

#include <sbf/core/fk_state.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace rbf {
namespace {

double signed_distance_to_aabb(const Eigen::Vector3d& point, const Obstacle& obstacle) {
    double outside_sq = 0.0;
    double inside_margin = std::numeric_limits<double>::infinity();
    bool inside = true;
    for (int axis = 0; axis < 3; ++axis) {
        const double lo = static_cast<double>(obstacle.bounds[axis]);
        const double hi = static_cast<double>(obstacle.bounds[axis + 3]);
        if (point[axis] < lo) {
            const double delta = lo - point[axis];
            outside_sq += delta * delta;
            inside = false;
        } else if (point[axis] > hi) {
            const double delta = point[axis] - hi;
            outside_sq += delta * delta;
            inside = false;
        } else {
            inside_margin = std::min(inside_margin, point[axis] - lo);
            inside_margin = std::min(inside_margin, hi - point[axis]);
        }
    }
    if (!inside) {
        return std::sqrt(outside_sq);
    }
    return -std::max(0.0, inside_margin);
}

Eigen::Vector3d fk_translation(const FKState& state, int frame) {
    return {state.prefix_lo[frame][3], state.prefix_lo[frame][7], state.prefix_lo[frame][11]};
}

Eigen::Vector3d fk_z_axis(const FKState& state, int frame) {
    Eigen::Vector3d axis(state.prefix_lo[frame][2],
                         state.prefix_lo[frame][6],
                         state.prefix_lo[frame][10]);
    const double norm = axis.norm();
    if (norm > 0.0) {
        axis /= norm;
    }
    return axis;
}

bool robot_all_revolute(const Robot& robot) {
    for (const auto& dh : robot.dh_params()) {
        if (dh.joint_type != 0) {
            return false;
        }
    }
    return true;
}

double revolute_material_point_motion_bound(const Robot& robot,
                                            const std::vector<Interval>& intervals,
                                            const FKState& center_fk,
                                            const Eigen::Vector3d& world_point,
                                            int frame_limit) {
    const int n = std::min({robot.n_joints(), frame_limit, static_cast<int>(intervals.size())});
    double bound = 0.0;
    for (int joint = 0; joint < n; ++joint) {
        const double delta = std::max(std::abs(intervals[static_cast<std::size_t>(joint)].center() -
                                               intervals[static_cast<std::size_t>(joint)].lo),
                                      std::abs(intervals[static_cast<std::size_t>(joint)].hi -
                                               intervals[static_cast<std::size_t>(joint)].center()));
        if (delta <= 0.0) {
            continue;
        }
        const Eigen::Vector3d origin = fk_translation(center_fk, joint);
        const Eigen::Vector3d axis = fk_z_axis(center_fk, joint);
        if (axis.squaredNorm() <= 0.0) {
            continue;
        }
        const double lever = (world_point - origin).cross(axis).norm();
        const double capped_delta = std::min(delta, PI);
        bound += 2.0 * lever * std::sin(0.5 * capped_delta);
    }
    return bound;
}

}  // namespace

std::optional<MaterialPointOccupiedWitness> try_material_point_occupied_witness(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const LinkEnvelope& envelope,
    const Scene& scene,
    const OccupiedCertificateConfig& config) {
    (void)envelope;
    if (scene.empty() || static_cast<int>(intervals.size()) != robot.n_joints()) {
        return std::nullopt;
    }
    // The current production checker models AABB obstacles. Their signed
    // distance is exact and 1-Lipschitz, so they can serve as the SDF source.
    // The material points below are fixed points on active revolute-link
    // centerlines. Prismatic links are intentionally excluded until the checker
    // exposes a material-coordinate witness for variable-length links.
    if (!robot_all_revolute(robot)) {
        return std::nullopt;
    }

    std::vector<Interval> center_intervals = intervals;
    for (auto& interval : center_intervals) {
        const double c = interval.center();
        interval.lo = c;
        interval.hi = c;
    }
    const FKState center_fk = compute_fk_full(robot, center_intervals);
    if (!center_fk.valid) {
        return std::nullopt;
    }

    std::optional<MaterialPointOccupiedWitness> best;
    constexpr std::array<double, 5> sample_ts = {0.0, 0.25, 0.5, 0.75, 1.0};
    const int n_active = robot.n_active_links();
    const int* active_map = robot.active_link_map();
    for (int active = 0; active < n_active; ++active) {
        const int link = active_map[active];
        if (link < 0 || link + 1 >= center_fk.n_tf) {
            continue;
        }
        const Eigen::Vector3d origin = fk_translation(center_fk, link);
        const Eigen::Vector3d target = fk_translation(center_fk, link + 1);
        for (double t : sample_ts) {
            const Eigen::Vector3d world_point = origin + t * (target - origin);
            const double motion_bound =
                revolute_material_point_motion_bound(robot, intervals, center_fk, world_point, link + 1);
            for (int obs = 0; obs < scene.n_obstacles(); ++obs) {
                const double signed_distance =
                    signed_distance_to_aabb(world_point,
                                            scene.obstacles()[static_cast<std::size_t>(obs)]);
                MaterialPointOccupiedWitness witness;
                witness.link_id = link;
                witness.obstacle_id = obs;
                witness.link_point = world_point;
                witness.center_signed_distance = signed_distance;
                witness.motion_bound = motion_bound;
                witness.epsilon_num = config.numerical_epsilon +
                    std::max(0.0, config.min_penetration_margin);
                if (!witness.certifies_occupied()) {
                    continue;
                }
                if (!best ||
                    signed_distance + motion_bound <
                        best->center_signed_distance + best->motion_bound) {
                    best = witness;
                }
            }
        }
    }
    return best;
}

}  // namespace rbf
