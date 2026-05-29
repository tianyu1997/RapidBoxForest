#include <sbf/envelope/ifk_aa_source.h>

#include <sbf/core/aa_fk.h>
#include <sbf/envelope/dh_enumerate.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <queue>
#include <vector>

namespace rbf {

EndpointIAABBResult compute_endpoint_iaabb_ifk_aa(
    const Robot& robot,
    const std::vector<Interval>& intervals)
{
    EndpointIAABBResult result = compute_endpoint_iaabb_hifk_aa(
        robot,
        intervals,
        0,
        1,
        0.0);
    result.source = EndpointSource::IFK;
    return result;
}

EndpointIAABBResult compute_endpoint_iaabb_ifk_aa_stateful(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    AaFkPrefixState& state)
{
    EndpointIAABBResult result;
    result.source = EndpointSource::IFK;
    result.is_safe = true;
    result.safety_level = EndpointSafetyLevel::Certified;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(result.endpoint_iaabb_len());

    if (robot.n_joints() <= 0) {
        state.valid = false;
        return result;
    }

    if (!aa_fk_endpoint_incremental(robot, intervals, result.endpoint_iaabbs.data(), state)) {
        aa_fk_endpoint_full(robot, intervals, result.endpoint_iaabbs.data(), state);
    }
    return result;
}

namespace {

constexpr double kPlanarExtentEpsilon = 1e-12;

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

int widest_positive_width_dim(const std::vector<Interval>& intervals) {
    int best_dim = 0;
    double best_width = -1.0;
    for (int joint = 0; joint < static_cast<int>(intervals.size()); ++joint) {
        const double width = intervals[static_cast<std::size_t>(joint)].width();
        if (width > best_width) {
            best_width = width;
            best_dim = joint;
        }
    }
    return best_dim;
}

bool allowed_hifk_dimension(
    const std::vector<Interval>& intervals,
    int dim,
    double min_width)
{
    return dim >= 0 && dim < static_cast<int>(intervals.size()) &&
           intervals[static_cast<std::size_t>(dim)].width() > min_width;
}

std::vector<int> widest_root_order(const std::vector<Interval>& root_intervals, int n_dims) {
    std::vector<int> order(static_cast<std::size_t>(std::max(0, n_dims)));
    for (int dim = 0; dim < n_dims; ++dim) {
        order[static_cast<std::size_t>(dim)] = dim;
    }
    std::stable_sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        const double lhs_width = lhs < static_cast<int>(root_intervals.size())
            ? std::max(0.0, root_intervals[static_cast<std::size_t>(lhs)].width())
            : 0.0;
        const double rhs_width = rhs < static_cast<int>(root_intervals.size())
            ? std::max(0.0, root_intervals[static_cast<std::size_t>(rhs)].width())
            : 0.0;
        if (lhs_width != rhs_width) {
            return lhs_width > rhs_width;
        }
        return lhs < rhs;
    });
    return order;
}

int aafk_volume_min_dim(
    const Robot& robot,
    const std::vector<Interval>& intervals)
{
    const int n_joints = static_cast<int>(intervals.size());
    const int n_active_links = robot.n_active_links();
    int best_dim = -1;
    double best_value = std::numeric_limits<double>::infinity();

    for (int joint = 0; joint < n_joints; ++joint) {
        const double width = intervals[static_cast<std::size_t>(joint)].width();
        if (width <= 0.0) {
            continue;
        }

        const double mid =
            (intervals[static_cast<std::size_t>(joint)].lo + intervals[static_cast<std::size_t>(joint)].hi) * 0.5;
        std::vector<Interval> left(intervals);
        std::vector<Interval> right(intervals);
        left[static_cast<std::size_t>(joint)].hi = mid;
        right[static_cast<std::size_t>(joint)].lo = mid;

        std::vector<float> left_iaabbs(static_cast<std::size_t>(n_active_links) * 2 * 6);
        std::vector<float> right_iaabbs(static_cast<std::size_t>(n_active_links) * 2 * 6);
        compute_endpoint_iaabb_aa_fk_raw(robot, left, left_iaabbs.data());
        compute_endpoint_iaabb_aa_fk_raw(robot, right, right_iaabbs.data());

        const double left_measure = endpoint_split_measure(left_iaabbs, n_active_links);
        const double right_measure = endpoint_split_measure(right_iaabbs, n_active_links);
        const double minimax = std::max(left_measure, right_measure);
        if (minimax < best_value) {
            best_value = minimax;
            best_dim = joint;
        }
    }

    return best_dim >= 0 ? best_dim : widest_positive_width_dim(intervals);
}

struct HifkDepthSplitSchedule {
    HifkSplitStrategy strategy = HifkSplitStrategy::RoundRobin;
    std::vector<int> depth_dims;
    std::vector<int> root_order;
    int depth_offset = 0;
    double min_width = 0.0;

    void reset(const EndpointSourceConfig& config,
               const std::vector<Interval>& intervals,
               int capacity_hint) {
        strategy = config.hifk_split_strategy;
        depth_offset = std::max(0, config.hifk_depth_offset);
        min_width = std::max(0.0, config.hifk_min_split_width);
        depth_dims.clear();
        root_order.clear();

        if (strategy == HifkSplitStrategy::FixedDepthSchedule) {
            depth_dims = config.hifk_depth_dimensions;
            if (depth_dims.empty() && capacity_hint > 0) {
                depth_dims.assign(static_cast<std::size_t>(capacity_hint), -1);
            }
            return;
        }

        if (strategy == HifkSplitStrategy::WidestRoot) {
            const std::vector<Interval>& root_intervals =
                config.hifk_root_intervals.empty() ? intervals : config.hifk_root_intervals;
            root_order = widest_root_order(root_intervals, static_cast<int>(intervals.size()));
        }
    }

    int resolve(
        int depth_from_root,
        const std::vector<Interval>& intervals) const
    {
        if (depth_from_root < 0) {
            return widest_positive_width_dim(intervals);
        }

        const int global_depth = depth_from_root + depth_offset;
        if (strategy == HifkSplitStrategy::FixedDepthSchedule) {
            if (global_depth >= 0 && global_depth < static_cast<int>(depth_dims.size())) {
                const int stored = depth_dims[static_cast<std::size_t>(global_depth)];
                if (allowed_hifk_dimension(intervals, stored, min_width)) {
                    return stored;
                }
            }
            return widest_positive_width_dim(intervals);
        }

        if (strategy == HifkSplitStrategy::WidestRoot && !root_order.empty()) {
            const int stored = root_order[static_cast<std::size_t>(global_depth % static_cast<int>(root_order.size()))];
            if (allowed_hifk_dimension(intervals, stored, min_width)) {
                return stored;
            }
            return widest_positive_width_dim(intervals);
        }

        const int stored = global_depth % static_cast<int>(intervals.size());
        if (allowed_hifk_dimension(intervals, stored, min_width)) {
            return stored;
        }
        return widest_positive_width_dim(intervals);
    }
};

struct HifkAaWs {
    std::vector<AffineMatrix4> prefix;
    std::vector<float> leaf;
    int n_joints = 0;
    int n_tf = 0;

    AffineMatrix4* at(int depth) {
        return prefix.data() + static_cast<std::size_t>(depth) * (n_tf + 1);
    }
};

void hifk_aa_recurse(
    const Robot& robot,
    std::vector<Interval>& intervals,
    HifkDepthSplitSchedule& schedule,
    HifkAaWs& workspace,
    int depth,
    int k_valid,
    float* out_hull,
    int n_active_links,
    int depth_remaining)
{
    const int n_joints = workspace.n_joints;
    AffineMatrix4* current = workspace.at(depth);

    if (depth_remaining == 0) {
        aa_fk_update_prefix_from(current, robot, intervals, k_valid, n_joints);
        aa_fk_extract_from_prefix(
            current,
            robot.active_link_map(),
            n_active_links,
            n_joints,
            workspace.leaf.data());
        hull_endpoint_iaabbs(out_hull, workspace.leaf.data(), n_active_links * 2);
        return;
    }

    const int split_joint = schedule.resolve(depth, intervals);

    if (split_joint > k_valid) {
        aa_fk_extend_prefix(current, robot, intervals, k_valid, split_joint, n_joints);
    }

    AffineMatrix4* child = workspace.at(depth + 1);
    std::memcpy(child, current, static_cast<std::size_t>(split_joint + 1) * sizeof(AffineMatrix4));

    const double mid = (intervals[split_joint].lo + intervals[split_joint].hi) * 0.5;
    const double original_lo = intervals[split_joint].lo;
    const double original_hi = intervals[split_joint].hi;

    intervals[split_joint].hi = mid;
    hifk_aa_recurse(
        robot,
        intervals,
        schedule,
        workspace,
        depth + 1,
        split_joint,
        out_hull,
        n_active_links,
        depth_remaining - 1);
    intervals[split_joint].hi = original_hi;

    intervals[split_joint].lo = mid;
    hifk_aa_recurse(
        robot,
        intervals,
        schedule,
        workspace,
        depth + 1,
        split_joint,
        out_hull,
        n_active_links,
        depth_remaining - 1);
    intervals[split_joint].lo = original_lo;
}

void hifk_aa_bfs_adaptive(
    const Robot& robot,
    const std::vector<Interval>& intervals_in,
    const EndpointSourceConfig& config,
    int max_depth,
    float* out_hull,
    int n_active_links,
    double vol_ratio_thresh)
{
    const int iaabb_size = n_active_links * 2 * 6;

    struct Node {
        std::vector<Interval> intervals;
        std::vector<float> endpoint_iaabbs;
        double measure = 0.0;
        int depth = 0;

        bool operator<(const Node& other) const { return measure < other.measure; }
    };

    auto make_node = [&](const std::vector<Interval>& intervals, int depth) {
        Node node;
        node.intervals = intervals;
        node.depth = depth;
        node.endpoint_iaabbs.resize(static_cast<std::size_t>(iaabb_size));
        compute_endpoint_iaabb_aa_fk_raw(robot, node.intervals, node.endpoint_iaabbs.data());
        node.measure = endpoint_split_measure(node.endpoint_iaabbs, n_active_links);
        return node;
    };

    std::priority_queue<Node> queue;
    HifkDepthSplitSchedule schedule;
    schedule.reset(config, intervals_in, max_depth);
    queue.push(make_node(intervals_in, 0));

    while (!queue.empty()) {
        Node node = queue.top();
        queue.pop();

        if (node.depth >= max_depth) {
            hull_endpoint_iaabbs(out_hull, node.endpoint_iaabbs.data(), n_active_links * 2);
            continue;
        }

        const int split_joint = schedule.resolve(node.depth, node.intervals);

        const double mid = (node.intervals[split_joint].lo + node.intervals[split_joint].hi) * 0.5;
        const double original_lo = node.intervals[split_joint].lo;
        const double original_hi = node.intervals[split_joint].hi;

        node.intervals[split_joint].hi = mid;
        Node left = make_node(node.intervals, node.depth + 1);
        node.intervals[split_joint].hi = original_hi;
        node.intervals[split_joint].lo = mid;
        Node right = make_node(node.intervals, node.depth + 1);
        node.intervals[split_joint].lo = original_lo;

        std::vector<float> hull = left.endpoint_iaabbs;
        hull_endpoint_iaabbs(hull.data(), right.endpoint_iaabbs.data(), n_active_links * 2);
        const double hull_measure = endpoint_split_measure(hull, n_active_links);

        bool split = true;
        if (node.measure > 1e-15) {
            const double reduction = (node.measure - hull_measure) / node.measure;
            if (reduction < vol_ratio_thresh) {
                split = false;
            }
        }

        if (split) {
            queue.push(std::move(left));
            queue.push(std::move(right));
        } else {
            hull_endpoint_iaabbs(out_hull, node.endpoint_iaabbs.data(), n_active_links * 2);
        }
    }
}

}  // namespace

std::vector<int> aafk_volume_min_depth_schedule(
    const Robot& robot,
    const std::vector<Interval>& root_intervals,
    int max_depth)
{
    std::vector<int> schedule;
    const int effective_max_depth = std::max(0, max_depth);
    schedule.reserve(static_cast<std::size_t>(effective_max_depth));
    std::vector<Interval> intervals(root_intervals);
    for (int depth = 0; depth < effective_max_depth; ++depth) {
        const int dim = aafk_volume_min_dim(robot, intervals);
        if (dim < 0 || dim >= static_cast<int>(intervals.size())) {
            break;
        }
        schedule.push_back(dim);
        const double mid = 0.5 * (intervals[static_cast<std::size_t>(dim)].lo +
                                  intervals[static_cast<std::size_t>(dim)].hi);
        intervals[static_cast<std::size_t>(dim)].hi = mid;
    }
    return schedule;
}

EndpointIAABBResult compute_endpoint_iaabb_hifk_aa(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config)
{
    EndpointIAABBResult result;
    result.source = EndpointSource::HIFK;
    result.is_safe = true;
    result.safety_level = EndpointSafetyLevel::Certified;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(result.endpoint_iaabb_len());

    const int n_joints = robot.n_joints();
    if (n_joints <= 0) {
        return result;
    }

    const int effective_max_depth = std::max(0, config.hifk_max_depth);
    const int n_tf = n_joints + (robot.has_tool() ? 1 : 0);
    const int n_active_links = result.n_active_links;

    init_endpoints_inf(result.endpoint_iaabbs.data(), n_active_links);

    if (config.hifk_vol_ratio_thresh > 0.0) {
        hifk_aa_bfs_adaptive(
            robot,
            intervals,
            config,
            effective_max_depth,
            result.endpoint_iaabbs.data(),
            n_active_links,
            config.hifk_vol_ratio_thresh);
    } else {
        HifkDepthSplitSchedule schedule;
        schedule.reset(config, intervals, effective_max_depth);
        HifkAaWs workspace;
        workspace.n_joints = n_joints;
        workspace.n_tf = n_tf;
        workspace.prefix.resize(static_cast<std::size_t>(effective_max_depth + 1) * (n_tf + 1));
        workspace.leaf.resize(static_cast<std::size_t>(n_active_links) * 2 * 6);

        AffineMatrix4* root = workspace.at(0);
        AffineMatrix4 identity{};
        identity.m[0].ctr = 1.0;
        identity.m[5].ctr = 1.0;
        identity.m[10].ctr = 1.0;
        identity.m[15].ctr = 1.0;
        root[0] = identity;

        std::vector<Interval> working_intervals(intervals);
        hifk_aa_recurse(
            robot,
            working_intervals,
            schedule,
            workspace,
            0,
            0,
            result.endpoint_iaabbs.data(),
            n_active_links,
            effective_max_depth);
    }

    return result;
}

EndpointIAABBResult compute_endpoint_iaabb_hifk_aa(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int max_depth,
    int n_threads,
    double vol_ratio_thresh)
{
    EndpointSourceConfig config;
    config.source = EndpointSource::HIFK;
    config.hifk_max_depth = max_depth;
    config.hifk_n_threads = n_threads;
    config.hifk_vol_ratio_thresh = vol_ratio_thresh;
    return compute_endpoint_iaabb_hifk_aa(robot, intervals, config);
}

}  // namespace rbf