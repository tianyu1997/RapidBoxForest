#include <sbf/envelope/ifk_aa_source.h>

#include <sbf/core/aa_fk.h>
#include <sbf/envelope/dh_enumerate.h>
#include <sbf/envelope/support_hull.h>

#include <algorithm>
#include <array>
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
    AaFkPrefixState& state,
    bool* used_incremental)
{
    EndpointIAABBResult result;
    result.source = EndpointSource::IFK;
    result.is_safe = true;
    result.safety_level = EndpointSafetyLevel::Certified;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(result.endpoint_iaabb_len());

    if (robot.n_joints() <= 0) {
        state.valid = false;
        if (used_incremental != nullptr) {
            *used_incremental = false;
        }
        return result;
    }

    const bool reused_state = aa_fk_endpoint_incremental(
        robot, intervals, result.endpoint_iaabbs.data(), state);
    if (!reused_state) {
        aa_fk_endpoint_full(robot, intervals, result.endpoint_iaabbs.data(), state);
    }
    if (used_incremental != nullptr) {
        *used_incremental = reused_state;
    }
    return result;
}

namespace {

constexpr double kPlanarExtentEpsilon = 1e-12;

bool same_interval_list(
    const std::vector<Interval>& lhs,
    const std::vector<Interval>& rhs)
{
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t index = 0; index < lhs.size(); ++index) {
        if (lhs[index].lo != rhs[index].lo || lhs[index].hi != rhs[index].hi) {
            return false;
        }
    }
    return true;
}

bool hifk_supports_leaf_incremental(
    const EndpointSourceConfig& config,
    int effective_max_depth)
{
    if (config.hifk_vol_ratio_thresh > 0.0 ||
        config.hifk_min_split_width > 0.0 ||
        effective_max_depth < 0 ||
        effective_max_depth >= static_cast<int>(std::numeric_limits<std::size_t>::digits - 1)) {
        return false;
    }
    if (config.hifk_split_strategy == HifkSplitStrategy::FixedDepthSchedule &&
        config.hifk_depth_dimensions.empty()) {
        return false;
    }
    if (config.hifk_split_strategy == HifkSplitStrategy::WidestRoot &&
        config.hifk_root_intervals.empty()) {
        return false;
    }
    return true;
}

bool hifk_state_matches(
    const HifkAaState& state,
    const EndpointSourceConfig& config,
    int effective_max_depth,
    int n_joints)
{
    return state.valid &&
           state.n_joints == n_joints &&
           state.effective_max_depth == effective_max_depth &&
           state.depth_offset == config.hifk_depth_offset &&
           state.split_strategy == config.hifk_split_strategy &&
           state.depth_dimensions == config.hifk_depth_dimensions &&
           same_interval_list(state.root_intervals, config.hifk_root_intervals);
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

// --- Support-hull volume measure (seed/scene-independent) ----------------
//
// The certification-time envelope is a SupportHull: each active link is bounded
// by the convex hull swept between its proximal and distal endpoint AABBs
// (padded by the link radius). The schedule below selects split dimensions that
// minimise this hull volume, matching the envelope actually used at runtime
// rather than the looser sum of two disjoint endpoint boxes. The measure is a
// pure function of (robot, canonical intervals) so it never depends on a query
// seed or scene.

// Exact 2D convex hull area (monotone chain) of points projected onto two axes.
double convex_hull_area_2d(std::vector<std::array<double, 2>> pts) {
    std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) {
        return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
    });
    pts.erase(std::unique(pts.begin(), pts.end(), [](const auto& a, const auto& b) {
                  return a[0] == b[0] && a[1] == b[1];
              }),
              pts.end());
    const std::size_t n = pts.size();
    if (n < 3) {
        return 0.0;
    }
    const auto cross = [](const std::array<double, 2>& o,
                          const std::array<double, 2>& a,
                          const std::array<double, 2>& b) {
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    };
    std::vector<std::array<double, 2>> hull(2 * n);
    std::size_t k = 0;
    for (std::size_t i = 0; i < n; ++i) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0.0) {
            --k;
        }
        hull[k++] = pts[i];
    }
    const std::size_t lower = k + 1;
    for (std::size_t i = n; i-- > 0;) {
        while (k >= lower && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0.0) {
            --k;
        }
        hull[k++] = pts[i];
    }
    hull.resize(k - 1);
    double area2 = 0.0;
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const auto& p = hull[i];
        const auto& q = hull[(i + 1) % hull.size()];
        area2 += p[0] * q[1] - q[0] * p[1];
    }
    return std::abs(area2) * 0.5;
}

// Exact convex hull volume of a small 3D point set via incremental hull, with
// automatic dimension reduction when the set is flat (planar/linear robots).
double convex_hull_volume_3d(std::vector<std::array<double, 3>> pts) {
    constexpr double kEps = 1e-12;
    std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) {
        return a < b;
    });
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
    if (pts.size() < 4) {
        return 0.0;
    }

    std::array<double, 3> lo = pts.front();
    std::array<double, 3> hi = pts.front();
    for (const auto& p : pts) {
        for (int axis = 0; axis < 3; ++axis) {
            lo[axis] = std::min(lo[axis], p[axis]);
            hi[axis] = std::max(hi[axis], p[axis]);
        }
    }
    std::array<double, 3> ext{hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]};
    std::array<int, 3> wide_axes{};
    int wide_count = 0;
    for (int axis = 0; axis < 3; ++axis) {
        if (ext[axis] > kEps) {
            wide_axes[static_cast<std::size_t>(wide_count++)] = axis;
        }
    }
    if (wide_count <= 1) {
        return 0.0;  // degenerate line/point: contributes negligible volume.
    }
    if (wide_count == 2) {
        std::vector<std::array<double, 2>> projected;
        projected.reserve(pts.size());
        for (const auto& p : pts) {
            projected.push_back({p[wide_axes[0]], p[wide_axes[1]]});
        }
        return convex_hull_area_2d(std::move(projected));
    }

    const auto sub = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
        return std::array<double, 3>{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    };
    const auto cross = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
        return std::array<double, 3>{
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
    };
    const auto dot = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };

    const int n = static_cast<int>(pts.size());
    // Initial tetrahedron: spread across the four extreme directions.
    int i0 = 0;
    int i1 = 1;
    double best = -1.0;
    for (int i = 1; i < n; ++i) {
        const auto d = sub(pts[static_cast<std::size_t>(i)], pts[0]);
        const double len = dot(d, d);
        if (len > best) {
            best = len;
            i1 = i;
        }
    }
    int i2 = -1;
    best = -1.0;
    for (int i = 0; i < n; ++i) {
        const auto area_vec = cross(sub(pts[static_cast<std::size_t>(i1)], pts[static_cast<std::size_t>(i0)]),
                                    sub(pts[static_cast<std::size_t>(i)], pts[static_cast<std::size_t>(i0)]));
        const double area = dot(area_vec, area_vec);
        if (area > best) {
            best = area;
            i2 = i;
        }
    }
    if (i2 < 0) {
        return 0.0;
    }
    const auto base_normal = cross(sub(pts[static_cast<std::size_t>(i1)], pts[static_cast<std::size_t>(i0)]),
                                   sub(pts[static_cast<std::size_t>(i2)], pts[static_cast<std::size_t>(i0)]));
    int i3 = -1;
    best = kEps;
    for (int i = 0; i < n; ++i) {
        const double vol = std::abs(dot(base_normal, sub(pts[static_cast<std::size_t>(i)], pts[static_cast<std::size_t>(i0)])));
        if (vol > best) {
            best = vol;
            i3 = i;
        }
    }
    if (i3 < 0) {
        return 0.0;
    }

    struct Face {
        int a;
        int b;
        int c;
    };
    const std::array<double, 3> centroid{
        0.25 * (pts[static_cast<std::size_t>(i0)][0] + pts[static_cast<std::size_t>(i1)][0] +
                pts[static_cast<std::size_t>(i2)][0] + pts[static_cast<std::size_t>(i3)][0]),
        0.25 * (pts[static_cast<std::size_t>(i0)][1] + pts[static_cast<std::size_t>(i1)][1] +
                pts[static_cast<std::size_t>(i2)][1] + pts[static_cast<std::size_t>(i3)][1]),
        0.25 * (pts[static_cast<std::size_t>(i0)][2] + pts[static_cast<std::size_t>(i1)][2] +
                pts[static_cast<std::size_t>(i2)][2] + pts[static_cast<std::size_t>(i3)][2])};
    const auto make_outward = [&](int a, int b, int c) -> Face {
        const auto normal = cross(sub(pts[static_cast<std::size_t>(b)], pts[static_cast<std::size_t>(a)]),
                                  sub(pts[static_cast<std::size_t>(c)], pts[static_cast<std::size_t>(a)]));
        if (dot(normal, sub(centroid, pts[static_cast<std::size_t>(a)])) > 0.0) {
            return Face{a, c, b};
        }
        return Face{a, b, c};
    };

    std::vector<Face> faces{
        make_outward(i0, i1, i2),
        make_outward(i0, i1, i3),
        make_outward(i0, i2, i3),
        make_outward(i1, i2, i3)};

    const auto face_visible = [&](const Face& f, const std::array<double, 3>& q) {
        const auto normal = cross(sub(pts[static_cast<std::size_t>(f.b)], pts[static_cast<std::size_t>(f.a)]),
                                  sub(pts[static_cast<std::size_t>(f.c)], pts[static_cast<std::size_t>(f.a)]));
        return dot(normal, sub(q, pts[static_cast<std::size_t>(f.a)])) > kEps;
    };

    for (int i = 0; i < n; ++i) {
        if (i == i0 || i == i1 || i == i2 || i == i3) {
            continue;
        }
        const auto& q = pts[static_cast<std::size_t>(i)];
        std::vector<Face> visible;
        std::vector<Face> kept;
        for (const Face& f : faces) {
            if (face_visible(f, q)) {
                visible.push_back(f);
            } else {
                kept.push_back(f);
            }
        }
        if (visible.empty()) {
            continue;  // q inside current hull.
        }
        // Horizon: directed edges of visible faces whose reverse is not visible.
        std::vector<std::array<int, 2>> edges;
        const auto add_edge = [&](int u, int v) { edges.push_back({u, v}); };
        for (const Face& f : visible) {
            add_edge(f.a, f.b);
            add_edge(f.b, f.c);
            add_edge(f.c, f.a);
        }
        faces = std::move(kept);
        for (const auto& e : edges) {
            bool internal = false;
            for (const auto& other : edges) {
                if (other[0] == e[1] && other[1] == e[0]) {
                    internal = true;
                    break;
                }
            }
            if (!internal) {
                faces.push_back(Face{e[0], e[1], i});
            }
        }
    }

    const auto& ref = pts[static_cast<std::size_t>(i0)];
    double signed_volume = 0.0;
    for (const Face& f : faces) {
        signed_volume += dot(sub(pts[static_cast<std::size_t>(f.a)], ref),
                             cross(sub(pts[static_cast<std::size_t>(f.b)], ref),
                                   sub(pts[static_cast<std::size_t>(f.c)], ref)));
    }
    return std::abs(signed_volume) / 6.0;
}

// Convex hull volume of two axis-aligned boxes (proximal, distal endpoint
// AABBs) inflated by the link radius. Each box is [minx,miny,minz,maxx,maxy,maxz].
double support_hull_pair_volume(const float* prox, const float* dist, double radius) {
    std::vector<std::array<double, 3>> corners;
    corners.reserve(16);
    const float* boxes[2] = {prox, dist};
    for (const float* box : boxes) {
        const double xlo = static_cast<double>(box[0]) - radius;
        const double ylo = static_cast<double>(box[1]) - radius;
        const double zlo = static_cast<double>(box[2]) - radius;
        const double xhi = static_cast<double>(box[3]) + radius;
        const double yhi = static_cast<double>(box[4]) + radius;
        const double zhi = static_cast<double>(box[5]) + radius;
        for (int cx = 0; cx < 2; ++cx) {
            for (int cy = 0; cy < 2; ++cy) {
                for (int cz = 0; cz < 2; ++cz) {
                    corners.push_back({cx ? xhi : xlo, cy ? yhi : ylo, cz ? zhi : zlo});
                }
            }
        }
    }
    return convex_hull_volume_3d(std::move(corners));
}

double support_hull_split_measure(
    const std::vector<float>& iaabbs,
    int n_active_links,
    const double* link_radii) {
    double measure = 0.0;
    for (int link = 0; link < n_active_links; ++link) {
        const float* prox = iaabbs.data() + static_cast<std::size_t>(link * 2) * 6U;
        const float* dist = iaabbs.data() + static_cast<std::size_t>(link * 2 + 1) * 6U;
        const double radius = link_radii != nullptr ? link_radii[link] : 0.0;
        measure += support_hull_pair_volume(prox, dist, radius);
    }
    return measure;
}

// Selects which seed/scene-independent envelope volume the schedule minimises.
struct ScheduleMeasure {
    bool support_hull = false;
    const double* link_radii = nullptr;
};

double schedule_measure(
    const std::vector<float>& iaabbs,
    int n_active_links,
    const ScheduleMeasure& measure) {
    if (measure.support_hull) {
        return support_hull_split_measure(iaabbs, n_active_links, measure.link_radii);
    }
    return endpoint_split_measure(iaabbs, n_active_links);
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

struct AafkSampleNode {
    std::vector<Interval> intervals;
    double measure = 0.0;
};

double node_endpoint_measure(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_active_links,
    const ScheduleMeasure& measure)
{
    std::vector<float> endpoint_iaabbs(static_cast<std::size_t>(n_active_links) * 2 * 6);
    compute_endpoint_iaabb_aa_fk_raw(robot, intervals, endpoint_iaabbs.data());
    return schedule_measure(endpoint_iaabbs, n_active_links, measure);
}

AafkSampleNode make_aafk_sample_node(
    const Robot& robot,
    std::vector<Interval> intervals,
    int n_active_links,
    const ScheduleMeasure& measure)
{
    AafkSampleNode node;
    node.measure = node_endpoint_measure(robot, intervals, n_active_links, measure);
    node.intervals = std::move(intervals);
    return node;
}

double aafk_volume_min_split_score(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int joint,
    int n_active_links,
    const ScheduleMeasure& measure)
{
    if (joint < 0 || joint >= static_cast<int>(intervals.size())) {
        return std::numeric_limits<double>::infinity();
    }

    const double width = intervals[static_cast<std::size_t>(joint)].width();
    if (width <= 0.0) {
        return std::numeric_limits<double>::infinity();
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

    const double left_measure = schedule_measure(left_iaabbs, n_active_links, measure);
    const double right_measure = schedule_measure(right_iaabbs, n_active_links, measure);
    return std::max(left_measure, right_measure);
}

int widest_positive_width_dim(const std::vector<AafkSampleNode>& nodes) {
    if (nodes.empty()) {
        return -1;
    }

    const int n_joints = static_cast<int>(nodes.front().intervals.size());
    int best_dim = 0;
    double best_width = -1.0;
    for (int joint = 0; joint < n_joints; ++joint) {
        double aggregate_width = 0.0;
        for (const AafkSampleNode& node : nodes) {
            if (joint >= static_cast<int>(node.intervals.size())) {
                continue;
            }
            aggregate_width += std::max(0.0, node.intervals[static_cast<std::size_t>(joint)].width());
        }
        if (aggregate_width > best_width) {
            best_width = aggregate_width;
            best_dim = joint;
        }
    }
    return best_dim;
}

int resolve_aafk_schedule_dim(const std::vector<Interval>& intervals, int scheduled_dim) {
    if (scheduled_dim >= 0 &&
        scheduled_dim < static_cast<int>(intervals.size()) &&
        intervals[static_cast<std::size_t>(scheduled_dim)].width() > 0.0) {
        return scheduled_dim;
    }
    return widest_positive_width_dim(intervals);
}

int aafk_volume_min_dim(
    const Robot& robot,
    const std::vector<AafkSampleNode>& nodes,
    const ScheduleMeasure& measure)
{
    if (nodes.empty()) {
        return -1;
    }

    const int n_joints = static_cast<int>(nodes.front().intervals.size());
    const int n_active_links = robot.n_active_links();
    int best_dim = -1;
    int best_invalid_count = std::numeric_limits<int>::max();
    double best_value = std::numeric_limits<double>::infinity();

    for (int joint = 0; joint < n_joints; ++joint) {
        int invalid_count = 0;
        bool has_valid_split = false;
        double aggregate_value = 0.0;
        for (const AafkSampleNode& node : nodes) {
            const double split_score = aafk_volume_min_split_score(robot, node.intervals, joint, n_active_links, measure);
            if (std::isfinite(split_score)) {
                has_valid_split = true;
                aggregate_value += split_score;
            } else {
                ++invalid_count;
                aggregate_value += node.measure;
            }
        }
        if (!has_valid_split) {
            continue;
        }
        if (invalid_count < best_invalid_count ||
            (invalid_count == best_invalid_count && aggregate_value < best_value)) {
            best_invalid_count = invalid_count;
            best_value = aggregate_value;
            best_dim = joint;
        }
    }

    return best_dim >= 0 ? best_dim : widest_positive_width_dim(nodes);
}

std::vector<AafkSampleNode> downsample_aafk_nodes(
    std::vector<AafkSampleNode> nodes,
    int sample_nodes_per_depth)
{
    const int effective_budget = std::max(1, sample_nodes_per_depth);
    if (static_cast<int>(nodes.size()) <= effective_budget) {
        return nodes;
    }

    std::vector<AafkSampleNode> sampled;
    sampled.reserve(static_cast<std::size_t>(effective_budget));
    if (effective_budget == 1) {
        sampled.push_back(std::move(nodes.front()));
        return sampled;
    }

    const std::size_t last = nodes.size() - 1;
    const std::size_t denom = static_cast<std::size_t>(effective_budget - 1);
    for (int index = 0; index < effective_budget; ++index) {
        const std::size_t offset = (static_cast<std::size_t>(index) * last) / denom;
        sampled.push_back(std::move(nodes[offset]));
    }
    return sampled;
}

std::vector<AafkSampleNode> advance_aafk_sampled_nodes(
    const Robot& robot,
    const std::vector<AafkSampleNode>& nodes,
    int scheduled_dim,
    int sample_nodes_per_depth,
    const ScheduleMeasure& measure)
{
    if (nodes.empty()) {
        return {};
    }

    const int n_active_links = robot.n_active_links();
    std::vector<AafkSampleNode> next;
    next.reserve(nodes.size() * 2);
    for (const AafkSampleNode& node : nodes) {
        const int resolved_dim = resolve_aafk_schedule_dim(node.intervals, scheduled_dim);
        if (resolved_dim < 0 || resolved_dim >= static_cast<int>(node.intervals.size())) {
            continue;
        }

        const double mid = 0.5 * (node.intervals[static_cast<std::size_t>(resolved_dim)].lo +
                                  node.intervals[static_cast<std::size_t>(resolved_dim)].hi);
        std::vector<Interval> left(node.intervals);
        std::vector<Interval> right(node.intervals);
        left[static_cast<std::size_t>(resolved_dim)].hi = mid;
        right[static_cast<std::size_t>(resolved_dim)].lo = mid;
        next.push_back(make_aafk_sample_node(robot, std::move(left), n_active_links, measure));
        next.push_back(make_aafk_sample_node(robot, std::move(right), n_active_links, measure));
    }
    return downsample_aafk_nodes(std::move(next), sample_nodes_per_depth);
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
    const std::vector<Interval>& intervals,
    const ScheduleMeasure& measure)
{
    const int n_joints = static_cast<int>(intervals.size());
    const int n_active_links = robot.n_active_links();
    int best_dim = -1;
    double best_value = std::numeric_limits<double>::infinity();

    for (int joint = 0; joint < n_joints; ++joint) {
        const double minimax = aafk_volume_min_split_score(robot, intervals, joint, n_active_links, measure);
        if (std::isfinite(minimax) && minimax < best_value) {
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
    HifkAaState* state,
    bool allow_leaf_incremental,
    int& leaf_index,
    bool& used_leaf_incremental,
    int depth,
    int k_valid,
    float* out_hull,
    int n_active_links,
    int depth_remaining)
{
    const int n_joints = workspace.n_joints;
    const int n_tf = workspace.n_tf;
    AffineMatrix4* current = workspace.at(depth);

    if (depth_remaining == 0) {
        AaFkPrefixState* leaf_state = nullptr;
        if (state != nullptr &&
            leaf_index >= 0 &&
            leaf_index < static_cast<int>(state->leaf_states.size())) {
            leaf_state = &state->leaf_states[static_cast<std::size_t>(leaf_index)];
        }

        const bool reused_leaf =
            allow_leaf_incremental &&
            leaf_state != nullptr &&
            aa_fk_endpoint_incremental(robot, intervals, workspace.leaf.data(), *leaf_state);

        if (!reused_leaf) {
            aa_fk_update_prefix_from(current, robot, intervals, k_valid, n_joints);
            aa_fk_extract_from_prefix(
                current,
                robot.active_link_map(),
                n_active_links,
                n_joints,
                workspace.leaf.data());
            if (leaf_state != nullptr) {
                std::memcpy(leaf_state->prefix,
                            current,
                            static_cast<std::size_t>(n_tf + 1) * sizeof(AffineMatrix4));
                leaf_state->intervals = intervals;
                leaf_state->n_joints = n_joints;
                leaf_state->valid = true;
            }
        } else {
            used_leaf_incremental = true;
        }

        leaf_index += 1;
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
        state,
        allow_leaf_incremental,
        leaf_index,
        used_leaf_incremental,
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
        state,
        allow_leaf_incremental,
        leaf_index,
        used_leaf_incremental,
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

namespace {

std::vector<int> volume_min_depth_schedule_impl(
    const Robot& robot,
    const std::vector<Interval>& root_intervals,
    int max_depth,
    int sample_nodes_per_depth,
    const ScheduleMeasure& measure)
{
    std::vector<int> schedule;
    const int effective_max_depth = std::max(0, max_depth);
    schedule.reserve(static_cast<std::size_t>(effective_max_depth));

    if (sample_nodes_per_depth <= 1) {
        std::vector<Interval> intervals(root_intervals);
        for (int depth = 0; depth < effective_max_depth; ++depth) {
            const int dim = aafk_volume_min_dim(robot, intervals, measure);
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

    std::vector<AafkSampleNode> nodes;
    nodes.reserve(static_cast<std::size_t>(std::max(1, sample_nodes_per_depth)));
    nodes.push_back(make_aafk_sample_node(robot, root_intervals, robot.n_active_links(), measure));
    std::vector<Interval> intervals(root_intervals);
    for (int depth = 0; depth < effective_max_depth; ++depth) {
        const int dim = aafk_volume_min_dim(robot, nodes, measure);
        if (dim < 0 || dim >= static_cast<int>(root_intervals.size())) {
            break;
        }
        schedule.push_back(dim);
        nodes = advance_aafk_sampled_nodes(robot, nodes, dim, sample_nodes_per_depth, measure);
        if (nodes.empty()) {
            break;
        }
    }
    return schedule;
}

}  // namespace

std::vector<int> aafk_volume_min_depth_schedule(
    const Robot& robot,
    const std::vector<Interval>& root_intervals,
    int max_depth,
    int sample_nodes_per_depth)
{
    return volume_min_depth_schedule_impl(
        robot, root_intervals, max_depth, sample_nodes_per_depth, ScheduleMeasure{});
}

std::vector<int> support_hull_volume_min_depth_schedule(
    const Robot& robot,
    const std::vector<Interval>& root_intervals,
    int max_depth,
    int sample_nodes_per_depth)
{
    ScheduleMeasure measure;
    measure.support_hull = true;
    measure.link_radii = robot.active_link_radii();
    return volume_min_depth_schedule_impl(
        robot, root_intervals, max_depth, sample_nodes_per_depth, measure);
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
        int leaf_index = 0;
        bool used_leaf_incremental = false;
        hifk_aa_recurse(
            robot,
            working_intervals,
            schedule,
            workspace,
            nullptr,
            false,
            leaf_index,
            used_leaf_incremental,
            0,
            0,
            result.endpoint_iaabbs.data(),
            n_active_links,
            effective_max_depth);
    }

    return result;
}

EndpointIAABBResult compute_endpoint_iaabb_hifk_aa_stateful(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config,
    HifkAaState& state,
    bool* used_incremental)
{
    const int n_joints = robot.n_joints();
    const int effective_max_depth = std::max(0, config.hifk_max_depth);
    if (used_incremental != nullptr) {
        *used_incremental = false;
    }
    if (n_joints <= 0) {
        state.valid = false;
        return compute_endpoint_iaabb_hifk_aa(robot, intervals, config);
    }
    if (!hifk_supports_leaf_incremental(config, effective_max_depth)) {
        state.valid = false;
        return compute_endpoint_iaabb_hifk_aa(robot, intervals, config);
    }

    EndpointIAABBResult result;
    result.source = EndpointSource::HIFK;
    result.is_safe = true;
    result.safety_level = EndpointSafetyLevel::Certified;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(result.endpoint_iaabb_len());

    const int n_tf = n_joints + (robot.has_tool() ? 1 : 0);
    const int n_active_links = result.n_active_links;
    const std::size_t leaf_count = std::size_t{1} << effective_max_depth;

    init_endpoints_inf(result.endpoint_iaabbs.data(), n_active_links);

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

    state.leaf_states.resize(leaf_count);
    const bool allow_leaf_incremental = hifk_state_matches(state, config, effective_max_depth, n_joints);
    int leaf_index = 0;
    bool used_leaf_incremental = false;
    std::vector<Interval> working_intervals(intervals);
    hifk_aa_recurse(
        robot,
        working_intervals,
        schedule,
        workspace,
        &state,
        allow_leaf_incremental,
        leaf_index,
        used_leaf_incremental,
        0,
        0,
        result.endpoint_iaabbs.data(),
        n_active_links,
        effective_max_depth);

    state.depth_dimensions = config.hifk_depth_dimensions;
    state.root_intervals = config.hifk_root_intervals;
    state.split_strategy = config.hifk_split_strategy;
    state.effective_max_depth = effective_max_depth;
    state.depth_offset = config.hifk_depth_offset;
    state.n_joints = n_joints;
    state.valid = true;
    if (used_incremental != nullptr) {
        *used_incremental = used_leaf_incremental;
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