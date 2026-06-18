#pragma once
/// @file envelope_collision.h
/// @brief Conservative collision checks for link envelopes against AABB obstacles.

#include <sbf/core/types.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/kdop.h>
#include <sbf/envelope/support_hull.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace rbf {

enum class CollisionResultKind : uint8_t {
    DefinitelyFree = 0,
    MaybeColliding = 1,
};

enum class EnvelopeCollisionMode : uint8_t {
    Auto = 0,
    LinkAABB = 1,
    KDOP = 2,
    GJK = 3,
    KDOPThenGJK = 4,
};

enum class EnvelopeCollisionBlockerStage : uint8_t {
    None = 0,
    InvalidEnvelope = 1,
    LinkAABB = 2,
    KDOP = 3,
    GJK = 4,
};

struct EnvelopeCollisionBlocker {
    int active_link_index = -1;
    int obstacle_index = -1;
    int stage = static_cast<int>(EnvelopeCollisionBlockerStage::None);
    double margin = 0.0;
    double overlap_depth = 0.0;
    double overlap_volume_ratio = 0.0;
};

struct EnvelopeCollisionOptions {
    EnvelopeCollisionMode mode = EnvelopeCollisionMode::Auto;
    bool use_link_aabb_broadphase = true;
    bool skip_aabb_broadphase = false;
    bool direct_support_hull_collision = false;
    bool count_all_pairs = false;
    double safety_epsilon = 0.0;
    double overlap_tolerance = 0.0;
};

struct EnvelopeCollisionStats {
    int64_t envelope_aabb_tests = 0;
    int64_t envelope_aabb_rejects = 0;
    int64_t link_union_aabb_tests = 0;
    int64_t link_union_aabb_rejects = 0;
    int64_t link_aabb_tests = 0;
    int64_t link_aabb_rejects = 0;
    int64_t kdop_tests = 0;
    int64_t kdop_rejects = 0;
    int64_t kdop_axes_tested = 0;
    int64_t gjk_tests = 0;
    int64_t gjk_rejects = 0;
    int64_t gjk_iterations = 0;
    int64_t maybe_pairs = 0;
    int64_t overlap_tolerance_rejects = 0;
    double maybe_pair_overlap_depth_sum = 0.0;
    double maybe_pair_overlap_depth_max = 0.0;
    double maybe_pair_overlap_volume_sum = 0.0;
    double envelope_volume_sum = 0.0;
    double maybe_pair_overlap_volume_ratio_max = 0.0;
    int dominant_blocker_active_link_index = -1;
    int dominant_blocker_obstacle_index = -1;
    int dominant_blocker_stage = static_cast<int>(EnvelopeCollisionBlockerStage::None);
    double dominant_blocker_overlap_depth = 0.0;
    double dominant_blocker_overlap_volume_ratio = 0.0;
    double dominant_blocker_margin = 0.0;
    std::vector<EnvelopeCollisionBlocker> blockers;
};

namespace detail {

inline constexpr float kCollisionEps = 1e-6f;

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

inline Vec3 make_vec3(float x, float y, float z) {
    return {x, y, z};
}

inline Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 operator-(const Vec3& v) {
    return {-v.x, -v.y, -v.z};
}

inline Vec3 operator*(const Vec3& v, float scale) {
    return {v.x * scale, v.y * scale, v.z * scale};
}

inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

inline float norm_sq(const Vec3& v) {
    return dot(v, v);
}

inline bool same_direction(const Vec3& direction, const Vec3& toward) {
    return dot(direction, toward) > kCollisionEps;
}

inline Vec3 fallback_perpendicular(const Vec3& direction) {
    const Vec3 axis = std::abs(direction.x) < 0.9f
        ? make_vec3(1.0f, 0.0f, 0.0f)
        : make_vec3(0.0f, 1.0f, 0.0f);
    const Vec3 perp = cross(direction, axis);
    return norm_sq(perp) > kCollisionEps ? perp : make_vec3(0.0f, 0.0f, 1.0f);
}

inline Vec3 reject_from_line(const Vec3& line, const Vec3& toward_origin) {
    const Vec3 perp = cross(cross(line, toward_origin), line);
    return norm_sq(perp) > kCollisionEps ? perp : fallback_perpendicular(line);
}

struct Simplex {
    std::array<Vec3, 4> points{};
    int size = 0;

    void push_front(const Vec3& point) {
        for (int idx = std::min(size, 3); idx > 0; --idx) {
            points[static_cast<std::size_t>(idx)] = points[static_cast<std::size_t>(idx - 1)];
        }
        points[0] = point;
        size = std::min(size + 1, 4);
    }
};

inline bool aabb_overlap_padded(const float* a, const float* b, float pad) {
    return a[0] - pad <= b[3] && b[0] <= a[3] + pad &&
           a[1] - pad <= b[4] && b[1] <= a[4] + pad &&
           a[2] - pad <= b[5] && b[2] <= a[5] + pad;
}

inline float aabb_overlap_depth_padded(const float* a, const float* b, float pad) {
    if (!aabb_overlap_padded(a, b, pad)) {
        return 0.0f;
    }
    const float ax0 = a[0] - pad;
    const float ay0 = a[1] - pad;
    const float az0 = a[2] - pad;
    const float ax1 = a[3] + pad;
    const float ay1 = a[4] + pad;
    const float az1 = a[5] + pad;
    const float dx = std::min(ax1 - b[0], b[3] - ax0);
    const float dy = std::min(ay1 - b[1], b[4] - ay0);
    const float dz = std::min(az1 - b[2], b[5] - az0);
    return std::max(0.0f, std::min({dx, dy, dz}));
}

inline float aabb_volume_padded(const float* a, float pad) {
    const float ax0 = a[0] - pad;
    const float ay0 = a[1] - pad;
    const float az0 = a[2] - pad;
    const float ax1 = a[3] + pad;
    const float ay1 = a[4] + pad;
    const float az1 = a[5] + pad;
    return std::max((ax1 - ax0) * (ay1 - ay0) * (az1 - az0), kCollisionEps);
}

inline float aabb_overlap_volume_padded(const float* a, const float* b, float pad) {
    if (!aabb_overlap_padded(a, b, pad)) {
        return 0.0f;
    }
    const float ax0 = a[0] - pad;
    const float ay0 = a[1] - pad;
    const float az0 = a[2] - pad;
    const float ax1 = a[3] + pad;
    const float ay1 = a[4] + pad;
    const float az1 = a[5] + pad;
    const float ix = std::max(0.0f, std::min(ax1, b[3]) - std::max(ax0, b[0]));
    const float iy = std::max(0.0f, std::min(ay1, b[4]) - std::max(ay0, b[1]));
    const float iz = std::max(0.0f, std::min(az1, b[5]) - std::max(az0, b[2]));
    return ix * iy * iz;
}

inline float aabb_overlap_volume_ratio_padded(const float* a, const float* b, float pad) {
    const float overlap_volume = aabb_overlap_volume_padded(a, b, pad);
    const float volume = aabb_volume_padded(a, pad);
    return std::max(0.0f, std::min(1.0f, overlap_volume / volume));
}

inline double point_aabb_distance_sq_at_t(const double origin[3],
                                          const double dir[3],
                                          const float* obstacle,
                                          double t) {
    double dist_sq = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const double lo = static_cast<double>(obstacle[axis]);
        const double hi = static_cast<double>(obstacle[axis + 3]);
        const double value = origin[axis] + t * dir[axis];
        if (value < lo) {
            const double delta = lo - value;
            dist_sq += delta * delta;
        } else if (value > hi) {
            const double delta = value - hi;
            dist_sq += delta * delta;
        }
    }
    return dist_sq;
}

inline double segment_aabb_distance_sq(const double origin[3],
                                       const double dir[3],
                                       const float* obstacle) {
    std::array<double, 8> breaks{};
    int n_breaks = 0;
    auto append_break = [&](double value) {
        if (value >= 0.0 && value <= 1.0 && n_breaks < static_cast<int>(breaks.size())) {
            breaks[static_cast<std::size_t>(n_breaks++)] = value;
        }
    };
    append_break(0.0);
    append_break(1.0);
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(dir[axis]) < 1e-15) {
            continue;
        }
        const double inv = 1.0 / dir[axis];
        const double t_lo = (static_cast<double>(obstacle[axis]) - origin[axis]) * inv;
        const double t_hi = (static_cast<double>(obstacle[axis + 3]) - origin[axis]) * inv;
        if (t_lo > 0.0 && t_lo < 1.0) append_break(t_lo);
        if (t_hi > 0.0 && t_hi < 1.0) append_break(t_hi);
    }
    std::sort(breaks.begin(), breaks.begin() + n_breaks);
    int unique_count = 0;
    for (int idx = 0; idx < n_breaks; ++idx) {
        if (unique_count == 0 ||
            std::abs(breaks[static_cast<std::size_t>(idx)] -
                     breaks[static_cast<std::size_t>(unique_count - 1)]) > 1e-14) {
            breaks[static_cast<std::size_t>(unique_count++)] = breaks[static_cast<std::size_t>(idx)];
        }
    }

    double best = std::numeric_limits<double>::infinity();
    auto eval = [&](double t) {
        best = std::min(best, point_aabb_distance_sq_at_t(origin, dir, obstacle, t));
    };
    for (int idx = 0; idx < unique_count; ++idx) {
        eval(breaks[static_cast<std::size_t>(idx)]);
    }
    for (int idx = 0; idx + 1 < unique_count; ++idx) {
        const double left = breaks[static_cast<std::size_t>(idx)];
        const double right = breaks[static_cast<std::size_t>(idx + 1)];
        if (right - left <= 1e-14) {
            continue;
        }
        const double mid = 0.5 * (left + right);
        bool inside_all_axes = true;
        double numerator = 0.0;
        double denominator = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            const double lo = static_cast<double>(obstacle[axis]);
            const double hi = static_cast<double>(obstacle[axis + 3]);
            const double value = origin[axis] + mid * dir[axis];
            double bound = 0.0;
            if (value < lo) {
                bound = lo;
            } else if (value > hi) {
                bound = hi;
            } else {
                continue;
            }
            inside_all_axes = false;
            numerator += dir[axis] * (origin[axis] - bound);
            denominator += dir[axis] * dir[axis];
        }
        if (inside_all_axes) {
            return 0.0;
        }
        if (denominator > 0.0) {
            const double t_star = -numerator / denominator;
            if (t_star >= left && t_star <= right) {
                eval(t_star);
            }
        }
    }
    return best;
}

inline void include_aabb(float dst[6], const float* src) {
    dst[0] = std::min(dst[0], src[0]);
    dst[1] = std::min(dst[1], src[1]);
    dst[2] = std::min(dst[2], src[2]);
    dst[3] = std::max(dst[3], src[3]);
    dst[4] = std::max(dst[4], src[4]);
    dst[5] = std::max(dst[5], src[5]);
}

inline float link_radius_for_box(const LinkEnvelope& envelope, int box_index) {
    if (envelope.active_link_radii.empty()) return 0.0f;
    const int n_sub = std::max(1, envelope.n_subdivisions);
    const int link_index = std::clamp(box_index / n_sub, 0, envelope.n_active_links - 1);
    return static_cast<float>(envelope.active_link_radii[static_cast<std::size_t>(link_index)]);
}

inline Vec3 obstacle_center(const float* obstacle) {
    return make_vec3(
        0.5f * (obstacle[0] + obstacle[3]),
        0.5f * (obstacle[1] + obstacle[4]),
        0.5f * (obstacle[2] + obstacle[5]));
}

inline Vec3 box_center(const float* box) {
    return make_vec3(
        0.5f * (box[0] + box[3]),
        0.5f * (box[1] + box[4]),
        0.5f * (box[2] + box[5]));
}

inline Vec3 support_aabb(const float* box, const Vec3& dir, float xyz_pad = 0.0f) {
    return make_vec3(
        dir.x >= 0.0f ? box[3] + xyz_pad : box[0] - xyz_pad,
        dir.y >= 0.0f ? box[4] + xyz_pad : box[1] - xyz_pad,
        dir.z >= 0.0f ? box[5] + xyz_pad : box[2] - xyz_pad);
}

inline Vec3 support_support_hull(const float* record, const Vec3& dir) {
    const Vec3 prox = support_aabb(record, dir);
    const Vec3 dist = support_aabb(record + 6, dir);
    return dot(prox, dir) >= dot(dist, dir) ? prox : dist;
}

inline Vec3 support_minkowski_support_hull_vs_obstacle(
    const float* record,
    const float* obstacle,
    float xyz_pad,
    const Vec3& dir)
{
    return support_support_hull(record, dir) - support_aabb(obstacle, -dir, xyz_pad);
}

inline bool support_hull_capsule_bound_separates_obstacle(
    const float* record,
    const float* obstacle,
    float xyz_pad)
{
    double origin[3];
    double target[3];
    double bound_radius_sq = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const double p_lo = static_cast<double>(record[axis]);
        const double p_hi = static_cast<double>(record[axis + 3]);
        const double d_lo = static_cast<double>(record[axis + 6]);
        const double d_hi = static_cast<double>(record[axis + 9]);
        origin[axis] = 0.5 * (p_lo + p_hi);
        target[axis] = 0.5 * (d_lo + d_hi);
        const double p_half = 0.5 * std::max(0.0, p_hi - p_lo);
        const double d_half = 0.5 * std::max(0.0, d_hi - d_lo);
        bound_radius_sq += std::max(p_half, d_half) * std::max(p_half, d_half);
    }
    const double dir[3] = {
        target[0] - origin[0],
        target[1] - origin[1],
        target[2] - origin[2],
    };
    const double distance_sq = segment_aabb_distance_sq(origin, dir, obstacle);
    const double effective_radius = static_cast<double>(std::max(0.0f, xyz_pad)) +
        std::sqrt(bound_radius_sq);
    return distance_sq > effective_radius * effective_radius + 1e-14;
}

inline bool handle_line(Simplex& simplex, Vec3& direction) {
    const Vec3 a = simplex.points[0];
    const Vec3 b = simplex.points[1];
    const Vec3 ao = -a;
    const Vec3 ab = b - a;
    if (same_direction(ab, ao)) {
        direction = reject_from_line(ab, ao);
    } else {
        simplex.points[0] = a;
        simplex.size = 1;
        direction = norm_sq(ao) > kCollisionEps ? ao : make_vec3(1.0f, 0.0f, 0.0f);
    }
    return false;
}

inline bool handle_triangle(Simplex& simplex, Vec3& direction) {
    const Vec3 a = simplex.points[0];
    const Vec3 b = simplex.points[1];
    const Vec3 c = simplex.points[2];
    const Vec3 ao = -a;
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 abc = cross(ab, ac);

    const Vec3 ac_perp = cross(abc, ac);
    if (same_direction(ac_perp, ao)) {
        if (same_direction(ac, ao)) {
            simplex.points[1] = c;
            simplex.size = 2;
            direction = reject_from_line(ac, ao);
            return false;
        }
        simplex.points[1] = b;
        simplex.size = 2;
        return handle_line(simplex, direction);
    }

    const Vec3 ab_perp = cross(ab, abc);
    if (same_direction(ab_perp, ao)) {
        simplex.points[1] = b;
        simplex.size = 2;
        return handle_line(simplex, direction);
    }

    if (same_direction(abc, ao)) {
        direction = abc;
    } else {
        simplex.points[1] = c;
        simplex.points[2] = b;
        direction = -abc;
    }
    if (norm_sq(direction) <= kCollisionEps) {
        direction = fallback_perpendicular(ab);
    }
    return false;
}

inline bool handle_tetrahedron(Simplex& simplex, Vec3& direction) {
    const Vec3 a = simplex.points[0];
    const Vec3 b = simplex.points[1];
    const Vec3 c = simplex.points[2];
    const Vec3 d = simplex.points[3];
    const Vec3 ao = -a;
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ad = d - a;

    const Vec3 abc = cross(ab, ac);
    if (same_direction(abc, ao)) {
        simplex.points[1] = b;
        simplex.points[2] = c;
        simplex.size = 3;
        direction = abc;
        return false;
    }

    const Vec3 acd = cross(ac, ad);
    if (same_direction(acd, ao)) {
        simplex.points[1] = c;
        simplex.points[2] = d;
        simplex.size = 3;
        direction = acd;
        return false;
    }

    const Vec3 adb = cross(ad, ab);
    if (same_direction(adb, ao)) {
        simplex.points[1] = d;
        simplex.points[2] = b;
        simplex.size = 3;
        direction = adb;
        return false;
    }

    return true;
}

inline bool update_simplex(Simplex& simplex, Vec3& direction) {
    switch (simplex.size) {
        case 2:
            return handle_line(simplex, direction);
        case 3:
            return handle_triangle(simplex, direction);
        case 4:
            return handle_tetrahedron(simplex, direction);
        default:
            direction = -simplex.points[0];
            if (norm_sq(direction) <= kCollisionEps) {
                direction = make_vec3(1.0f, 0.0f, 0.0f);
            }
            return false;
    }
}

inline bool has_kdop_payload(const LinkEnvelope& envelope, int n_boxes) {
    return envelope.kdop_n_axes > 0 &&
        static_cast<int>(envelope.kdop_intervals.size()) >= n_boxes * envelope.kdop_n_axes * 2;
}

inline bool has_support_hull_payload(const LinkEnvelope& envelope, int n_boxes) {
    return static_cast<int>(envelope.support_hulls.size()) >= n_boxes * kSupportHullRecordSize;
}

inline EnvelopeCollisionMode resolve_collision_mode(
    const LinkEnvelope& envelope,
    EnvelopeCollisionMode requested,
    int n_boxes)
{
    if (requested != EnvelopeCollisionMode::Auto) {
        return requested;
    }
    if (envelope.type == EnvelopeType::KDOP && has_kdop_payload(envelope, n_boxes)) {
        return EnvelopeCollisionMode::KDOP;
    }
    if (envelope.type == EnvelopeType::SupportHull && has_support_hull_payload(envelope, n_boxes)) {
        return has_kdop_payload(envelope, n_boxes)
            ? EnvelopeCollisionMode::KDOPThenGJK
            : EnvelopeCollisionMode::GJK;
    }
    return EnvelopeCollisionMode::LinkAABB;
}

inline void project_obstacle_on_axis(
    const float* obstacle,
    const Vec3& axis,
    float interval_pad,
    float& out_lo,
    float& out_hi)
{
    const Vec3 center = obstacle_center(obstacle);
    const Vec3 half = make_vec3(
        0.5f * (obstacle[3] - obstacle[0]),
        0.5f * (obstacle[4] - obstacle[1]),
        0.5f * (obstacle[5] - obstacle[2]));
    const float projected_center = dot(center, axis);
    const float projected_radius =
        std::abs(axis.x) * half.x + std::abs(axis.y) * half.y + std::abs(axis.z) * half.z + interval_pad;
    out_lo = projected_center - projected_radius;
    out_hi = projected_center + projected_radius;
}

inline bool kdop_separates_obstacle(
    const LinkEnvelope& envelope,
    int box_index,
    const float* obstacle,
    float interval_pad,
    EnvelopeCollisionStats& stats)
{
    const int n_boxes = envelope.n_active_links * std::max(1, envelope.n_subdivisions);
    if (!has_kdop_payload(envelope, n_boxes)) {
        return false;
    }
    const int n_axes = envelope.kdop_n_axes;
    const auto axes = kdop_axis_table(envelope.kdop_direction_set);
    const float* intervals = envelope.kdop_intervals.data() +
        static_cast<std::size_t>(box_index * n_axes * 2);
    stats.kdop_tests += 1;
    stats.kdop_axes_tested += n_axes;
    for (int axis_index = 0; axis_index < n_axes; ++axis_index) {
        const Vec3 axis = make_vec3(
            axes[static_cast<std::size_t>(axis_index * 3 + 0)],
            axes[static_cast<std::size_t>(axis_index * 3 + 1)],
            axes[static_cast<std::size_t>(axis_index * 3 + 2)]);
        float obstacle_lo = 0.0f;
        float obstacle_hi = 0.0f;
        project_obstacle_on_axis(obstacle, axis, interval_pad, obstacle_lo, obstacle_hi);
        const float envelope_lo = intervals[static_cast<std::size_t>(axis_index * 2)];
        const float envelope_hi = intervals[static_cast<std::size_t>(axis_index * 2 + 1)];
        if (envelope_hi < obstacle_lo || obstacle_hi < envelope_lo) {
            stats.kdop_rejects += 1;
            return true;
        }
    }
    return false;
}

inline bool support_hull_separates_obstacle(
    const LinkEnvelope& envelope,
    int box_index,
    const float* obstacle,
    float xyz_pad,
    EnvelopeCollisionStats& stats)
{
    const int n_boxes = envelope.n_active_links * std::max(1, envelope.n_subdivisions);
    if (!has_support_hull_payload(envelope, n_boxes)) {
        return false;
    }

    const float* record = envelope.support_hulls.data() +
        static_cast<std::size_t>(box_index * kSupportHullRecordSize);
    if (support_hull_capsule_bound_separates_obstacle(record, obstacle, xyz_pad)) {
        stats.gjk_tests += 1;
        stats.gjk_rejects += 1;
        return true;
    }
    Vec3 direction = obstacle_center(obstacle) -
        (box_center(record) + box_center(record + 6)) * 0.5f;
    if (norm_sq(direction) <= kCollisionEps) {
        direction = make_vec3(1.0f, 0.0f, 0.0f);
    }

    Simplex simplex;
    simplex.push_front(support_minkowski_support_hull_vs_obstacle(record, obstacle, xyz_pad, direction));
    direction = -simplex.points[0];
    stats.gjk_tests += 1;

    if (norm_sq(direction) <= kCollisionEps) {
        return false;
    }

    constexpr int kMaxIterations = 32;
    for (int iter = 0; iter < kMaxIterations; ++iter) {
        stats.gjk_iterations += 1;
        const Vec3 support = support_minkowski_support_hull_vs_obstacle(record, obstacle, xyz_pad, direction);
        if (dot(support, direction) < 0.0f) {
            stats.gjk_rejects += 1;
            return true;
        }
        simplex.push_front(support);
        if (update_simplex(simplex, direction)) {
            return false;
        }
    }

    return false;
}

}  // namespace detail

/// Conservative envelope-vs-AABB collision with shared AABB broadphase and
/// representation-aware narrow phases.
///
/// LinkIAABB keeps the original padded AABB test. KDOP uses interval
/// projections on the stored k-DOP axes. SupportHull uses a convex support-map
/// GJK check against an expanded obstacle AABB; when both KDOP and SupportHull
/// payloads are present, KDOP runs as a cheap prefilter before GJK.
inline CollisionResultKind collide_envelope_aabbs(
    const LinkEnvelope& envelope,
    const Obstacle* obstacles,
    int n_obstacles,
    const EnvelopeCollisionOptions& options = {},
    EnvelopeCollisionStats* stats = nullptr)
{
    EnvelopeCollisionStats local_stats;
    EnvelopeCollisionStats& s = stats ? *stats : local_stats;

    if (n_obstacles <= 0 || obstacles == nullptr) {
        return CollisionResultKind::DefinitelyFree;
    }
    const int n_sub = std::max(1, envelope.n_subdivisions);
    const int n_boxes = envelope.n_active_links * n_sub;
    if (n_boxes <= 0 || static_cast<int>(envelope.link_iaabbs.size()) < n_boxes * 6) {
        s.maybe_pairs += static_cast<int64_t>(std::max(0, n_obstacles));
        s.dominant_blocker_stage = static_cast<int>(EnvelopeCollisionBlockerStage::InvalidEnvelope);
        return CollisionResultKind::MaybeColliding;
    }
    EnvelopeCollisionMode resolved_mode = detail::resolve_collision_mode(
        envelope,
        options.mode,
        n_boxes);
    const bool direct_support_hull =
        options.direct_support_hull_collision &&
        envelope.type == EnvelopeType::SupportHull &&
        detail::has_support_hull_payload(envelope, n_boxes);
    const bool skip_aabb_broadphase =
        (direct_support_hull || options.skip_aabb_broadphase) &&
        envelope.type == EnvelopeType::SupportHull &&
        detail::has_support_hull_payload(envelope, n_boxes);
    if (direct_support_hull) {
        resolved_mode = EnvelopeCollisionMode::GJK;
    }

    auto stage_for_maybe_pair = [&]() {
        switch (resolved_mode) {
            case EnvelopeCollisionMode::KDOP:
                return EnvelopeCollisionBlockerStage::KDOP;
            case EnvelopeCollisionMode::GJK:
            case EnvelopeCollisionMode::KDOPThenGJK:
                return EnvelopeCollisionBlockerStage::GJK;
            case EnvelopeCollisionMode::Auto:
            case EnvelopeCollisionMode::LinkAABB:
            default:
                return EnvelopeCollisionBlockerStage::LinkAABB;
        }
    };

    s.envelope_volume_sum = 0.0;
    for (int box = 0; box < n_boxes; ++box) {
        const float* link_box = envelope.link_iaabbs.data() + box * 6;
        const float pad = detail::link_radius_for_box(envelope, box) +
            static_cast<float>(std::max(0.0, options.safety_epsilon));
        s.envelope_volume_sum += static_cast<double>(detail::aabb_volume_padded(link_box, pad));
    }

    float envelope_bounds[6] = {
        envelope.link_iaabbs[0], envelope.link_iaabbs[1], envelope.link_iaabbs[2],
        envelope.link_iaabbs[3], envelope.link_iaabbs[4], envelope.link_iaabbs[5]
    };
    for (int i = 1; i < n_boxes; ++i) {
        detail::include_aabb(envelope_bounds, envelope.link_iaabbs.data() + i * 6);
    }
    float max_radius = static_cast<float>(std::max(0.0, options.safety_epsilon));
    for (double radius : envelope.active_link_radii) {
        max_radius = std::max(max_radius, static_cast<float>(radius + options.safety_epsilon));
    }

    for (int obs = 0; obs < n_obstacles; ++obs) {
        const float* obstacle = obstacles[obs].bounds;
        if (!skip_aabb_broadphase) {
            s.envelope_aabb_tests += 1;
            if (!detail::aabb_overlap_padded(envelope_bounds, obstacle, max_radius)) {
                s.envelope_aabb_rejects += 1;
                continue;
            }
        }

        bool obstacle_maybe = false;
        for (int box = 0; box < n_boxes; ++box) {
            const float* link_box = envelope.link_iaabbs.data() + box * 6;
            const float pad = detail::link_radius_for_box(envelope, box) +
                static_cast<float>(std::max(0.0, options.safety_epsilon));
            if (options.use_link_aabb_broadphase && !skip_aabb_broadphase) {
                s.link_aabb_tests += 1;
                if (!detail::aabb_overlap_padded(link_box, obstacle, pad)) {
                    s.link_aabb_rejects += 1;
                    continue;
                }
            }

            bool separated = false;
            if (resolved_mode == EnvelopeCollisionMode::KDOP ||
                resolved_mode == EnvelopeCollisionMode::KDOPThenGJK) {
                separated = detail::kdop_separates_obstacle(envelope, box, obstacle, pad, s);
            }
            if (!separated &&
                (resolved_mode == EnvelopeCollisionMode::GJK ||
                 resolved_mode == EnvelopeCollisionMode::KDOPThenGJK)) {
                separated = detail::support_hull_separates_obstacle(envelope, box, obstacle, pad, s);
            }
            if (separated) {
                continue;
            }

            const double overlap_depth =
                static_cast<double>(detail::aabb_overlap_depth_padded(link_box, obstacle, pad));
            if (overlap_depth <= std::max(0.0, options.overlap_tolerance)) {
                s.overlap_tolerance_rejects += 1;
                continue;
            }
            s.maybe_pairs += 1;
            s.maybe_pair_overlap_depth_sum += overlap_depth;
            s.maybe_pair_overlap_depth_max =
                std::max(s.maybe_pair_overlap_depth_max, overlap_depth);
            const double overlap_volume =
                static_cast<double>(detail::aabb_overlap_volume_padded(link_box, obstacle, pad));
            s.maybe_pair_overlap_volume_sum += overlap_volume;
            const double overlap_ratio = s.envelope_volume_sum > 0.0
                ? s.maybe_pair_overlap_volume_sum / s.envelope_volume_sum
                : 0.0;
            const double pair_overlap_ratio = s.envelope_volume_sum > 0.0
                ? overlap_volume / s.envelope_volume_sum
                : 0.0;
            s.maybe_pair_overlap_volume_ratio_max =
                std::max(s.maybe_pair_overlap_volume_ratio_max,
                         std::max(0.0, std::min(1.0, overlap_ratio)));
            if (overlap_depth > s.dominant_blocker_overlap_depth) {
                const int active_link_index = n_sub > 0 ? box / n_sub : box;
                s.dominant_blocker_active_link_index = active_link_index;
                s.dominant_blocker_obstacle_index = obs;
                s.dominant_blocker_stage = static_cast<int>(stage_for_maybe_pair());
                s.dominant_blocker_overlap_depth = overlap_depth;
                s.dominant_blocker_overlap_volume_ratio =
                    std::max(0.0, std::min(1.0, pair_overlap_ratio));
                s.dominant_blocker_margin = -overlap_depth;
            }
            const int active_link_index = n_sub > 0 ? box / n_sub : box;
            s.blockers.push_back(EnvelopeCollisionBlocker{
                active_link_index,
                obs,
                static_cast<int>(stage_for_maybe_pair()),
                -overlap_depth,
                overlap_depth,
                std::max(0.0, std::min(1.0, pair_overlap_ratio)),
            });
            obstacle_maybe = true;
            if (!options.count_all_pairs) {
                return CollisionResultKind::MaybeColliding;
            }
        }
        if (!obstacle_maybe) {
            s.link_union_aabb_rejects += 1;
        }
        s.link_union_aabb_tests += 1;
    }

    return s.maybe_pairs == 0
        ? CollisionResultKind::DefinitelyFree
        : CollisionResultKind::MaybeColliding;
}

}  // namespace rbf
