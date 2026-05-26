#pragma once
/// @file envelope_collision.h
/// @brief Conservative collision checks for link envelopes against AABB obstacles.

#include <sbf/core/types.h>
#include <sbf/envelope/envelope_type.h>

#include <algorithm>
#include <cstdint>

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
};

struct EnvelopeCollisionOptions {
    EnvelopeCollisionMode mode = EnvelopeCollisionMode::Auto;
    bool use_link_aabb_broadphase = true;
    bool count_all_pairs = false;
    double safety_epsilon = 0.0;
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
};

namespace detail {

inline bool aabb_overlap_padded(const float* a, const float* b, float pad) {
    return a[0] - pad <= b[3] && b[0] <= a[3] + pad &&
           a[1] - pad <= b[4] && b[1] <= a[4] + pad &&
           a[2] - pad <= b[5] && b[2] <= a[5] + pad;
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

}  // namespace detail

/// Conservative AABB-backed fallback for KDOP/SupportHull compatibility.
///
/// The current standalone dependency stores centerline link AABBs plus copied
/// radii. This routine inflates each link AABB by radius + safety epsilon and
/// only returns `DefinitelyFree` when every inflated box is separated from all
/// obstacles. Any overlap remains `MaybeColliding` for downstream refinement.
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
        return CollisionResultKind::MaybeColliding;
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
        s.envelope_aabb_tests += 1;
        if (!detail::aabb_overlap_padded(envelope_bounds, obstacle, max_radius)) {
            s.envelope_aabb_rejects += 1;
            continue;
        }

        bool obstacle_maybe = false;
        for (int box = 0; box < n_boxes; ++box) {
            const float* link_box = envelope.link_iaabbs.data() + box * 6;
            const float pad = detail::link_radius_for_box(envelope, box) +
                static_cast<float>(std::max(0.0, options.safety_epsilon));
            s.link_aabb_tests += 1;
            if (!detail::aabb_overlap_padded(link_box, obstacle, pad)) {
                s.link_aabb_rejects += 1;
                continue;
            }
            s.kdop_tests += options.mode == EnvelopeCollisionMode::KDOP ||
                envelope.type == EnvelopeType::KDOP ? 1 : 0;
            s.kdop_axes_tested += options.mode == EnvelopeCollisionMode::KDOP ||
                envelope.type == EnvelopeType::KDOP ? 6 : 0;
            s.maybe_pairs += 1;
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