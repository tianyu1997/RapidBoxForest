// RBFPlanningForest v6 — Unified Link Envelope
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/kdop.h>
#include <sbf/envelope/link_iaabb.h>
#include <sbf/envelope/support_hull.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace rbf {

namespace {

void populate_aabb_payloads(LinkEnvelope& result, const double* link_radii) {
    const int n_sub = std::max(1, result.n_subdivisions);
    const int n_boxes = result.n_active_links * n_sub;
    if (n_boxes <= 0 || static_cast<int>(result.link_iaabbs.size()) < n_boxes * 6) {
        result.inflated_link_iaabbs.clear();
        result.envelope_aabb.clear();
        result.link_union_iaabbs.clear();
        return;
    }

    result.inflated_link_iaabbs = result.link_iaabbs;
    result.link_union_iaabbs.assign(static_cast<std::size_t>(result.n_active_links * 6), 0.0f);
    result.envelope_aabb = {
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
    };

    for (int link = 0; link < result.n_active_links; ++link) {
        float link_union[6] = {
            std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity(),
        };
        const float radius = link_radii != nullptr ? static_cast<float>(link_radii[link]) : 0.0f;
        for (int sub = 0; sub < n_sub; ++sub) {
            const int box = link * n_sub + sub;
            float* inflated = result.inflated_link_iaabbs.data() + static_cast<std::size_t>(box) * 6U;
            inflated[0] -= radius;
            inflated[1] -= radius;
            inflated[2] -= radius;
            inflated[3] += radius;
            inflated[4] += radius;
            inflated[5] += radius;
            for (int axis = 0; axis < 3; ++axis) {
                link_union[axis] = std::min(link_union[axis], inflated[axis]);
                link_union[axis + 3] = std::max(link_union[axis + 3], inflated[axis + 3]);
                result.envelope_aabb[static_cast<std::size_t>(axis)] =
                    std::min(result.envelope_aabb[static_cast<std::size_t>(axis)], inflated[axis]);
                result.envelope_aabb[static_cast<std::size_t>(axis + 3)] =
                    std::max(result.envelope_aabb[static_cast<std::size_t>(axis + 3)], inflated[axis + 3]);
            }
        }
        float* dst = result.link_union_iaabbs.data() + static_cast<std::size_t>(link) * 6U;
        std::copy(link_union, link_union + 6, dst);
    }
}

void populate_shape_payloads(LinkEnvelope& result, const EnvelopeTypeConfig& config) {
    const bool wants_kdop = config.type == EnvelopeType::KDOP ||
        config.type == EnvelopeType::SupportHull;
    const bool wants_support_hull = config.type == EnvelopeType::SupportHull;
    result.kdop_direction_set = config.kdop_config.direction_set;
    result.kdop_n_axes = wants_kdop ? kdop_axis_count(config.kdop_config.direction_set) : 0;
    if (wants_kdop && !result.inflated_link_iaabbs.empty()) {
        result.kdop_intervals = compute_kdop_intervals_from_aabbs(
            result.inflated_link_iaabbs, config.kdop_config.direction_set);
    } else {
        result.kdop_intervals.clear();
    }
    if (wants_support_hull && !result.inflated_link_iaabbs.empty()) {
        result.support_hulls = compute_support_hulls_from_aabbs(result.inflated_link_iaabbs);
    } else {
        result.support_hulls.clear();
    }
}

}  // namespace

LinkEnvelope compute_link_envelope(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    const EnvelopeTypeConfig& config)
{
    LinkEnvelope result;
    result.type = config.type;
    result.n_active_links = n_active_links;
    if (link_radii != nullptr && n_active_links > 0) {
        result.active_link_radii.assign(link_radii, link_radii + n_active_links);
    }

    const int n_sub = std::max(1, config.n_subdivisions);
    result.n_subdivisions = n_sub;

    if (n_active_links <= 0 || endpoint_iaabbs == nullptr) {
        return result;
    }

    // Compute TIGHT link iAABBs (without link_radii inflation).
    if (n_sub <= 1) {
        result.link_iaabbs.resize(n_active_links * 6);
        derive_link_iaabb_paired(endpoint_iaabbs, n_active_links,
                                 nullptr, result.link_iaabbs.data());
    } else {
        result.link_iaabbs.resize(n_active_links * n_sub * 6);
        derive_link_iaabb_subdivided(endpoint_iaabbs, n_active_links,
                                     nullptr, n_sub,
                                     result.link_iaabbs.data());
    }

    populate_aabb_payloads(result, link_radii);
    populate_shape_payloads(result, config);

    return result;
}

}  // namespace rbf
