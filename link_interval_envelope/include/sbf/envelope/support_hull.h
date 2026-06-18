#pragma once

#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/link_iaabb.h>

#include <algorithm>
#include <array>
#include <vector>

namespace rbf {

// Each record stores the raw endpoint AABBs of a short-link segment plus an
// auxiliary scalar used for compatibility/debugging. Envelope payloads stay
// radius-free; collision-time padding accounts for link radius and epsilon.
// [proximal_box(6), distal_box(6), auxiliary_scalar(1)].
inline constexpr int kSupportHullRecordSize = 13;

inline std::vector<float> compute_support_hulls_from_aabbs(const std::vector<float>& aabbs) {
    if (aabbs.size() % 6U != 0U) {
        return {};
    }
    const int n_boxes = static_cast<int>(aabbs.size() / 6U);
    std::vector<float> records(static_cast<std::size_t>(n_boxes * kSupportHullRecordSize), 0.0f);
    for (int box = 0; box < n_boxes; ++box) {
        const float* aabb = aabbs.data() + static_cast<std::size_t>(box) * 6U;
        float* out = records.data() + static_cast<std::size_t>(box * kSupportHullRecordSize);
        std::copy(aabb, aabb + 6, out);
        std::copy(aabb, aabb + 6, out + 6);
        out[12] = 0.0f;
    }
    return records;
}

inline std::vector<float> compute_support_hulls_from_endpoint_iaabbs(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    int n_subdivisions)
{
    if (endpoint_iaabbs == nullptr || n_active_links <= 0) {
        return {};
    }
    const int n_sub = std::max(1, n_subdivisions);
    std::vector<float> records(
        static_cast<std::size_t>(n_active_links * n_sub * kSupportHullRecordSize),
        0.0f);

    for (int link = 0; link < n_active_links; ++link) {
        const float* prox = endpoint_iaabbs + static_cast<std::size_t>(link * 2) * 6U;
        const float* dist = endpoint_iaabbs + static_cast<std::size_t>(link * 2 + 1) * 6U;
        const float radius = link_radii != nullptr ? static_cast<float>(link_radii[link]) : 0.0f;
        for (int sub = 0; sub < n_sub; ++sub) {
            const float t0 = static_cast<float>(sub) / static_cast<float>(n_sub);
            const float t1 = static_cast<float>(sub + 1) / static_cast<float>(n_sub);
            float* out = records.data() +
                static_cast<std::size_t>((link * n_sub + sub) * kSupportHullRecordSize);
            interpolate_endpoint_box(prox, dist, t0, out);
            interpolate_endpoint_box(prox, dist, t1, out + 6);
            out[12] = radius;
        }
    }
    return records;
}

inline std::vector<float> compute_support_hulls_from_endpoint_iaabbs(
    const float* endpoint_iaabbs,
    int n_active_links,
    int n_subdivisions,
    float safety_epsilon = 0.0f)
{
    (void)safety_epsilon;
    return compute_support_hulls_from_endpoint_iaabbs(endpoint_iaabbs,
                                                      n_active_links,
                                                      nullptr,
                                                      n_subdivisions);
}

}  // namespace rbf
