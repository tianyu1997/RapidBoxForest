#pragma once

#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/link_iaabb.h>

#include <algorithm>
#include <array>
#include <vector>

namespace rbf {

// Each record stores the two endpoint AABBs of a short-link segment:
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
        const float cx = 0.5f * (aabb[0] + aabb[3]);
        const float cy = 0.5f * (aabb[1] + aabb[4]);
        const float cz = 0.5f * (aabb[2] + aabb[5]);
        const float hx = std::max(0.0f, 0.5f * (aabb[3] - aabb[0]));
        const float hy = std::max(0.0f, 0.5f * (aabb[4] - aabb[1]));
        const float hz = std::max(0.0f, 0.5f * (aabb[5] - aabb[2]));
        out[0] = cx;
        out[1] = cy;
        out[2] = cz;
        out[3] = hx;
        out[4] = hy;
        out[5] = hz;
        out[6] = aabb[0];
        out[7] = aabb[1];
        out[8] = aabb[2];
        out[9] = aabb[3];
        out[10] = aabb[4];
        out[11] = aabb[5];
        out[12] = std::max({hx, hy, hz});
    }
    return records;
}

inline std::vector<float> compute_support_hulls_from_endpoint_iaabbs(
    const float* endpoint_iaabbs,
    int n_active_links,
    int n_subdivisions,
    float safety_epsilon = 0.0f)
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
        for (int sub = 0; sub < n_sub; ++sub) {
            const float t0 = static_cast<float>(sub) / static_cast<float>(n_sub);
            const float t1 = static_cast<float>(sub + 1) / static_cast<float>(n_sub);
            float* out = records.data() +
                static_cast<std::size_t>((link * n_sub + sub) * kSupportHullRecordSize);
            interpolate_endpoint_box(prox, dist, t0, out);
            interpolate_endpoint_box(prox, dist, t1, out + 6);
            expand_box_in_place(out, safety_epsilon);
            expand_box_in_place(out + 6, safety_epsilon);
            out[12] = 0.5f * std::max({
                out[3] - out[0],
                out[4] - out[1],
                out[5] - out[2],
                out[9] - out[6],
                out[10] - out[7],
                out[11] - out[8],
            });
        }
    }
    return records;
}

}  // namespace rbf
