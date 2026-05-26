#pragma once

#include <sbf/envelope/envelope_type.h>

#include <algorithm>
#include <array>
#include <vector>

namespace rbf {

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

}  // namespace rbf
