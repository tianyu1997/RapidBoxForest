#pragma once

#include <sbf/envelope/envelope_type.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace rbf {

inline int kdop_axis_count(KdopDirectionSet direction_set) {
    switch (direction_set) {
        case KdopDirectionSet::DOP6: return 3;
        case KdopDirectionSet::DOP18: return 9;
        case KdopDirectionSet::DOP26: return 13;
    }
    return 13;
}

inline std::array<float, 39> kdop_axis_table(KdopDirectionSet direction_set) {
    std::array<float, 39> axes{};
    const std::array<std::array<float, 3>, 13> base{{
        {{1.0f, 0.0f, 0.0f}},
        {{0.0f, 1.0f, 0.0f}},
        {{0.0f, 0.0f, 1.0f}},
        {{1.0f, 1.0f, 0.0f}},
        {{1.0f, -1.0f, 0.0f}},
        {{1.0f, 0.0f, 1.0f}},
        {{1.0f, 0.0f, -1.0f}},
        {{0.0f, 1.0f, 1.0f}},
        {{0.0f, 1.0f, -1.0f}},
        {{1.0f, 1.0f, 1.0f}},
        {{1.0f, 1.0f, -1.0f}},
        {{1.0f, -1.0f, 1.0f}},
        {{-1.0f, 1.0f, 1.0f}},
    }};
    const int count = kdop_axis_count(direction_set);
    for (int axis = 0; axis < count; ++axis) {
        const auto& src = base[static_cast<std::size_t>(axis)];
        const float norm = std::sqrt(src[0] * src[0] + src[1] * src[1] + src[2] * src[2]);
        axes[static_cast<std::size_t>(axis * 3 + 0)] = src[0] / norm;
        axes[static_cast<std::size_t>(axis * 3 + 1)] = src[1] / norm;
        axes[static_cast<std::size_t>(axis * 3 + 2)] = src[2] / norm;
    }
    return axes;
}

inline std::vector<float> compute_kdop_intervals_from_aabbs(
    const std::vector<float>& aabbs,
    KdopDirectionSet direction_set) {
    if (aabbs.size() % 6U != 0U) {
        return {};
    }
    const int n_boxes = static_cast<int>(aabbs.size() / 6U);
    const int n_axes = kdop_axis_count(direction_set);
    const auto axes = kdop_axis_table(direction_set);
    std::vector<float> intervals(static_cast<std::size_t>(n_boxes * n_axes * 2), 0.0f);
    for (int box = 0; box < n_boxes; ++box) {
        const float* aabb = aabbs.data() + static_cast<std::size_t>(box) * 6U;
        for (int axis = 0; axis < n_axes; ++axis) {
            const float dx = axes[static_cast<std::size_t>(axis * 3 + 0)];
            const float dy = axes[static_cast<std::size_t>(axis * 3 + 1)];
            const float dz = axes[static_cast<std::size_t>(axis * 3 + 2)];
            float lo = 0.0f;
            float hi = 0.0f;
            const float xs[2] = {aabb[0], aabb[3]};
            const float ys[2] = {aabb[1], aabb[4]};
            const float zs[2] = {aabb[2], aabb[5]};
            bool first = true;
            for (float x : xs) {
                for (float y : ys) {
                    for (float z : zs) {
                        const float projection = dx * x + dy * y + dz * z;
                        if (first) {
                            lo = projection;
                            hi = projection;
                            first = false;
                        } else {
                            lo = std::min(lo, projection);
                            hi = std::max(hi, projection);
                        }
                    }
                }
            }
            const std::size_t offset = static_cast<std::size_t>((box * n_axes + axis) * 2);
            intervals[offset] = lo;
            intervals[offset + 1U] = hi;
        }
    }
    return intervals;
}

}  // namespace rbf
