#pragma once
/// @file link_iaabb.h
/// @brief Link IAABB derivation from endpoint iAABBs (Phase C1).
///
/// Converts proximal+distal endpoint iAABBs into per-link bounding boxes.
/// Two modes:
///   - **Paired**: hull of proximal and distal endpoints + radius inflation.
///   - **Subdivided**: each link split into n_sub segments via linear
///     interpolation between proximal and distal intervals.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>

#include <algorithm>
#include <vector>

namespace rbf {

// ─── Paired derive: proximal+distal → link AABB ────────────────────────────
// endpoint_iaabbs layout: [n_active × 2 × 6]
//   (ci*2+0)*6 = proximal iAABB, (ci*2+1)*6 = distal iAABB
// out_link_iaabbs layout: [n_active × 6]: [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
void derive_link_iaabb_paired(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    float* out_link_iaabbs);

// ─── Subdivided derive: each link split into n_sub segments ────────────────
// Linear interpolation between proximal and distal endpoint intervals.
// out_sub_iaabbs layout: [n_active × n_sub × 6]
void derive_link_iaabb_subdivided(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    int n_subdivisions,
    float* out_sub_iaabbs);

inline void interpolate_endpoint_box(
    const float* proximal_box,
    const float* distal_box,
    float t,
    float* out_box)
{
    for (int i = 0; i < 6; ++i) {
        out_box[i] = proximal_box[i] * (1.0f - t) + distal_box[i] * t;
    }
}

inline void expand_box_in_place(float* box, float padding) {
    if (padding <= 0.0f) return;
    box[0] -= padding;
    box[1] -= padding;
    box[2] -= padding;
    box[3] += padding;
    box[4] += padding;
    box[5] += padding;
}

}  // namespace rbf
