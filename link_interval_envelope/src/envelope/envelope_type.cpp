// RBFPlanningForest v6 — Unified Link Envelope (Phase C4, N6: Hull16_Grid)
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/kdop.h>
#include <sbf/envelope/link_iaabb.h>
#include <sbf/envelope/link_grid.h>
#include <sbf/envelope/support_hull.h>
#include <sbf/voxel/voxel_grid.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

namespace rbf {

namespace {

using Clock = std::chrono::steady_clock;

void include_bounds(double lo[3], double hi[3], const float* box) {
    for (int a = 0; a < 3; ++a) {
        lo[a] = std::min(lo[a], static_cast<double>(box[a]));
        hi[a] = std::max(hi[a], static_cast<double>(box[a + 3]));
    }
}

void finalise_grid_metrics(LinkEnvelope& result, const voxel::SparseVoxelGrid& grid,
                           Clock::time_point start, Clock::time_point stop) {
    result.grid_fill_time_us = std::chrono::duration<double, std::micro>(stop - start).count();
    result.grid_grow_count = grid.grow_count();
    result.grid_reserve_count = grid.reserve_count();
    result.grid_capacity = grid.capacity();
    result.grid_range_write_count = grid.range_write_count();
    result.grid_local_range_write_count = grid.local_range_write_count();
    result.grid_brick_write_count = grid.brick_write_count();
    result.grid_fallback_range_write_count = grid.fallback_range_write_count();
}

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
        config.type == EnvelopeType::SupportHull ||
        config.type == EnvelopeType::Hull16_Grid;
    const bool wants_support_hull = config.type == EnvelopeType::SupportHull ||
        config.type == EnvelopeType::Hull16_Grid;
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
    // Inflation by link_radii is deferred to the grid-fill step so that
    // each sub-AABB is only padded once at the point of use.
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

    // LinkIAABB_Grid: rasterize (sub-)AABBs into sparse grid, inflated by r+pad
    if (config.type == EnvelopeType::LinkIAABB_Grid) {
        const double delta = config.grid_config.voxel_delta;
        auto sg = std::make_unique<voxel::SparseVoxelGrid>(delta);

        int n_boxes = n_active_links * n_sub;
        const float pad = static_cast<float>(sg->safety_pad());
        double grid_lo[3] = {
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
        };
        double grid_hi[3] = {
            -std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),
        };
        for (int i = 0; i < n_boxes; ++i) {
            const float* src = result.link_iaabbs.data() + i * 6;
            int ci = i / n_sub;
            float r = (link_radii != nullptr)
                          ? static_cast<float>(link_radii[ci]) + pad : pad;
            float inflated[6] = {
                src[0] - r, src[1] - r, src[2] - r,
                src[3] + r, src[4] + r, src[5] + r
            };
            include_bounds(grid_lo, grid_hi, inflated);
        }
        if (n_boxes > 0) sg->reserve_from_bounds(grid_lo, grid_hi);

        const auto grid_start = Clock::now();
        for (int i = 0; i < n_boxes; ++i) {
            const float* src = result.link_iaabbs.data() + i * 6;
            int ci = i / n_sub;
            float r = (link_radii != nullptr)
                          ? static_cast<float>(link_radii[ci]) + pad : pad;
            float inflated[6] = {
                src[0] - r, src[1] - r, src[2] - r,
                src[3] + r, src[4] + r, src[5] + r
            };
            sg->fill_aabb(inflated);
        }
        const auto grid_stop = Clock::now();
        finalise_grid_metrics(result, *sg, grid_start, grid_stop);

        result.sparse_grid = std::move(sg);
    }

    // Hull16_Grid: rasterize per-link convex hull segments into sparse grid
    if (config.type == EnvelopeType::Hull16_Grid) {
        const double delta = config.grid_config.voxel_delta;
        auto sg = std::make_unique<voxel::SparseVoxelGrid>(delta);
        double grid_lo[3] = {
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
        };
        double grid_hi[3] = {
            -std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),
        };

        const double pad = sg->safety_pad();
        for (int i = 0; i < n_active_links; ++i) {
            const float* prox = endpoint_iaabbs + i * 12;
            const float* dist = endpoint_iaabbs + i * 12 + 6;
            const double r = (link_radii ? link_radii[i] : 0.0) + pad;
            for (int a = 0; a < 3; ++a) {
                const double lo = std::min(static_cast<double>(prox[a]),
                                           static_cast<double>(dist[a])) - r;
                const double hi = std::max(static_cast<double>(prox[a + 3]),
                                           static_cast<double>(dist[a + 3])) + r;
                grid_lo[a] = std::min(grid_lo[a], lo);
                grid_hi[a] = std::max(grid_hi[a], hi);
            }
        }
        if (n_active_links > 0) sg->reserve_from_bounds(grid_lo, grid_hi);

        // endpoint_iaabbs layout: [n_active × 2 × 6]
        // per link: prox[6] + dist[6]
        const auto grid_start = Clock::now();
        for (int i = 0; i < n_active_links; ++i) {
            const float* prox = endpoint_iaabbs + i * 12;
            const float* dist = endpoint_iaabbs + i * 12 + 6;
            const double r = link_radii ? link_radii[i] : 0.0;
            sg->fill_hull16(prox, dist, r);
        }
        const auto grid_stop = Clock::now();
        finalise_grid_metrics(result, *sg, grid_start, grid_stop);

        result.sparse_grid = std::move(sg);
    }

    return result;
}

}  // namespace rbf
