#pragma once
/// @file envelope_type.h
/// @brief Envelope type enumeration, configuration, and unified computation entry point.
///
/// The envelope pipeline converts per-node endpoint iAABBs into link envelopes
/// that bound the workspace swept volume of each robot link.  Three types:
///   - `LinkIAABB` — AABB hull per link (fast, coarse).
///   - `LinkIAABB_Grid` — AABB + voxel grid rasterization (tighter, slower).
///   - `Hull16_Grid` — reserved for future hull-16 turbo scanline.

#include <sbf/envelope/link_grid.h>
#include <sbf/voxel/voxel_grid.h>
#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace rbf {

// ─── Envelope type ──────────────────────────────────────────────────────────
enum class EnvelopeType : uint8_t {
    LinkIAABB      = 0,  // AABB(sub=n)
    LinkIAABB_Grid = 1,  // AABB + voxel grid
    Hull16_Grid    = 2,  // reserved for grid-envelope variants
    Hull_Grid      = Hull16_Grid,  // backward-compatible SBF/LECT alias
    KDOP           = Hull16_Grid,  // compatibility alias for SBF evidence presets
    SupportHull    = Hull16_Grid   // compatibility alias for SBF evidence presets
};

inline const char* envelope_type_name(EnvelopeType t) {
    switch (t) {
        case EnvelopeType::LinkIAABB:      return "LinkIAABB";
        case EnvelopeType::LinkIAABB_Grid: return "LinkIAABB_Grid";
        case EnvelopeType::Hull16_Grid:    return "Hull16_Grid";
        default:                           return "Unknown";
    }
}

// ─── Configuration ──────────────────────────────────────────────────────────
enum class KdopDirectionSet : uint8_t {
    DOP6 = 0,
    DOP18 = 1,
    DOP26 = 2,
};

struct KdopConfig {
    KdopDirectionSet direction_set = KdopDirectionSet::DOP26;
    double safety_epsilon = 0.0;
};

struct SupportHullConfig {
    bool keep_kdop = true;
    double safety_epsilon = 0.0;
};

struct EnvelopeTypeConfig {
    EnvelopeType type = EnvelopeType::LinkIAABB;
    int n_subdivisions = 1;     // sub=1 whole link, sub=n subdivided
    GridConfig grid_config;     // only used by Grid types
    KdopConfig kdop_config;     // compatibility for KDOP evidence presets
    SupportHullConfig support_hull_config;  // compatibility for SupportHull presets
};

// ─── Result ─────────────────────────────────────────────────────────────────
struct LinkEnvelope {
    EnvelopeType type = EnvelopeType::LinkIAABB;
    int n_active_links = 0;
    int n_subdivisions = 1;                   // sub count used to produce link_iaabbs
    int kdop_n_axes = 0;
    KdopDirectionSet kdop_direction_set = KdopDirectionSet::DOP26;
    std::vector<float> link_iaabbs;           // [n_active * n_sub × 6], always present
    std::vector<float> active_link_radii;      // copied for direct collision compatibility
    std::vector<float> inflated_link_iaabbs;   // [n_active * n_sub × 6], link_iaabbs inflated by radius
    std::vector<float> envelope_aabb;          // aggregate [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
    std::vector<float> link_union_iaabbs;      // [n_active × 6], union across subdivisions
    std::vector<float> kdop_intervals;         // [n_active * n_sub × kdop_n_axes × 2]
    std::vector<float> support_hulls;          // [n_active * n_sub × kSupportHullRecordSize]
    std::unique_ptr<voxel::SparseVoxelGrid> sparse_grid;  // non-null for Grid types
    double grid_fill_time_us = 0.0;
    int grid_grow_count = 0;
    int grid_reserve_count = 0;
    int grid_capacity = 0;
    int64_t grid_range_write_count = 0;
    int64_t grid_local_range_write_count = 0;
    int64_t grid_brick_write_count = 0;
    int64_t grid_fallback_range_write_count = 0;

    bool has_grid() const { return sparse_grid != nullptr; }
};

// ─── Unified entry point ────────────────────────────────────────────────────
LinkEnvelope compute_link_envelope(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    const EnvelopeTypeConfig& config);

}  // namespace rbf
