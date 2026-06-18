#pragma once
/// @file endpoint_source.h
/// @brief Endpoint iAABB computation: multiple source methods + source substitution.
///
/// Endpoint sources to compute endpoint (proximal + distal) interval AABBs:
///   - **IFK** (safe) — unsplit AA-FK, certified outer bound.
///   - **CritSample** (unsafe) — Critical-point boundary enumeration.
///   - **Analytical** (safe) — Multi-phase closed-form critical-point solve.
///   - **GCPC** (safe) — Pre-computed interior critical points cache.
///   - **MC** (unsafe) — Pure Monte Carlo sampling within interval box.
///   - **HIFK** (safe) — Hierarchical bisection with AA-FK leaves.
///
/// A substitution matrix (`kSourceSubstitutionMatrix`) determines when
/// cached data from one source can serve a request for another source.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>

#include <cstdint>
#include <vector>

namespace rbf {

// Forward declaration
class GcpcCache;

// ─── Endpoint source method ─────────────────────────────────────────────────
enum class EndpointSource : uint8_t {
    IFK         = 0,
    CritSample  = 1,
    Analytical  = 2,
    GCPC        = 3,
    MC          = 4,
    HIFK        = 5
};

enum class EndpointSafetyLevel : uint8_t {
    UnsafeHeuristic = 0,
    Provisional = 1,
    Certified = 2,
};

inline EndpointSafetyLevel endpoint_source_default_safety(EndpointSource s) {
    switch (s) {
        case EndpointSource::IFK:
        case EndpointSource::Analytical:
        case EndpointSource::GCPC:
        case EndpointSource::HIFK:
            return EndpointSafetyLevel::Certified;
        case EndpointSource::CritSample:
        case EndpointSource::MC:
        default:
            return EndpointSafetyLevel::UnsafeHeuristic;
    }
}

inline bool endpoint_safety_is_certified(EndpointSafetyLevel level) {
    return level == EndpointSafetyLevel::Certified;
}

inline const char* endpoint_source_name(EndpointSource s) {
    switch (s) {
        case EndpointSource::IFK:        return "IFK";
        case EndpointSource::CritSample: return "CritSample";
        case EndpointSource::Analytical: return "Analytical";
        case EndpointSource::GCPC:       return "GCPC";
        case EndpointSource::MC:         return "MC";
        case EndpointSource::HIFK:       return "HIFK";
        default:                         return "Unknown";
    }
}

enum class HifkSplitStrategy : uint8_t {
    RoundRobin = 0,
    WidestRoot = 1,
    FixedDepthSchedule = 2,
};

// ─── Configuration ──────────────────────────────────────────────────────────
struct EndpointSourceConfig {
    EndpointSource source = EndpointSource::IFK;
    int n_samples_crit = 1000;          // CritSample
    int n_threads = 1;                  // CritSample internal enumeration threads; 1 = serial, 0 = auto
    int parallel_min_combos = 0;        // Minimum combo count before internal parallelism; <=0 = auto
    int max_phase_analytical = 3;       // Analytical (0..3)
    bool bypass_narrow_skip = false;    // skip kPhase123Threshold early exit
    // GCPC parity mode: when true, GCPC returns analytical(max_phase)
    // directly (same volume baseline, mainly for controlled experiments).
    bool gcpc_match_analytical = false;
    const GcpcCache* gcpc_cache = nullptr;  // GCPC (not owned)
    int hifk_max_depth = 9;               // HIFK total bisection depth; <0 selects interval-aware auto depth
    int hifk_n_threads = 1;               // reserved for future parallel HIFK
    double hifk_vol_ratio_thresh = 0.0;  // >0 enables adaptive BFS splitting
    HifkSplitStrategy hifk_split_strategy = HifkSplitStrategy::RoundRobin;
    int hifk_depth_offset = 0;
    double hifk_min_split_width = 0.0;
    std::vector<int> hifk_depth_dimensions;
    std::vector<Interval> hifk_root_intervals;
};

// ─── Result ─────────────────────────────────────────────────────────────────
struct EndpointIAABBResult {
    std::vector<float> endpoint_iaabbs;  // [n_active × 2 × 6]
    EndpointSource source = EndpointSource::IFK;
    bool is_safe = false;
    EndpointSafetyLevel safety_level = EndpointSafetyLevel::UnsafeHeuristic;
    int n_active_links = 0;

    // Interval-FK state for sources that explicitly build it; AA-backed IFK/HIFK
    // currently leave this empty.
    FKState fk_state;

    // Analytical source: number of links fully pruned by AA Gap Pruning
    int n_pruned_links = 0;

    // Instrumentation for endpoint sources that enumerate candidate combinations.
    int64_t combo_count = 0;
    int enumerate_threads = 1;
    double enumerate_time_us = 0.0;
    int changed_dim = -1;
    int64_t parallel_min_combos_used = 0;
    int64_t enumerate_chunk_size = 0;
    int enumerate_chunk_count = 0;
    int candidate_dirty_count = 0;
    int predh_rebuild_count = 0;
    bool endpoint_cache_reused = false;

    int endpoint_iaabb_len() const { return n_active_links * 2 * 6; }
};

// ─── Unified entry point ────────────────────────────────────────────────────
EndpointIAABBResult compute_endpoint_iaabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config,
    FKState* fk = nullptr,
    int changed_dim = -1);

int recommend_hifk_depth(
    const std::vector<Interval>& intervals,
    int max_depth_cap = 5);

int recommend_hifk_depth(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int max_depth_cap = 5);

// ─── Source substitution matrix ─────────────────────────────────────────────────
// Row = cached source, Column = requested source.
// Order: IFK(0), CritSample(1), Analytical(2), GCPC(3), MC(4), HIFK(5).
inline constexpr int kEndpointSourceCount = 6;

inline constexpr bool kSourceSubstitutionMatrix[kEndpointSourceCount][kEndpointSourceCount] = {
    // requested:            IFK    Crit   Analyt GCPC   MC     HIFK
    /* cached IFK */        {true,  false, false, false, false, false},
    /* cached Crit */       {false, true,  false, false, false, false},
    /* cached Analytical */ {true,  true,  true,  false, false, false},
    /* cached GCPC */       {true,  true,  true,  true,  false, false},
    /* cached MC */         {false, false, false, false, true,  false},
    /* cached HIFK */       {false, false, false, false, false, true },
};

static_assert(static_cast<int>(EndpointSource::IFK) == 0,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::CritSample) == 1,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::Analytical) == 2,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::GCPC) == 3,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::MC) == 4,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::HIFK) == 5,
              "EndpointSource enum order changed: update substitution matrix");

/// Can a cached result (from `cached`) serve a request for `requested`?
inline bool source_can_serve(EndpointSource cached, EndpointSource requested) {
    const int ci = static_cast<int>(cached);
    const int ri = static_cast<int>(requested);
    if (ci < 0 || ci >= kEndpointSourceCount ||
        ri < 0 || ri >= kEndpointSourceCount)
        return false;
    return kSourceSubstitutionMatrix[ci][ri];
}

/// Map endpoint source to channel index: safe → CH_SAFE (0), else → CH_UNSAFE (1).
inline int source_channel(EndpointSource s) {
    switch (s) {
        case EndpointSource::IFK:
        case EndpointSource::HIFK:
            return 0;
        default:
            return 1;
    }
}

// ─── Hull (union) of endpoint iAABBs ───────────────────────────────────────────────
// Element-wise hull: dst[i] = min(dst[i], src[i]) for lo, max for hi.
// Layout: n_endpoints × 6 floats  [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
void hull_endpoint_iaabbs(float* dst, const float* src, int n_endpoints);
}  // namespace rbf
