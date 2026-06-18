#pragma once
/// @file crit_source.h
/// @brief Critical-point boundary enumeration endpoint source (unsafe).
///
/// Two stages:
///   1. Critical point enumeration: {lo, hi, kπ/2} boundary combos → FK → AABB.
///   2. (Future) Local optimization at discovered extrema.
///
/// Result `is_safe = false` — may miss true extrema beyond sampled points.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/dh_enumerate.h>
#include <sbf/envelope/endpoint_source.h>

#include <cstdint>
#include <vector>

namespace rbf {

struct CritSampleState {
    uint64_t robot_fingerprint = 0;
    int n_joints = 0;
    int n_active_links = 0;
    bool valid = false;
    bool endpoint_valid = false;
    std::vector<Interval> intervals;
    std::vector<std::vector<double>> raw_candidates;
    std::vector<std::vector<double>> candidates;
    std::vector<std::vector<PreDH>> pre_dh;
    std::vector<int> n_cands;
    std::vector<float> endpoint_iaabbs;
    int64_t combo_count = 0;
    int enumerate_threads = 1;
    double enumerate_time_us = 0.0;
    int64_t parallel_min_combos_used = 0;
    int64_t enumerate_chunk_size = 0;
    int enumerate_chunk_count = 0;
    int candidate_dirty_count = 0;
    int predh_rebuild_count = 0;

    void reset();
};

// Compute endpoint iAABBs via critical-point boundary enumeration.
// Two stages:
//   1. Critical point enumeration: {lo, hi, k*pi/2 within range} combos → FK → AABB
//      Narrow intervals (< 0.01 rad) collapse to midpoint only.
//   2. (Optional future) Local optimization
// Result is_safe = false (UNSAFE — may miss extrema).
EndpointIAABBResult compute_endpoint_iaabb_crit(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_samples = 1000,
    uint64_t seed = 42,
    int changed_dim = -1,
    FKState* fk = nullptr,
    int n_threads = 1,
    int parallel_min_combos = 0);

EndpointIAABBResult compute_endpoint_iaabb_crit_incremental(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_samples = 1000,
    uint64_t seed = 42,
    int changed_dim = -1,
    FKState* fk = nullptr,
    CritSampleState* state = nullptr,
    int n_threads = 1,
    int parallel_min_combos = 0);

}  // namespace rbf
