// RBFPlanningForest v6 — Critical Sampling Source (Phase B2)
#include <sbf/envelope/crit_source.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <vector>

namespace rbf {

namespace {

using Clock = std::chrono::steady_clock;

static constexpr int64_t kMaxCombos = 8192;

bool same_candidates(const std::vector<double>& a, const std::vector<double>& b) {
    return a == b;
}

bool same_interval(const Interval& a, const Interval& b) {
    return a.lo == b.lo && a.hi == b.hi;
}

struct CritSyncStats {
    int candidate_dirty_count = 0;
    int predh_rebuild_count = 0;
    bool candidates_changed = false;
};

std::vector<double> collect_candidates_for_interval(const Interval& interval) {
    std::vector<double> out;
    collect_kpi2(interval.lo, interval.hi, out);
    return out;
}

std::vector<std::vector<double>> cap_candidates(
    const std::vector<Interval>& intervals,
    const std::vector<std::vector<double>>& raw_candidates)
{
    std::vector<std::vector<double>> capped = raw_candidates;
    int64_t total = 1;
    for (const auto& candidates : capped) {
        total *= static_cast<int64_t>(candidates.size());
        if (total > kMaxCombos * 16) break;
    }

    while (total > kMaxCombos) {
        int worst = 0;
        for (int j = 1; j < static_cast<int>(capped.size()); ++j) {
            if (capped[j].size() > capped[worst].size()) worst = j;
        }
        if (capped[worst].size() <= 3) break;

        const double lo = intervals[worst].lo;
        const double hi = intervals[worst].hi;
        capped[worst] = {lo, 0.5 * (lo + hi), hi};

        total = 1;
        for (const auto& candidates : capped) {
            total *= static_cast<int64_t>(candidates.size());
        }
    }
    return capped;
}

void rebuild_pre_dh_for_joint(
    const Robot& robot,
    int joint_idx,
    const std::vector<double>& candidates,
    CritSampleState& state)
{
    const auto& dh = robot.dh_params();
    state.n_cands[joint_idx] = static_cast<int>(candidates.size());
    state.pre_dh[joint_idx].resize(candidates.size());
    for (int k = 0; k < static_cast<int>(candidates.size()); ++k) {
        build_dh_matrix(dh[joint_idx], candidates[k], state.pre_dh[joint_idx][k].A);
    }
}

CritSyncStats sync_crit_state(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int changed_dim,
    CritSampleState& state)
{
    CritSyncStats stats;
    const int n = robot.n_joints();
    const bool compatible = state.valid &&
        state.robot_fingerprint == robot.fingerprint() &&
        state.n_joints == n &&
        static_cast<int>(state.raw_candidates.size()) == n &&
        static_cast<int>(state.pre_dh.size()) == n &&
        static_cast<int>(state.n_cands.size()) == n;

    if (!compatible || changed_dim < 0 || changed_dim >= n) {
        state.raw_candidates.assign(static_cast<std::size_t>(n), {});
        for (int j = 0; j < n; ++j) {
            state.raw_candidates[j] = collect_candidates_for_interval(intervals[j]);
        }
    } else {
        if (state.intervals.size() != intervals.size() ||
            !same_interval(state.intervals[static_cast<std::size_t>(changed_dim)],
                           intervals[static_cast<std::size_t>(changed_dim)])) {
            state.raw_candidates[changed_dim] = collect_candidates_for_interval(intervals[changed_dim]);
        }
    }

    auto capped = cap_candidates(intervals, state.raw_candidates);
    if (!compatible) {
        state.candidates.assign(static_cast<std::size_t>(n), {});
        state.pre_dh.assign(static_cast<std::size_t>(n), {});
        state.n_cands.assign(static_cast<std::size_t>(n), 0);
    }

    for (int j = 0; j < n; ++j) {
        if (!compatible || j >= static_cast<int>(state.candidates.size()) ||
            !same_candidates(state.candidates[j], capped[j])) {
            rebuild_pre_dh_for_joint(robot, j, capped[j], state);
            ++stats.candidate_dirty_count;
            ++stats.predh_rebuild_count;
        }
    }
    stats.candidates_changed = stats.candidate_dirty_count > 0;

    state.candidates = std::move(capped);
    state.intervals = intervals;
    state.robot_fingerprint = robot.fingerprint();
    state.n_joints = n;
    state.n_active_links = robot.n_active_links();
    state.valid = true;
    if (stats.candidates_changed) {
        state.endpoint_valid = false;
    }
    state.candidate_dirty_count = stats.candidate_dirty_count;
    state.predh_rebuild_count = stats.predh_rebuild_count;
    return stats;
}

void update_fk_result_if_requested(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int changed_dim,
    FKState* fk,
    EndpointIAABBResult& result)
{
    if (!fk) return;
    if (changed_dim >= 0 && fk->valid) {
        update_fk_inplace(*fk, robot, intervals, changed_dim);
    } else {
        *fk = compute_fk_full(robot, intervals);
    }
    result.fk_state = *fk;
}

}  // namespace

void CritSampleState::reset() {
    *this = CritSampleState{};
}

EndpointIAABBResult compute_endpoint_iaabb_crit(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_samples,
    uint64_t seed,
    int changed_dim,
    FKState* fk,
    int n_threads,
    int parallel_min_combos)
{
    return compute_endpoint_iaabb_crit_incremental(
        robot, intervals, n_samples, seed, changed_dim, fk, nullptr,
        n_threads, parallel_min_combos);
}

EndpointIAABBResult compute_endpoint_iaabb_crit_incremental(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_samples,
    uint64_t seed,
    int changed_dim,
    FKState* fk,
    CritSampleState* state,
    int n_threads,
    int parallel_min_combos)
{
    (void)seed;
    (void)n_samples;

    EndpointIAABBResult result;
    result.source = EndpointSource::CritSample;
    result.is_safe = false;
    result.safety_level = EndpointSafetyLevel::UnsafeHeuristic;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(result.endpoint_iaabb_len());
    result.changed_dim = changed_dim;

    const int n_act = result.n_active_links;
    const int* alm = robot.active_link_map();

    init_endpoints_inf(result.endpoint_iaabbs.data(), n_act);

    CritSampleState local_state;
    CritSampleState& active_state = state ? *state : local_state;
    const CritSyncStats sync_stats = sync_crit_state(robot, intervals, changed_dim, active_state);
    result.candidate_dirty_count = sync_stats.candidate_dirty_count;
    result.predh_rebuild_count = sync_stats.predh_rebuild_count;

    if (active_state.endpoint_valid && !sync_stats.candidates_changed) {
        result.endpoint_iaabbs = active_state.endpoint_iaabbs;
        result.combo_count = active_state.combo_count;
        result.enumerate_threads = 0;
        result.enumerate_time_us = 0.0;
        result.parallel_min_combos_used = active_state.parallel_min_combos_used;
        result.enumerate_chunk_size = active_state.enumerate_chunk_size;
        result.enumerate_chunk_count = active_state.enumerate_chunk_count;
        result.endpoint_cache_reused = true;
        update_fk_result_if_requested(robot, intervals, changed_dim, fk, result);
        return result;
    }

    const auto enumerate_start = Clock::now();
    const DHEnumerationStats enum_stats = enumerate_critical_parallel(
        robot,
        active_state.pre_dh,
        active_state.n_cands,
        alm,
        n_act,
        result.endpoint_iaabbs.data(),
        n_threads,
        parallel_min_combos);
    const auto enumerate_stop = Clock::now();
    result.combo_count = enum_stats.combo_count;
    result.enumerate_threads = enum_stats.threads_used;
    result.parallel_min_combos_used = enum_stats.parallel_min_combos_used;
    result.enumerate_chunk_size = enum_stats.chunk_size;
    result.enumerate_chunk_count = enum_stats.chunk_count;
    result.enumerate_time_us = std::chrono::duration<double, std::micro>(
        enumerate_stop - enumerate_start).count();

    active_state.endpoint_iaabbs = result.endpoint_iaabbs;
    active_state.endpoint_valid = true;
    active_state.combo_count = result.combo_count;
    active_state.enumerate_threads = result.enumerate_threads;
    active_state.enumerate_time_us = result.enumerate_time_us;
    active_state.parallel_min_combos_used = result.parallel_min_combos_used;
    active_state.enumerate_chunk_size = result.enumerate_chunk_size;
    active_state.enumerate_chunk_count = result.enumerate_chunk_count;

    update_fk_result_if_requested(robot, intervals, changed_dim, fk, result);

    return result;
}

}  // namespace rbf
