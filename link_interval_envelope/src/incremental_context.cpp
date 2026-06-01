#include <link_interval_envelope/incremental_context.h>

#include <chrono>
#include <cmath>
#include <utility>

namespace link_interval_envelope {

namespace {

using Clock = std::chrono::steady_clock;

bool same_interval(const rbf::Interval& a, const rbf::Interval& b) {
    return a.lo == b.lo && a.hi == b.hi;
}

bool differs_only_in_dim(const std::vector<rbf::Interval>& lhs,
                         const std::vector<rbf::Interval>& rhs,
                         int changed_dim) {
    if (changed_dim < 0 || lhs.size() != rhs.size()) {
        return false;
    }

    bool saw_change = false;
    for (int index = 0; index < static_cast<int>(lhs.size()); ++index) {
        if (same_interval(lhs[static_cast<std::size_t>(index)], rhs[static_cast<std::size_t>(index)])) {
            continue;
        }
        if (index != changed_dim || saw_change) {
            return false;
        }
        saw_change = true;
    }
    return saw_change;
}

}  // namespace

IncrementalEnvelopeContext::IncrementalEnvelopeContext(
    rbf::Robot robot,
    rbf::EndpointSourceConfig endpoint_config,
    rbf::EnvelopeTypeConfig envelope_config)
    : robot_(std::move(robot))
    , endpoint_config_(endpoint_config)
    , envelope_config_(envelope_config)
{}

void IncrementalEnvelopeContext::reset() {
    fk_state_ = rbf::FKState{};
    crit_state_.reset();
    aa_fk_state_ = rbf::AaFkPrefixState{};
    hifk_aa_state_ = rbf::HifkAaState{};
    last_intervals_.clear();
}

int IncrementalEnvelopeContext::infer_changed_dim(
    const std::vector<rbf::Interval>& intervals) const
{
    if (last_intervals_.size() != intervals.size()) {
        return -1;
    }

    int changed = -2;
    for (int i = 0; i < static_cast<int>(intervals.size()); ++i) {
        if (same_interval(last_intervals_[i], intervals[i])) continue;
        if (changed != -2) return -1;
        changed = i;
    }
    return changed;
}

rbf::EndpointIAABBResult IncrementalEnvelopeContext::endpoint_from_current_fk() const {
    rbf::EndpointIAABBResult result;
    result.source = endpoint_config_.source;
    result.is_safe = rbf::endpoint_safety_is_certified(
        rbf::endpoint_source_default_safety(endpoint_config_.source));
    result.safety_level = rbf::endpoint_source_default_safety(endpoint_config_.source);
    result.n_active_links = robot_.n_active_links();
    result.fk_state = fk_state_;
    result.endpoint_iaabbs.resize(result.endpoint_iaabb_len());
    rbf::extract_endpoint_iaabbs(
        fk_state_,
        robot_.active_link_map(),
        robot_.n_active_links(),
        result.endpoint_iaabbs.data());
    return result;
}

rbf::EndpointIAABBResult IncrementalEnvelopeContext::endpoint_from_crit_cache() const {
    rbf::EndpointIAABBResult result;
    result.source = rbf::EndpointSource::CritSample;
    result.is_safe = false;
    result.safety_level = rbf::EndpointSafetyLevel::UnsafeHeuristic;
    result.n_active_links = crit_state_.n_active_links;
    result.endpoint_iaabbs = crit_state_.endpoint_iaabbs;
    result.fk_state = fk_state_;
    result.combo_count = crit_state_.combo_count;
    result.enumerate_threads = 0;
    result.enumerate_time_us = 0.0;
    result.parallel_min_combos_used = crit_state_.parallel_min_combos_used;
    result.enumerate_chunk_size = crit_state_.enumerate_chunk_size;
    result.enumerate_chunk_count = crit_state_.enumerate_chunk_count;
    result.candidate_dirty_count = 0;
    result.predh_rebuild_count = 0;
    result.endpoint_cache_reused = true;
    return result;
}

IncrementalEnvelopeResult IncrementalEnvelopeContext::compute(
    const std::vector<rbf::Interval>& intervals,
    int changed_dim)
{
    IncrementalEnvelopeResult out;
    int effective_changed_dim = changed_dim;
    if (effective_changed_dim < 0) {
        effective_changed_dim = infer_changed_dim(intervals);
    } else if (endpoint_config_.source == rbf::EndpointSource::CritSample &&
               !differs_only_in_dim(last_intervals_, intervals, effective_changed_dim)) {
        reset();
        effective_changed_dim = -1;
    }
    out.changed_dim = effective_changed_dim == -2 ? -1 : effective_changed_dim;

    const bool can_use_incremental_crit =
        endpoint_config_.source == rbf::EndpointSource::CritSample &&
        crit_state_.valid &&
        effective_changed_dim >= 0 &&
        effective_changed_dim < static_cast<int>(intervals.size());

    const bool can_reuse_crit_cache =
        endpoint_config_.source == rbf::EndpointSource::CritSample &&
        crit_state_.valid &&
        crit_state_.endpoint_valid &&
        effective_changed_dim == -2;

    const auto endpoint_start = Clock::now();
    if (can_reuse_crit_cache) {
        out.endpoint = endpoint_from_crit_cache();
        out.reused_endpoint_cache = true;
    } else if (endpoint_config_.source == rbf::EndpointSource::CritSample) {
        out.endpoint = rbf::compute_endpoint_iaabb_crit_incremental(
            robot_,
            intervals,
            endpoint_config_.n_samples_crit,
            42,
            can_use_incremental_crit ? effective_changed_dim : -1,
            &fk_state_,
            &crit_state_,
            endpoint_config_.n_threads,
            endpoint_config_.parallel_min_combos);
        out.used_source_incremental_state = can_use_incremental_crit;
    } else if (endpoint_config_.source == rbf::EndpointSource::IFK) {
        out.endpoint = rbf::compute_endpoint_iaabb_ifk_aa_stateful(
            robot_, intervals, aa_fk_state_, &out.used_source_incremental_state);
    } else if (endpoint_config_.source == rbf::EndpointSource::HIFK) {
        out.endpoint = rbf::compute_endpoint_iaabb_hifk_aa_stateful(
            robot_, intervals, endpoint_config_, hifk_aa_state_, &out.used_source_incremental_state);
    } else {
        out.endpoint = rbf::compute_endpoint_iaabb(
            robot_,
            intervals,
            endpoint_config_,
            &fk_state_,
            -1);
        out.used_incremental_fk = false;
    }
    const auto endpoint_stop = Clock::now();

    const auto envelope_start = Clock::now();
    out.envelope = rbf::compute_link_envelope(
        out.endpoint.endpoint_iaabbs.data(),
        out.endpoint.n_active_links,
        robot_.active_link_radii(),
        envelope_config_);
    const auto envelope_stop = Clock::now();

    out.endpoint_time_us = std::chrono::duration<double, std::micro>(
        endpoint_stop - endpoint_start).count();
    out.envelope_time_us = std::chrono::duration<double, std::micro>(
        envelope_stop - envelope_start).count();
    last_intervals_ = intervals;
    return out;
}

}  // namespace link_interval_envelope
