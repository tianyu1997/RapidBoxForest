#include <link_interval_envelope/batch.h>

#include <sbf/core/fk_state.h>
#include <sbf/envelope/crit_source.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace link_interval_envelope {

namespace {

using Clock = std::chrono::steady_clock;

bool same_interval(const rbf::Interval& a, const rbf::Interval& b) {
    return a.lo == b.lo && a.hi == b.hi;
}

int infer_changed_dim(const std::vector<rbf::Interval>& previous,
                      const std::vector<rbf::Interval>& current) {
    if (previous.size() != current.size()) return -1;
    int changed = -2;
    for (int i = 0; i < static_cast<int>(current.size()); ++i) {
        if (same_interval(previous[static_cast<std::size_t>(i)], current[static_cast<std::size_t>(i)])) {
            continue;
        }
        if (changed != -2) return -1;
        changed = i;
    }
    return changed;
}

struct WorkerCritCache {
    rbf::FKState fk;
    rbf::CritSampleState state;
    std::vector<rbf::Interval> previous_intervals;
};

EnvelopeBatchResult compute_one_box(
    const rbf::Robot& robot,
    const std::vector<rbf::Interval>& intervals,
    const rbf::EndpointSourceConfig& endpoint_config,
    const rbf::EnvelopeTypeConfig& envelope_config,
    WorkerCritCache* crit_cache,
    bool nested_parallelism_suppressed)
{
    EnvelopeBatchResult out;

    const auto endpoint_start = Clock::now();
    rbf::EndpointIAABBResult endpoint;
    if (endpoint_config.source == rbf::EndpointSource::CritSample && crit_cache != nullptr) {
        const int changed_dim = infer_changed_dim(crit_cache->previous_intervals, intervals);
        endpoint = rbf::compute_endpoint_iaabb_crit_incremental(
            robot,
            intervals,
            endpoint_config.n_samples_crit,
            42,
            changed_dim >= 0 ? changed_dim : -1,
            &crit_cache->fk,
            &crit_cache->state,
            endpoint_config.n_threads,
            endpoint_config.parallel_min_combos);
        crit_cache->previous_intervals = intervals;
    } else {
        endpoint = rbf::compute_endpoint_iaabb(
            robot,
            intervals,
            endpoint_config,
            nullptr,
            -1);
    }
    const auto endpoint_stop = Clock::now();

    const auto envelope_start = Clock::now();
    out.envelope = rbf::compute_link_envelope(
        endpoint.endpoint_iaabbs.data(),
        endpoint.n_active_links,
        robot.active_link_radii(),
        envelope_config);
    const auto envelope_stop = Clock::now();

    out.source = endpoint.source;
    out.is_safe = endpoint.is_safe;
    out.n_active_links = endpoint.n_active_links;
    out.n_pruned_links = endpoint.n_pruned_links;
    out.combo_count = endpoint.combo_count;
    out.enumerate_threads = endpoint.enumerate_threads;
    out.enumerate_time_us = endpoint.enumerate_time_us;
    out.changed_dim = endpoint.changed_dim;
    out.parallel_min_combos_used = endpoint.parallel_min_combos_used;
    out.enumerate_chunk_size = endpoint.enumerate_chunk_size;
    out.enumerate_chunk_count = endpoint.enumerate_chunk_count;
    out.candidate_dirty_count = endpoint.candidate_dirty_count;
    out.predh_rebuild_count = endpoint.predh_rebuild_count;
    out.endpoint_cache_reused = endpoint.endpoint_cache_reused;
    out.endpoint_safety_level = endpoint.safety_level;
    out.nested_parallelism_suppressed = nested_parallelism_suppressed;
    out.endpoint_iaabbs = std::move(endpoint.endpoint_iaabbs);
    out.endpoint_time_us = std::chrono::duration<double, std::micro>(
        endpoint_stop - endpoint_start).count();
    out.envelope_time_us = std::chrono::duration<double, std::micro>(
        envelope_stop - envelope_start).count();
    return out;
}

}  // namespace

int resolve_thread_count(int requested_threads, int n_items) {
    if (n_items <= 0) return 1;
    if (requested_threads > 0) {
        return std::max(1, std::min(requested_threads, n_items));
    }
    const unsigned hw = std::thread::hardware_concurrency();
    const int available = hw == 0 ? 1 : static_cast<int>(hw);
    return std::max(1, std::min({available, n_items, 8}));
}

std::vector<EnvelopeBatchResult> compute_envelope_batch(
    const rbf::Robot& robot,
    const std::vector<std::vector<rbf::Interval>>& interval_boxes,
    const rbf::EndpointSourceConfig& endpoint_config,
    const rbf::EnvelopeTypeConfig& envelope_config,
    int n_threads)
{
    if (endpoint_config.source != rbf::EndpointSource::IFK &&
        endpoint_config.source != rbf::EndpointSource::CritSample &&
        endpoint_config.source != rbf::EndpointSource::HIFK) {
        throw std::invalid_argument(
            "compute_envelope_batch currently supports IFK, CritSample, and HIFK only");
    }

    const int n_items = static_cast<int>(interval_boxes.size());
    std::vector<EnvelopeBatchResult> results(static_cast<std::size_t>(n_items));
    if (n_items == 0) return results;

    const int workers = resolve_thread_count(n_threads, n_items);
    rbf::EndpointSourceConfig worker_endpoint_config = endpoint_config;
    const bool suppress_nested_parallelism =
        workers > 1 &&
        endpoint_config.source == rbf::EndpointSource::CritSample &&
        endpoint_config.n_threads != 1;
    if (suppress_nested_parallelism) {
        worker_endpoint_config.n_threads = 1;
    }
    std::atomic<int> next{0};
    std::exception_ptr first_error;
    std::mutex error_mutex;

    auto run_worker = [&]() {
        WorkerCritCache crit_cache;
        while (true) {
            const int idx = next.fetch_add(1);
            if (idx >= n_items) break;
            try {
                results[static_cast<std::size_t>(idx)] = compute_one_box(
                    robot,
                    interval_boxes[static_cast<std::size_t>(idx)],
                    worker_endpoint_config,
                    envelope_config,
                    endpoint_config.source == rbf::EndpointSource::CritSample ? &crit_cache : nullptr,
                    suppress_nested_parallelism);
            } catch (...) {
                std::lock_guard<std::mutex> lock(error_mutex);
                if (!first_error) first_error = std::current_exception();
            }
        }
    };

    if (workers == 1) {
        run_worker();
    } else {
        std::vector<std::thread> threads;
        threads.reserve(static_cast<std::size_t>(workers));
        for (int i = 0; i < workers; ++i) {
            threads.emplace_back(run_worker);
        }
        for (auto& thread : threads) thread.join();
    }

    if (first_error) std::rethrow_exception(first_error);
    return results;
}

}  // namespace link_interval_envelope