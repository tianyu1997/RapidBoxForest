#pragma once
/// @file batch.h
/// @brief Parallel batch envelope computation for supported endpoint sources.

#include <link_interval_envelope/endpoint.h>
#include <link_interval_envelope/envelope.h>
#include <link_interval_envelope/robot.h>
#include <link_interval_envelope/types.h>

#include <vector>

namespace link_interval_envelope {

struct EnvelopeBatchResult {
    rbf::EndpointSource source = rbf::EndpointSource::IFK;
    bool is_safe = true;
    int n_active_links = 0;
    int n_pruned_links = 0;
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
    rbf::EndpointSafetyLevel endpoint_safety_level = rbf::EndpointSafetyLevel::UnsafeHeuristic;
    bool nested_parallelism_suppressed = false;
    std::vector<float> endpoint_iaabbs;
    rbf::LinkEnvelope envelope;
    double endpoint_time_us = 0.0;
    double envelope_time_us = 0.0;
};

int resolve_thread_count(int requested_threads, int n_items);

std::vector<EnvelopeBatchResult> compute_envelope_batch(
    const rbf::Robot& robot,
    const std::vector<std::vector<rbf::Interval>>& interval_boxes,
    const rbf::EndpointSourceConfig& endpoint_config,
    const rbf::EnvelopeTypeConfig& envelope_config,
    int n_threads = 0);

}  // namespace link_interval_envelope
