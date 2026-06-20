#include <LECTDatabase/sbf/oracle.h>

#include "oracle_canonical.h"
#include "oracle_endpoint_materialization.h"
#include "oracle_impl.h"
#include "oracle_support.h"

#include <sbf/envelope/crit_source.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/ifk_aa_source.h>

#include <chrono>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace rbf {
namespace {

using Clock = std::chrono::steady_clock;

lect_database::NodeId to_database_node(OracleNodeId node) {
    if (node < 0 || static_cast<std::uint64_t>(node) > std::numeric_limits<lect_database::NodeId>::max()) {
        return lect_database::kInvalidNodeId;
    }
    return static_cast<lect_database::NodeId>(node);
}

}  // namespace

lect_database::EvidenceKey DatabaseBoxOracle::endpoint_key(OracleNodeId node) const {
    lect_database::EvidenceKey key;
    key.node_id = to_database_node(node);
    const auto topology = database_.topology(key.node_id);
    key.node_path = topology.path;
    key.node_path_valid = lect_database::valid_node_id(topology.id);
    key.sector = lect_database::kPrimarySector;
    key.channel = database_channel_for_endpoint(endpoint_config_.source);
    key.endpoint_source = endpoint_config_.source;
    key.payload_kind = lect_database::EvidencePayloadKind::EndpointEnvelope;
    return key;
}

std::optional<DatabaseBoxOracle::EndpointPayload> DatabaseBoxOracle::endpoint_payload_for_node(
    OracleNodeId node,
    const std::vector<Interval>& intervals,
    int changed_dim,
    bool allow_external_evidence) {
    const auto key = endpoint_key(node);
    const auto evidence_frame = canonical_evidence_frame_for_intervals(robot_, database_, intervals);
    const int normalized_sector = normalize_sector(evidence_frame.sector);
    const std::uint64_t envelope_cache_key = make_envelope_cache_key(key.node_id, normalized_sector);
    if (!evidence_frame.valid ||
        (evidence_frame.symmetry &&
         intervals_straddle_sector_boundary(*evidence_frame.symmetry, intervals))) {
        counters_.canonical_frame_invalid += 1;
        throw std::runtime_error(
            "canonical evidence frame is invalid: native interval cannot be represented as one canonical sector");
    }
    auto reflect_payload = [&](EndpointPayload payload) -> EndpointPayload {
        if (!evidence_frame.symmetry || normalize_sector(evidence_frame.sector) == 0 || payload.payload.empty()) {
            return payload;
        }
        const int n_endpoint_boxes = static_cast<int>(payload.payload.size() / 6u);
        if (n_endpoint_boxes <= 0 || payload.payload.size() != static_cast<std::size_t>(n_endpoint_boxes) * 6u) {
            return payload;
        }
        EndpointPayload reflected;
        reflected.owned_payload.resize(payload.payload.size());
        evidence_frame.symmetry->transform_all_endpoint_iaabbs(
            payload.payload.data(),
            n_endpoint_boxes,
            normalize_sector(evidence_frame.sector),
            reflected.owned_payload.data());
        reflected.payload = reflected.owned_payload;
        reflected.envelope_cache_key = payload.envelope_cache_key;
        reflected.envelope_cacheable = payload.envelope_cacheable;
        reflected.reused_external_evidence = payload.reused_external_evidence;
        return reflected;
    };
    const bool use_endpoint_evidence_cache = validation_config_.enable_endpoint_evidence_cache;
    const bool store_endpoint_evidence_cache =
        use_endpoint_evidence_cache && validation_config_.store_endpoint_evidence_cache;
    const bool read_local_endpoint_cache =
        use_endpoint_evidence_cache && (online_cache_ != nullptr || store_endpoint_evidence_cache);
    const bool external_evidence_allowed =
        allow_external_evidence &&
        validation_config_.external_evidence_materialization &&
        external_evidence_source_ != nullptr;
    auto lookup_external_payload = [&]() -> std::optional<EndpointPayload> {
        if (!external_evidence_allowed) {
            return std::nullopt;
        }
        const auto external_lookup_start = Clock::now();
        std::optional<lect_database::EvidenceRecordView> cached;
        bool direct_external_checked = false;
        bool direct_external_compatible = false;
        if (direct_external_evidence_database_ != nullptr) {
            direct_external_checked = true;
            counters_.interval_replay_compatibility_checks += 1;
            const auto compatibility =
                interval_evidence_compatibility(database_, direct_external_evidence_database_, evidence_frame);
            direct_external_compatible = compatibility.compatible;
            if (direct_external_compatible) {
                counters_.interval_replay_compatible += 1;
            } else {
                counters_.interval_replay_incompatible += 1;
                counters_.interval_replay_key_only_blocked += 1;
            }
        }
        if (direct_external_compatible) {
            std::lock_guard<std::mutex> lock(external_direct_lookup_mutex());
            cached = direct_external_evidence_database_->endpoint_for_box_exact(
                direct_external_evidence_database_->make_box_key(evidence_frame.lookup_intervals),
                key);
            if (cached) {
                counters_.interval_replay_direct_exact_hits += 1;
            }
        }
        const bool allow_source_lookup =
            !direct_external_checked || direct_external_compatible;
        if (!cached && allow_source_lookup) {
            cached = external_evidence_source_->endpoint_for_box_exact(evidence_frame.lookup_intervals, key);
        }
        counters_.materialization_external_lookup_time_us += elapsed_us(external_lookup_start);
        if (!cached) {
            counters_.materialization_external_exact_misses += 1;
            return std::nullopt;
        }
        counters_.materialization_external_exact_hits += 1;
        counters_.materialization_reused_external_evidence += 1;
        last_validation_detail_.reused_external_evidence = true;
        if (validation_config_.external_evidence_backfill_active && store_endpoint_evidence_cache) {
            lect_database::EvidenceRecord backfill;
            backfill.key = key;
            backfill.child_hull = cached->child_hull;
            backfill.unavailable = cached->unavailable;
            backfill.payload.assign(cached->payload.begin(), cached->payload.end());
            if (online_cache_ != nullptr) {
                online_cache_->put_evidence(backfill);
            } else {
                database_.put_evidence(std::move(backfill));
            }
        }
        const auto read_start = Clock::now();
        EndpointPayload payload;
        payload.record_storage = cached->storage;
        payload.storage_owner = cached->storage_owner;
        payload.payload = cached->payload;
        payload.envelope_cache_key = make_envelope_cache_key(cached->key, normalized_sector);
        payload.envelope_cacheable = true;
        payload.reused_external_evidence = true;
        counters_.materialization_external_read_time_us += elapsed_us(read_start);
        return reflect_payload(std::move(payload));
    };
    const bool prefer_external_first =
        external_evidence_allowed &&
        !validation_config_.external_evidence_backfill_active;
    if (prefer_external_first) {
        if (auto payload = lookup_external_payload()) {
            return payload;
        }
    }
    if (read_local_endpoint_cache) {
        if (online_cache_ != nullptr) {
            const auto lookup_start = Clock::now();
            auto cached = online_cache_->evidence(key);
            counters_.materialization_cache_lookup_time_us += elapsed_us(lookup_start);
            if (cached) {
                const auto read_start = Clock::now();
                EndpointPayload payload;
                payload.owned_payload = std::move(cached->payload);
                payload.payload = payload.owned_payload;
                payload.envelope_cache_key = make_envelope_cache_key(cached->key, normalized_sector);
                payload.envelope_cacheable = true;
                counters_.materialization_cache_read_time_us += elapsed_us(read_start);
                counters_.materialization_reused_endpoint_cache += 1;
                return reflect_payload(std::move(payload));
            }
        }
        const auto database_lookup_start = Clock::now();
        auto local_cached = database_.evidence(key);
        counters_.materialization_cache_lookup_time_us += elapsed_us(database_lookup_start);
        if (local_cached) {
            const auto read_start = Clock::now();
            EndpointPayload payload;
            payload.record_storage = local_cached->storage;
            payload.storage_owner = local_cached->storage_owner;
            payload.payload = local_cached->payload;
            payload.envelope_cache_key = make_envelope_cache_key(local_cached->key, normalized_sector);
            payload.envelope_cacheable = true;
            counters_.materialization_cache_read_time_us += elapsed_us(read_start);
            counters_.materialization_reused_endpoint_cache += 1;
            return reflect_payload(std::move(payload));
        }
    } else {
        counters_.materialization_skipped_endpoint_cache += 1;
    }
    if (!prefer_external_first) {
        if (auto payload = lookup_external_payload()) {
            return payload;
        }
    }
    if (external_evidence_allowed) {
        counters_.materialization_external_live_fallbacks += 1;
    }

    const bool use_shared_endpoint_cache =
        shared_endpoint_cache_ != nullptr &&
        validation_config_.enable_endpoint_evidence_cache &&
        validation_config_.enable_worker_shared_endpoint_cache;
    if (use_shared_endpoint_cache) {
        const auto shared_lookup_start = Clock::now();
        auto shared_cached =
            shared_endpoint_cache_->endpoint_for_box_exact(evidence_frame.lookup_intervals, key);
        counters_.materialization_cache_lookup_time_us += elapsed_us(shared_lookup_start);
        if (shared_cached) {
            counters_.materialization_reused_shared_endpoint_cache += 1;
            last_validation_detail_.reused_endpoint_cache = true;
            EndpointPayload payload;
            payload.record_storage = shared_cached->storage;
            payload.storage_owner = shared_cached->storage_owner;
            payload.payload = shared_cached->payload;
            payload.envelope_cache_key = make_envelope_cache_key(shared_cached->key, normalized_sector);
            payload.envelope_cacheable = true;
            return reflect_payload(std::move(payload));
        }
    }

    const auto endpoint_start = Clock::now();
    EndpointSourceConfig materialization_config = hifk_config_for_materialization(*this, node, endpoint_config_);
    bool used_source_incremental_state = false;
    EndpointIAABBResult endpoint;
    if (!validation_config_.stateless_materialization_context &&
        materialization_config.source == EndpointSource::IFK) {
        endpoint = compute_endpoint_iaabb_ifk_aa_stateful(
            robot_, evidence_frame.lookup_intervals, impl_->aa_fk_prefix_state, &used_source_incremental_state);
    } else if (!validation_config_.stateless_materialization_context &&
               materialization_config.source == EndpointSource::CritSample) {
        const bool can_use_incremental_crit =
            impl_->crit_sample_state.valid &&
            differs_only_in_dim_exact(impl_->crit_sample_state.intervals,
                                      evidence_frame.lookup_intervals,
                                      changed_dim);
        endpoint = compute_endpoint_iaabb_crit_incremental(
            robot_,
            evidence_frame.lookup_intervals,
            materialization_config.n_samples_crit,
            42,
            can_use_incremental_crit ? changed_dim : -1,
            nullptr,
            &impl_->crit_sample_state,
            materialization_config.n_threads,
            materialization_config.parallel_min_combos);
        used_source_incremental_state = can_use_incremental_crit;
    } else if (!validation_config_.stateless_materialization_context &&
               materialization_config.source == EndpointSource::HIFK) {
        endpoint = compute_endpoint_iaabb_hifk_aa_stateful(
            robot_, evidence_frame.lookup_intervals, materialization_config, impl_->hifk_aa_state, &used_source_incremental_state);
    } else {
        endpoint = compute_endpoint_iaabb(
            robot_, evidence_frame.lookup_intervals, materialization_config, nullptr, changed_dim);
    }
    const bool used_incremental_fk =
        used_source_incremental_state &&
        (materialization_config.source == EndpointSource::IFK || materialization_config.source == EndpointSource::HIFK);
    last_validation_detail_.changed_dim = changed_dim;
    last_validation_detail_.used_incremental_fk = used_incremental_fk;
    last_validation_detail_.used_source_incremental_state = used_source_incremental_state;
    if (used_incremental_fk) {
        counters_.materialization_incremental_fk += 1;
    }
    if (used_source_incremental_state) {
        counters_.materialization_source_incremental_state += 1;
    }
    counters_.materialization_candidate_dirty_count += endpoint.candidate_dirty_count;
    counters_.materialization_predh_rebuild_count += endpoint.predh_rebuild_count;
    if (endpoint.endpoint_cache_reused) {
        counters_.materialization_reused_endpoint_cache += 1;
    }
    counters_.materialization_endpoint_wall_time_us += elapsed_us(endpoint_start);
    if (endpoint.endpoint_iaabbs.empty()) {
        return std::nullopt;
    }
    EndpointPayload payload;
    payload.owned_payload = std::move(endpoint.endpoint_iaabbs);
    payload.payload = payload.owned_payload;
    payload.envelope_cache_key = envelope_cache_key;
    payload.envelope_cacheable = true;
    lect_database::EvidenceRecord record;
    record.key = key;
    record.payload = payload.owned_payload;
    record.child_hull = false;
    if (store_endpoint_evidence_cache) {
        if (online_cache_ != nullptr) {
            online_cache_->put_evidence(record);
        } else {
            database_.put_evidence(record);
        }
    }
    if (use_shared_endpoint_cache) {
        shared_endpoint_cache_->put(evidence_frame.lookup_intervals, key,
                                    payload.owned_payload, /*child_hull=*/false,
                                    /*unavailable=*/false);
        counters_.materialization_stored_shared_endpoint_cache += 1;
    }
    counters_.materializations += 1;
    if (store_endpoint_evidence_cache) {
        counters_.materialization_stored_endpoint += 1;
    }
    counters_.materialization_endpoint_time_us += endpoint.enumerate_time_us;
    return reflect_payload(std::move(payload));
}

}  // namespace rbf
