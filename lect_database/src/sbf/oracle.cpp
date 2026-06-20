#include <LECTDatabase/sbf/oracle.h>

#include "oracle_best_tighten.h"
#include "oracle_canonical.h"
#include "oracle_endpoint_materialization.h"
#include "oracle_material_point.h"
#include "oracle_options.h"
#include "oracle_support.h"

#include <sbf/core/joint_symmetry.h>
#include <sbf/envelope/crit_source.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/ifk_aa_source.h>

#include <link_interval_envelope/incremental_context.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>

namespace rbf {
namespace {

using lect_database::NodeId;
using Clock = std::chrono::steady_clock;

lect_database::NodeId to_database_node(OracleNodeId node) {
    if (node < 0 || static_cast<std::uint64_t>(node) > std::numeric_limits<lect_database::NodeId>::max()) {
        return lect_database::kInvalidNodeId;
    }
    return static_cast<lect_database::NodeId>(node);
}

OracleNodeId from_database_node(lect_database::NodeId node) {
    return lect_database::valid_node_id(node) ? static_cast<OracleNodeId>(node) : kInvalidOracleNodeId;
}

}  // namespace

struct DatabaseBoxOracle::Impl {
    // Incremental AA-backed endpoint state reused along a single-threaded
    // parent->child descent. IFK reuses a single AA-FK chain prefix; HIFK
    // reuses per-leaf AA-FK states when the split schedule is deterministic.
    // Not shared across threads (each worker owns its own oracle).
    AaFkPrefixState aa_fk_prefix_state;
    CritSampleState crit_sample_state;
    HifkAaState hifk_aa_state;
};

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                                     lect_database::LectDatabase& database,
                                     Scene scene,
                                     EndpointSourceConfig endpoint_config,
                                     EnvelopeTypeConfig envelope_config,
                         OracleValidationConfig validation_config,
                                                                                                                                                 const lect_database::LectExternalEvidenceSource* external_evidence_source,
                                                                                                                                                 const lect_database::LectDatabase* direct_external_evidence_database)
    : robot_(std::move(robot)),
      database_(database),
        external_evidence_source_(external_evidence_source),
        direct_external_evidence_database_(direct_external_evidence_database),
      endpoint_config_(std::move(endpoint_config)),
      envelope_config_(std::move(envelope_config)),
      validation_config_(std::move(validation_config)),
      scene_(std::move(scene)),
      checker_(robot_, scene_),
      impl_(std::make_unique<Impl>()) {
    seed_best_tighten_schedule_from_policy();
}

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                         lect_database::OnlineEnvelopeCacheTree& online_cache,
                         Scene scene,
                         EndpointSourceConfig endpoint_config,
                         EnvelopeTypeConfig envelope_config,
                         OracleValidationConfig validation_config,
                                                                         const lect_database::LectExternalEvidenceSource* external_evidence_source,
                                                                         const lect_database::LectDatabase* direct_external_evidence_database)
    : robot_(std::move(robot)),
    database_(online_cache.database()),
    online_cache_(&online_cache),
    external_evidence_source_(external_evidence_source),
    direct_external_evidence_database_(direct_external_evidence_database),
    endpoint_config_(std::move(endpoint_config)),
    envelope_config_(std::move(envelope_config)),
    validation_config_(std::move(validation_config)),
    scene_(std::move(scene)),
    checker_(robot_, scene_),
    impl_(std::make_unique<Impl>()) {
    seed_best_tighten_schedule_from_policy();
}

DatabaseBoxOracle::~DatabaseBoxOracle() = default;

void DatabaseBoxOracle::seed_best_tighten_schedule_from_policy() {
    // Pre-seed the per-depth best-tighten split dimensions from the canonical
    // FixedDepthSchedule so the grower replays a (robot, domain)-determined
    // dimension sequence instead of lazily fixing it from the first query that
    // reaches each depth. This keeps node paths canonical (and therefore the
    // external-evidence keys stable / reusable) regardless of seed or query
    // order. Only applies to the AAFKVolumeMin FixedDepthSchedule strategy.
    const auto& policy = database_.split_policy_descriptor();
    if (policy.strategy != lect_database::SplitStrategy::AAFKVolumeMin) {
        return;
    }
    if (policy.depth_dimensions.empty()) {
        return;
    }
    best_tighten_depth_dims_ = policy.depth_dimensions;
}

SplitNodeResult DatabaseBoxOracle::split_node(OracleNodeId node,
                                              const std::vector<Interval>& intervals,
                                              int changed_dim,
                                              const OracleSplitOptions& options) {
    SplitNodeResult result;
    std::pair<lect_database::NodeId, lect_database::NodeId> children{
        lect_database::kInvalidNodeId,
        lect_database::kInvalidNodeId,
    };

    if (options.use_best_tighten && !intervals.empty()) {
        auto best_tighten_options = with_fk_effective_split_filter(options.best_tighten, robot_);
        try {
            const auto scoring_endpoint_config = hifk_config_for_materialization(*this, node, endpoint_config_);
            link_interval_envelope::IncrementalEnvelopeContext probe(
                robot_, scoring_endpoint_config, envelope_config_);
            int next_changed_dim_hint = changed_dim;
            if (best_tighten_reference_volumes_.empty()) {
                link_interval_envelope::IncrementalEnvelopeContext reference_probe(
                    robot_, endpoint_config_, envelope_config_);
                const auto reference = reference_probe.compute(robot_.joint_limits().limits, -1);
                best_tighten_reference_volumes_ = link_aabb_volumes(reference.envelope.link_iaabbs);
            }
            const auto symmetry = best_tighten_options.prefer_sector_boundary
                ? primary_database_symmetry(robot_, database_)
                : std::nullopt;
            const auto scorer = [this, &probe, &next_changed_dim_hint](const std::vector<Interval>& candidate_intervals) {
                next_changed_dim_hint = -1;
                // Best-tighten scores arbitrary candidate child intervals that jump
                // around the tree (parent -> left -> right -> next-dim left ...). The
                // incremental endpoint state (notably the heuristic CritSample source)
                // is only valid for single-dim diffs branching from the *same* parent,
                // not for these arbitrary jumps, which corrupts the scored envelope and
                // makes children appear larger than their parent. Force a full,
                // stateless recompute per candidate so the minimax scores are correct.
                probe.reset();
                const auto scored = probe.compute(candidate_intervals, -1);
                counters_.scoring_evaluations += 1;
                counters_.scoring_endpoint_time_us += scored.endpoint_time_us;
                counters_.scoring_envelope_time_us += scored.envelope_time_us;
                counters_.scoring_candidate_dirty_count += scored.endpoint.candidate_dirty_count;
                counters_.scoring_predh_rebuild_count += scored.endpoint.predh_rebuild_count;
                if (scored.changed_dim >= 0) {
                    counters_.scoring_changed_dim_inferred += 1;
                }
                if (scored.used_incremental_fk) {
                    counters_.scoring_incremental_fk += 1;
                }
                if (scored.used_source_incremental_state) {
                    counters_.scoring_source_incremental_state += 1;
                }
                if (scored.reused_fk) {
                    counters_.scoring_reused_fk += 1;
                }
                if (scored.reused_endpoint_cache) {
                    counters_.scoring_reused_endpoint_cache += 1;
                }
                return normalized_link_aabb_volume_score(scored.envelope.link_iaabbs,
                                                         best_tighten_reference_volumes_);
            };

            const auto candidate = choose_best_tighten_split(intervals,
                                                             depth(node),
                                                             scorer,
                                                             symmetry,
                                                             best_tighten_options,
                                                             best_tighten_depth_dims_,
                                                             best_tighten_recent_dims_);
            children = database_.split_leaf(to_database_node(node), candidate.dim, candidate.split_val);
            if (oracle_best_tighten_debug_enabled()) {
                static std::atomic<long> ok{0};
                long n = ++ok;
                if (n <= 20 || n % 200 == 0) {
                    std::fprintf(stderr,
                                 "[BT_OK] n=%ld dim=%d val=%.5f valid_children=%d\n",
                                 n, candidate.dim, candidate.split_val,
                                 (int)(lect_database::valid_node_id(children.first) &&
                                       lect_database::valid_node_id(children.second)));
                }
            }
        } catch (const std::exception& e) {
            if (oracle_best_tighten_debug_enabled()) {
                static std::atomic<long> bad{0};
                long n = ++bad;
                if (n <= 20 || n % 200 == 0) {
                    std::fprintf(stderr, "[BT_EXC] n=%ld what=%s\n", n, e.what());
                }
            }
        }
    }

    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        children = online_cache_ != nullptr
            ? online_cache_->split_leaf(to_database_node(node))
            : database_.split_leaf(to_database_node(node));
    }
    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        return result;
    }
    const auto topology = database_.topology(to_database_node(node));
    result.split = true;
    result.node = node;
    result.left = from_database_node(children.first);
    result.right = from_database_node(children.second);
    result.split_dim = topology.split_dim;
    result.split_value = topology.split_value;
    unexplored_leaf_cache_dirty_ = true;
    return result;
}

SplitNodeResult DatabaseBoxOracle::split_node_at(OracleNodeId node, int split_dim, double split_value) {
    SplitNodeResult result;
    const auto children = database_.split_leaf(to_database_node(node), split_dim, split_value);
    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        return result;
    }
    const auto topology = database_.topology(to_database_node(node));
    result.split = true;
    result.node = node;
    result.left = from_database_node(children.first);
    result.right = from_database_node(children.second);
    result.split_dim = topology.split_dim;
    result.split_value = topology.split_value;
    unexplored_leaf_cache_dirty_ = true;
    return result;
}

bool DatabaseBoxOracle::point_in_collision(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    return checker_.check_config(q);
}

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

    // Cross-task shared endpoint cache (interval-keyed, thread-safe). Lets a
    // worker reuse an endpoint another worker (or an earlier batch) already
    // computed for the identical canonical box, the way a persistent oracle
    // reuses evidence across queries.
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

BoxValidation DatabaseBoxOracle::classify_payload(OracleNodeId node,
                                                  const std::vector<Interval>& intervals,
                                                  const EndpointPayload& endpoint_payload) {
    (void)intervals;
    const auto envelope_read_start = Clock::now();
    const LinkEnvelope* envelope = nullptr;
    const bool use_envelope_cache = endpoint_payload.envelope_cacheable &&
        enable_envelope_cache_ &&
        !database_.bulk_prewarm_mode_enabled() &&
        !database_.streaming_prewarm_mode_enabled();
    const std::uint64_t envelope_cache_key = use_envelope_cache
        ? (endpoint_payload.envelope_cache_key ^ endpoint_payload_hash(endpoint_payload.payload))
        : endpoint_payload.envelope_cache_key;
    if (use_envelope_cache) {
        const auto cache_it = envelope_cache_.find(envelope_cache_key);
        counters_.materialization_envelope_read_time_us += elapsed_us(envelope_read_start);
        if (cache_it != envelope_cache_.end()) {
            counters_.materialization_reused_cached_envelope += 1;
            envelope = &cache_it->second;
        }
    } else {
        counters_.materialization_envelope_read_time_us += elapsed_us(envelope_read_start);
    }

    LinkEnvelope computed_envelope;
    double envelope_compute_us = 0.0;
    if (envelope == nullptr) {
        const auto envelope_start = Clock::now();
        computed_envelope = compute_link_envelope(endpoint_payload.payload.data(),
                                                  robot_.n_active_links(),
                                                  robot_.active_link_radii(),
                                                  envelope_config_);
        envelope_compute_us = elapsed_us(envelope_start);
        counters_.materialization_envelope_compute_time_us += envelope_compute_us;
        if (use_envelope_cache) {
            auto [cache_it, inserted] = envelope_cache_.try_emplace(envelope_cache_key,
                                                                    std::move(computed_envelope));
            if (!inserted) {
                cache_it->second = std::move(computed_envelope);
            }
            envelope = &cache_it->second;
        } else {
            envelope = &computed_envelope;
        }
    }
    EnvelopeCollisionStats collision_stats;
    EnvelopeCollisionOptions collision_options;
    collision_options.safety_epsilon = std::max(envelope_config_.kdop_config.safety_epsilon,
                                                envelope_config_.support_hull_config.safety_epsilon);
    collision_options.overlap_tolerance = std::max(envelope_config_.kdop_config.overlap_tolerance,
                                                   envelope_config_.support_hull_config.overlap_tolerance);
    collision_options.skip_aabb_broadphase =
        envelope_config_.support_hull_config.skip_aabb_broadphase;
    collision_options.direct_support_hull_collision =
        envelope_config_.support_hull_config.direct_collision;
    collision_options.count_all_pairs = validation_config_.collect_full_overlap_stats;
    if (oracle_envelope_debug_enabled()) {
        std::fprintf(stderr,
            "[ENVDBG] cfg.type=%d n_sub=%d keep_kdop=%d || env.type=%d env.n_sub=%d env.n_active=%d link_iaabbs=%zu support_hulls=%zu kdop_intervals=%zu kdop_n_axes=%d\n",
            static_cast<int>(envelope_config_.type), envelope_config_.n_subdivisions,
            static_cast<int>(envelope_config_.support_hull_config.keep_kdop),
            static_cast<int>(envelope->type), envelope->n_subdivisions, envelope->n_active_links,
            envelope->link_iaabbs.size(), envelope->support_hulls.size(),
            envelope->kdop_intervals.size(), envelope->kdop_n_axes);
    }
    const auto collision_start = Clock::now();
    const CollisionResultKind collision = collide_envelope_aabbs(*envelope,
                                                                 scene_.obstacles().data(),
                                                                 scene_.n_obstacles(),
                                                                 collision_options,
                                                                 &collision_stats);
    const double collision_us = elapsed_us(collision_start);
    counters_.materialization_envelope_collision_time_us += collision_us;
    counters_.materialization_envelope_time_us += envelope_compute_us + collision_us;
    record_envelope_collision(counters_, collision_stats);
    counters_.envelope_collision_queries += 1;
    if (oracle_envelope_debug_enabled()) {
        EnvelopeCollisionStats all_stats;
        EnvelopeCollisionOptions all_opts = collision_options;
        all_opts.count_all_pairs = true;
        const CollisionResultKind all = collide_envelope_aabbs(*envelope,
            scene_.obstacles().data(), scene_.n_obstacles(), all_opts, &all_stats);
        std::fprintf(stderr,
            "[ENVDBG] result=%d(0=Free,1=Maybe) all=%d || aabb_tests=%lld aabb_rej=%lld link_tests=%lld link_rej=%lld kdop_tests=%lld kdop_rej=%lld gjk_tests=%lld gjk_rej=%lld maybe_pairs=%lld\n",
            static_cast<int>(collision), static_cast<int>(all),
            (long long)all_stats.envelope_aabb_tests, (long long)all_stats.envelope_aabb_rejects,
            (long long)all_stats.link_aabb_tests, (long long)all_stats.link_aabb_rejects,
            (long long)all_stats.kdop_tests, (long long)all_stats.kdop_rejects,
            (long long)all_stats.gjk_tests, (long long)all_stats.gjk_rejects,
            (long long)all_stats.maybe_pairs);
    }
    last_validation_detail_.node = node;
    last_validation_detail_.depth = depth(node);
    last_validation_detail_.mode = validation_config_.mode;
    last_validation_detail_.endpoint_source = endpoint_config_.source;
    last_validation_detail_.endpoint_safety_level = endpoint_source_default_safety(endpoint_config_.source);
    last_validation_detail_.endpoint_is_safe = endpoint_safety_is_certified(last_validation_detail_.endpoint_safety_level);
    last_validation_detail_.materialized = true;
    last_validation_detail_.aabb_overlap = collision != CollisionResultKind::DefinitelyFree;
    last_validation_detail_.aabb_overlap_depth = collision_stats.maybe_pair_overlap_depth_max;
    last_validation_detail_.aabb_overlap_volume_ratio = collision_stats.maybe_pair_overlap_volume_ratio_max;
    last_validation_detail_.blocker_active_link_index =
        collision_stats.dominant_blocker_active_link_index;
    last_validation_detail_.blocker_link_id =
        active_link_index_to_link_id(robot_, collision_stats.dominant_blocker_active_link_index);
    last_validation_detail_.blocker_obstacle_id =
        collision_stats.dominant_blocker_obstacle_index;
    last_validation_detail_.blocker_stage =
        collision_stats.dominant_blocker_stage;
    last_validation_detail_.blocker_margin =
        collision_stats.dominant_blocker_margin;
    last_validation_detail_.blocker_overlap_depth =
        collision_stats.dominant_blocker_overlap_depth;
    last_validation_detail_.blocker_overlap_volume_ratio =
        collision_stats.dominant_blocker_overlap_volume_ratio;
    last_validation_detail_.blocker_affected_joints =
        affected_joints_for_link(robot_, last_validation_detail_.blocker_link_id);
    last_validation_detail_.blockers =
        make_oracle_blockers(robot_, collision_stats);
    last_validation_detail_.blocker_signature_hash =
        blocker_signature_hash(last_validation_detail_.blockers);
    if (collision == CollisionResultKind::DefinitelyFree) {
        counters_.envelope_collision_free += 1;
        counters_.certified_free += 1;
        last_validation_detail_.validation = BoxValidation::Free;
        last_validation_detail_.safety_status = BoxSafetyStatus::CertifiedFree;
        last_validation_detail_.strict_audit_required = false;
        last_validation_detail_.collision_possible = false;
        return BoxValidation::Free;
    }
    last_validation_detail_.occupied_certificate_checked =
        validation_config_.occupied_certificate.enabled;
    if (validation_config_.occupied_certificate.enabled) {
        auto witness = try_material_point_occupied_witness(robot_,
                                                           intervals,
                                                           *envelope,
                                                           scene_,
                                                           validation_config_.occupied_certificate);
        if (witness && certifies_occupied(*witness)) {
            counters_.envelope_collision_maybe += 1;
            counters_.certified_occupied += 1;
            last_validation_detail_.validation = BoxValidation::Occupied;
            last_validation_detail_.safety_status = BoxSafetyStatus::Unknown;
            last_validation_detail_.strict_audit_required = false;
            last_validation_detail_.collision_possible = true;
            last_validation_detail_.occupied_certificate_found = true;
            last_validation_detail_.occupied_witness_link_id = witness->link_id;
            last_validation_detail_.occupied_witness_obstacle_id = witness->obstacle_id;
            last_validation_detail_.occupied_witness_center_signed_distance =
                witness->center_signed_distance;
            last_validation_detail_.occupied_witness_motion_bound = witness->motion_bound;
            return BoxValidation::Occupied;
        }
    }
    counters_.collision_possible += 1;
    counters_.envelope_collision_maybe += 1;
    last_validation_detail_.validation = BoxValidation::Unknown;
    last_validation_detail_.safety_status = BoxSafetyStatus::Unknown;
    last_validation_detail_.strict_audit_required = collision == CollisionResultKind::DefinitelyFree;
    last_validation_detail_.collision_possible = true;
    return BoxValidation::Unknown;
}

BoxValidation DatabaseBoxOracle::validate_node(OracleNodeId node,
                                               const std::vector<Interval>& intervals,
                                               int changed_dim) {
    const auto total_start = Clock::now();
    counters_.node_validations += 1;
    last_validation_detail_ = {};
    last_validation_detail_.node = node;
    last_validation_detail_.depth = depth(node);
    last_validation_detail_.mode = validation_config_.mode;
    const double preamble_us = elapsed_us(total_start);
    counters_.validate_node_preamble_time_us += preamble_us;
    if (scene_.empty()) {
        counters_.certified_free += 1;
        last_validation_detail_.validation = BoxValidation::Free;
        last_validation_detail_.safety_status = BoxSafetyStatus::CertifiedFree;
        last_validation_detail_.collision_possible = false;
        const double total_us = elapsed_us(total_start);
        counters_.validate_node_total_time_us += total_us;
        counters_.validate_node_overhead_time_us += std::max(0.0, total_us - preamble_us);
        return BoxValidation::Free;
    }
    const bool use_validation_cache =
        validation_config_.enable_validation_cache &&
        validation_config_.validation_cache_max_entries > 0;
    const std::uint64_t cache_key =
        use_validation_cache ? validation_cache_key(node, intervals, changed_dim) : 0;
    if (use_validation_cache) {
        const auto cache_it = validation_cache_.find(cache_key);
        if (cache_it != validation_cache_.end()) {
            counters_.validation_cache_hits += 1;
            last_validation_detail_ = cache_it->second.detail;
            const double total_us = elapsed_us(total_start);
            counters_.validate_node_total_time_us += total_us;
            counters_.validate_node_overhead_time_us += std::max(0.0, total_us - preamble_us);
            return cache_it->second.result;
        }
        counters_.validation_cache_misses += 1;
    }
    auto store_validation_cache = [&](BoxValidation result) {
        if (!use_validation_cache) {
            return;
        }
        if (validation_cache_.size() >=
            static_cast<std::size_t>(validation_config_.validation_cache_max_entries)) {
            validation_cache_.clear();
        }
        validation_cache_[cache_key] = ValidationCacheEntry{result, last_validation_detail_};
    };
    const auto endpoint_path_start = Clock::now();
    auto payload = endpoint_payload_for_node(node, intervals, changed_dim);
    const double endpoint_path_us = elapsed_us(endpoint_path_start);
    counters_.validate_node_endpoint_path_time_us += endpoint_path_us;
    if (!payload) {
        counters_.collision_possible += 1;
        last_validation_detail_.validation = BoxValidation::Unknown;
        last_validation_detail_.safety_status = BoxSafetyStatus::Unknown;
        last_validation_detail_.collision_possible = true;
        const double total_us = elapsed_us(total_start);
        counters_.validate_node_total_time_us += total_us;
        counters_.validate_node_overhead_time_us += std::max(0.0, total_us - preamble_us - endpoint_path_us);
        store_validation_cache(BoxValidation::Unknown);
        return BoxValidation::Unknown;
    }
    const auto classify_start = Clock::now();
    BoxValidation result = classify_payload(node, intervals, *payload);
    if (result == BoxValidation::Unknown &&
        validation_config_.external_evidence_live_retry_on_maybe &&
        payload->reused_external_evidence) {
        counters_.materialization_external_maybe_live_retries += 1;
        const auto live_endpoint_start = Clock::now();
        auto live_payload = endpoint_payload_for_node(node, intervals, changed_dim, /*allow_external_evidence=*/false);
        counters_.validate_node_endpoint_path_time_us += elapsed_us(live_endpoint_start);
        if (live_payload) {
            result = classify_payload(node, intervals, *live_payload);
            if (result == BoxValidation::Free) {
                counters_.materialization_external_maybe_live_retry_free += 1;
            }
        }
    }
    const double classify_us = elapsed_us(classify_start);
    counters_.validate_node_classify_time_us += classify_us;
    const double total_us = elapsed_us(total_start);
    counters_.validate_node_total_time_us += total_us;
    counters_.validate_node_overhead_time_us +=
        std::max(0.0, total_us - preamble_us - endpoint_path_us - classify_us);
    store_validation_cache(result);
    return result;
}

bool DatabaseBoxOracle::validate_intervals(const std::vector<Interval>& intervals) {
    counters_.interval_validations += 1;
    return !checker_.check_box(intervals);
}

bool DatabaseBoxOracle::is_reserved(OracleNodeId node) const {
    if (node < 0) {
        return false;
    }
    if (active_tree_is_primary_canonical_sector(robot_, database_)) {
        return false;
    }
    return node_to_box_.find(node) != node_to_box_.end();
}

std::optional<int> DatabaseBoxOracle::reservation_owner(OracleNodeId node) const {
    if (node < 0) {
        return std::nullopt;
    }
    if (active_tree_is_primary_canonical_sector(robot_, database_)) {
        return std::nullopt;
    }
    const auto it = node_to_box_.find(node);
    return it == node_to_box_.end() ? std::nullopt : std::optional<int>(it->second);
}

void DatabaseBoxOracle::reserve_node(OracleNodeId node, int box_id) {
    if (node < 0) {
        return;
    }
    if (active_tree_is_primary_canonical_sector(robot_, database_)) {
        return;
    }
    node_to_box_[node] = box_id;
    box_to_node_[box_id] = node;
}

void DatabaseBoxOracle::release_node(OracleNodeId node) {
    if (node < 0) {
        return;
    }
    const auto it = node_to_box_.find(node);
    if (it == node_to_box_.end()) {
        return;
    }
    box_to_node_.erase(it->second);
    node_to_box_.erase(it);
}

void DatabaseBoxOracle::release_box(int box_id) {
    const auto it = box_to_node_.find(box_id);
    if (it == box_to_node_.end()) {
        return;
    }
    node_to_box_.erase(it->second);
    box_to_node_.erase(it);
}

void DatabaseBoxOracle::clear_reservations() {
    node_to_box_.clear();
    box_to_node_.clear();
}

void DatabaseBoxOracle::record_visit(OracleNodeId node) {
    if (node >= 0) {
        ++visit_counts_[node];
    }
}

OracleNodeId DatabaseBoxOracle::select_unexplored_node() const {
    if (unexplored_leaf_cache_dirty_) {
        unexplored_leaf_cache_.clear();
        for (lect_database::NodeId node_id : database_.node_ids()) {
            const auto topology = database_.topology(node_id);
            if (!topology.leaf) {
                continue;
            }
            const OracleNodeId node = from_database_node(node_id);
            unexplored_leaf_cache_.push_back({node, interval_volume(node_intervals(node))});
        }
        unexplored_leaf_cache_dirty_ = false;
    }
    OracleNodeId best_node = kInvalidOracleNodeId;
    double best_weight = -1.0;
    for (const auto& entry : unexplored_leaf_cache_) {
        const OracleNodeId node = entry.node;
        if (node < 0 || is_reserved(node) || !is_leaf(node)) {
            continue;
        }
        const auto visit_it = visit_counts_.find(node);
        const double visit_count = visit_it == visit_counts_.end()
            ? 0.0
            : static_cast<double>(visit_it->second);
        const double weight = entry.volume / (visit_count + 1.0);
        if (weight > best_weight) {
            best_weight = weight;
            best_node = node;
        }
    }
    if (best_node >= 0) {
        ++visit_counts_[best_node];
    }
    return best_node;
}

int DatabaseBoxOracle::common_ancestor_depth(OracleNodeId lhs_node, OracleNodeId rhs_node) const {
    const auto lhs = to_database_node(lhs_node);
    const auto rhs = to_database_node(rhs_node);
    if (lhs_node < 0 || rhs_node < 0 || !database_.node(lhs) || !database_.node(rhs)) {
        return -1;
    }
    const auto ancestor = database_.lca(lhs, rhs);
    if (!lect_database::valid_node_id(ancestor)) {
        return -1;
    }
    return database_.topology(ancestor).depth;
}

std::unique_ptr<BoxOracleSession> DatabaseBoxOracle::make_session(const OracleSessionConfig& config) {
    return std::make_unique<DatabaseBoxOracleSession>(*this, config);
}

std::shared_ptr<lect_database::SharedEndpointEvidenceCache> DatabaseBoxOracle::shared_endpoint_cache() {
    if (!shared_endpoint_cache_) {
        shared_endpoint_cache_ = std::make_shared<lect_database::SharedEndpointEvidenceCache>(
            validation_config_.shared_endpoint_cache_max_entries,
            validation_config_.shared_endpoint_cache_max_bytes);
    }
    return shared_endpoint_cache_;
}

void DatabaseBoxOracle::set_scene(Scene scene) {
    scene_ = std::move(scene);
    checker_.set_scene(scene_);
    validation_cache_.clear();
}

std::unique_ptr<BoxOracleSession> DatabaseBoxOracleFactory::make_session(const OracleSessionConfig& config) {
    return master_.make_session(config);
}

}  // namespace rbf
