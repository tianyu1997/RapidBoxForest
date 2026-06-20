#include <LECTDatabase/sbf/oracle.h>

#include "oracle_material_point.h"
#include "oracle_options.h"
#include "oracle_support.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <utility>

namespace rbf {
namespace {

using Clock = std::chrono::steady_clock;

}  // namespace

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

}  // namespace rbf
