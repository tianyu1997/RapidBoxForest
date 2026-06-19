#include "oracle_support.h"

#include <algorithm>
#include <atomic>
#include <cstring>

namespace rbf {
namespace {

std::uint64_t hash_mix(std::uint64_t seed, std::uint64_t value) {
    seed ^= value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
    return seed;
}

std::uint64_t hash_double_bits(double value) {
    if (value == 0.0) {
        value = 0.0;
    }
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(value));
    return bits;
}

bool same_interval_exact(const Interval& a, const Interval& b) {
    return a.lo == b.lo && a.hi == b.hi;
}

}  // namespace

std::mutex& external_direct_lookup_mutex() {
    static std::mutex mutex;
    return mutex;
}

std::uint64_t validation_cache_key(OracleNodeId node,
                                   const std::vector<Interval>& intervals,
                                   int changed_dim) {
    std::uint64_t key = 0xcbf29ce484222325ull;
    key = hash_mix(key, static_cast<std::uint64_t>(node));
    key = hash_mix(key, static_cast<std::uint64_t>(changed_dim + 4096));
    key = hash_mix(key, static_cast<std::uint64_t>(intervals.size()));
    for (const auto& interval : intervals) {
        key = hash_mix(key, hash_double_bits(interval.lo));
        key = hash_mix(key, hash_double_bits(interval.hi));
    }
    return key;
}

double elapsed_us(OracleClock::time_point start) {
    return std::chrono::duration<double, std::micro>(OracleClock::now() - start).count();
}

bool differs_only_in_dim_exact(const std::vector<Interval>& lhs,
                               const std::vector<Interval>& rhs,
                               int changed_dim) {
    if (changed_dim < 0 || lhs.size() != rhs.size()) {
        return false;
    }
    bool saw_change = false;
    for (int index = 0; index < static_cast<int>(lhs.size()); ++index) {
        if (same_interval_exact(lhs[static_cast<std::size_t>(index)],
                                rhs[static_cast<std::size_t>(index)])) {
            continue;
        }
        if (index != changed_dim || saw_change) {
            return false;
        }
        saw_change = true;
    }
    return saw_change;
}

double interval_volume(const std::vector<Interval>& intervals) {
    double volume = 1.0;
    for (const auto& interval : intervals) {
        volume *= std::max(0.0, interval.width());
    }
    return volume;
}

int active_link_index_to_link_id(const Robot& robot, int active_link_index) {
    if (active_link_index < 0 || active_link_index >= robot.n_active_links()) {
        return -1;
    }
    const int* active_map = robot.active_link_map();
    if (active_map == nullptr) {
        return active_link_index;
    }
    return active_map[active_link_index];
}

std::vector<int> affected_joints_for_link(const Robot& robot, int link_id) {
    std::vector<int> joints;
    if (link_id < 0 || robot.n_joints() <= 0) {
        return joints;
    }
    const int last_joint = std::clamp(link_id, 0, robot.n_joints() - 1);
    joints.reserve(static_cast<std::size_t>(last_joint + 1));
    for (int joint = 0; joint <= last_joint; ++joint) {
        joints.push_back(joint);
    }
    return joints;
}

std::vector<OracleValidationBlocker> make_oracle_blockers(
    const Robot& robot,
    const EnvelopeCollisionStats& collision_stats) {
    std::vector<OracleValidationBlocker> blockers;
    blockers.reserve(collision_stats.blockers.size());
    for (const auto& source : collision_stats.blockers) {
        OracleValidationBlocker blocker;
        blocker.active_link_index = source.active_link_index;
        blocker.link_id = active_link_index_to_link_id(robot, source.active_link_index);
        blocker.obstacle_id = source.obstacle_index;
        blocker.stage = source.stage;
        blocker.margin = source.margin;
        blocker.overlap_depth = source.overlap_depth;
        blocker.overlap_volume_ratio = source.overlap_volume_ratio;
        blocker.affected_joints = affected_joints_for_link(robot, blocker.link_id);
        blockers.push_back(std::move(blocker));
    }
    std::sort(blockers.begin(), blockers.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.overlap_depth != rhs.overlap_depth) {
            return lhs.overlap_depth > rhs.overlap_depth;
        }
        if (lhs.link_id != rhs.link_id) {
            return lhs.link_id < rhs.link_id;
        }
        if (lhs.obstacle_id != rhs.obstacle_id) {
            return lhs.obstacle_id < rhs.obstacle_id;
        }
        return lhs.stage < rhs.stage;
    });
    return blockers;
}

std::uint64_t blocker_signature_hash(const std::vector<OracleValidationBlocker>& blockers,
                                     std::size_t top_k) {
    std::uint64_t seed = 0x4f2a7c15c0ffee21ull;
    const std::size_t count = std::min(top_k, blockers.size());
    seed = hash_mix(seed, static_cast<std::uint64_t>(count));
    for (std::size_t index = 0; index < count; ++index) {
        const auto& blocker = blockers[index];
        seed = hash_mix(seed, static_cast<std::uint64_t>(blocker.link_id + 4096));
        seed = hash_mix(seed, static_cast<std::uint64_t>(blocker.obstacle_id + 4096));
        seed = hash_mix(seed, static_cast<std::uint64_t>(blocker.stage + 4096));
    }
    return count == 0 ? 0 : seed;
}

std::uint64_t make_envelope_cache_key(lect_database::NodeId node_id, int sector) {
    std::uint64_t key = static_cast<std::uint64_t>(node_id);
    const std::uint64_t sector_bits = static_cast<std::uint64_t>(static_cast<std::uint32_t>(sector));
    key ^= sector_bits + 0x9e3779b97f4a7c15ull + (key << 6) + (key >> 2);
    return key;
}

std::uint64_t make_envelope_cache_key(const lect_database::EvidenceKey& key, int fallback_sector) {
    const int sector = key.sector == lect_database::kPrimarySector ? fallback_sector : static_cast<int>(key.sector);
    return make_envelope_cache_key(key.node_id, sector);
}

std::uint64_t endpoint_payload_hash(std::span<const float> payload) {
    // Envelope evidence for the same node/sector may come from a direct box or
    // a child-hull/external exact lookup. Include the payload bytes so those
    // distinct envelopes cannot alias in the per-oracle envelope cache.
    std::uint64_t hash = 1469598103934665603ull;
    const auto* bytes = reinterpret_cast<const unsigned char*>(payload.data());
    const std::size_t n_bytes = payload.size() * sizeof(float);
    for (std::size_t i = 0; i < n_bytes; ++i) {
        hash ^= static_cast<std::uint64_t>(bytes[i]);
        hash *= 1099511628211ull;
    }
    hash ^= static_cast<std::uint64_t>(payload.size()) + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2);
    return hash;
}

std::uint64_t next_session_id() {
    static std::atomic<std::uint64_t> counter{0};
    return counter.fetch_add(1, std::memory_order_relaxed);
}

void record_envelope_collision(OracleCounters& counters, const EnvelopeCollisionStats& stats) {
    counters.envelope_collision_envelope_aabb_tests += stats.envelope_aabb_tests;
    counters.envelope_collision_envelope_aabb_rejects += stats.envelope_aabb_rejects;
    counters.envelope_collision_link_union_aabb_tests += stats.link_union_aabb_tests;
    counters.envelope_collision_link_union_aabb_rejects += stats.link_union_aabb_rejects;
    counters.envelope_collision_link_aabb_tests += stats.link_aabb_tests;
    counters.envelope_collision_link_aabb_rejects += stats.link_aabb_rejects;
    counters.envelope_collision_kdop_tests += stats.kdop_tests;
    counters.envelope_collision_kdop_rejects += stats.kdop_rejects;
    counters.envelope_collision_kdop_axes_tested += stats.kdop_axes_tested;
    counters.envelope_collision_gjk_tests += stats.gjk_tests;
    counters.envelope_collision_gjk_rejects += stats.gjk_rejects;
    counters.envelope_collision_gjk_iterations += stats.gjk_iterations;
    counters.envelope_collision_overlap_depth_sum += stats.maybe_pair_overlap_depth_sum;
    counters.envelope_collision_overlap_depth_max =
        std::max(counters.envelope_collision_overlap_depth_max,
                 stats.maybe_pair_overlap_depth_max);
    counters.envelope_collision_overlap_volume_ratio_max =
        std::max(counters.envelope_collision_overlap_volume_ratio_max,
                 stats.maybe_pair_overlap_volume_ratio_max);
}

bool certifies_occupied(const MaterialPointOccupiedWitness& witness) {
    return witness.center_signed_distance + witness.motion_bound + witness.epsilon_num < 0.0;
}

lect_database::EvidenceChannel database_channel_for_endpoint(EndpointSource source) {
    return source_channel(source) == 0 ? lect_database::EvidenceChannel::Safe
                                       : lect_database::EvidenceChannel::Rapid;
}

}  // namespace rbf
