#pragma once

#include <LECTDatabase/sbf/oracle.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <span>
#include <vector>

namespace rbf {

using OracleClock = std::chrono::steady_clock;

std::mutex& external_direct_lookup_mutex();

std::uint64_t validation_cache_key(OracleNodeId node,
                                   const std::vector<Interval>& intervals,
                                   int changed_dim);
double elapsed_us(OracleClock::time_point start);
bool differs_only_in_dim_exact(const std::vector<Interval>& lhs,
                               const std::vector<Interval>& rhs,
                               int changed_dim);
double interval_volume(const std::vector<Interval>& intervals);

int active_link_index_to_link_id(const Robot& robot, int active_link_index);
std::vector<int> affected_joints_for_link(const Robot& robot, int link_id);
std::vector<OracleValidationBlocker> make_oracle_blockers(
    const Robot& robot,
    const EnvelopeCollisionStats& collision_stats);
std::uint64_t blocker_signature_hash(const std::vector<OracleValidationBlocker>& blockers,
                                     std::size_t top_k = 3);

std::uint64_t make_envelope_cache_key(lect_database::NodeId node_id, int sector);
std::uint64_t make_envelope_cache_key(const lect_database::EvidenceKey& key, int fallback_sector);
std::uint64_t endpoint_payload_hash(std::span<const float> payload);
std::uint64_t next_session_id();

void record_envelope_collision(OracleCounters& counters, const EnvelopeCollisionStats& stats);
bool certifies_occupied(const MaterialPointOccupiedWitness& witness);
lect_database::EvidenceChannel database_channel_for_endpoint(EndpointSource source);

}  // namespace rbf
