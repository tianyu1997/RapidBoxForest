#include <SBF/safe_box_forest.h>

#include <SBF/adaptive_grid_partition.h>
#include <SBF/box_graph.h>
#include <SBF/oracle.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>

#include "adaptive_grid_partition_options.h"
#include "planning_forest_dynamic_collision_cache_state.h"

namespace rbf {

RBFPlanningForest::~RBFPlanningForest() = default;

const OracleCounters* RBFPlanningForest::oracle_counters() const {
    return oracle_ ? &oracle_->counters() : nullptr;
}

void RBFPlanningForest::clear_forest() {
    boxes_.clear();
    raw_boxes_.clear();
    adjacency_.clear();
    segment_edges_.clear();
    adaptive_partition_.reset();
    adaptive_partition_query_enabled_ = false;
    has_adaptive_partition_config_ = false;
    clear_dynamic_collision_cache();
    if (oracle_) {
        oracle_->clear_reservations();
    }
    invalidate_query_cache();
}

void RBFPlanningForest::reset_oracle(Scene scene) {
    if (!online_cache_) {
        throw std::runtime_error("SBF online envelope cache is not initialised");
    }
    online_cache_->clear_payloads();
    oracle_ = std::make_unique<DatabaseBoxOracle>(
        robot_, *online_cache_, std::move(scene), config_.endpoint_source, config_.envelope_type, config_.validation,
        external_evidence_source_, direct_external_evidence_database_);
    // Preserve the interval-keyed endpoint cache across oracle resets so it
    // persists across queries (endpoints are scene-independent). The cache is
    // memory-bounded by the validation config (OOM guard).
    if (shared_endpoint_cache_) {
        oracle_->set_shared_endpoint_cache(shared_endpoint_cache_);
    } else {
        shared_endpoint_cache_ = oracle_->shared_endpoint_cache();
    }
}

void RBFPlanningForest::reserve_existing_boxes() {
    if (!oracle_) {
        return;
    }
    oracle_->clear_reservations();
    for (const auto& box : boxes_) {
        if (box.tree_id >= 0) {
            oracle_->reserve_node(box.tree_id, box.id);
        }
    }
}

void RBFPlanningForest::rebuild_adjacency() {
    adjacency_ = compute_adjacency(boxes_, config_.query.adjacency_tolerance);
    apply_segment_edges_to_adjacency(segment_edges_, adjacency_);
    invalidate_query_cache();
}

void RBFPlanningForest::invalidate_query_cache() const {
    query_cache_dirty_ = true;
}

const QueryGraphCache& RBFPlanningForest::query_cache() const {
    if (partition_native_mode()) {
        throw std::logic_error(
            "partition_native mode forbids QueryGraphCache fallback; "
            "use AdaptiveGridPartition query/locate/connect APIs instead");
    }
    if (query_cache_dirty_) {
        query_cache_ = build_query_graph_cache(boxes_, adjacency_, segment_edges_);
        query_cache_dirty_ = false;
    }
    return query_cache_;
}

int RBFPlanningForest::next_box_id() const {
    int next = 0;
    for (const auto& box : boxes_) {
        next = std::max(next, box.id + 1);
    }
    return next;
}

} // namespace rbf
