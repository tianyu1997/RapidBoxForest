#pragma once

#include "connector_internal.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace rbf {

void record_chain_pave_boundary_ffb_failure(const FindFreeBoxResult& result,
                                            const Eigen::VectorXd& seed,
                                            BoxOracle& oracle,
                                            const ChainPaveConfig& config,
                                            StageContext& context);

void record_optional_chain_pave_boundary_failure_payload(const FindFreeBoxResult& result,
                                                         const Eigen::VectorXd& seed,
                                                         BoxOracle& oracle,
                                                         const ChainPaveConfig& config);

std::vector<Eigen::VectorXd> chain_pave_boundary_seed_candidates(const BoxNode& box,
                                                                 const Eigen::VectorXd& from,
                                                                 const Eigen::VectorXd& target,
                                                                 double requested_step,
                                                                 double adjacency_tolerance,
                                                                 double gap_fill_min_step);

Eigen::VectorXd chain_pave_closest_point_in_box(const BoxNode& box,
                                                const Eigen::VectorXd& point);

std::uint64_t chain_pave_boundary_seed_key(int parent_id,
                                           std::size_t segment_index,
                                           const BoxNode& parent_box,
                                           const Eigen::VectorXd& cursor,
                                           const Eigen::VectorXd& seed,
                                           double adjacency_tolerance,
                                           double gap_fill_min_step);

struct ChainPaveConnectedStats {
    int segments = 0;
    int steps = 0;
    int reach_failures = 0;
    int target_hits = 0;
};

struct ChainPaveCommitContext {
    std::vector<BoxNode>& boxes;
    BoxOracle& oracle;
    AdjacencyGraph& graph;
    int& next_box_id;
    const ChainPaveConfig& config;
    StageContext& context;
    int& added;
    std::unordered_map<int, std::size_t> box_index;
    std::unordered_map<OracleNodeId, int> tree_owner;

    ChainPaveCommitContext(std::vector<BoxNode>& boxes,
                           BoxOracle& oracle,
                           AdjacencyGraph& graph,
                           int& next_box_id,
                           const ChainPaveConfig& config,
                           StageContext& context,
                           int& added);

    void index_box(std::size_t index);
    BoxNode* box_by_id(int id);
    void append_graph_edge(int lhs, int rhs);
    int find_existing_cover(const Eigen::VectorXd& point, int preferred_id = -1);
    int find_box_owning_node_covering(OracleNodeId node, const Eigen::VectorXd& point);
    int find_box_owning_node(OracleNodeId node) const;
    int commit_box(FindFreeBoxResult& result,
                   const Eigen::VectorXd& seed,
                   int parent_id,
                   bool allow_duplicate_node = false);
    int commit_reserved_cap_box(const FindFreeBoxResult& result,
                                const Eigen::VectorXd& seed,
                                int parent_id);
};

void record_chain_pave_connected_stats(StageContext& context,
                                       int added,
                                       int max_chain,
                                       const ChainPaveConnectedStats& stats);

}  // namespace rbf
