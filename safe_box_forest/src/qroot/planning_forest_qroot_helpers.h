#pragma once

#include <SBF/box_graph.h>
#include <SBF/find_free_box_types.h>
#include <SBF/runtime.h>
#include <LECTDatabase/sbf/oracle_types.h>

#include <Eigen/Core>

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rbf {

inline constexpr double kQueryRootBoundaryContainmentTolerance = 1e-3;

bool intervals_contain_point_with_boundary_tolerance(const std::vector<Interval>& intervals,
                                                     const Eigen::Ref<const Eigen::VectorXd>& point,
                                                     double tolerance);

bool expand_intervals_to_contain_boundary_seed(std::vector<Interval>& intervals,
                                               const Eigen::Ref<const Eigen::VectorXd>& point,
                                               double tolerance);

bool box_contains_box_exact_local(const BoxNode& outer, const BoxNode& inner);

bool intervals_subset_local(const std::vector<Interval>& inner,
                            const std::vector<Interval>& outer,
                            double tolerance = 0.0);

void append_local_edge(AdjacencyGraph& graph, int lhs, int rhs);

void connect_incremental_boxes(AdjacencyGraph& graph,
                               const std::vector<BoxNode>& boxes,
                               std::size_t first_new_index,
                               double tolerance);

bool allow_dynamic_commit(BoxOracle& oracle,
                          FindFreeBoxResult& result,
                          BoxCommitPolicy policy);

struct BuildDisjointSet {
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> rank;

    void add(int id);
    int find(int id);
    void unite(int lhs, int rhs);
    bool connected(int lhs, int rhs);
    int island_count();
};

struct BoxSpatialIndex {
    int index_dim = -1;
    double origin = 0.0;
    double bin_width = 1.0;
    std::unordered_map<long long, std::vector<int>> bins;

    static int choose_dim(const std::vector<BoxNode>& boxes);
    static long long bin_of(double value, double origin_value, double width);

    void rebuild(const std::vector<BoxNode>& boxes, double tolerance);
    void add_box(const BoxNode& box, int index, double tolerance);
    std::vector<int> interval_candidates(const std::vector<Interval>& intervals, double tolerance) const;
    std::vector<int> point_candidates(const Eigen::Ref<const Eigen::VectorXd>& point) const;
    int covering_box(const std::vector<BoxNode>& boxes,
                     const Eigen::Ref<const Eigen::VectorXd>& point,
                     double tolerance) const;
};

struct QueryRootGrowResult {
    int boxes_added = 0;
    int endpoint_anchors_added = 0;
    int endpoint_root_fallbacks = 0;
    int uncovered_endpoints = 0;
    int ffb_success = 0;
    int ffb_fail = 0;
    int contained_rejects = 0;
    int domain_rejects = 0;
    int adjacency_rejects = 0;
    int adjacency_edges_added = 0;
    int commit_rejects = 0;
    int adjacency_candidates_tested = 0;
    int pair_attempts = 0;
    int pairs_total = 0;
    int pairs_connected_before = 0;
    int pairs_connected_after = 0;
    int islands_before = 0;
    int islands_after = 0;
    double index_rebuild_ms = 0.0;
    double index_query_ms = 0.0;
    double total_ms = 0.0;
};

BuildDisjointSet make_dsu_from_graph(const std::vector<BoxNode>& boxes,
                                     const AdjacencyGraph& graph);

int commit_query_root_box(BoxOracle& oracle,
                          const FindFreeBoxOptions& options,
                          BoxCommitPolicy commit_policy,
                          const std::function<FindFreeBoxResult(const Eigen::VectorXd&,
                                                                 const std::vector<Interval>&,
                                                                 StageContext&,
                                                                 const FindFreeBoxOptions&)>& find_in_domain,
                          const Eigen::Ref<const Eigen::VectorXd>& seed,
                          const BoxNode& domain,
                          int parent_box_id,
                          int root_id,
                          std::vector<BoxNode>& boxes,
                          std::vector<BoxNode>& raw_boxes,
                          AdjacencyGraph& graph,
                          BoxSpatialIndex& box_index,
                          BuildDisjointSet& dsu,
                          int& next_id,
                          StageContext& context,
                          QueryRootGrowResult& stats,
                          double adjacency_tolerance);

} // namespace rbf
