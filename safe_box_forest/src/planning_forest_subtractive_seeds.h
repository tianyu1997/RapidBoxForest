#pragma once

#include <SBF/safe_box_forest.h>
#include <SBF/box_graph.h>

#include <unordered_set>
#include <vector>

namespace rbf {

struct SubtractiveSeedCandidate {
    Eigen::VectorXd seed;
    int parent_box_id = -1;
    int root_id = -1;
    int domain_index = -1;
};

int containing_domain_index(const std::vector<BoxNode>& domains,
                            const Eigen::Ref<const Eigen::VectorXd>& point,
                            double tolerance);

int containing_domain_index(const std::vector<BoxNode>& domains,
                            const std::vector<Interval>& intervals,
                            double tolerance);

std::vector<SubtractiveSeedCandidate> make_subtractive_regrow_seeds(
    const std::vector<BoxNode>& live_boxes,
    const std::vector<BoxNode>& removed_boxes,
    const std::unordered_set<int>& removed_box_ids,
    const AdjacencyGraph& previous_adjacency,
    const std::vector<Eigen::VectorXd>& anchor_points,
    const DynamicUpdateConfig& config,
    double adjacency_tolerance,
    double boundary_epsilon);

}  // namespace rbf
