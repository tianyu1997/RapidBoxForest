#pragma once

#include <LECTDatabase/sbf/oracle_types.h>

#include <Eigen/Core>

#include <functional>
#include <optional>
#include <vector>

namespace rbf::detail {

// Computes the seed-containing LECT cell at a target depth from the split
// policy alone. This lets BinaryDepth FFB probe deep cells without first
// materializing every ancestor in the database tree.
struct VirtualSeedCell {
    int depth = 0;
    int changed_dim = -1;
    std::vector<Interval> tree_intervals;
    std::vector<Interval> query_intervals;
};

struct MaterializedSeedCell {
    OracleNodeId node = kInvalidOracleNodeId;
    int changed_dim = -1;
    int splits = 0;
    std::vector<Interval> tree_intervals;
    std::vector<Interval> query_intervals;
};

struct VirtualSeedPathEntry {
    int depth = 0;
    int changed_dim = -1;
    std::vector<Interval> tree_intervals;
};

using MaterializeSplitObserver =
    std::function<void(const SplitNodeResult&, const std::vector<Interval>&, int, double)>;

bool split_policy_supports_virtual_cells(const OracleSplitPolicyDescriptor& descriptor,
                                         int target_depth);
std::optional<VirtualSeedCell> virtual_seed_cell_at_depth(BoxOracle& oracle,
                                                          const Eigen::Ref<const Eigen::VectorXd>& seed,
                                                          int target_depth);
std::optional<std::vector<VirtualSeedPathEntry>> virtual_seed_path_to_depth(
    BoxOracle& oracle,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    int target_depth);
bool intervals_equal_with_tolerance(const std::vector<Interval>& lhs,
                                    const std::vector<Interval>& rhs,
                                    double tolerance);
std::optional<MaterializedSeedCell> materialize_seed_path_to_depth(
    BoxOracle& oracle,
    const Eigen::Ref<const Eigen::VectorXd>& seed,
    int target_depth,
    const OracleSplitOptions& split_options,
    const MaterializeSplitObserver& observe_split);

}  // namespace rbf::detail
