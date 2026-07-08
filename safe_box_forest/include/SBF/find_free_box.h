#pragma once

#include <SBF/find_free_box_types.h>
#include <SBF/runtime_fwd.h>

#include <Eigen/Core>

#include <chrono>
#include <functional>

namespace rbf {

class FindFreeBoxService {
public:
	using AcceptCandidate = std::function<bool(const FindFreeBoxResult&)>;
	explicit FindFreeBoxService(BoxOracle& oracle) : oracle_(oracle) {}
	FindFreeBoxResult find(const Eigen::Ref<const Eigen::VectorXd>& seed,
						   const FindFreeBoxOptions& options = {});
	FindFreeBoxResult find(const Eigen::Ref<const Eigen::VectorXd>& seed,
						   StageContext& context,
						   const FindFreeBoxOptions& options = {});
	FindFreeBoxResult find_incremental(const Eigen::Ref<const Eigen::VectorXd>& seed,
										StageContext& context,
										const FindFreeBoxOptions& options,
										const AcceptCandidate& accept);

private:
	FindFreeBoxResult find_binary_depth(const Eigen::Ref<const Eigen::VectorXd>& seed,
										StageContext& context,
										const FindFreeBoxOptions& options,
										const OracleSplitOptions& split_options,
										int effective_max_depth,
										std::chrono::steady_clock::time_point start);
	BoxOracle& oracle_;
};

}  // namespace rbf
