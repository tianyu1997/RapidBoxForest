#pragma once

#include <cstdint>
#include <vector>

namespace rbf {

struct QueryBridgeBatchOptions {
	/// Local indices in the starts/goals batch that should be bridged even if
	/// the current graph query already satisfies the acceptance thresholds.
	std::vector<int> forced_query_indices;
	/// Optional stable query ids, one per starts/goals batch entry. These ids are
	/// stored on query-bridge edges so later graph searches can distinguish
	/// same-query and cross-query reuse without relying on process environment.
	std::vector<int> global_query_indices;
};

#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
enum class CorridorRefineMode : std::uint8_t {
	SegmentBridge = 0,
	BoxOnlyLongPath = 1,
};
#endif

} // namespace rbf
