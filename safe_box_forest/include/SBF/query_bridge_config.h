#pragma once

#include <cstdint>
#include <vector>

namespace rbf {

enum class PortalMembershipPolicy : std::uint8_t {
	GlobalForestOnly = 0,
	PortalInteriorIndex = 1,
};

struct RBFQueryRuntimeOptions {
	/// Stable query id used to distinguish same-query and foreign query repair
	/// edges during graph search. -1 disables query ownership penalties.
	int active_query_index = -1;
};

struct QueryBridgeBatchOptions {
	/// Local indices in the starts/goals batch that should be bridged even if
	/// the current graph query already satisfies the acceptance thresholds.
	std::vector<int> forced_query_indices;
	/// Optional stable query ids, one per starts/goals batch entry. These ids are
	/// stored on query-bridge edges so later graph searches can distinguish
	/// same-query and cross-query reuse without relying on process environment.
	std::vector<int> global_query_indices;
};

enum class CorridorRefineMode : std::uint8_t {
	SegmentBridge = 0,
	BoxOnlyLongPath = 1,
};

struct EndpointMainBoxCorridorConfig {
	int target_k = 8;
	double coarse_step = 0.08;
	double fine_step = 0.02;
	int max_ffb_calls = 48;
	int max_boxes = 64;
	double residual_segment_max_length = 0.25;
	double lateral_offset = 0.03;
	int lateral_rounds = 2;
	double face_epsilon = 1e-6;
};

} // namespace rbf
