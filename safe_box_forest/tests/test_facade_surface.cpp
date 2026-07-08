#include <SBF/safe_box_forest.h>

#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
#include <SBF/debug.h>
#endif

#include <type_traits>

namespace {

template <typename T, typename = void>
struct has_debug_chain_pave : std::false_type {};

template <typename T>
struct has_debug_chain_pave<T, std::void_t<decltype(&T::debug_chain_pave)>>
	: std::true_type {};

template <typename T, typename = void>
struct has_debug_chain_pave_waypoints : std::false_type {};

template <typename T>
struct has_debug_chain_pave_waypoints<T, std::void_t<decltype(&T::debug_chain_pave_waypoints)>>
	: std::true_type {};

template <typename T, typename = void>
struct has_refine_query_corridor : std::false_type {};

template <typename T>
struct has_refine_query_corridor<
	T,
	std::void_t<decltype(static_cast<int (T::*)(const Eigen::Ref<const Eigen::VectorXd>&,
	                                            const Eigen::Ref<const Eigen::VectorXd>&,
	                                            int)>(&T::refine_query_corridor))>>
	: std::true_type {};

template <typename T, typename = void>
struct has_connect_update_segment_fallback : std::false_type {};

template <typename T>
struct has_connect_update_segment_fallback<
	T,
	std::void_t<decltype(&T::connect_update_segment_fallback)>>
	: std::true_type {};

template <typename T, typename = void>
struct has_connect_update_endpoint_segment_fallback : std::false_type {};

template <typename T>
struct has_connect_update_endpoint_segment_fallback<
	T,
	std::void_t<decltype(&T::connect_update_endpoint_segment_fallback)>>
	: std::true_type {};

template <typename T, typename = void>
struct has_oracle_counters : std::false_type {};

template <typename T>
struct has_oracle_counters<T, std::void_t<decltype(&T::oracle_counters)>>
	: std::true_type {};

#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
template <typename T, typename = void>
struct has_build_subtractive_default_options : std::false_type {};

template <typename T>
struct has_build_subtractive_default_options<
	T,
	std::void_t<decltype(static_cast<rbf::BuildProfile (T::*)(
		const std::vector<rbf::SubtractiveObstacleGroup>&,
		const std::vector<Eigen::VectorXd>&)>(&T::build_subtractive))>>
	: std::true_type {};

template <typename T, typename = void>
struct has_build_subtractive_explicit_options : std::false_type {};

template <typename T>
struct has_build_subtractive_explicit_options<
	T,
	std::void_t<decltype(static_cast<rbf::BuildProfile (T::*)(
		const std::vector<rbf::SubtractiveObstacleGroup>&,
		const std::vector<Eigen::VectorXd>&,
		const rbf::SubtractiveBuildOptions&)>(&T::build_subtractive))>>
	: std::true_type {};
#else
template <typename T, typename = void>
struct has_build_subtractive : std::false_type {};

template <typename T>
struct has_build_subtractive<T, std::void_t<decltype(&T::build_subtractive)>>
	: std::true_type {};
#endif

template <typename T, typename = void>
struct has_add_obstacle_and_rebuild : std::false_type {};

template <typename T>
struct has_add_obstacle_and_rebuild<T, std::void_t<decltype(&T::add_obstacle_and_rebuild)>>
	: std::true_type {};

template <typename T, typename = void>
struct has_add_obstacles_and_rebuild : std::false_type {};

template <typename T>
struct has_add_obstacles_and_rebuild<T, std::void_t<decltype(&T::add_obstacles_and_rebuild)>>
	: std::true_type {};

template <typename T, typename = void>
struct has_remove_obstacle_and_regrow : std::false_type {};

template <typename T>
struct has_remove_obstacle_and_regrow<T, std::void_t<decltype(&T::remove_obstacle_and_regrow)>>
	: std::true_type {};

template <typename T, typename = void>
struct has_remove_obstacle_suffix_and_regrow : std::false_type {};

template <typename T>
struct has_remove_obstacle_suffix_and_regrow<T, std::void_t<decltype(&T::remove_obstacle_suffix_and_regrow)>>
	: std::true_type {};

template <typename T, typename = void>
struct has_run_pure_ffb : std::false_type {};

template <typename T>
struct has_run_pure_ffb<T, std::void_t<decltype(&T::run_pure_ffb)>>
	: std::true_type {};

template <typename T, typename = void>
struct has_dynamic_update_config : std::false_type {};

template <typename T>
struct has_dynamic_update_config<T, std::void_t<decltype(&T::dynamic_update)>>
	: std::true_type {};

} // namespace

int main() {
#if defined(SBF_DIAGNOSTIC_API) && SBF_DIAGNOSTIC_API
	static_assert(sizeof(rbf::DebugBoundaryFfbFailure) > 0,
	              "diagnostic builds should expose DebugBoundaryFfbFailure");
	static_assert(sizeof(rbf::DebugChainPaveResult) > 0,
	              "diagnostic builds should expose DebugChainPaveResult");
	static_assert(has_dynamic_update_config<rbf::RBFPlanningConfig>::value,
	              "diagnostic builds should expose dynamic_update config");
	static_assert(has_debug_chain_pave<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose debug_chain_pave");
	static_assert(has_debug_chain_pave_waypoints<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose debug_chain_pave_waypoints");
	static_assert(has_refine_query_corridor<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose refine_query_corridor");
	static_assert(has_connect_update_segment_fallback<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose connect_update_segment_fallback");
	static_assert(has_connect_update_endpoint_segment_fallback<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose connect_update_endpoint_segment_fallback");
	static_assert(has_oracle_counters<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose oracle_counters");
	static_assert(has_build_subtractive_default_options<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose build_subtractive default-options overload");
	static_assert(has_build_subtractive_explicit_options<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose build_subtractive explicit-options overload");
	static_assert(has_add_obstacle_and_rebuild<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose add_obstacle_and_rebuild");
	static_assert(has_add_obstacles_and_rebuild<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose add_obstacles_and_rebuild");
	static_assert(has_remove_obstacle_and_regrow<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose remove_obstacle_and_regrow");
	static_assert(has_remove_obstacle_suffix_and_regrow<rbf::RBFPlanningForest>::value,
	              "diagnostic builds should expose remove_obstacle_suffix_and_regrow");
#else
	static_assert(!has_dynamic_update_config<rbf::RBFPlanningConfig>::value,
	              "default builds must not expose dynamic_update config");
	static_assert(!has_debug_chain_pave<rbf::RBFPlanningForest>::value,
	              "default builds must not expose debug_chain_pave");
	static_assert(!has_debug_chain_pave_waypoints<rbf::RBFPlanningForest>::value,
	              "default builds must not expose debug_chain_pave_waypoints");
	static_assert(!has_refine_query_corridor<rbf::RBFPlanningForest>::value,
	              "default builds must not expose refine_query_corridor");
	static_assert(!has_connect_update_segment_fallback<rbf::RBFPlanningForest>::value,
	              "default builds must not expose connect_update_segment_fallback");
	static_assert(!has_connect_update_endpoint_segment_fallback<rbf::RBFPlanningForest>::value,
	              "default builds must not expose connect_update_endpoint_segment_fallback");
	static_assert(!has_oracle_counters<rbf::RBFPlanningForest>::value,
	              "default builds must not expose oracle_counters");
	static_assert(!has_build_subtractive<rbf::RBFPlanningForest>::value,
	              "default builds must not expose build_subtractive");
	static_assert(!has_add_obstacle_and_rebuild<rbf::RBFPlanningForest>::value,
	              "default builds must not expose add_obstacle_and_rebuild");
	static_assert(!has_add_obstacles_and_rebuild<rbf::RBFPlanningForest>::value,
	              "default builds must not expose add_obstacles_and_rebuild");
	static_assert(!has_remove_obstacle_and_regrow<rbf::RBFPlanningForest>::value,
	              "default builds must not expose remove_obstacle_and_regrow");
	static_assert(!has_remove_obstacle_suffix_and_regrow<rbf::RBFPlanningForest>::value,
	              "default builds must not expose remove_obstacle_suffix_and_regrow");
#endif
	static_assert(!has_run_pure_ffb<rbf::RBFPlanningForest>::value,
	              "run_pure_ffb was an unimplemented benchmark facade and must stay removed");
	return 0;
}
