#include <SBF/leaf_sweep_grower.h>

#include "leaf_sweep_grower_diagnostics.h"
#include "leaf_sweep_grower_internal.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace rbf {

LeafSweepGrower::LeafSweepGrower(DatabaseBoxOracle& oracle,
								 LeafSweepConfig config,
								 OracleSplitOptions split_options)
	: oracle_(oracle), config_(std::move(config)), split_options_(std::move(split_options)) {}

LeafSweepResult LeafSweepGrower::sweep(const std::vector<Obstacle>& obstacles,
									   int start_depth,
									   int max_depth) {
	RuntimeConfig runtime;
	const int threads = std::max(1, config_.n_threads);
	runtime.mode = threads > 1 ? ExecutionMode::Parallel : ExecutionMode::Inline;
	runtime.n_threads = threads;
	runtime.batch_size = config_.validation_batch_size;
	StageContext context(runtime, Deadline::after_ms(config_.timeout_ms));
	return sweep(obstacles, start_depth, max_depth, context);
}

LeafSweepResult LeafSweepGrower::sweep(const std::vector<Obstacle>& obstacles,
									   int start_depth,
									   int max_depth,
									   StageContext& context) {
	if (start_depth < 0 || max_depth < 0) {
		throw std::invalid_argument("LeafSweepGrower depths must be non-negative");
	}
	if (start_depth > max_depth) {
		throw std::invalid_argument("LeafSweepGrower start_depth must be <= max_depth");
	}
	const int effective_max_depth = std::max(0, std::min(max_depth, oracle_.max_tree_depth() - 1));
	if (start_depth > effective_max_depth) {
		throw std::invalid_argument("LeafSweepGrower start_depth exceeds oracle max tree depth");
	}

	LeafSweepResult result;
	ScopedOracleEnvelopeCache envelope_cache_scope(oracle_, false);
	const bool collect_full_overlap_stats =
		config_.collision_overlap_prune_ratio_threshold > 0.0 &&
		config_.collision_overlap_prune_min_depth >= 0;
	ScopedOracleFullOverlapStats overlap_stats_scope(oracle_, collect_full_overlap_stats);
	const auto total_start = Clock::now();
	oracle_.set_scene(Scene(obstacles));
	set_value(result, context, "leaf_sweep.start_depth", static_cast<double>(start_depth));
	set_value(result, context, "leaf_sweep.max_depth", static_cast<double>(effective_max_depth));
	set_value(result, context, "leaf_sweep.obstacles", static_cast<double>(obstacles.size()));
	set_value(result, context, "leaf_sweep.executor_threads", static_cast<double>(context.executor().n_threads()));
	set_value(result, context, "leaf_sweep.validation_batch_size",
			  static_cast<double>(std::max(1, config_.validation_batch_size)));
	set_value(result, context, "leaf_sweep.collect_full_overlap_stats",
			  collect_full_overlap_stats ? 1.0 : 0.0);
	if (config_.pre_split_to_max_depth) {
		const auto pre_split_start = Clock::now();
		const bool ok = oracle_.database().ensure_depth(effective_max_depth);
		set_value(result,
				  context,
				  "leaf_sweep.pre_split_to_max_depth",
				  ok ? 1.0 : 0.0);
		set_value(result,
				  context,
				  "leaf_sweep.pre_split_ms",
				  std::chrono::duration<double, std::milli>(Clock::now() - pre_split_start).count());
	}

	const auto init_start = Clock::now();
	auto start_frontier = materialize_start_frontier(start_depth, effective_max_depth, context, result);
	result.initialize_ms = elapsed_ms(init_start);
	set_value(result, context, "leaf_sweep.start_frontier_nodes", static_cast<double>(start_frontier.size()));

	auto groups = cluster_obstacles(obstacles, result, context);
	if (config_.use_virtual_topology && groups.size() != 1) {
		throw std::invalid_argument("LeafSweepGrower virtual topology currently requires a single obstacle group");
	}
	if (config_.checkpoint_callback && groups.size() != 1) {
		throw std::invalid_argument("LeafSweepGrower checkpoint callback currently requires a single obstacle group");
	}
	set_value(result,
			  context,
			  "leaf_sweep.virtual_topology",
			  config_.use_virtual_topology ? 1.0 : 0.0);
	const auto group_start = Clock::now();
	for (auto& group : groups) {
		if (context.should_stop()) {
			result.deadline_reached = context.deadline().expired();
			add_counter(result, context, "leaf_sweep.deadline_before_group");
			break;
		}
		sweep_group(group, start_frontier, effective_max_depth, context, result);
	}
	result.group_sweep_ms = elapsed_ms(group_start);

	const auto compose_start = Clock::now();
	if (groups.size() == 1) {
		if (config_.store_group_results) {
			result.free_boxes = groups.front().result.free_boxes;
			result.collision_boxes = groups.front().result.collision_boxes;
		} else {
			result.free_boxes = std::move(groups.front().result.free_boxes);
			result.collision_boxes = std::move(groups.front().result.collision_boxes);
		}
		result.collision_box_obstacle_indices.assign(result.collision_boxes.size(),
													 groups.front().result.obstacle_indices);
		set_value(result, context, "leaf_sweep.single_group_compose_fast_path", 1.0);
	} else {
		compose_final_sets(groups, start_depth, effective_max_depth, context, result);
	}
	result.compose_ms = elapsed_ms(compose_start);

	if (config_.store_group_results) {
		result.groups.reserve(groups.size());
		for (auto& group : groups) {
			result.groups.push_back(std::move(group.result));
		}
	}
	result.total_ms = elapsed_ms(total_start);
	set_value(result, context, "leaf_sweep.free_boxes", static_cast<double>(result.free_boxes.size()));
	set_value(result, context, "leaf_sweep.collision_boxes", static_cast<double>(result.collision_boxes.size()));
	set_value(result, context, "leaf_sweep.initialize_ms", result.initialize_ms);
	set_value(result, context, "leaf_sweep.group_sweep_ms", result.group_sweep_ms);
	set_value(result, context, "leaf_sweep.compose_ms", result.compose_ms);
	set_value(result, context, "leaf_sweep.total_ms", result.total_ms);
	for (const auto& [key, value] : context.diagnostics().snapshot()) {
		result.diagnostics[key] = value;
	}
	return result;
}

}  // namespace rbf
