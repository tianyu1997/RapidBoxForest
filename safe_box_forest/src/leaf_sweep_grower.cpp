#include <SBF/leaf_sweep_grower.h>

#include "leaf_sweep_grower_diagnostics.h"
#include "leaf_sweep_grower_internal.h"

#include <rbf/lect_database.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace rbf {

LeafSweepGrower::LeafSweepGrower(DatabaseBoxOracle& oracle,
								 LeafSweepConfig config,
								 OracleSplitOptions split_options)
	: oracle_(oracle), config_(std::move(config)), split_options_(std::move(split_options)) {}

int LeafSweepGrower::virtual_depth(OracleNodeId node) const {
	if (node <= 0) {
		return 0;
	}
	std::uint64_t value = static_cast<std::uint64_t>(node) + 1u;
	int depth = -1;
	while (value != 0u) {
		value >>= 1u;
		++depth;
	}
	return std::max(0, depth);
}

bool LeafSweepGrower::virtual_split_node(const PendingNode& item,
										 int depth,
										 PendingNode& left,
										 PendingNode& right) const {
	if (item.node < 0 || item.intervals.empty()) {
		return false;
	}
	const auto& descriptor = oracle_.database().split_policy_descriptor();
	int split_dim = -1;
	if (!descriptor.depth_dimensions.empty() &&
		depth >= 0 &&
		depth < static_cast<int>(descriptor.depth_dimensions.size())) {
		split_dim = descriptor.depth_dimensions[static_cast<std::size_t>(depth)];
	} else if (!item.intervals.empty()) {
		split_dim = depth % static_cast<int>(item.intervals.size());
	}
	if (split_dim < 0 || split_dim >= static_cast<int>(item.intervals.size())) {
		return false;
	}
	const auto dim = static_cast<std::size_t>(split_dim);
	const double split_value = item.intervals[dim].center();
	if (!(split_value > item.intervals[dim].lo && split_value < item.intervals[dim].hi)) {
		return false;
	}
	left.node = static_cast<OracleNodeId>(2 * static_cast<std::uint64_t>(item.node) + 1u);
	left.changed_dim = split_dim;
	left.intervals = item.intervals;
	left.intervals[dim].hi = split_value;
	right.node = static_cast<OracleNodeId>(2 * static_cast<std::uint64_t>(item.node) + 2u);
	right.changed_dim = split_dim;
	right.intervals = item.intervals;
	right.intervals[dim].lo = split_value;
	return true;
}

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

void LeafSweepGrower::sweep_group(GroupWork& group,
								  const std::vector<PendingNode>& start_frontier,
								  int max_depth,
								  StageContext& context,
								  LeafSweepResult& result) {
	oracle_.set_scene(Scene(group.result.obstacles));
	const auto planning_domain = oracle_.planning_intervals();
	std::vector<PendingNode> pending = start_frontier;
	std::size_t index = 0;
	int next_free_id = 0;
	for (const auto& box : group.result.free_boxes) {
		next_free_id = std::max(next_free_id, box.id + 1);
	}
	int next_collision_id = 0;
	for (const auto& box : group.result.collision_boxes) {
		next_collision_id = std::max(next_collision_id, box.id + 1);
	}
	auto append_collision_box = [&](OracleNodeId node,
									const std::vector<Interval>& intervals,
									BoxSafetyStatus status = BoxSafetyStatus::Occupied) {
		if (config_.max_collision_boxes > 0 &&
			static_cast<int>(group.result.collision_boxes.size()) >= config_.max_collision_boxes) {
			add_counter(result, context, "leaf_sweep.collision_boxes_dropped_by_cap");
			return;
		}
		group.collision_nodes.push_back(node);
		group.result.collision_boxes.push_back(
			make_box_from_intervals(intervals,
									node,
									next_collision_id++,
									status));
	};
	auto record_dropped_pending_after_stop = [&](std::size_t dropped) {
		if (dropped > 0) {
			add_counter(result,
						context,
						"leaf_sweep.pending_nodes_dropped_after_stop",
						static_cast<double>(dropped));
		}
	};
	auto append_free_box = [&](OracleNodeId node,
							   const std::vector<Interval>& intervals,
							   const OracleValidationDetail& detail) {
		if (config_.max_free_boxes > 0 &&
			static_cast<int>(group.result.free_boxes.size()) >= config_.max_free_boxes) {
			add_counter(result, context, "leaf_sweep.free_boxes_dropped_by_cap");
			return;
		}
		group.free_nodes.push_back(node);
		group.result.free_boxes.push_back(
			make_box_from_intervals(intervals,
									node,
									next_free_id++,
									detail.safety_status,
									detail.strict_audit_required));
	};
	std::vector<int> checkpoint_depths = config_.checkpoint_depths;
	checkpoint_depths.erase(
		std::remove_if(checkpoint_depths.begin(),
					   checkpoint_depths.end(),
					   [&](int depth) { return depth < 0 || depth > max_depth; }),
		checkpoint_depths.end());
	std::sort(checkpoint_depths.begin(), checkpoint_depths.end());
	checkpoint_depths.erase(std::unique(checkpoint_depths.begin(), checkpoint_depths.end()),
							checkpoint_depths.end());
	const bool checkpoint_enabled =
		static_cast<bool>(config_.checkpoint_callback) && !checkpoint_depths.empty();
	std::size_t checkpoint_index = 0;
	int active_max_depth = checkpoint_enabled ? checkpoint_depths.front() : max_depth;
	auto make_checkpoint_snapshot = [&](int depth) {
		LeafSweepResult snapshot;
		snapshot.free_boxes = group.result.free_boxes;
		snapshot.collision_boxes = group.result.collision_boxes;
		snapshot.collision_box_obstacle_indices.assign(snapshot.collision_boxes.size(),
													   group.result.obstacle_indices);
		snapshot.deadline_reached = result.deadline_reached;
		snapshot.group_sweep_ms = result.group_sweep_ms;
		snapshot.total_ms = result.total_ms;
		snapshot.diagnostics = result.diagnostics;
		set_value(snapshot, context, "leaf_sweep.checkpoint_depth", static_cast<double>(depth));
		set_value(snapshot,
				  context,
				  "leaf_sweep.checkpoint_free_boxes",
				  static_cast<double>(snapshot.free_boxes.size()));
		set_value(snapshot,
				  context,
				  "leaf_sweep.checkpoint_collision_boxes",
				  static_cast<double>(snapshot.collision_boxes.size()));
		return snapshot;
	};
	auto advance_checkpoint_or_finish = [&]() -> bool {
		if (!checkpoint_enabled) {
			return true;
		}
		const int checkpoint_depth = active_max_depth;
		auto snapshot = make_checkpoint_snapshot(checkpoint_depth);
		const bool stop = config_.checkpoint_callback(snapshot, checkpoint_depth);
		add_counter(result, context, "leaf_sweep.checkpoints_evaluated");
		if (stop) {
			add_counter(result, context, "leaf_sweep.checkpoint_stop");
			return true;
		}
		if (checkpoint_depth >= max_depth) {
			return true;
		}
		pending.clear();
		pending.reserve(group.result.collision_boxes.size());
		for (const auto& box : group.result.collision_boxes) {
			if (!valid_oracle_node(box.tree_id)) {
				add_counter(result, context, "leaf_sweep.checkpoint_invalid_frontier_nodes");
				continue;
			}
			PendingNode item;
			item.node = box.tree_id;
			item.changed_dim = -1;
			item.intervals = box.joint_intervals;
			pending.push_back(std::move(item));
		}
		group.result.collision_boxes.clear();
		group.collision_nodes.clear();
		next_collision_id = 0;
		index = 0;
		if (pending.empty()) {
			add_counter(result, context, "leaf_sweep.checkpoint_empty_frontier");
			return true;
		}
		while (checkpoint_index + 1 < checkpoint_depths.size() &&
			   checkpoint_depths[checkpoint_index] <= checkpoint_depth) {
			++checkpoint_index;
		}
		active_max_depth = checkpoint_index < checkpoint_depths.size()
			? checkpoint_depths[checkpoint_index]
			: max_depth;
		if (active_max_depth <= checkpoint_depth) {
			active_max_depth = max_depth;
		}
		add_counter(result, context, "leaf_sweep.checkpoint_continue");
		return false;
	};
	if (config_.use_virtual_topology && config_.parallel_virtual_validation && context.executor().n_threads() > 1) {
		struct ValidationOutcome {
			BoxValidation validation = BoxValidation::Unknown;
			OracleValidationDetail detail;
			std::vector<Interval> commit_intervals;
			bool outside_planning_domain = false;
			bool clipped_to_planning_domain = false;
			bool exception = false;
		};
		const int n_workers = std::max(1, context.executor().n_threads());
		std::vector<std::unique_ptr<DatabaseBoxOracle>> worker_oracles;
		worker_oracles.reserve(static_cast<std::size_t>(n_workers));
		for (int worker = 0; worker < n_workers; ++worker) {
			auto worker_validation_config = oracle_.validation_config();
			worker_validation_config.store_endpoint_evidence_cache = false;
			worker_validation_config.external_evidence_backfill_active = false;
			worker_validation_config.collect_full_overlap_stats =
				config_.collision_overlap_prune_ratio_threshold > 0.0 &&
				config_.collision_overlap_prune_min_depth >= 0;
			auto worker_oracle = std::make_unique<DatabaseBoxOracle>(oracle_.robot(),
																	 oracle_.database(),
																	 oracle_.scene(),
																	 oracle_.endpoint_config(),
																	 oracle_.envelope_config(),
																	 worker_validation_config,
																	 oracle_.external_evidence_source(),
																	 nullptr);
			worker_oracle->set_envelope_cache_enabled(oracle_.envelope_cache_enabled());
			if (worker_validation_config.enable_worker_shared_endpoint_cache) {
				worker_oracle->set_shared_endpoint_cache(oracle_.shared_endpoint_cache());
			}
			worker_oracles.push_back(std::move(worker_oracle));
		}
		add_counter(result, context, "leaf_sweep.parallel_validation_sessions", static_cast<double>(worker_oracles.size()));
		const std::size_t batch_size = static_cast<std::size_t>(std::max(1, config_.validation_batch_size));
		while (true) {
			while (index < pending.size()) {
				if (context.should_stop()) {
					result.deadline_reached = context.deadline().expired();
					add_counter(result, context, "leaf_sweep.group_deadline");
					record_dropped_pending_after_stop(pending.size() - index);
					index = pending.size();
					break;
				}
				const std::size_t batch_begin = index;
				const std::size_t batch_end = std::min(pending.size(), batch_begin + batch_size);
				std::vector<ValidationOutcome> outcomes(batch_end - batch_begin);
				context.executor().parallel_for(0, static_cast<int>(outcomes.size()), [&](int local_index) {
					const int worker_id = std::max(0, current_worker_id());
					BoxOracle& worker_oracle =
						worker_id < static_cast<int>(worker_oracles.size()) &&
							  worker_oracles[static_cast<std::size_t>(worker_id)]
							? static_cast<BoxOracle&>(*worker_oracles[static_cast<std::size_t>(worker_id)])
							: static_cast<BoxOracle&>(oracle_);
					const PendingNode& item = pending[batch_begin + static_cast<std::size_t>(local_index)];
					auto& outcome = outcomes[static_cast<std::size_t>(local_index)];
					try {
						const auto intervals = item.intervals.empty() ? worker_oracle.node_intervals(item.node) : item.intervals;
						if (!intervals_overlap_domain(intervals, planning_domain, 0.0)) {
							outcome.outside_planning_domain = true;
							return;
						}
						outcome.commit_intervals = intervals;
						if (!clip_intervals_to_domain(outcome.commit_intervals, planning_domain)) {
							outcome.outside_planning_domain = true;
							outcome.commit_intervals.clear();
							return;
						}
						outcome.clipped_to_planning_domain = !intervals_same_exact(outcome.commit_intervals, intervals);
						outcome.validation = worker_oracle.validate_node(item.node, intervals, item.changed_dim);
						outcome.detail = worker_oracle.last_validation_detail();
					} catch (...) {
						outcome.validation = BoxValidation::Unknown;
						outcome.exception = true;
					}
				});
				add_counter(result, context, "leaf_sweep.parallel_validation_batches");
				for (std::size_t local = 0; local < outcomes.size(); ++local) {
					const PendingNode& item = pending[batch_begin + local];
					const auto& outcome = outcomes[local];
					if (!valid_oracle_node(item.node)) {
						add_counter(result, context, "leaf_sweep.invalid_pending_nodes");
						continue;
					}
					if (outcome.outside_planning_domain) {
						add_counter(result, context, "leaf_sweep.valid_domain_pruned_boxes");
						continue;
					}
					if (outcome.clipped_to_planning_domain) {
						add_counter(result, context, "leaf_sweep.valid_domain_clipped_boxes");
					}
					if (outcome.exception) {
						add_counter(result, context, "leaf_sweep.validation_exceptions");
					}
					add_counter(result, context, "leaf_sweep.node_validations");
					if (outcome.validation == BoxValidation::Free) {
						append_free_box(item.node, outcome.commit_intervals, outcome.detail);
						continue;
					}
					const int depth = virtual_depth(item.node);
					if (should_prune_by_overlap(config_, depth, outcome.detail)) {
						add_counter(result, context, "leaf_sweep.collision_overlap_pruned");
						append_collision_box(item.node, outcome.commit_intervals);
						continue;
					}
					if (depth >= active_max_depth) {
						append_collision_box(item.node, outcome.commit_intervals);
						continue;
					}
					PendingNode left;
					PendingNode right;
					if (!virtual_split_node(item, depth, left, right)) {
						add_counter(result, context, "leaf_sweep.split_failures");
						append_collision_box(item.node, outcome.commit_intervals);
						continue;
					}
					add_counter(result, context, "leaf_sweep.splits");
					if (intervals_overlap_domain(left.intervals, planning_domain, 0.0)) {
						pending.push_back(std::move(left));
					} else {
						add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
					}
					if (intervals_overlap_domain(right.intervals, planning_domain, 0.0)) {
						pending.push_back(std::move(right));
					} else {
						add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
					}
				}
				index = batch_end;
			}
			if (context.should_stop()) {
				if (checkpoint_enabled) {
					(void)advance_checkpoint_or_finish();
				}
				break;
			}
			if (advance_checkpoint_or_finish()) {
				break;
			}
		}
		for (const auto& worker_oracle : worker_oracles) {
			if (!worker_oracle) {
				continue;
			}
			record_leaf_sweep_worker_oracle_counters(result,
													 context,
													 worker_oracle->counters());
		}
		add_counter(result, context, "leaf_sweep.group_free_boxes",
					static_cast<double>(group.result.free_boxes.size()));
		add_counter(result, context, "leaf_sweep.group_collision_boxes",
					static_cast<double>(group.result.collision_boxes.size()));
		return;
	}
	while (true) {
		while (index < pending.size()) {
			const PendingNode item = pending[index++];
			if (context.should_stop()) {
				result.deadline_reached = context.deadline().expired();
				add_counter(result, context, "leaf_sweep.group_deadline");
				record_dropped_pending_after_stop(pending.size() - index + 1);
				index = pending.size();
				break;
			}
			if (!valid_oracle_node(item.node)) {
				add_counter(result, context, "leaf_sweep.invalid_pending_nodes");
				continue;
			}
			const auto intervals = item.intervals.empty() ? oracle_.node_intervals(item.node) : item.intervals;
			if (!intervals_overlap_domain(intervals, planning_domain, 0.0)) {
				add_counter(result, context, "leaf_sweep.valid_domain_pruned_boxes");
				continue;
			}
			auto commit_intervals = intervals;
			if (!clip_intervals_to_domain(commit_intervals, planning_domain)) {
				add_counter(result, context, "leaf_sweep.valid_domain_pruned_boxes");
				continue;
			}
			const bool clipped_to_planning_domain = !intervals_same_exact(commit_intervals, intervals);
			if (clipped_to_planning_domain) {
				add_counter(result, context, "leaf_sweep.valid_domain_clipped_boxes");
			}
			BoxValidation validation = BoxValidation::Unknown;
			try {
				validation = oracle_.validate_node(item.node, intervals, item.changed_dim);
			} catch (...) {
				add_counter(result, context, "leaf_sweep.validation_exceptions");
				validation = BoxValidation::Unknown;
			}
			const auto detail = oracle_.last_validation_detail();
			add_counter(result, context, "leaf_sweep.node_validations");
			if (validation == BoxValidation::Free) {
				append_free_box(item.node, commit_intervals, detail);
				continue;
			}

			const int depth = config_.use_virtual_topology ? virtual_depth(item.node) : oracle_.depth(item.node);
			if (should_prune_by_overlap(config_, depth, detail)) {
				add_counter(result, context, "leaf_sweep.collision_overlap_pruned");
				append_collision_box(item.node, commit_intervals);
				continue;
			}
			if (depth >= active_max_depth) {
				append_collision_box(item.node, commit_intervals);
				continue;
			}

			if (config_.use_virtual_topology) {
				PendingNode left;
				PendingNode right;
				if (!virtual_split_node(item, depth, left, right)) {
					add_counter(result, context, "leaf_sweep.split_failures");
					append_collision_box(item.node, commit_intervals);
					continue;
				}
				add_counter(result, context, "leaf_sweep.splits");
				if (intervals_overlap_domain(left.intervals, planning_domain, 0.0)) {
					pending.push_back(std::move(left));
				} else {
					add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
				}
				if (intervals_overlap_domain(right.intervals, planning_domain, 0.0)) {
					pending.push_back(std::move(right));
				} else {
					add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
				}
				continue;
			}

			if (oracle_.is_leaf(item.node)) {
				SplitNodeResult split;
				try {
					const auto split_start = Clock::now();
					split = oracle_.split_node(item.node, intervals, item.changed_dim, split_options_);
					record_timing(result,
								  context,
								  "profile.oracle.split_node",
								  std::chrono::duration<double, std::milli>(Clock::now() - split_start).count());
				} catch (...) {
					add_counter(result, context, "leaf_sweep.split_exceptions");
				}
				if (!split.split) {
					add_counter(result, context, "leaf_sweep.split_failures");
					append_collision_box(item.node, commit_intervals);
					continue;
				}
				add_counter(result, context, "leaf_sweep.splits");
			}
			const int split_dim = oracle_.split_dim(item.node);
			const OracleNodeId left = oracle_.left_child(item.node);
			const OracleNodeId right = oracle_.right_child(item.node);
			if (valid_oracle_node(left)) {
				PendingNode child;
				child.node = left;
				child.changed_dim = split_dim;
				if (intervals_overlap_domain(oracle_.node_intervals(child.node), planning_domain, 0.0)) {
					pending.push_back(std::move(child));
				} else {
					add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
				}
			}
			if (valid_oracle_node(right)) {
				PendingNode child;
				child.node = right;
				child.changed_dim = split_dim;
				if (intervals_overlap_domain(oracle_.node_intervals(child.node), planning_domain, 0.0)) {
					pending.push_back(std::move(child));
				} else {
					add_counter(result, context, "leaf_sweep.valid_domain_pruned_frontier");
				}
			}
		}
		if (context.should_stop()) {
			if (checkpoint_enabled) {
				(void)advance_checkpoint_or_finish();
			}
			break;
		}
		if (advance_checkpoint_or_finish()) {
			break;
		}
	}
	add_counter(result, context, "leaf_sweep.group_free_boxes",
				static_cast<double>(group.result.free_boxes.size()));
	add_counter(result, context, "leaf_sweep.group_collision_boxes",
				static_cast<double>(group.result.collision_boxes.size()));
}

}  // namespace rbf
