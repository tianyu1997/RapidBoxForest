#include <SBF/leaf_sweep_grower.h>

#include <rbf/lect_database.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace rbf {

namespace {

using Clock = std::chrono::steady_clock;

bool valid_oracle_node(OracleNodeId node) {
	return node >= 0;
}

double elapsed_ms(Clock::time_point start) {
	return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

void add_counter(LeafSweepResult& result,
				 StageContext& context,
				 const std::string& key,
				 double value = 1.0) {
	context.diagnostics().add_counter(key, value);
	result.diagnostics[key] += value;
}

void set_value(LeafSweepResult& result,
			   StageContext& context,
			   const std::string& key,
			   double value) {
	context.diagnostics().set_value(key, value);
	result.diagnostics[key] = value;
}

void record_timing(LeafSweepResult& result,
				   StageContext& context,
				   const std::string& key,
				   double milliseconds) {
	context.diagnostics().record_timing(key, milliseconds);
	result.diagnostics[key + ".total_ms"] += milliseconds;
	result.diagnostics[key + ".count"] += 1.0;
}

Eigen::VectorXd center_of_intervals(const std::vector<Interval>& intervals) {
	Eigen::VectorXd center(static_cast<int>(intervals.size()));
	for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
		center[dim] = intervals[static_cast<std::size_t>(dim)].center();
	}
	return center;
}

BoxNode make_box(DatabaseBoxOracle& oracle,
				 OracleNodeId node,
				 int id,
				 BoxSafetyStatus status,
				 bool strict_audit_required = false) {
	BoxNode box;
	box.id = id;
	box.joint_intervals = oracle.node_intervals(node);
	box.seed_config = center_of_intervals(box.joint_intervals);
	box.tree_id = node;
	box.parent_box_id = -1;
	box.root_id = id;
	box.safety_status = status;
	box.strict_audit_required = strict_audit_required;
	box.compute_volume();
	return box;
}

BoxNode make_box_from_intervals(const std::vector<Interval>& intervals,
								OracleNodeId node,
								int id,
								BoxSafetyStatus status,
								bool strict_audit_required = false) {
	BoxNode box;
	box.id = id;
	box.joint_intervals = intervals;
	box.seed_config = center_of_intervals(box.joint_intervals);
	box.tree_id = node;
	box.parent_box_id = -1;
	box.root_id = id;
	box.safety_status = status;
	box.strict_audit_required = strict_audit_required;
	box.compute_volume();
	return box;
}

Obstacle aggregate_obstacles(const std::vector<Obstacle>& obstacles) {
	if (obstacles.empty()) {
		return {};
	}
	Obstacle aggregate = obstacles.front();
	for (std::size_t i = 1; i < obstacles.size(); ++i) {
		for (int axis = 0; axis < 3; ++axis) {
			aggregate.bounds[axis] = std::min(aggregate.bounds[axis], obstacles[i].bounds[axis]);
			aggregate.bounds[axis + 3] = std::max(aggregate.bounds[axis + 3], obstacles[i].bounds[axis + 3]);
		}
	}
	return aggregate;
}

double aabb_distance(const Obstacle& lhs, const Obstacle& rhs) {
	double distance_sq = 0.0;
	for (int axis = 0; axis < 3; ++axis) {
		double gap = 0.0;
		if (lhs.bounds[axis + 3] < rhs.bounds[axis]) {
			gap = static_cast<double>(rhs.bounds[axis] - lhs.bounds[axis + 3]);
		} else if (rhs.bounds[axis + 3] < lhs.bounds[axis]) {
			gap = static_cast<double>(lhs.bounds[axis] - rhs.bounds[axis + 3]);
		}
		distance_sq += gap * gap;
	}
	return std::sqrt(distance_sq);
}

class UnionFind {
public:
	explicit UnionFind(int n) : parent_(static_cast<std::size_t>(n)), rank_(static_cast<std::size_t>(n), 0) {
		for (int i = 0; i < n; ++i) {
			parent_[static_cast<std::size_t>(i)] = i;
		}
	}

	int find(int item) {
		const std::size_t index = static_cast<std::size_t>(item);
		if (parent_[index] != item) {
			parent_[index] = find(parent_[index]);
		}
		return parent_[index];
	}

	void unite(int lhs, int rhs) {
		int root_lhs = find(lhs);
		int root_rhs = find(rhs);
		if (root_lhs == root_rhs) {
			return;
		}
		if (rank_[static_cast<std::size_t>(root_lhs)] < rank_[static_cast<std::size_t>(root_rhs)]) {
			std::swap(root_lhs, root_rhs);
		}
		parent_[static_cast<std::size_t>(root_rhs)] = root_lhs;
		if (rank_[static_cast<std::size_t>(root_lhs)] == rank_[static_cast<std::size_t>(root_rhs)]) {
			rank_[static_cast<std::size_t>(root_lhs)] += 1;
		}
	}

private:
	std::vector<int> parent_;
	std::vector<int> rank_;
};

class ScopedOracleEnvelopeCache {
public:
	ScopedOracleEnvelopeCache(DatabaseBoxOracle& oracle, bool enabled)
		: oracle_(oracle), previous_(oracle.envelope_cache_enabled()) {
		oracle_.set_envelope_cache_enabled(enabled);
	}

	~ScopedOracleEnvelopeCache() {
		oracle_.set_envelope_cache_enabled(previous_);
	}

	ScopedOracleEnvelopeCache(const ScopedOracleEnvelopeCache&) = delete;
	ScopedOracleEnvelopeCache& operator=(const ScopedOracleEnvelopeCache&) = delete;

private:
	DatabaseBoxOracle& oracle_;
	bool previous_ = true;
};

}  // namespace

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
	const auto total_start = Clock::now();
	oracle_.set_scene(Scene(obstacles));
	set_value(result, context, "leaf_sweep.start_depth", static_cast<double>(start_depth));
	set_value(result, context, "leaf_sweep.max_depth", static_cast<double>(effective_max_depth));
	set_value(result, context, "leaf_sweep.obstacles", static_cast<double>(obstacles.size()));
	set_value(result, context, "leaf_sweep.executor_threads", static_cast<double>(context.executor().n_threads()));
	set_value(result, context, "leaf_sweep.validation_batch_size",
			  static_cast<double>(std::max(1, config_.validation_batch_size)));
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

std::vector<LeafSweepGrower::GroupWork> LeafSweepGrower::cluster_obstacles(
	const std::vector<Obstacle>& obstacles,
	LeafSweepResult& result,
	StageContext& context) const {
	std::vector<GroupWork> groups;
	result.obstacle_group_ids.assign(obstacles.size(), -1);
	if (obstacles.empty()) {
		GroupWork group;
		group.result.group_id = 0;
		group.result.aggregate_obstacle = {};
		groups.push_back(std::move(group));
		set_value(result, context, "leaf_sweep.groups", 1.0);
		return groups;
	}

	const int n = static_cast<int>(obstacles.size());
	UnionFind uf(n);
	const double cluster_gap = std::max(0.0, config_.obstacle_cluster_gap);
	for (int i = 0; i < n; ++i) {
		for (int j = i + 1; j < n; ++j) {
			if (aabb_distance(obstacles[static_cast<std::size_t>(i)],
							  obstacles[static_cast<std::size_t>(j)]) <= cluster_gap) {
				uf.unite(i, j);
			}
		}
	}

	std::unordered_map<int, int> root_to_group;
	for (int i = 0; i < n; ++i) {
		const int root = uf.find(i);
		auto it = root_to_group.find(root);
		if (it == root_to_group.end()) {
			const int group_id = static_cast<int>(groups.size());
			it = root_to_group.emplace(root, group_id).first;
			GroupWork group;
			group.result.group_id = group_id;
			groups.push_back(std::move(group));
		}
		const int group_id = it->second;
		result.obstacle_group_ids[static_cast<std::size_t>(i)] = group_id;
		auto& group = groups[static_cast<std::size_t>(group_id)].result;
		group.obstacle_indices.push_back(i);
		group.obstacles.push_back(obstacles[static_cast<std::size_t>(i)]);
	}

	for (auto& group : groups) {
		group.result.aggregate_obstacle = aggregate_obstacles(group.result.obstacles);
	}
	set_value(result, context, "leaf_sweep.groups", static_cast<double>(groups.size()));
	return groups;
}

std::vector<LeafSweepGrower::PendingNode> LeafSweepGrower::materialize_start_frontier(
	int start_depth,
	int max_depth,
	StageContext& context,
	LeafSweepResult& result) {
	(void)max_depth;
	std::vector<PendingNode> frontier;
	PendingNode root;
	root.node = oracle_.root_node();
	root.changed_dim = -1;
	if (config_.use_virtual_topology) {
		root.intervals = oracle_.root_intervals();
	}
	frontier.push_back(std::move(root));
	for (int depth = 0; depth < start_depth; ++depth) {
		std::vector<PendingNode> next;
		for (const auto& item : frontier) {
			if (context.should_stop()) {
				result.deadline_reached = context.deadline().expired();
				add_counter(result, context, "leaf_sweep.initialize_deadline");
				next.push_back(item);
				continue;
			}
			if (!valid_oracle_node(item.node)) {
				add_counter(result, context, "leaf_sweep.invalid_frontier_nodes");
				continue;
			}
			if (config_.use_virtual_topology) {
				PendingNode left;
				PendingNode right;
				if (!virtual_split_node(item, depth, left, right)) {
					add_counter(result, context, "leaf_sweep.initialize_split_failures");
					next.push_back(item);
					continue;
				}
				add_counter(result, context, "leaf_sweep.initialize_splits");
				next.push_back(std::move(left));
				next.push_back(std::move(right));
				continue;
			}
			if (oracle_.is_leaf(item.node)) {
				const auto intervals = oracle_.node_intervals(item.node);
				const auto split_start = Clock::now();
				const auto split = oracle_.split_node(item.node, intervals, item.changed_dim, split_options_);
				record_timing(result,
							  context,
							  "profile.oracle.split_node",
							  std::chrono::duration<double, std::milli>(Clock::now() - split_start).count());
				if (!split.split) {
					add_counter(result, context, "leaf_sweep.initialize_split_failures");
					next.push_back(item);
					continue;
				}
				add_counter(result, context, "leaf_sweep.initialize_splits");
			}
			const int split_dim = oracle_.split_dim(item.node);
			const OracleNodeId left = oracle_.left_child(item.node);
			const OracleNodeId right = oracle_.right_child(item.node);
			if (valid_oracle_node(left)) {
				PendingNode child;
				child.node = left;
				child.changed_dim = split_dim;
				next.push_back(std::move(child));
			}
			if (valid_oracle_node(right)) {
				PendingNode child;
				child.node = right;
				child.changed_dim = split_dim;
				next.push_back(std::move(child));
			}
		}
		frontier = std::move(next);
		if (frontier.empty()) {
			break;
		}
	}
	return frontier;
}

void LeafSweepGrower::sweep_group(GroupWork& group,
								  const std::vector<PendingNode>& start_frontier,
								  int max_depth,
								  StageContext& context,
								  LeafSweepResult& result) {
	oracle_.set_scene(Scene(group.result.obstacles));
	std::vector<PendingNode> pending = start_frontier;
	std::size_t index = 0;
	int next_free_id = 0;
	int next_collision_id = 0;
	if (config_.use_virtual_topology && config_.parallel_virtual_validation && context.executor().n_threads() > 1) {
		struct ValidationOutcome {
			BoxValidation validation = BoxValidation::Unknown;
			OracleValidationDetail detail;
			bool exception = false;
		};
		const int n_workers = std::max(1, context.executor().n_threads());
		std::vector<std::unique_ptr<DatabaseBoxOracle>> worker_oracles;
		worker_oracles.reserve(static_cast<std::size_t>(n_workers));
		for (int worker = 0; worker < n_workers; ++worker) {
			auto worker_validation_config = oracle_.validation_config();
			worker_validation_config.store_endpoint_evidence_cache = false;
			worker_validation_config.external_evidence_backfill_active = false;
			auto worker_oracle = std::make_unique<DatabaseBoxOracle>(oracle_.robot(),
																	 oracle_.database(),
																	 oracle_.scene(),
																	 oracle_.endpoint_config(),
																	 oracle_.envelope_config(),
																	 worker_validation_config,
																	 oracle_.external_evidence_source(),
																	 oracle_.direct_external_evidence_database());
			worker_oracle->set_envelope_cache_enabled(oracle_.envelope_cache_enabled());
			if (worker_validation_config.enable_worker_shared_endpoint_cache) {
				worker_oracle->set_shared_endpoint_cache(oracle_.shared_endpoint_cache());
			}
			worker_oracles.push_back(std::move(worker_oracle));
		}
		add_counter(result, context, "leaf_sweep.parallel_validation_sessions", static_cast<double>(worker_oracles.size()));
		const std::size_t batch_size = static_cast<std::size_t>(std::max(1, config_.validation_batch_size));
		while (index < pending.size()) {
			if (context.should_stop()) {
				result.deadline_reached = context.deadline().expired();
				add_counter(result, context, "leaf_sweep.group_deadline");
				for (; index < pending.size(); ++index) {
					if (!valid_oracle_node(pending[index].node)) {
						continue;
					}
					group.collision_nodes.push_back(pending[index].node);
					group.result.collision_boxes.push_back(
						make_box_from_intervals(pending[index].intervals,
												pending[index].node,
												next_collision_id++,
												BoxSafetyStatus::Occupied));
				}
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
					outcome.validation = worker_oracle.validate_node(item.node, item.intervals, item.changed_dim);
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
				if (outcome.exception) {
					add_counter(result, context, "leaf_sweep.validation_exceptions");
				}
				add_counter(result, context, "leaf_sweep.node_validations");
				if (outcome.validation == BoxValidation::Free) {
					group.free_nodes.push_back(item.node);
					group.result.free_boxes.push_back(
						make_box_from_intervals(item.intervals,
												item.node,
												next_free_id++,
												outcome.detail.safety_status,
												outcome.detail.strict_audit_required));
					continue;
				}
				const int depth = virtual_depth(item.node);
				if (depth >= max_depth) {
					group.collision_nodes.push_back(item.node);
					group.result.collision_boxes.push_back(
						make_box_from_intervals(item.intervals,
												item.node,
												next_collision_id++,
												BoxSafetyStatus::Occupied));
					continue;
				}
				PendingNode left;
				PendingNode right;
				if (!virtual_split_node(item, depth, left, right)) {
					add_counter(result, context, "leaf_sweep.split_failures");
					group.collision_nodes.push_back(item.node);
					group.result.collision_boxes.push_back(
						make_box_from_intervals(item.intervals,
												item.node,
												next_collision_id++,
												BoxSafetyStatus::Occupied));
					continue;
				}
				add_counter(result, context, "leaf_sweep.splits");
				pending.push_back(std::move(left));
				pending.push_back(std::move(right));
			}
			index = batch_end;
		}
		for (const auto& worker_oracle : worker_oracles) {
			if (!worker_oracle) {
				continue;
			}
			const auto& counters = worker_oracle->counters();
			add_counter(result,
						context,
						"leaf_sweep.worker_oracle.node_validations",
						static_cast<double>(counters.node_validations));
			add_counter(result,
						context,
						"leaf_sweep.worker_oracle.materialization_reused_external_evidence",
						static_cast<double>(counters.materialization_reused_external_evidence));
			add_counter(result,
						context,
						"leaf_sweep.worker_oracle.validate_node_total_time_us",
						counters.validate_node_total_time_us);
			add_counter(result,
						context,
						"leaf_sweep.worker_oracle.materialization_external_lookup_time_us",
						counters.materialization_external_lookup_time_us);
			add_counter(result,
						context,
						"leaf_sweep.worker_oracle.materialization_envelope_compute_time_us",
						counters.materialization_envelope_compute_time_us);
			add_counter(result,
						context,
						"leaf_sweep.worker_oracle.materialization_envelope_collision_time_us",
						counters.materialization_envelope_collision_time_us);
		}
		add_counter(result, context, "leaf_sweep.group_free_boxes",
					static_cast<double>(group.result.free_boxes.size()));
		add_counter(result, context, "leaf_sweep.group_collision_boxes",
					static_cast<double>(group.result.collision_boxes.size()));
		return;
	}
	while (index < pending.size()) {
		const PendingNode item = pending[index++];
		if (context.should_stop()) {
			result.deadline_reached = context.deadline().expired();
			add_counter(result, context, "leaf_sweep.group_deadline");
			if (valid_oracle_node(item.node)) {
				group.collision_nodes.push_back(item.node);
				group.result.collision_boxes.push_back(
					item.intervals.empty()
						? make_box(oracle_, item.node, next_collision_id++, BoxSafetyStatus::Occupied)
						: make_box_from_intervals(item.intervals,
												  item.node,
												  next_collision_id++,
												  BoxSafetyStatus::Occupied));
			}
			for (; index < pending.size(); ++index) {
				if (!valid_oracle_node(pending[index].node)) {
					continue;
				}
				group.collision_nodes.push_back(pending[index].node);
				group.result.collision_boxes.push_back(
					pending[index].intervals.empty()
						? make_box(oracle_, pending[index].node, next_collision_id++, BoxSafetyStatus::Occupied)
						: make_box_from_intervals(pending[index].intervals,
												  pending[index].node,
												  next_collision_id++,
												  BoxSafetyStatus::Occupied));
			}
			break;
		}
		if (!valid_oracle_node(item.node)) {
			add_counter(result, context, "leaf_sweep.invalid_pending_nodes");
			continue;
		}
		const auto intervals = item.intervals.empty() ? oracle_.node_intervals(item.node) : item.intervals;
		BoxValidation validation = BoxValidation::Unknown;
		try {
			validation = oracle_.validate_node(item.node, intervals, item.changed_dim);
		} catch (...) {
			add_counter(result, context, "leaf_sweep.validation_exceptions");
			validation = BoxValidation::Unknown;
		}
		add_counter(result, context, "leaf_sweep.node_validations");
		if (validation == BoxValidation::Free) {
			const auto detail = oracle_.last_validation_detail();
			group.free_nodes.push_back(item.node);
			group.result.free_boxes.push_back(
				item.intervals.empty()
					? make_box(oracle_,
							   item.node,
							   next_free_id++,
							   detail.safety_status,
							   detail.strict_audit_required)
					: make_box_from_intervals(item.intervals,
											  item.node,
											  next_free_id++,
											  detail.safety_status,
											  detail.strict_audit_required));
			continue;
		}

		const int depth = config_.use_virtual_topology ? virtual_depth(item.node) : oracle_.depth(item.node);
		if (depth >= max_depth) {
			group.collision_nodes.push_back(item.node);
			group.result.collision_boxes.push_back(
				item.intervals.empty()
					? make_box(oracle_, item.node, next_collision_id++, BoxSafetyStatus::Occupied)
					: make_box_from_intervals(item.intervals,
											  item.node,
											  next_collision_id++,
											  BoxSafetyStatus::Occupied));
			continue;
		}

		if (config_.use_virtual_topology) {
			PendingNode left;
			PendingNode right;
			if (!virtual_split_node(item, depth, left, right)) {
				add_counter(result, context, "leaf_sweep.split_failures");
				group.collision_nodes.push_back(item.node);
				group.result.collision_boxes.push_back(
					make_box_from_intervals(item.intervals,
											item.node,
											next_collision_id++,
											BoxSafetyStatus::Occupied));
				continue;
			}
			add_counter(result, context, "leaf_sweep.splits");
			pending.push_back(std::move(left));
			pending.push_back(std::move(right));
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
				group.collision_nodes.push_back(item.node);
				group.result.collision_boxes.push_back(
					make_box(oracle_, item.node, next_collision_id++, BoxSafetyStatus::Occupied));
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
			pending.push_back(std::move(child));
		}
		if (valid_oracle_node(right)) {
			PendingNode child;
			child.node = right;
			child.changed_dim = split_dim;
			pending.push_back(std::move(child));
		}
	}
	add_counter(result, context, "leaf_sweep.group_free_boxes",
				static_cast<double>(group.result.free_boxes.size()));
	add_counter(result, context, "leaf_sweep.group_collision_boxes",
				static_cast<double>(group.result.collision_boxes.size()));
}

void LeafSweepGrower::compose_final_sets(const std::vector<GroupWork>& groups,
										 int start_depth,
										 int max_depth,
										 StageContext& context,
										 LeafSweepResult& result) const {
	if (groups.size() > 64) {
		throw std::invalid_argument("LeafSweepGrower compose currently supports up to 64 obstacle groups");
	}
	std::vector<std::unordered_set<OracleNodeId>> free_sets;
	std::vector<std::unordered_set<OracleNodeId>> collision_sets;
	free_sets.reserve(groups.size());
	collision_sets.reserve(groups.size());
	for (const auto& group : groups) {
		free_sets.emplace_back(group.free_nodes.begin(), group.free_nodes.end());
		collision_sets.emplace_back(group.collision_nodes.begin(), group.collision_nodes.end());
	}
	const std::uint64_t all_free_mask =
		groups.size() == 64 ? ~std::uint64_t{0}
							: ((std::uint64_t{1} << groups.size()) - std::uint64_t{1});

	struct ComposeNode {
		OracleNodeId node = kInvalidOracleNodeId;
		int changed_dim = -1;
		std::uint64_t free_mask = 0;
		std::uint64_t collision_mask = 0;
	};

	std::queue<ComposeNode> queue;
	queue.push({oracle_.root_node(), -1, 0, 0});
	int next_free_id = 0;
	int next_collision_id = 0;
	while (!queue.empty()) {
		if (context.should_stop()) {
			result.deadline_reached = context.deadline().expired();
			add_counter(result, context, "leaf_sweep.compose_deadline");
			break;
		}
		ComposeNode item = queue.front();
		queue.pop();
		if (!valid_oracle_node(item.node)) {
			continue;
		}
		for (std::size_t group_index = 0; group_index < groups.size(); ++group_index) {
			const std::uint64_t bit = std::uint64_t{1} << group_index;
			if ((item.collision_mask & bit) == 0 &&
				collision_sets[group_index].find(item.node) != collision_sets[group_index].end()) {
				item.collision_mask |= bit;
			}
			if ((item.free_mask & bit) == 0 &&
				free_sets[group_index].find(item.node) != free_sets[group_index].end()) {
				item.free_mask |= bit;
			}
		}
		const int depth = oracle_.depth(item.node);
		if (depth < start_depth) {
			if (oracle_.is_leaf(item.node)) {
				add_counter(result, context, "leaf_sweep.compose_shallow_leaf_collision");
				result.collision_boxes.push_back(
					make_box(oracle_, item.node, next_collision_id++, BoxSafetyStatus::Occupied));
				continue;
			}
			const int split_dim = oracle_.split_dim(item.node);
			queue.push({oracle_.left_child(item.node), split_dim, item.free_mask, item.collision_mask});
			queue.push({oracle_.right_child(item.node), split_dim, item.free_mask, item.collision_mask});
			continue;
		}

		if (item.collision_mask != 0) {
			result.collision_boxes.push_back(
				make_box(oracle_, item.node, next_collision_id++, BoxSafetyStatus::Occupied));
			continue;
		}

		if (item.free_mask == all_free_mask) {
			result.free_boxes.push_back(
				make_box(oracle_, item.node, next_free_id++, BoxSafetyStatus::CertifiedFree));
			continue;
		}

		if (depth >= max_depth || oracle_.is_leaf(item.node)) {
			add_counter(result, context, "leaf_sweep.compose_unclassified_collision");
			result.collision_boxes.push_back(
				make_box(oracle_, item.node, next_collision_id++, BoxSafetyStatus::Occupied));
			continue;
		}

		const int split_dim = oracle_.split_dim(item.node);
		queue.push({oracle_.left_child(item.node), split_dim, item.free_mask, item.collision_mask});
		queue.push({oracle_.right_child(item.node), split_dim, item.free_mask, item.collision_mask});
	}
}

}  // namespace rbf
