#include <LECTDatabase/sbf/oracle.h>

#include "oracle_best_tighten.h"
#include "oracle_canonical.h"
#include "oracle_endpoint_materialization.h"
#include "oracle_impl.h"
#include "oracle_options.h"
#include "oracle_support.h"

#include <sbf/core/joint_symmetry.h>
#include <link_interval_envelope/incremental_context.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <stdexcept>

namespace rbf {
namespace {

using lect_database::NodeId;
using Clock = std::chrono::steady_clock;

lect_database::NodeId to_database_node(OracleNodeId node) {
    if (node < 0 || static_cast<std::uint64_t>(node) > std::numeric_limits<lect_database::NodeId>::max()) {
        return lect_database::kInvalidNodeId;
    }
    return static_cast<lect_database::NodeId>(node);
}

OracleNodeId from_database_node(lect_database::NodeId node) {
    return lect_database::valid_node_id(node) ? static_cast<OracleNodeId>(node) : kInvalidOracleNodeId;
}

}  // namespace

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                                     lect_database::LectDatabase& database,
                                     Scene scene,
                                     EndpointSourceConfig endpoint_config,
                                     EnvelopeTypeConfig envelope_config,
                         OracleValidationConfig validation_config,
                                                                                                                                                 const lect_database::LectExternalEvidenceSource* external_evidence_source,
                                                                                                                                                 const lect_database::LectDatabase* direct_external_evidence_database)
    : robot_(std::move(robot)),
      database_(database),
        external_evidence_source_(external_evidence_source),
        direct_external_evidence_database_(direct_external_evidence_database),
      endpoint_config_(std::move(endpoint_config)),
      envelope_config_(std::move(envelope_config)),
      validation_config_(std::move(validation_config)),
      scene_(std::move(scene)),
      checker_(robot_, scene_),
      impl_(std::make_unique<Impl>()) {
    seed_best_tighten_schedule_from_policy();
}

DatabaseBoxOracle::DatabaseBoxOracle(Robot robot,
                         lect_database::OnlineEnvelopeCacheTree& online_cache,
                         Scene scene,
                         EndpointSourceConfig endpoint_config,
                         EnvelopeTypeConfig envelope_config,
                         OracleValidationConfig validation_config,
                                                                         const lect_database::LectExternalEvidenceSource* external_evidence_source,
                                                                         const lect_database::LectDatabase* direct_external_evidence_database)
    : robot_(std::move(robot)),
    database_(online_cache.database()),
    online_cache_(&online_cache),
    external_evidence_source_(external_evidence_source),
    direct_external_evidence_database_(direct_external_evidence_database),
    endpoint_config_(std::move(endpoint_config)),
    envelope_config_(std::move(envelope_config)),
    validation_config_(std::move(validation_config)),
    scene_(std::move(scene)),
    checker_(robot_, scene_),
    impl_(std::make_unique<Impl>()) {
    seed_best_tighten_schedule_from_policy();
}

DatabaseBoxOracle::~DatabaseBoxOracle() = default;

void DatabaseBoxOracle::seed_best_tighten_schedule_from_policy() {
    // Pre-seed the per-depth best-tighten split dimensions from the canonical
    // FixedDepthSchedule so the grower replays a (robot, domain)-determined
    // dimension sequence instead of lazily fixing it from the first query that
    // reaches each depth. This keeps node paths canonical (and therefore the
    // external-evidence keys stable / reusable) regardless of seed or query
    // order. Only applies to the AAFKVolumeMin FixedDepthSchedule strategy.
    const auto& policy = database_.split_policy_descriptor();
    if (policy.strategy != lect_database::SplitStrategy::AAFKVolumeMin) {
        return;
    }
    if (policy.depth_dimensions.empty()) {
        return;
    }
    best_tighten_depth_dims_ = policy.depth_dimensions;
}

SplitNodeResult DatabaseBoxOracle::split_node(OracleNodeId node,
                                              const std::vector<Interval>& intervals,
                                              int changed_dim,
                                              const OracleSplitOptions& options) {
    SplitNodeResult result;
    std::pair<lect_database::NodeId, lect_database::NodeId> children{
        lect_database::kInvalidNodeId,
        lect_database::kInvalidNodeId,
    };

    if (options.use_best_tighten && !intervals.empty()) {
        auto best_tighten_options = with_fk_effective_split_filter(options.best_tighten, robot_);
        try {
            const auto scoring_endpoint_config = hifk_config_for_materialization(*this, node, endpoint_config_);
            link_interval_envelope::IncrementalEnvelopeContext probe(
                robot_, scoring_endpoint_config, envelope_config_);
            int next_changed_dim_hint = changed_dim;
            if (best_tighten_reference_volumes_.empty()) {
                link_interval_envelope::IncrementalEnvelopeContext reference_probe(
                    robot_, endpoint_config_, envelope_config_);
                const auto reference = reference_probe.compute(robot_.joint_limits().limits, -1);
                best_tighten_reference_volumes_ = link_aabb_volumes(reference.envelope.link_iaabbs);
            }
            const auto symmetry = best_tighten_options.prefer_sector_boundary
                ? primary_database_symmetry(robot_, database_)
                : std::nullopt;
            const auto scorer = [this, &probe, &next_changed_dim_hint](const std::vector<Interval>& candidate_intervals) {
                next_changed_dim_hint = -1;
                // Best-tighten scores arbitrary candidate child intervals that jump
                // around the tree (parent -> left -> right -> next-dim left ...). The
                // incremental endpoint state (notably the heuristic CritSample source)
                // is only valid for single-dim diffs branching from the *same* parent,
                // not for these arbitrary jumps, which corrupts the scored envelope and
                // makes children appear larger than their parent. Force a full,
                // stateless recompute per candidate so the minimax scores are correct.
                probe.reset();
                const auto scored = probe.compute(candidate_intervals, -1);
                counters_.scoring_evaluations += 1;
                counters_.scoring_endpoint_time_us += scored.endpoint_time_us;
                counters_.scoring_envelope_time_us += scored.envelope_time_us;
                counters_.scoring_candidate_dirty_count += scored.endpoint.candidate_dirty_count;
                counters_.scoring_predh_rebuild_count += scored.endpoint.predh_rebuild_count;
                if (scored.changed_dim >= 0) {
                    counters_.scoring_changed_dim_inferred += 1;
                }
                if (scored.used_incremental_fk) {
                    counters_.scoring_incremental_fk += 1;
                }
                if (scored.used_source_incremental_state) {
                    counters_.scoring_source_incremental_state += 1;
                }
                if (scored.reused_fk) {
                    counters_.scoring_reused_fk += 1;
                }
                if (scored.reused_endpoint_cache) {
                    counters_.scoring_reused_endpoint_cache += 1;
                }
                return normalized_link_aabb_volume_score(scored.envelope.link_iaabbs,
                                                         best_tighten_reference_volumes_);
            };

            const auto candidate = choose_best_tighten_split(intervals,
                                                             depth(node),
                                                             scorer,
                                                             symmetry,
                                                             best_tighten_options,
                                                             best_tighten_depth_dims_,
                                                             best_tighten_recent_dims_);
            children = database_.split_leaf(to_database_node(node), candidate.dim, candidate.split_val);
            if (oracle_best_tighten_debug_enabled()) {
                static std::atomic<long> ok{0};
                long n = ++ok;
                if (n <= 20 || n % 200 == 0) {
                    std::fprintf(stderr,
                                 "[BT_OK] n=%ld dim=%d val=%.5f valid_children=%d\n",
                                 n, candidate.dim, candidate.split_val,
                                 (int)(lect_database::valid_node_id(children.first) &&
                                       lect_database::valid_node_id(children.second)));
                }
            }
        } catch (const std::exception& e) {
            if (oracle_best_tighten_debug_enabled()) {
                static std::atomic<long> bad{0};
                long n = ++bad;
                if (n <= 20 || n % 200 == 0) {
                    std::fprintf(stderr, "[BT_EXC] n=%ld what=%s\n", n, e.what());
                }
            }
        }
    }

    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        children = online_cache_ != nullptr
            ? online_cache_->split_leaf(to_database_node(node))
            : database_.split_leaf(to_database_node(node));
    }
    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        return result;
    }
    const auto topology = database_.topology(to_database_node(node));
    result.split = true;
    result.node = node;
    result.left = from_database_node(children.first);
    result.right = from_database_node(children.second);
    result.split_dim = topology.split_dim;
    result.split_value = topology.split_value;
    unexplored_leaf_cache_dirty_ = true;
    return result;
}

SplitNodeResult DatabaseBoxOracle::split_node_at(OracleNodeId node, int split_dim, double split_value) {
    SplitNodeResult result;
    const auto children = database_.split_leaf(to_database_node(node), split_dim, split_value);
    if (!lect_database::valid_node_id(children.first) || !lect_database::valid_node_id(children.second)) {
        return result;
    }
    const auto topology = database_.topology(to_database_node(node));
    result.split = true;
    result.node = node;
    result.left = from_database_node(children.first);
    result.right = from_database_node(children.second);
    result.split_dim = topology.split_dim;
    result.split_value = topology.split_value;
    unexplored_leaf_cache_dirty_ = true;
    return result;
}

bool DatabaseBoxOracle::point_in_collision(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    return checker_.check_config(q);
}

BoxValidation DatabaseBoxOracle::validate_node(OracleNodeId node,
                                               const std::vector<Interval>& intervals,
                                               int changed_dim) {
    const auto total_start = Clock::now();
    counters_.node_validations += 1;
    last_validation_detail_ = {};
    last_validation_detail_.node = node;
    last_validation_detail_.depth = depth(node);
    last_validation_detail_.mode = validation_config_.mode;
    const double preamble_us = elapsed_us(total_start);
    counters_.validate_node_preamble_time_us += preamble_us;
    if (scene_.empty()) {
        counters_.certified_free += 1;
        last_validation_detail_.validation = BoxValidation::Free;
        last_validation_detail_.safety_status = BoxSafetyStatus::CertifiedFree;
        last_validation_detail_.collision_possible = false;
        const double total_us = elapsed_us(total_start);
        counters_.validate_node_total_time_us += total_us;
        counters_.validate_node_overhead_time_us += std::max(0.0, total_us - preamble_us);
        return BoxValidation::Free;
    }
    const bool use_validation_cache =
        validation_config_.enable_validation_cache &&
        validation_config_.validation_cache_max_entries > 0;
    const std::uint64_t cache_key =
        use_validation_cache ? validation_cache_key(node, intervals, changed_dim) : 0;
    if (use_validation_cache) {
        const auto cache_it = validation_cache_.find(cache_key);
        if (cache_it != validation_cache_.end()) {
            counters_.validation_cache_hits += 1;
            last_validation_detail_ = cache_it->second.detail;
            const double total_us = elapsed_us(total_start);
            counters_.validate_node_total_time_us += total_us;
            counters_.validate_node_overhead_time_us += std::max(0.0, total_us - preamble_us);
            return cache_it->second.result;
        }
        counters_.validation_cache_misses += 1;
    }
    auto store_validation_cache = [&](BoxValidation result) {
        if (!use_validation_cache) {
            return;
        }
        if (validation_cache_.size() >=
            static_cast<std::size_t>(validation_config_.validation_cache_max_entries)) {
            validation_cache_.clear();
        }
        validation_cache_[cache_key] = ValidationCacheEntry{result, last_validation_detail_};
    };
    const auto endpoint_path_start = Clock::now();
    auto payload = endpoint_payload_for_node(node, intervals, changed_dim);
    const double endpoint_path_us = elapsed_us(endpoint_path_start);
    counters_.validate_node_endpoint_path_time_us += endpoint_path_us;
    if (!payload) {
        counters_.collision_possible += 1;
        last_validation_detail_.validation = BoxValidation::Unknown;
        last_validation_detail_.safety_status = BoxSafetyStatus::Unknown;
        last_validation_detail_.collision_possible = true;
        const double total_us = elapsed_us(total_start);
        counters_.validate_node_total_time_us += total_us;
        counters_.validate_node_overhead_time_us += std::max(0.0, total_us - preamble_us - endpoint_path_us);
        store_validation_cache(BoxValidation::Unknown);
        return BoxValidation::Unknown;
    }
    const auto classify_start = Clock::now();
    BoxValidation result = classify_payload(node, intervals, *payload);
    if (result == BoxValidation::Unknown &&
        validation_config_.external_evidence_live_retry_on_maybe &&
        payload->reused_external_evidence) {
        counters_.materialization_external_maybe_live_retries += 1;
        const auto live_endpoint_start = Clock::now();
        auto live_payload = endpoint_payload_for_node(node, intervals, changed_dim, /*allow_external_evidence=*/false);
        counters_.validate_node_endpoint_path_time_us += elapsed_us(live_endpoint_start);
        if (live_payload) {
            result = classify_payload(node, intervals, *live_payload);
            if (result == BoxValidation::Free) {
                counters_.materialization_external_maybe_live_retry_free += 1;
            }
        }
    }
    const double classify_us = elapsed_us(classify_start);
    counters_.validate_node_classify_time_us += classify_us;
    const double total_us = elapsed_us(total_start);
    counters_.validate_node_total_time_us += total_us;
    counters_.validate_node_overhead_time_us +=
        std::max(0.0, total_us - preamble_us - endpoint_path_us - classify_us);
    store_validation_cache(result);
    return result;
}

bool DatabaseBoxOracle::validate_intervals(const std::vector<Interval>& intervals) {
    counters_.interval_validations += 1;
    return !checker_.check_box(intervals);
}

bool DatabaseBoxOracle::is_reserved(OracleNodeId node) const {
    if (node < 0) {
        return false;
    }
    if (active_tree_is_primary_canonical_sector(robot_, database_)) {
        return false;
    }
    return node_to_box_.find(node) != node_to_box_.end();
}

std::optional<int> DatabaseBoxOracle::reservation_owner(OracleNodeId node) const {
    if (node < 0) {
        return std::nullopt;
    }
    if (active_tree_is_primary_canonical_sector(robot_, database_)) {
        return std::nullopt;
    }
    const auto it = node_to_box_.find(node);
    return it == node_to_box_.end() ? std::nullopt : std::optional<int>(it->second);
}

void DatabaseBoxOracle::reserve_node(OracleNodeId node, int box_id) {
    if (node < 0) {
        return;
    }
    if (active_tree_is_primary_canonical_sector(robot_, database_)) {
        return;
    }
    node_to_box_[node] = box_id;
    box_to_node_[box_id] = node;
}

void DatabaseBoxOracle::release_node(OracleNodeId node) {
    if (node < 0) {
        return;
    }
    const auto it = node_to_box_.find(node);
    if (it == node_to_box_.end()) {
        return;
    }
    box_to_node_.erase(it->second);
    node_to_box_.erase(it);
}

void DatabaseBoxOracle::release_box(int box_id) {
    const auto it = box_to_node_.find(box_id);
    if (it == box_to_node_.end()) {
        return;
    }
    node_to_box_.erase(it->second);
    box_to_node_.erase(it);
}

void DatabaseBoxOracle::clear_reservations() {
    node_to_box_.clear();
    box_to_node_.clear();
}

void DatabaseBoxOracle::record_visit(OracleNodeId node) {
    if (node >= 0) {
        ++visit_counts_[node];
    }
}

OracleNodeId DatabaseBoxOracle::select_unexplored_node() const {
    if (unexplored_leaf_cache_dirty_) {
        unexplored_leaf_cache_.clear();
        for (lect_database::NodeId node_id : database_.node_ids()) {
            const auto topology = database_.topology(node_id);
            if (!topology.leaf) {
                continue;
            }
            const OracleNodeId node = from_database_node(node_id);
            unexplored_leaf_cache_.push_back({node, interval_volume(node_intervals(node))});
        }
        unexplored_leaf_cache_dirty_ = false;
    }
    OracleNodeId best_node = kInvalidOracleNodeId;
    double best_weight = -1.0;
    for (const auto& entry : unexplored_leaf_cache_) {
        const OracleNodeId node = entry.node;
        if (node < 0 || is_reserved(node) || !is_leaf(node)) {
            continue;
        }
        const auto visit_it = visit_counts_.find(node);
        const double visit_count = visit_it == visit_counts_.end()
            ? 0.0
            : static_cast<double>(visit_it->second);
        const double weight = entry.volume / (visit_count + 1.0);
        if (weight > best_weight) {
            best_weight = weight;
            best_node = node;
        }
    }
    if (best_node >= 0) {
        ++visit_counts_[best_node];
    }
    return best_node;
}

int DatabaseBoxOracle::common_ancestor_depth(OracleNodeId lhs_node, OracleNodeId rhs_node) const {
    const auto lhs = to_database_node(lhs_node);
    const auto rhs = to_database_node(rhs_node);
    if (lhs_node < 0 || rhs_node < 0 || !database_.node(lhs) || !database_.node(rhs)) {
        return -1;
    }
    const auto ancestor = database_.lca(lhs, rhs);
    if (!lect_database::valid_node_id(ancestor)) {
        return -1;
    }
    return database_.topology(ancestor).depth;
}

std::unique_ptr<BoxOracleSession> DatabaseBoxOracle::make_session(const OracleSessionConfig& config) {
    return std::make_unique<DatabaseBoxOracleSession>(*this, config);
}

std::shared_ptr<lect_database::SharedEndpointEvidenceCache> DatabaseBoxOracle::shared_endpoint_cache() {
    if (!shared_endpoint_cache_) {
        shared_endpoint_cache_ = std::make_shared<lect_database::SharedEndpointEvidenceCache>(
            validation_config_.shared_endpoint_cache_max_entries,
            validation_config_.shared_endpoint_cache_max_bytes);
    }
    return shared_endpoint_cache_;
}

void DatabaseBoxOracle::set_scene(Scene scene) {
    scene_ = std::move(scene);
    checker_.set_scene(scene_);
    validation_cache_.clear();
}

std::unique_ptr<BoxOracleSession> DatabaseBoxOracleFactory::make_session(const OracleSessionConfig& config) {
    return master_.make_session(config);
}

}  // namespace rbf
