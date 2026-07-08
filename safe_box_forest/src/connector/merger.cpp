#include <SBF/merger.h>
#include <SBF/box_graph.h>
#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rbf {
namespace {

bool intervals_same(const Interval& lhs, const Interval& rhs, double tolerance) {
    return std::abs(lhs.lo - rhs.lo) <= tolerance &&
           std::abs(lhs.hi - rhs.hi) <= tolerance;
}

bool intervals_connected(const Interval& lhs, const Interval& rhs, double tolerance) {
    return std::min(lhs.hi, rhs.hi) >= std::max(lhs.lo, rhs.lo) - tolerance;
}

int exact_merge_dimension(const BoxNode& lhs, const BoxNode& rhs, double tolerance) {
    if (lhs.n_dims() != rhs.n_dims()) {
        return -1;
    }
    int merge_dim = -1;
    for (int dim = 0; dim < lhs.n_dims(); ++dim) {
        if (intervals_same(lhs.joint_intervals[dim], rhs.joint_intervals[dim], tolerance)) {
            continue;
        }
        if (merge_dim >= 0 || !intervals_connected(lhs.joint_intervals[dim], rhs.joint_intervals[dim], tolerance)) {
            return -1;
        }
        merge_dim = dim;
    }
    return merge_dim;
}

}  // namespace

Consolidator::Consolidator(BoxOracle& oracle, MergerConfig config)
    : oracle_(oracle), config_(std::move(config)) {}

MergerResult Consolidator::run(std::vector<BoxNode>& boxes,
                               const std::unordered_set<int>& protected_ids) {
    RuntimeConfig runtime;
    runtime.mode = config_.n_threads > 1 ? ExecutionMode::Parallel : ExecutionMode::Inline;
    runtime.n_threads = config_.n_threads;
    runtime.batch_size = config_.candidate_batch_size;
    runtime.parallel_threshold = config_.parallel_threshold;
    runtime.deterministic_reduce = config_.deterministic_reduce;
    StageContext context(runtime);
    return run(boxes, context, protected_ids);
}

MergerResult Consolidator::run(std::vector<BoxNode>& boxes,
                               StageContext& context,
                               const std::unordered_set<int>& protected_ids) {
    MergerResult result;
    result.boxes_before = static_cast<int>(boxes.size());
    for (int round = 0; round < config_.max_rounds; ++round) {
        if (context.should_stop()) {
            break;
        }
        bool changed = false;
        if (config_.exact_face_merge && try_exact_merge(boxes, protected_ids)) {
            result.exact_merges += 1;
            changed = true;
        }
        if (config_.greedy_hull_merge && try_greedy_merge(boxes, context, protected_ids, result.greedy_merges)) {
            changed = true;
        }
        if (config_.containment_prune) {
            const int pruned = prune_contained(boxes, protected_ids);
            result.pruned_boxes += pruned;
            changed = changed || pruned > 0;
        }
        result.rounds += 1;
        if (config_.target_boxes > 0 && static_cast<int>(boxes.size()) <= config_.target_boxes) {
            break;
        }
        if (!changed) {
            break;
        }
    }
    result.boxes_after = static_cast<int>(boxes.size());
    return result;
}

bool Consolidator::try_exact_merge(std::vector<BoxNode>& boxes,
                                   const std::unordered_set<int>& protected_ids) {
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        if (protected_ids.find(boxes[i].id) != protected_ids.end()) {
            continue;
        }
        for (std::size_t j = i + 1; j < boxes.size(); ++j) {
            if (protected_ids.find(boxes[j].id) != protected_ids.end()) {
                continue;
            }
            const int merge_dim = exact_merge_dimension(boxes[i], boxes[j], config_.adjacency_tolerance);
            if (merge_dim < 0) {
                continue;
            }
            boxes[i].joint_intervals[merge_dim] = boxes[i].joint_intervals[merge_dim].hull(boxes[j].joint_intervals[merge_dim]);
            boxes[i].compute_volume();
            boxes[i].tree_id = -1;
            oracle_.release_box(boxes[j].id);
            boxes.erase(boxes.begin() + static_cast<std::ptrdiff_t>(j));
            return true;
        }
    }
    return false;
}

bool Consolidator::try_greedy_merge(std::vector<BoxNode>& boxes,
                                    StageContext& context,
                                    const std::unordered_set<int>& protected_ids,
                                    int& greedy_merges) {
    auto candidates = collect_greedy_candidates(boxes, context, protected_ids);
    auto validations = validate_candidates(candidates, context);
    for (const auto& validation : validations) {
        if (context.should_stop()) {
            break;
        }
        if (validation.valid && apply_merge_candidate(boxes, validation.candidate)) {
            greedy_merges += 1;
            return true;
        }
    }
    return false;
}

std::vector<MergeCandidate> Consolidator::collect_greedy_candidates(
    const std::vector<BoxNode>& boxes,
    StageContext& context,
    const std::unordered_set<int>& protected_ids) const {
    const int n_boxes = static_cast<int>(boxes.size());
    if (n_boxes <= 1) {
        return {};
    }

    auto collect_for_i = [&](int i, std::vector<MergeCandidate>& out) {
        if (protected_ids.find(boxes[static_cast<std::size_t>(i)].id) != protected_ids.end()) {
            return;
        }
        for (int j = i + 1; j < n_boxes; ++j) {
            const auto& lhs = boxes[static_cast<std::size_t>(i)];
            const auto& rhs = boxes[static_cast<std::size_t>(j)];
            if (protected_ids.find(rhs.id) != protected_ids.end()) {
                continue;
            }
            if (!boxes_connected(lhs, rhs, config_.adjacency_tolerance)) {
                continue;
            }
            auto candidate_hull = hull(lhs, rhs);
            double hull_volume = 1.0;
            for (const auto& interval : candidate_hull) {
                hull_volume *= std::max(0.0, interval.width());
            }
            const double sum_volume = std::max(1e-300, lhs.volume + rhs.volume);
            const double score = hull_volume / sum_volume;
            if (score <= config_.score_threshold) {
                out.push_back({lhs.id, rhs.id, score, std::move(candidate_hull)});
            }
        }
    };

    std::vector<std::vector<MergeCandidate>> per_lhs(static_cast<std::size_t>(n_boxes));
    const bool use_parallel = context.executor().n_threads() > 1 &&
        (config_.parallel_threshold <= 0 || n_boxes >= config_.parallel_threshold);
    if (use_parallel) {
        context.executor().parallel_for(0, n_boxes, [&](int i) {
            if (!context.should_stop()) {
                collect_for_i(i, per_lhs[static_cast<std::size_t>(i)]);
            }
        });
    } else {
        for (int i = 0; i < n_boxes; ++i) {
            if (context.should_stop()) {
                break;
            }
            collect_for_i(i, per_lhs[static_cast<std::size_t>(i)]);
        }
    }

    std::vector<MergeCandidate> candidates;
    for (auto& bucket : per_lhs) {
        for (auto& candidate : bucket) {
            candidates.push_back(std::move(candidate));
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const MergeCandidate& lhs, const MergeCandidate& rhs) {
        if (lhs.score != rhs.score) return lhs.score < rhs.score;
        if (lhs.lhs_box_id != rhs.lhs_box_id) return lhs.lhs_box_id < rhs.lhs_box_id;
        return lhs.rhs_box_id < rhs.rhs_box_id;
    });
    return candidates;
}

std::vector<MergeValidationResult> Consolidator::validate_candidates(
    const std::vector<MergeCandidate>& candidates,
    StageContext& context) {
    std::vector<MergeValidationResult> validations(candidates.size());
    if (candidates.empty()) {
        return validations;
    }

    const bool use_worker_sessions = context.executor().n_threads() > 1 &&
        (config_.parallel_threshold <= 0 || static_cast<int>(candidates.size()) >= config_.parallel_threshold);
    if (!use_worker_sessions) {
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (context.should_stop()) break;
            validations[i].candidate = candidates[i];
            validations[i].valid = oracle_.validate_intervals(candidates[i].hull_intervals);
        }
        return validations;
    }

    const int n_workers = std::max(1, std::min(context.executor().n_threads(), static_cast<int>(candidates.size())));
    std::vector<std::unique_ptr<BoxOracleSession>> sessions(static_cast<std::size_t>(n_workers));
    for (int worker = 0; worker < n_workers; ++worker) {
        OracleSessionConfig session_config;
        session_config.worker_id = worker;
        session_config.read_only = true;
        session_config.domain_root = oracle_.root_node();
        sessions[static_cast<std::size_t>(worker)] = oracle_.make_session(session_config);
        if (!sessions[static_cast<std::size_t>(worker)]) {
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                if (context.should_stop()) break;
                validations[i].candidate = candidates[i];
                validations[i].valid = oracle_.validate_intervals(candidates[i].hull_intervals);
            }
            return validations;
        }
    }
    context.diagnostics().add_counter("merger.parallel_validation_batches");
    context.diagnostics().add_counter("merger.parallel_validation_candidates", static_cast<double>(candidates.size()));
    context.diagnostics().add_counter("merger.parallel_validation_sessions", static_cast<double>(n_workers));

    context.executor().parallel_for(0, n_workers, [&](int worker) {
        if (context.should_stop()) {
            return;
        }
        const int begin = static_cast<int>((static_cast<std::size_t>(worker) * candidates.size()) /
                                           static_cast<std::size_t>(n_workers));
        const int end = static_cast<int>((static_cast<std::size_t>(worker + 1) * candidates.size()) /
                                         static_cast<std::size_t>(n_workers));
        auto& worker_oracle = sessions[static_cast<std::size_t>(worker)]->oracle();
        for (int index = begin; index < end; ++index) {
            if (context.should_stop()) {
                break;
            }
            auto& validation = validations[static_cast<std::size_t>(index)];
            validation.candidate = candidates[static_cast<std::size_t>(index)];
            validation.valid = worker_oracle.validate_intervals(
                candidates[static_cast<std::size_t>(index)].hull_intervals);
        }
    });
    return validations;
}

bool Consolidator::apply_merge_candidate(std::vector<BoxNode>& boxes,
                                         const MergeCandidate& candidate) {
    std::size_t lhs_index = boxes.size();
    std::size_t rhs_index = boxes.size();
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        if (boxes[i].id == candidate.lhs_box_id) {
            lhs_index = i;
        } else if (boxes[i].id == candidate.rhs_box_id) {
            rhs_index = i;
        }
    }
    if (lhs_index >= boxes.size() || rhs_index >= boxes.size() || lhs_index == rhs_index) {
        return false;
    }
    if (rhs_index < lhs_index) {
        std::swap(lhs_index, rhs_index);
    }
    if (!boxes_connected(boxes[lhs_index], boxes[rhs_index], config_.adjacency_tolerance)) {
        return false;
    }
    boxes[lhs_index].joint_intervals = candidate.hull_intervals;
    boxes[lhs_index].compute_volume();
    boxes[lhs_index].tree_id = -1;
    oracle_.release_box(boxes[rhs_index].id);
    boxes.erase(boxes.begin() + static_cast<std::ptrdiff_t>(rhs_index));
    return true;
}

int Consolidator::prune_contained(std::vector<BoxNode>& boxes,
                                  const std::unordered_set<int>& protected_ids) {
    int removed = 0;
    for (std::size_t i = 0; i < boxes.size();) {
        if (protected_ids.find(boxes[i].id) != protected_ids.end()) {
            ++i;
            continue;
        }
        bool contained = false;
        for (std::size_t j = 0; j < boxes.size(); ++j) {
            if (i == j) {
                continue;
            }
            if (contains_box(boxes[j], boxes[i], config_.adjacency_tolerance)) {
                contained = true;
                break;
            }
        }
        if (contained) {
            oracle_.release_box(boxes[i].id);
            boxes.erase(boxes.begin() + static_cast<std::ptrdiff_t>(i));
            removed += 1;
        } else {
            ++i;
        }
    }
    return removed;
}

std::vector<Interval> Consolidator::hull(const BoxNode& lhs, const BoxNode& rhs) {
    std::vector<Interval> intervals = lhs.joint_intervals;
    for (int dim = 0; dim < lhs.n_dims(); ++dim) {
        intervals[static_cast<std::size_t>(dim)] = intervals[static_cast<std::size_t>(dim)].hull(rhs.joint_intervals[dim]);
    }
    return intervals;
}

bool Consolidator::contains_box(const BoxNode& outer, const BoxNode& inner, double tolerance) {
    if (outer.n_dims() != inner.n_dims()) {
        return false;
    }
    for (int dim = 0; dim < outer.n_dims(); ++dim) {
        if (outer.joint_intervals[dim].lo > inner.joint_intervals[dim].lo + tolerance ||
            outer.joint_intervals[dim].hi < inner.joint_intervals[dim].hi - tolerance) {
            return false;
        }
    }
    return outer.id != inner.id;
}

}  // namespace rbf
