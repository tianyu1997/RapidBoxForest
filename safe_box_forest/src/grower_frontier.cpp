#include <SBF/grower.h>

#include "grower_frontier_helpers.h"
#include "grower_internal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

namespace rbf {

using namespace grower_frontier;

bool RrtGrower::seed_covered_by_frontier_cache(const std::vector<BoxNode>& boxes,
                                               const Eigen::Ref<const Eigen::VectorXd>& seed,
                                               StageContext* context) const {
    if (!config_.component_connect_frontier_cache) {
        return point_covered_by_existing_box(boxes, seed);
    }
    const std::string key = frontier_seed_cache_key(seed);
    {
        std::lock_guard<std::mutex> lock(frontier_cache_mutex_);
        if (covered_frontier_seed_cache_.find(key) != covered_frontier_seed_cache_.end()) {
            if (context != nullptr) {
                context->diagnostics().add_counter("grower.frontier_covered_cache_hits");
            }
            return true;
        }
    }
    const bool covered = point_covered_by_existing_box(boxes, seed);
    if (covered) {
        std::lock_guard<std::mutex> lock(frontier_cache_mutex_);
        covered_frontier_seed_cache_.insert(key);
        if (context != nullptr) {
            context->diagnostics().add_counter("grower.frontier_covered_cache_inserts");
        }
    }
    return covered;
}

bool RrtGrower::best_uncovered_directed_face_score(const std::vector<BoxNode>& boxes,
                                                   const BoxNode& parent,
                                                   const Eigen::Ref<const Eigen::VectorXd>& target,
                                                   double& best_score,
                                                   StageContext* context) const {
    const auto root = oracle_.planning_intervals();
    if (parent.n_dims() != target.size() || target.size() != static_cast<int>(root.size())) {
        return false;
    }
    const double seed_epsilon = std::max(config_.boundary_epsilon, 0.25 * config_.adjacency_tolerance);
    const Eigen::VectorXd parent_center = parent.center();
    bool found = false;
    best_score = std::numeric_limits<double>::infinity();
    int covered_faces = 0;
    for (int dim = 0; dim < parent.n_dims(); ++dim) {
        for (int side = 0; side <= 1; ++side) {
            const double direction = target[dim] - parent_center[dim];
            if ((side == 1 && direction <= 1e-12) ||
                (side == 0 && direction >= -1e-12)) {
                continue;
            }
            const double score = face_seed_score(parent, root, target, dim, side, seed_epsilon);
            if (!std::isfinite(score)) {
                continue;
            }
            const Eigen::VectorXd candidate_seed = make_face_seed(parent, root, target, dim, side, seed_epsilon);
            if (seed_covered_by_frontier_cache(boxes, candidate_seed, context)) {
                covered_faces += 1;
                continue;
            }
            best_score = std::min(best_score, score);
            found = true;
        }
    }
    if (!found && covered_faces > 0 && context != nullptr) {
        context->diagnostics().add_counter("grower.component_connect_closed_frontier_faces",
                                           static_cast<double>(covered_faces));
        if (config_.coverage_first_stop_loss) {
            context->diagnostics().add_counter("grower.hard_frontier_closed_frontier_faces",
                                               static_cast<double>(covered_faces));
        }
    }
    return found;
}

bool RrtGrower::prepare_frontier_seed_with_memory(const std::vector<BoxNode>& boxes,
                                                  const BoxNode& parent,
                                                  const Eigen::VectorXd& target,
                                                  int face_dim,
                                                  int side,
                                                  Eigen::VectorXd& seed,
                                                  StageContext* context) const {
    const auto root = oracle_.planning_intervals();
    if (parent.n_dims() != target.size() || target.size() != static_cast<int>(root.size())) {
        return false;
    }
    const double seed_epsilon = std::max(config_.boundary_epsilon, 0.25 * config_.adjacency_tolerance);
    seed = make_face_seed(parent, root, target, face_dim, side, seed_epsilon);
    if (!config_.frontier_face_memory) {
        return !seed_covered_by_frontier_cache(boxes, seed, context);
    }

    const int bins_per_dim = std::clamp(config_.frontier_face_bins_per_dim, 1, 16);
    const std::uint64_t total_bins = frontier_face_total_bins(parent.n_dims(), face_dim, bins_per_dim);
    int budget = frontier_face_attempt_budget(config_, parent, root, face_dim);
    budget = static_cast<int>(std::min<std::uint64_t>(static_cast<std::uint64_t>(budget), total_bins));
    if (context != nullptr) {
        set_max_diagnostic(*context, "grower.frontier_face_attempt_budget_max", static_cast<double>(budget));
        set_max_diagnostic(*context, "grower.frontier_face_total_bins_max", static_cast<double>(total_bins));
    }

    const std::uint64_t direct_bin = frontier_face_bin_for_seed(parent, seed, face_dim, bins_per_dim);
    std::uint64_t chosen_bin = direct_bin;
    bool direct_bin_reused = false;
    {
        std::lock_guard<std::mutex> lock(frontier_face_memory_mutex_);
        auto& used_bins = frontier_face_bins_[frontier_face_memory_key(parent.id, face_dim, side)];
        if (static_cast<int>(used_bins.size()) >= budget) {
            if (context != nullptr) {
                context->diagnostics().add_counter("grower.frontier_face_memory_exhausted");
            }
            return false;
        }
        if (used_bins.find(direct_bin) != used_bins.end()) {
            direct_bin_reused = true;
            const std::uint64_t start = (direct_bin + 2654435761ULL * static_cast<std::uint64_t>(used_bins.size() + 1)) %
                                        std::max<std::uint64_t>(1, total_bins);
            bool found = false;
            for (std::uint64_t offset = 0; offset < total_bins; ++offset) {
                const std::uint64_t candidate_bin = (start + offset) % total_bins;
                if (used_bins.find(candidate_bin) == used_bins.end()) {
                    chosen_bin = candidate_bin;
                    found = true;
                    break;
                }
            }
            if (!found) {
                if (context != nullptr) {
                    context->diagnostics().add_counter("grower.frontier_face_memory_no_free_bin");
                }
                return false;
            }
        }
        used_bins.insert(chosen_bin);
    }

    if (chosen_bin != direct_bin) {
        seed = make_face_seed_for_bin(parent, root, face_dim, side, seed_epsilon, bins_per_dim, chosen_bin);
        if (context != nullptr) {
            context->diagnostics().add_counter("grower.frontier_face_memory_lateral_bin");
        }
    } else if (context != nullptr) {
        context->diagnostics().add_counter("grower.frontier_face_memory_direct_bin");
    }
    if (direct_bin_reused && context != nullptr) {
        context->diagnostics().add_counter("grower.frontier_face_memory_reused_direct_bin");
    }
    if (seed_covered_by_frontier_cache(boxes, seed, context)) {
        if (context != nullptr) {
            context->diagnostics().add_counter("grower.frontier_face_memory_seed_covered_after_claim");
        }
        return false;
    }
    return true;
}

bool RrtGrower::make_frontier_seed(const std::vector<BoxNode>& boxes,
                                   const Eigen::VectorXd& target,
                                   Eigen::VectorXd& seed,
                                   int& parent_box_id,
                                   int& root_id,
                                   StageContext* context,
                                   GrowTraceFace* face,
                                   std::vector<GrowTraceFace>* face_candidates) const {
    return make_frontier_seed_for_root(boxes,
                                       -1,
                                       target,
                                       seed,
                                       parent_box_id,
                                       root_id,
                                       context,
                                       face,
                                       face_candidates);
}

bool RrtGrower::make_frontier_seed_for_root(const std::vector<BoxNode>& boxes,
                                            int source_root_id,
                                            const Eigen::VectorXd& target,
                                            Eigen::VectorXd& seed,
                                            int& parent_box_id,
                                            int& root_id,
                                            StageContext* context,
                                            GrowTraceFace* face,
                                            std::vector<GrowTraceFace>* face_candidates) const {
    if (context != nullptr) {
        context->diagnostics().add_counter("grower.frontier_seed_for_root_calls");
    }
    if (boxes.empty()) {
        return false;
    }
    const auto root = oracle_.planning_intervals();
    if (target.size() != static_cast<int>(root.size())) {
        return false;
    }
    const double seed_epsilon = std::max(config_.boundary_epsilon, 0.25 * config_.adjacency_tolerance);

    const int candidate_limit = std::max(1, config_.frontier_face_candidate_limit);
    std::priority_queue<FaceCandidate, std::vector<FaceCandidate>, WorseFaceCandidateFirst> candidates;
    int scanned_boxes = 0;
    int scanned_faces = 0;
    {
        std::unique_ptr<ScopedStageTimer> scan_timer;
        if (context != nullptr) {
            scan_timer = std::make_unique<ScopedStageTimer>(
                context->diagnostics(),
                "grower.rrt.make_frontier_seed_for_root.scan_faces");
        }
        for (int parent_index = 0; parent_index < static_cast<int>(boxes.size()); ++parent_index) {
            const BoxNode& box = boxes[static_cast<std::size_t>(parent_index)];
            if (source_root_id >= 0 && box.root_id != source_root_id) {
                continue;
            }
            if (box.n_dims() != target.size()) {
                continue;
            }
            scanned_boxes += 1;
            for (int dim = 0; dim < box.n_dims(); ++dim) {
                for (int side = 0; side <= 1; ++side) {
                    scanned_faces += 1;
                    const double score = face_seed_score(box, root, target, dim, side, seed_epsilon);
                    if (!std::isfinite(score)) {
                        continue;
                    }
                    FaceCandidate candidate{parent_index, dim, side, score};
                    if (static_cast<int>(candidates.size()) < candidate_limit) {
                        candidates.push(candidate);
                    } else if (candidate.score < candidates.top().score - 1e-18) {
                        candidates.pop();
                        candidates.push(candidate);
                    }
                }
            }
        }
    }
    if (context != nullptr) {
        context->diagnostics().add_counter("grower.frontier_seed_scanned_boxes", static_cast<double>(scanned_boxes));
        context->diagnostics().add_counter("grower.frontier_seed_scanned_faces", static_cast<double>(scanned_faces));
        context->diagnostics().add_counter("grower.frontier_seed_candidate_count", static_cast<double>(candidates.size()));
    }

    std::vector<FaceCandidate> ordered;
    ordered.reserve(candidates.size());
    {
        std::unique_ptr<ScopedStageTimer> sort_timer;
        if (context != nullptr) {
            sort_timer = std::make_unique<ScopedStageTimer>(
                context->diagnostics(),
                "grower.rrt.make_frontier_seed_for_root.sort_candidates");
        }
        while (!candidates.empty()) {
            ordered.push_back(candidates.top());
            candidates.pop();
        }
        std::sort(ordered.begin(), ordered.end(), face_candidate_less);
    }

    if (face_candidates != nullptr) {
        face_candidates->clear();
        const int trace_limit = std::max(0, config_.trace_face_candidate_limit);
        const int n_trace = std::min(trace_limit, static_cast<int>(ordered.size()));
        face_candidates->reserve(static_cast<std::size_t>(n_trace));
        for (int rank = 0; rank < n_trace; ++rank) {
            const FaceCandidate& candidate = ordered[static_cast<std::size_t>(rank)];
            const BoxNode& parent = boxes[static_cast<std::size_t>(candidate.parent_index)];
            const Eigen::VectorXd candidate_seed = make_face_seed(parent,
                                                                  root,
                                                                  target,
                                                                  candidate.dim,
                                                                  candidate.side,
                                                                  seed_epsilon);
            const bool covered = seed_covered_by_frontier_cache(boxes, candidate_seed, context);
            face_candidates->push_back(make_trace_face(boxes,
                                                       candidate,
                                                       scanned_boxes,
                                                       scanned_faces,
                                                       rank,
                                                       false,
                                                       covered));
        }
    }

    {
        std::unique_ptr<ScopedStageTimer> prepare_timer;
        if (context != nullptr) {
            prepare_timer = std::make_unique<ScopedStageTimer>(
                context->diagnostics(),
                "grower.rrt.make_frontier_seed_for_root.prepare_candidates");
        }
        for (int rank = 0; rank < static_cast<int>(ordered.size()); ++rank) {
            const FaceCandidate& candidate = ordered[static_cast<std::size_t>(rank)];
            const BoxNode& parent = boxes[static_cast<std::size_t>(candidate.parent_index)];
            Eigen::VectorXd candidate_seed;
            if (!prepare_frontier_seed_with_memory(boxes,
                                                   parent,
                                                   target,
                                                   candidate.dim,
                                                   candidate.side,
                                                   candidate_seed,
                                                   context)) {
                continue;
            }
            GrowTraceFace selected = make_trace_face(boxes,
                                                     candidate,
                                                     scanned_boxes,
                                                     scanned_faces,
                                                     rank,
                                                     true,
                                                     false);
            if (face_candidates != nullptr) {
                for (GrowTraceFace& traced : *face_candidates) {
                    if (traced.rank == rank) {
                        traced.selected = true;
                        traced.seed_covered = false;
                        break;
                    }
                }
            }
            seed = std::move(candidate_seed);
            parent_box_id = parent.id;
            root_id = parent.root_id;
            if (face != nullptr) {
                *face = selected;
            }
            if (context != nullptr) {
                context->diagnostics().add_counter("grower.frontier_seed_selected_rank", static_cast<double>(rank));
            }
            return true;
        }
    }
    if (context != nullptr) {
        context->diagnostics().add_counter("grower.frontier_no_uncovered_seed");
        if (config_.coverage_first_stop_loss) {
            context->diagnostics().add_counter("grower.hard_frontier_no_uncovered_seed");
            context->diagnostics().add_counter("grower.hard_frontier_closed_frontier");
        }
        if (source_root_id >= 0) {
            context->diagnostics().add_counter("grower.root_frontier_no_uncovered_seed");
            if (config_.coverage_first_stop_loss) {
                context->diagnostics().add_counter("grower.hard_frontier_closed_root_frontier");
            }
        }
        context->diagnostics().add_counter("grower.frontier_scanned_boxes", static_cast<double>(scanned_boxes));
        context->diagnostics().add_counter("grower.frontier_scanned_faces", static_cast<double>(scanned_faces));
    }
    return false;
}

bool RrtGrower::make_frontier_seed_from_parent(const std::vector<BoxNode>& boxes,
                                               int parent_index,
                                               const Eigen::VectorXd& target,
                                               Eigen::VectorXd& seed,
                                               int& parent_box_id,
                                               int& root_id,
                                               bool require_target_direction,
                                               GrowTraceFace* face,
                                               std::vector<GrowTraceFace>* face_candidates,
                                               StageContext* context) const {
    if (parent_index < 0 || parent_index >= static_cast<int>(boxes.size())) {
        return false;
    }
    const auto root = oracle_.planning_intervals();
    const BoxNode& parent = boxes[static_cast<std::size_t>(parent_index)];
    if (parent.n_dims() != target.size() || target.size() != static_cast<int>(root.size())) {
        return false;
    }
    const double seed_epsilon = std::max(config_.boundary_epsilon, 0.25 * config_.adjacency_tolerance);
    std::vector<FaceCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(2 * parent.n_dims()));
    const Eigen::VectorXd parent_center = parent.center();
    for (int dim = 0; dim < parent.n_dims(); ++dim) {
        for (int side = 0; side <= 1; ++side) {
            if (require_target_direction) {
                const double direction = target[dim] - parent_center[dim];
                if ((side == 1 && direction <= 1e-12) ||
                    (side == 0 && direction >= -1e-12)) {
                    continue;
                }
            }
            const double score = face_seed_score(parent, root, target, dim, side, seed_epsilon);
            if (std::isfinite(score)) {
                candidates.push_back({parent_index, dim, side, score});
            }
        }
    }
    std::sort(candidates.begin(), candidates.end(), face_candidate_less);

    if (face_candidates != nullptr) {
        face_candidates->clear();
        const int trace_limit = std::max(0, config_.trace_face_candidate_limit);
        const int n_trace = std::min(trace_limit, static_cast<int>(candidates.size()));
        face_candidates->reserve(static_cast<std::size_t>(n_trace));
        for (int rank = 0; rank < n_trace; ++rank) {
            const FaceCandidate& candidate = candidates[static_cast<std::size_t>(rank)];
            const Eigen::VectorXd candidate_seed = make_face_seed(parent,
                                                                  root,
                                                                  target,
                                                                  candidate.dim,
                                                                  candidate.side,
                                                                  seed_epsilon);
            const bool covered = seed_covered_by_frontier_cache(boxes, candidate_seed, context);
            face_candidates->push_back(make_trace_face(boxes,
                                                       candidate,
                                                       1,
                                                       static_cast<int>(candidates.size()),
                                                       rank,
                                                       false,
                                                       covered));
        }
    }

    for (int rank = 0; rank < static_cast<int>(candidates.size()); ++rank) {
        const FaceCandidate& candidate = candidates[static_cast<std::size_t>(rank)];
        Eigen::VectorXd candidate_seed;
        if (!prepare_frontier_seed_with_memory(boxes,
                                               parent,
                                               target,
                                               candidate.dim,
                                               candidate.side,
                                               candidate_seed,
                                               context)) {
            continue;
        }
        GrowTraceFace selected = make_trace_face(boxes,
                                                 candidate,
                                                 1,
                                                 static_cast<int>(candidates.size()),
                                                 rank,
                                                 true,
                                                 false);
        if (face_candidates != nullptr) {
            for (GrowTraceFace& traced : *face_candidates) {
                if (traced.rank == rank) {
                    traced.selected = true;
                    traced.seed_covered = false;
                    break;
                }
            }
        }
        seed = std::move(candidate_seed);
        parent_box_id = parent.id;
        root_id = parent.root_id;
        if (face != nullptr) {
            *face = selected;
        }
        return true;
    }
    return false;
}

}  // namespace rbf
