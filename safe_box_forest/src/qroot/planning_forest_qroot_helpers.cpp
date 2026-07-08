#include "planning_forest_qroot_helpers.h"

#include <SBF/oracle.h>

#include "planning_forest_query_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace rbf {

bool intervals_contain_point_with_boundary_tolerance(const std::vector<Interval>& intervals,
                                                     const Eigen::Ref<const Eigen::VectorXd>& point,
                                                     double tolerance) {
    return intervals_point_gap_local(intervals, point) <=
        std::max(tolerance, kQueryRootBoundaryContainmentTolerance);
}

bool expand_intervals_to_contain_boundary_seed(std::vector<Interval>& intervals,
                                               const Eigen::Ref<const Eigen::VectorXd>& point,
                                               double tolerance) {
    if (point.size() != static_cast<int>(intervals.size())) {
        return false;
    }
    bool expanded = false;
    const double allowed_gap = std::max(tolerance, kQueryRootBoundaryContainmentTolerance);
    for (int dim = 0; dim < point.size(); ++dim) {
        auto& interval = intervals[static_cast<std::size_t>(dim)];
        if (point[dim] < interval.lo) {
            const double gap = interval.lo - point[dim];
            if (gap > allowed_gap) {
                return false;
            }
            interval.lo = point[dim];
            expanded = true;
        } else if (point[dim] > interval.hi) {
            const double gap = point[dim] - interval.hi;
            if (gap > allowed_gap) {
                return false;
            }
            interval.hi = point[dim];
            expanded = true;
        }
    }
    return expanded;
}

bool box_contains_box_exact_local(const BoxNode& outer, const BoxNode& inner) {
    if (outer.n_dims() != inner.n_dims()) {
        return false;
    }
    for (int dim = 0; dim < outer.n_dims(); ++dim) {
        if (outer.joint_intervals[dim].lo > inner.joint_intervals[dim].lo ||
            outer.joint_intervals[dim].hi < inner.joint_intervals[dim].hi) {
            return false;
        }
    }
    return outer.id != inner.id;
}

bool intervals_subset_local(const std::vector<Interval>& inner,
                            const std::vector<Interval>& outer,
                            double tolerance) {
    if (inner.size() != outer.size()) {
        return false;
    }
    for (std::size_t dim = 0; dim < inner.size(); ++dim) {
        if (inner[dim].lo < outer[dim].lo - tolerance ||
            inner[dim].hi > outer[dim].hi + tolerance) {
            return false;
        }
    }
    return true;
}

void append_local_edge(AdjacencyGraph& graph, int lhs, int rhs) {
    if (lhs < 0 || rhs < 0 || lhs == rhs) {
        return;
    }
    auto append_one = [](std::vector<int>& neighbors, int value) {
        if (std::find(neighbors.begin(), neighbors.end(), value) == neighbors.end()) {
            neighbors.push_back(value);
        }
    };
    append_one(graph[lhs], rhs);
    append_one(graph[rhs], lhs);
}

void connect_incremental_boxes(AdjacencyGraph& graph,
                               const std::vector<BoxNode>& boxes,
                               std::size_t first_new_index,
                               double tolerance) {
    if (first_new_index >= boxes.size()) {
        return;
    }
    for (const auto& box : boxes) {
        graph[box.id];
    }
    for (std::size_t i = first_new_index; i < boxes.size(); ++i) {
        for (std::size_t j = 0; j < i; ++j) {
            if (boxes_connected(boxes[j], boxes[i], tolerance)) {
                append_local_edge(graph, boxes[j].id, boxes[i].id);
            }
        }
    }
}

bool allow_dynamic_commit(BoxOracle& oracle,
                          FindFreeBoxResult& result,
                          BoxCommitPolicy policy) {
    if (result.validation_detail.safety_status == BoxSafetyStatus::CertifiedFree) {
        return true;
    }
    if (result.validation_detail.safety_status != BoxSafetyStatus::ProvisionalFree) {
        return false;
    }
    if (policy == BoxCommitPolicy::CommitCertifiedOnly) {
        return false;
    }
    if (policy == BoxCommitPolicy::CommitProvisionalAllowed) {
        return true;
    }
    if (policy == BoxCommitPolicy::AuditBeforeCommit) {
        if (oracle.validate_intervals(result.intervals)) {
            result.validation_detail.safety_status = BoxSafetyStatus::CertifiedFree;
            result.validation_detail.strict_audit_required = false;
            return true;
        }
        return false;
    }
    return false;
}

void BuildDisjointSet::add(int id) {
    if (parent.find(id) == parent.end()) {
        parent[id] = id;
        rank[id] = 0;
    }
}

int BuildDisjointSet::find(int id) {
    add(id);
    int p = parent[id];
    if (p != id) {
        p = find(p);
        parent[id] = p;
    }
    return p;
}

void BuildDisjointSet::unite(int lhs, int rhs) {
    int left = find(lhs);
    int right = find(rhs);
    if (left == right) {
        return;
    }
    if (rank[left] < rank[right]) {
        std::swap(left, right);
    }
    parent[right] = left;
    if (rank[left] == rank[right]) {
        rank[left] += 1;
    }
}

bool BuildDisjointSet::connected(int lhs, int rhs) {
    return find(lhs) == find(rhs);
}

int BuildDisjointSet::island_count() {
    std::unordered_set<int> roots;
    roots.reserve(parent.size());
    for (const auto& [id, _] : parent) {
        roots.insert(find(id));
    }
    return static_cast<int>(roots.size());
}

int BoxSpatialIndex::choose_dim(const std::vector<BoxNode>& boxes) {
    if (boxes.empty()) {
        return -1;
    }
    const int nd = boxes.front().n_dims();
    int best_dim = -1;
    double best_span = -1.0;
    for (int dim = 0; dim < nd; ++dim) {
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        for (const auto& box : boxes) {
            if (box.n_dims() != nd) {
                continue;
            }
            lo = std::min(lo, box.joint_intervals[dim].lo);
            hi = std::max(hi, box.joint_intervals[dim].hi);
        }
        const double span = hi - lo;
        if (std::isfinite(span) && span > best_span) {
            best_span = span;
            best_dim = dim;
        }
    }
    return best_dim;
}

long long BoxSpatialIndex::bin_of(double value, double origin_value, double width) {
    return static_cast<long long>(std::floor((value - origin_value) / std::max(width, 1e-12)));
}

void BoxSpatialIndex::rebuild(const std::vector<BoxNode>& boxes, double tolerance) {
    bins.clear();
    index_dim = choose_dim(boxes);
    if (index_dim < 0 || boxes.empty()) {
        return;
    }
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    double width_sum = 0.0;
    int width_count = 0;
    for (const auto& box : boxes) {
        if (box.n_dims() <= index_dim) {
            continue;
        }
        const auto& interval = box.joint_intervals[static_cast<std::size_t>(index_dim)];
        lo = std::min(lo, interval.lo);
        hi = std::max(hi, interval.hi);
        width_sum += std::max(0.0, interval.width());
        width_count += 1;
    }
    origin = std::isfinite(lo) ? lo - tolerance : 0.0;
    const double span = std::max(hi - lo, 1e-9);
    const double avg_width = width_count > 0 ? width_sum / static_cast<double>(width_count) : span;
    bin_width = std::max({span / 128.0, avg_width, tolerance * 4.0, 1e-9});
    bins.reserve(std::max<std::size_t>(1, boxes.size() * 2));
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        add_box(boxes[static_cast<std::size_t>(index)], index, tolerance);
    }
}

void BoxSpatialIndex::add_box(const BoxNode& box, int index, double tolerance) {
    if (index_dim < 0 || box.n_dims() <= index_dim) {
        return;
    }
    const auto& interval = box.joint_intervals[static_cast<std::size_t>(index_dim)];
    const long long lo_bin = bin_of(interval.lo - tolerance, origin, bin_width);
    const long long hi_bin = bin_of(interval.hi + tolerance, origin, bin_width);
    for (long long bin = lo_bin; bin <= hi_bin; ++bin) {
        bins[bin].push_back(index);
    }
}

std::vector<int> BoxSpatialIndex::interval_candidates(const std::vector<Interval>& intervals,
                                                      double tolerance) const {
    std::vector<int> out;
    if (index_dim < 0 || index_dim >= static_cast<int>(intervals.size())) {
        return out;
    }
    std::unordered_set<int> seen;
    const auto& interval = intervals[static_cast<std::size_t>(index_dim)];
    const long long lo_bin = bin_of(interval.lo - tolerance, origin, bin_width);
    const long long hi_bin = bin_of(interval.hi + tolerance, origin, bin_width);
    for (long long bin = lo_bin; bin <= hi_bin; ++bin) {
        auto it = bins.find(bin);
        if (it == bins.end()) {
            continue;
        }
        for (int index : it->second) {
            if (seen.insert(index).second) {
                out.push_back(index);
            }
        }
    }
    return out;
}

std::vector<int> BoxSpatialIndex::point_candidates(const Eigen::Ref<const Eigen::VectorXd>& point) const {
    std::vector<int> out;
    if (index_dim < 0 || index_dim >= point.size()) {
        return out;
    }
    auto it = bins.find(bin_of(point[index_dim], origin, bin_width));
    if (it != bins.end()) {
        out = it->second;
    }
    return out;
}

int BoxSpatialIndex::covering_box(const std::vector<BoxNode>& boxes,
                                  const Eigen::Ref<const Eigen::VectorXd>& point,
                                  double tolerance) const {
    int best = -1;
    double best_volume = std::numeric_limits<double>::infinity();
    auto candidates = point_candidates(point);
    if (candidates.empty()) {
        candidates.reserve(boxes.size());
        for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
            candidates.push_back(index);
        }
    }
    for (int index : candidates) {
        if (index < 0 || index >= static_cast<int>(boxes.size())) {
            continue;
        }
        const auto& box = boxes[static_cast<std::size_t>(index)];
        if (intervals_contain_point_local(box.joint_intervals, point, tolerance) &&
            box.volume < best_volume) {
            best = index;
            best_volume = box.volume;
        }
    }
    return best;
}

BuildDisjointSet make_dsu_from_graph(const std::vector<BoxNode>& boxes,
                                     const AdjacencyGraph& graph) {
    BuildDisjointSet dsu;
    for (const auto& box : boxes) {
        dsu.add(box.id);
    }
    for (const auto& [id, neighbors] : graph) {
        dsu.add(id);
        for (int neighbor : neighbors) {
            dsu.unite(id, neighbor);
        }
    }
    return dsu;
}

int commit_query_root_box(BoxOracle& oracle,
                          const FindFreeBoxOptions& options,
                          BoxCommitPolicy commit_policy,
                          const std::function<FindFreeBoxResult(const Eigen::VectorXd&,
                                                                 const std::vector<Interval>&,
                                                                 StageContext&,
                                                                 const FindFreeBoxOptions&)>& find_in_domain,
                          const Eigen::Ref<const Eigen::VectorXd>& seed,
                          const BoxNode& domain,
                          int parent_box_id,
                          int root_id,
                          std::vector<BoxNode>& boxes,
                          std::vector<BoxNode>& raw_boxes,
                          AdjacencyGraph& graph,
                          BoxSpatialIndex& box_index,
                          BuildDisjointSet& dsu,
                          int& next_id,
                          StageContext& context,
                          QueryRootGrowResult& stats,
                          double adjacency_tolerance) {
    using Clock = std::chrono::steady_clock;
    const auto query_start = Clock::now();
    if (box_index.covering_box(boxes, seed, 0.0) >= 0) {
        stats.contained_rejects += 1;
        stats.index_query_ms += std::chrono::duration<double, std::milli>(Clock::now() - query_start).count();
        return -1;
    }
    stats.index_query_ms += std::chrono::duration<double, std::milli>(Clock::now() - query_start).count();

    auto result = find_in_domain(seed, domain.joint_intervals, context, options);
    if (!result.found) {
        stats.ffb_fail += 1;
        return -1;
    }
    stats.ffb_success += 1;
    if (!intervals_subset_local(result.intervals, domain.joint_intervals, 1e-12) ||
        !intervals_contain_point_with_boundary_tolerance(result.intervals, seed, adjacency_tolerance)) {
        stats.domain_rejects += 1;
        return -1;
    }
    if (!allow_dynamic_commit(oracle, result, commit_policy)) {
        stats.commit_rejects += 1;
        return -1;
    }

    BoxNode box;
    box.id = next_id++;
    box.joint_intervals = result.intervals;
    const bool boundary_expanded =
        expand_intervals_to_contain_boundary_seed(box.joint_intervals, seed, adjacency_tolerance);
    if (!intervals_subset_local(box.joint_intervals,
                                domain.joint_intervals,
                                kQueryRootBoundaryContainmentTolerance)) {
        stats.domain_rejects += 1;
        return -1;
    }
    box.seed_config = seed;
    box.tree_id = result.node;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id >= 0 ? root_id : box.id;
    box.safety_status = result.validation_detail.safety_status;
    box.strict_audit_required = result.validation_detail.strict_audit_required || boundary_expanded;
    box.compute_volume();

    const auto contained_start = Clock::now();
    auto containing_candidates = box_index.interval_candidates(box.joint_intervals, 0.0);
    if (containing_candidates.empty()) {
        containing_candidates.reserve(boxes.size());
        for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
            containing_candidates.push_back(index);
        }
    }
    for (int index : containing_candidates) {
        if (index >= 0 && index < static_cast<int>(boxes.size()) &&
            box_contains_box_exact_local(boxes[static_cast<std::size_t>(index)], box)) {
            stats.contained_rejects += 1;
            stats.index_query_ms += std::chrono::duration<double, std::milli>(Clock::now() - contained_start).count();
            return -1;
        }
    }
    stats.index_query_ms += std::chrono::duration<double, std::milli>(Clock::now() - contained_start).count();

    if (parent_box_id >= 0) {
        const BoxNode* parent = find_box_by_id(boxes, parent_box_id);
        if (parent == nullptr || !boxes_connected(*parent, box, adjacency_tolerance)) {
            stats.adjacency_rejects += 1;
            return -1;
        }
        box.root_id = parent->root_id >= 0 ? parent->root_id : parent->id;
    }

    oracle.reserve_node(box.tree_id, box.id);
    const int new_index = static_cast<int>(boxes.size());
    boxes.push_back(box);
    raw_boxes.push_back(box);
    graph[box.id];
    dsu.add(box.id);
    if (parent_box_id >= 0) {
        append_local_edge(graph, parent_box_id, box.id);
        dsu.unite(parent_box_id, box.id);
    }
    auto adjacency_candidates = box_index.interval_candidates(box.joint_intervals, adjacency_tolerance);
    stats.adjacency_candidates_tested += static_cast<int>(adjacency_candidates.size());
    for (int index : adjacency_candidates) {
        if (index < 0 || index >= new_index) {
            continue;
        }
        const BoxNode& existing = boxes[static_cast<std::size_t>(index)];
        if (boxes_connected(existing, box, adjacency_tolerance)) {
            append_local_edge(graph, existing.id, box.id);
            dsu.unite(existing.id, box.id);
        }
    }
    box_index.add_box(boxes.back(), new_index, adjacency_tolerance);
    stats.boxes_added += 1;
    return box.id;
}

} // namespace rbf
