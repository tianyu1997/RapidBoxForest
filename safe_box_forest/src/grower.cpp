#include <SBF/grower.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace rbf {
namespace {

double box_gap_squared(const BoxNode& lhs, const BoxNode& rhs) {
    const int nd = lhs.n_dims();
    if (nd != rhs.n_dims()) {
        return std::numeric_limits<double>::infinity();
    }
    double gap_sq = 0.0;
    for (int dim = 0; dim < nd; ++dim) {
        double gap = 0.0;
        if (lhs.joint_intervals[dim].hi < rhs.joint_intervals[dim].lo) {
            gap = rhs.joint_intervals[dim].lo - lhs.joint_intervals[dim].hi;
        } else if (rhs.joint_intervals[dim].hi < lhs.joint_intervals[dim].lo) {
            gap = lhs.joint_intervals[dim].lo - rhs.joint_intervals[dim].hi;
        }
        gap_sq += gap * gap;
    }
    return gap_sq;
}

double interval_point_gap(const Interval& interval, double value) {
    if (value < interval.lo) {
        return interval.lo - value;
    }
    if (value > interval.hi) {
        return value - interval.hi;
    }
    return 0.0;
}

double intervals_point_gap(const std::vector<Interval>& intervals,
                           const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (point.size() != static_cast<int>(intervals.size())) {
        return std::numeric_limits<double>::infinity();
    }
    double gap = 0.0;
    for (int dim = 0; dim < point.size(); ++dim) {
        gap = std::max(gap, interval_point_gap(intervals[static_cast<std::size_t>(dim)], point[dim]));
    }
    return gap;
}

bool intervals_contain_point(const std::vector<Interval>& intervals,
                             const Eigen::Ref<const Eigen::VectorXd>& point,
                             double tolerance) {
    return intervals_point_gap(intervals, point) <= tolerance;
}

double box_point_gap(const BoxNode& box, const Eigen::Ref<const Eigen::VectorXd>& point) {
    return intervals_point_gap(box.joint_intervals, point);
}

Eigen::VectorXd closest_point_in_box(const BoxNode& box,
                                     const Eigen::Ref<const Eigen::VectorXd>& point) {
    Eigen::VectorXd closest(box.n_dims());
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        closest[dim] = std::clamp(point[dim],
                                  box.joint_intervals[dim].lo,
                                  box.joint_intervals[dim].hi);
    }
    return closest;
}

std::vector<Interval> bounds_for_indices(const std::vector<BoxNode>& boxes,
                                         const std::vector<int>& indices) {
    std::vector<Interval> bounds;
    if (indices.empty()) {
        return bounds;
    }
    bounds = boxes[static_cast<std::size_t>(indices.front())].joint_intervals;
    for (int outer = 1; outer < static_cast<int>(indices.size()); ++outer) {
        const auto& intervals = boxes[static_cast<std::size_t>(indices[static_cast<std::size_t>(outer)])].joint_intervals;
        for (int dim = 0; dim < static_cast<int>(bounds.size()) && dim < static_cast<int>(intervals.size()); ++dim) {
            bounds[static_cast<std::size_t>(dim)].lo = std::min(bounds[static_cast<std::size_t>(dim)].lo,
                                                                intervals[static_cast<std::size_t>(dim)].lo);
            bounds[static_cast<std::size_t>(dim)].hi = std::max(bounds[static_cast<std::size_t>(dim)].hi,
                                                                intervals[static_cast<std::size_t>(dim)].hi);
        }
    }
    return bounds;
}

Eigen::VectorXd intervals_center(const std::vector<Interval>& intervals) {
    Eigen::VectorXd center(static_cast<int>(intervals.size()));
    for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
        center[dim] = intervals[static_cast<std::size_t>(dim)].center();
    }
    return center;
}

Eigen::VectorXd closest_point_in_intervals(const std::vector<Interval>& intervals,
                                           const Eigen::Ref<const Eigen::VectorXd>& point) {
    Eigen::VectorXd closest(static_cast<int>(intervals.size()));
    for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
        closest[dim] = std::clamp(point[dim],
                                  intervals[static_cast<std::size_t>(dim)].lo,
                                  intervals[static_cast<std::size_t>(dim)].hi);
    }
    return closest;
}

double interval_bounds_gap_squared(const std::vector<Interval>& lhs,
                                   const std::vector<Interval>& rhs) {
    if (lhs.size() != rhs.size()) {
        return std::numeric_limits<double>::infinity();
    }
    double gap_sq = 0.0;
    for (int dim = 0; dim < static_cast<int>(lhs.size()); ++dim) {
        double gap = 0.0;
        const auto& left = lhs[static_cast<std::size_t>(dim)];
        const auto& right = rhs[static_cast<std::size_t>(dim)];
        if (left.hi < right.lo) {
            gap = right.lo - left.hi;
        } else if (right.hi < left.lo) {
            gap = left.lo - right.hi;
        }
        gap_sq += gap * gap;
    }
    return gap_sq;
}

bool box_contains_box_exact(const BoxNode& outer, const BoxNode& inner) {
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

bool box_contains_point_exact(const BoxNode& box, const Eigen::Ref<const Eigen::VectorXd>& point) {
    if (box.n_dims() != point.size()) {
        return false;
    }
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        if (point[dim] < box.joint_intervals[dim].lo ||
            point[dim] > box.joint_intervals[dim].hi) {
            return false;
        }
    }
    return true;
}

bool point_covered_by_existing_box(const std::vector<BoxNode>& boxes,
                                   const Eigen::Ref<const Eigen::VectorXd>& point) {
    return std::any_of(boxes.begin(), boxes.end(), [&](const BoxNode& box) {
        return box_contains_point_exact(box, point);
    });
}

bool allow_box_commit(BoxOracle& oracle,
                      FindFreeBoxResult& result,
                      BoxCommitPolicy policy,
                      StageContext& context) {
    if (result.validation_detail.safety_status == BoxSafetyStatus::CertifiedFree) {
        return true;
    }
    if (result.validation_detail.safety_status != BoxSafetyStatus::ProvisionalFree) {
        context.diagnostics().add_counter("grower.commit_rejected_unknown_status");
        return false;
    }
    if (policy == BoxCommitPolicy::CommitCertifiedOnly) {
        context.diagnostics().add_counter("grower.commit_rejected_provisional");
        return false;
    }
    if (policy == BoxCommitPolicy::CommitProvisionalAllowed) {
        context.diagnostics().add_counter("grower.commit_provisional_allowed");
        return true;
    }
    if (policy == BoxCommitPolicy::AuditBeforeCommit) {
        context.diagnostics().add_counter("grower.commit_audit_attempted");
        if (oracle.validate_intervals(result.intervals)) {
            result.validation_detail.safety_status = BoxSafetyStatus::CertifiedFree;
            result.validation_detail.strict_audit_required = false;
            context.diagnostics().add_counter("grower.commit_audit_success");
            return true;
        }
        context.diagnostics().add_counter("grower.commit_audit_failed");
        return false;
    }
    return false;
}

bool can_step_outside_face(const BoxNode& box,
                           const std::vector<Interval>& root,
                           int dim,
                           int side,
                           double epsilon) {
    return side == 1
        ? box.joint_intervals[dim].hi + epsilon <= root[static_cast<std::size_t>(dim)].hi
        : box.joint_intervals[dim].lo - epsilon >= root[static_cast<std::size_t>(dim)].lo;
}

double face_seed_score(const BoxNode& box,
                       const std::vector<Interval>& root,
                       const Eigen::Ref<const Eigen::VectorXd>& target,
                       int face_dim,
                       int side,
                       double epsilon) {
    if (!can_step_outside_face(box, root, face_dim, side, epsilon)) {
        return std::numeric_limits<double>::infinity();
    }
    double score = 0.0;
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        double value = target[dim];
        if (dim == face_dim) {
            value = side == 1
                ? box.joint_intervals[dim].hi + epsilon
                : box.joint_intervals[dim].lo - epsilon;
        } else {
            const double lo = box.joint_intervals[dim].lo;
            const double hi = box.joint_intervals[dim].hi;
            const double safe_lo = lo + epsilon;
            const double safe_hi = hi - epsilon;
            value = safe_lo <= safe_hi
                ? std::clamp(target[dim], safe_lo, safe_hi)
                : std::clamp(target[dim], lo, hi);
        }
        const double delta = target[dim] - value;
        score += delta * delta;
    }
    return score;
}

Eigen::VectorXd make_face_seed(const BoxNode& box,
                               const std::vector<Interval>& root,
                               const Eigen::Ref<const Eigen::VectorXd>& target,
                               int face_dim,
                               int side,
                               double epsilon) {
    const int nd = box.n_dims();
    Eigen::VectorXd seed(nd);
    for (int dim = 0; dim < nd; ++dim) {
        if (dim == face_dim) {
            seed[dim] = side == 1
                ? box.joint_intervals[dim].hi + epsilon
                : box.joint_intervals[dim].lo - epsilon;
        } else {
            const double lo = box.joint_intervals[dim].lo;
            const double hi = box.joint_intervals[dim].hi;
            const double safe_lo = lo + epsilon;
            const double safe_hi = hi - epsilon;
            seed[dim] = safe_lo <= safe_hi
                ? std::clamp(target[dim], safe_lo, safe_hi)
                : std::clamp(target[dim], lo, hi);
        }
        seed[dim] = std::clamp(seed[dim],
                               root[static_cast<std::size_t>(dim)].lo,
                               root[static_cast<std::size_t>(dim)].hi);
    }
    return seed;
}

std::uint64_t frontier_face_memory_key(int parent_box_id, int face_dim, int side) {
    const std::uint64_t parent = static_cast<std::uint64_t>(static_cast<std::uint32_t>(parent_box_id));
    const std::uint64_t dim = static_cast<std::uint64_t>(static_cast<std::uint32_t>(std::max(0, face_dim)));
    const std::uint64_t side_bit = side == 1 ? 1ULL : 0ULL;
    return (parent << 8) ^ (dim << 1) ^ side_bit;
}

std::uint64_t frontier_face_total_bins(int nd, int face_dim, int bins_per_dim) {
    const int bins = std::max(1, bins_per_dim);
    std::uint64_t total = 1;
    for (int dim = 0; dim < nd; ++dim) {
        if (dim == face_dim) {
            continue;
        }
        if (total > std::numeric_limits<std::uint64_t>::max() / static_cast<std::uint64_t>(bins)) {
            return std::numeric_limits<std::uint64_t>::max();
        }
        total *= static_cast<std::uint64_t>(bins);
    }
    return total;
}

std::uint64_t frontier_face_bin_for_seed(const BoxNode& box,
                                         const Eigen::Ref<const Eigen::VectorXd>& seed,
                                         int face_dim,
                                         int bins_per_dim) {
    const int bins = std::max(1, bins_per_dim);
    std::uint64_t code = 0;
    std::uint64_t stride = 1;
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        if (dim == face_dim) {
            continue;
        }
        const double lo = box.joint_intervals[static_cast<std::size_t>(dim)].lo;
        const double width = std::max(box.joint_intervals[static_cast<std::size_t>(dim)].width(), 1e-12);
        const double normalized = std::clamp((seed[dim] - lo) / width, 0.0, 1.0 - 1e-12);
        const int bin = std::clamp(static_cast<int>(std::floor(normalized * bins)), 0, bins - 1);
        code += static_cast<std::uint64_t>(bin) * stride;
        stride *= static_cast<std::uint64_t>(bins);
    }
    return code;
}

Eigen::VectorXd make_face_seed_for_bin(const BoxNode& box,
                                       const std::vector<Interval>& root,
                                       int face_dim,
                                       int side,
                                       double epsilon,
                                       int bins_per_dim,
                                       std::uint64_t bin_code) {
    const int nd = box.n_dims();
    const int bins = std::max(1, bins_per_dim);
    Eigen::VectorXd seed(nd);
    for (int dim = 0; dim < nd; ++dim) {
        if (dim == face_dim) {
            seed[dim] = side == 1
                ? box.joint_intervals[static_cast<std::size_t>(dim)].hi + epsilon
                : box.joint_intervals[static_cast<std::size_t>(dim)].lo - epsilon;
        } else {
            const int bin = static_cast<int>(bin_code % static_cast<std::uint64_t>(bins));
            bin_code /= static_cast<std::uint64_t>(bins);
            const double lo = box.joint_intervals[static_cast<std::size_t>(dim)].lo;
            const double hi = box.joint_intervals[static_cast<std::size_t>(dim)].hi;
            const double width = std::max(hi - lo, 0.0);
            seed[dim] = lo + (static_cast<double>(bin) + 0.5) * width / static_cast<double>(bins);
        }
        seed[dim] = std::clamp(seed[dim],
                               root[static_cast<std::size_t>(dim)].lo,
                               root[static_cast<std::size_t>(dim)].hi);
    }
    return seed;
}

Eigen::VectorXd clip_to_root_intervals(const Eigen::Ref<const Eigen::VectorXd>& q,
                                       const std::vector<Interval>& root) {
    Eigen::VectorXd clipped = q;
    if (clipped.size() != static_cast<int>(root.size())) {
        return clipped;
    }
    for (int dim = 0; dim < clipped.size(); ++dim) {
        clipped[dim] = std::clamp(clipped[dim],
                                  root[static_cast<std::size_t>(dim)].lo,
                                  root[static_cast<std::size_t>(dim)].hi);
    }
    return clipped;
}

int frontier_face_attempt_budget(const GrowerConfig& config,
                                 const BoxNode& box,
                                 const std::vector<Interval>& root,
                                 int face_dim) {
    if (box.n_dims() <= 1) {
        return std::max(1, config.frontier_face_min_attempts);
    }
    double log_span = 0.0;
    int counted = 0;
    for (int dim = 0; dim < box.n_dims(); ++dim) {
        if (dim == face_dim) {
            continue;
        }
        const double root_width = std::max(root[static_cast<std::size_t>(dim)].width(), 1e-12);
        const double normalized_width = std::clamp(
            box.joint_intervals[static_cast<std::size_t>(dim)].width() / root_width,
            1e-12,
            1.0);
        log_span += std::log(normalized_width);
        counted += 1;
    }
    const double geometric_span = counted > 0 ? std::exp(log_span / static_cast<double>(counted)) : 1.0;
    const int area_budget = static_cast<int>(std::ceil(std::max(0.0, config.frontier_face_area_attempt_scale) *
                                                       geometric_span));
    const int raw_budget = std::max(std::max(1, config.frontier_face_min_attempts), area_budget);
    return std::max(1, std::min(std::max(1, config.frontier_face_max_attempts), raw_budget));
}

struct FaceCandidate {
    int parent_index = -1;
    int dim = -1;
    int side = 0;
    double score = std::numeric_limits<double>::infinity();
};

struct WorseFaceCandidateFirst {
    bool operator()(const FaceCandidate& lhs, const FaceCandidate& rhs) const {
        if (std::abs(lhs.score - rhs.score) > 1e-18) {
            return lhs.score < rhs.score;
        }
        if (lhs.parent_index != rhs.parent_index) {
            return lhs.parent_index < rhs.parent_index;
        }
        if (lhs.dim != rhs.dim) {
            return lhs.dim < rhs.dim;
        }
        return lhs.side < rhs.side;
    }
};

GrowTraceFace make_trace_face(const std::vector<BoxNode>& boxes,
                              const FaceCandidate& candidate,
                              int scanned_boxes,
                              int scanned_faces,
                              int rank,
                              bool selected,
                              bool seed_covered) {
    GrowTraceFace face;
    if (candidate.parent_index < 0 || candidate.parent_index >= static_cast<int>(boxes.size())) {
        return face;
    }
    const BoxNode& parent = boxes[static_cast<std::size_t>(candidate.parent_index)];
    face.valid = true;
    face.rank = rank;
    face.selected = selected;
    face.seed_covered = seed_covered;
    face.parent_index = candidate.parent_index;
    face.parent_box_id = parent.id;
    face.dim = candidate.dim;
    face.side = candidate.side == 1 ? 1 : -1;
    face.face_value = candidate.side == 1
        ? parent.joint_intervals[static_cast<std::size_t>(candidate.dim)].hi
        : parent.joint_intervals[static_cast<std::size_t>(candidate.dim)].lo;
    face.score = candidate.score;
    face.scanned_boxes = scanned_boxes;
    face.scanned_faces = scanned_faces;
    return face;
}

struct RootGroups {
    std::unordered_map<int, std::vector<int>> by_root;
    std::vector<int> roots;
};

RootGroups group_boxes_by_root(const std::vector<BoxNode>& boxes) {
    RootGroups groups;
    for (int index = 0; index < static_cast<int>(boxes.size()); ++index) {
        const int root = boxes[static_cast<std::size_t>(index)].root_id;
        if (root >= 0) {
            groups.by_root[root].push_back(index);
        }
    }
    groups.roots.reserve(groups.by_root.size());
    for (const auto& [root, _] : groups.by_root) {
        groups.roots.push_back(root);
    }
    std::sort(groups.roots.begin(), groups.roots.end());
    return groups;
}

struct RootComponent {
    int id = -1;
    std::vector<int> roots;
    std::vector<int> indices;
    std::vector<Interval> bounds;
    Eigen::VectorXd center;
};

struct RootComponentGraph {
    RootGroups groups;
    std::unordered_map<int, int> root_to_component;
    std::vector<RootComponent> components;
    int connected_cross_root_pairs = 0;
};

struct LocalDisjointSet {
    std::vector<int> parent;
    std::vector<int> rank;

    explicit LocalDisjointSet(int n) : parent(static_cast<std::size_t>(n)), rank(static_cast<std::size_t>(n), 0) {
        for (int i = 0; i < n; ++i) parent[static_cast<std::size_t>(i)] = i;
    }

    int find(int value) {
        int& p = parent[static_cast<std::size_t>(value)];
        if (p != value) p = find(p);
        return p;
    }

    void unite(int lhs, int rhs) {
        int left = find(lhs);
        int right = find(rhs);
        if (left == right) return;
        if (rank[static_cast<std::size_t>(left)] < rank[static_cast<std::size_t>(right)]) {
            std::swap(left, right);
        }
        parent[static_cast<std::size_t>(right)] = left;
        if (rank[static_cast<std::size_t>(left)] == rank[static_cast<std::size_t>(right)]) {
            rank[static_cast<std::size_t>(left)] += 1;
        }
    }
};

RootComponentGraph build_root_component_graph(const std::vector<BoxNode>& boxes,
                                              double adjacency_tolerance,
                                              bool island_aware) {
    RootComponentGraph graph;
    graph.groups = group_boxes_by_root(boxes);
    if (graph.groups.roots.empty()) {
        return graph;
    }

    std::unordered_map<int, int> root_to_ordinal;
    root_to_ordinal.reserve(graph.groups.roots.size());
    for (int ordinal = 0; ordinal < static_cast<int>(graph.groups.roots.size()); ++ordinal) {
        root_to_ordinal[graph.groups.roots[static_cast<std::size_t>(ordinal)]] = ordinal;
    }

    LocalDisjointSet dsu(static_cast<int>(graph.groups.roots.size()));
    if (island_aware) {
        for (int lhs_index = 0; lhs_index < static_cast<int>(boxes.size()); ++lhs_index) {
            const BoxNode& lhs = boxes[static_cast<std::size_t>(lhs_index)];
            if (lhs.root_id < 0) continue;
            for (int rhs_index = lhs_index + 1; rhs_index < static_cast<int>(boxes.size()); ++rhs_index) {
                const BoxNode& rhs = boxes[static_cast<std::size_t>(rhs_index)];
                if (rhs.root_id < 0 || rhs.root_id == lhs.root_id) continue;
                if (!boxes_connected(lhs, rhs, adjacency_tolerance)) continue;
                dsu.unite(root_to_ordinal.at(lhs.root_id), root_to_ordinal.at(rhs.root_id));
                graph.connected_cross_root_pairs += 1;
            }
        }
    }

    std::unordered_map<int, int> dsu_to_component;
    for (int root : graph.groups.roots) {
        const int dsu_root = dsu.find(root_to_ordinal.at(root));
        auto [component_it, inserted] = dsu_to_component.emplace(dsu_root, static_cast<int>(graph.components.size()));
        if (inserted) {
            RootComponent component;
            component.id = component_it->second;
            graph.components.push_back(std::move(component));
        }
        const int component_index = component_it->second;
        graph.root_to_component[root] = component_index;
        RootComponent& component = graph.components[static_cast<std::size_t>(component_index)];
        component.roots.push_back(root);
        const auto group_it = graph.groups.by_root.find(root);
        if (group_it != graph.groups.by_root.end()) {
            component.indices.insert(component.indices.end(), group_it->second.begin(), group_it->second.end());
        }
    }

    for (RootComponent& component : graph.components) {
        std::sort(component.roots.begin(), component.roots.end());
        component.bounds = bounds_for_indices(boxes, component.indices);
        component.center = intervals_center(component.bounds);
    }
    return graph;
}

std::uint64_t component_pair_key(int lhs_root_id, int rhs_root_id) {
    const std::uint32_t lo = static_cast<std::uint32_t>(std::min(lhs_root_id, rhs_root_id));
    const std::uint32_t hi = static_cast<std::uint32_t>(std::max(lhs_root_id, rhs_root_id));
    return (static_cast<std::uint64_t>(lo) << 32) | static_cast<std::uint64_t>(hi);
}

std::string frontier_seed_cache_key(const Eigen::Ref<const Eigen::VectorXd>& seed) {
    std::ostringstream out;
    out << seed.size() << ':' << std::setprecision(17);
    for (int dim = 0; dim < seed.size(); ++dim) {
        out << seed[dim] << ',';
    }
    return out.str();
}

double normalized_linf_distance(const std::vector<Interval>& root,
                                const Eigen::Ref<const Eigen::VectorXd>& lhs,
                                const Eigen::Ref<const Eigen::VectorXd>& rhs) {
    if (lhs.size() != rhs.size() || lhs.size() != static_cast<int>(root.size())) {
        return 0.0;
    }
    double distance = 0.0;
    for (int dim = 0; dim < lhs.size(); ++dim) {
        const double width = std::max(root[static_cast<std::size_t>(dim)].width(), 1e-12);
        distance = std::max(distance, std::abs(lhs[dim] - rhs[dim]) / width);
    }
    return distance;
}

int common_ancestor_depth(const BoxOracle& oracle, OracleNodeId lhs_node, OracleNodeId rhs_node) {
    return oracle.common_ancestor_depth(lhs_node, rhs_node);
}

void set_max_diagnostic(StageContext& context, const std::string& key, double value) {
    context.diagnostics().set_value(key, std::max(context.diagnostics().value(key), value));
}

int select_depth_stage_index(const GrowerConfig& config, int box_count) {
    if (config.depth_stages.empty()) {
        return -1;
    }
    for (int index = 0; index < static_cast<int>(config.depth_stages.size()); ++index) {
        const auto& stage = config.depth_stages[static_cast<std::size_t>(index)];
        if (stage.box_limit <= 0 || box_count < stage.box_limit) {
            return index;
        }
    }
    return static_cast<int>(config.depth_stages.size()) - 1;
}

const GrowerConfig::DepthStage* depth_stage_or_null(const GrowerConfig& config, int stage_index) {
    if (stage_index < 0 || stage_index >= static_cast<int>(config.depth_stages.size())) {
        return nullptr;
    }
    return &config.depth_stages[static_cast<std::size_t>(stage_index)];
}

FindFreeBoxOptions staged_ffb_options(const GrowerConfig& config, int stage_index) {
    FindFreeBoxOptions options = config.find_free_box;
    const auto* stage = depth_stage_or_null(config, stage_index);
    if (stage != nullptr && stage->ffb_depth > 0) {
        options.max_depth = stage->ffb_depth;
    }
    return options;
}

FindFreeBoxOptions component_connect_ffb_options(const GrowerConfig& config,
                                                 StageContext& context,
                                                 const FindFreeBoxOptions& base_options,
                                                 int stage_index,
                                                 int pair_unknown_failures) {
    FindFreeBoxOptions options = base_options;
    if (!config.component_connect_adaptive_ffb) {
        return options;
    }
    if (config.component_connect_depth_after_unknown_only && pair_unknown_failures <= 0) {
        context.diagnostics().add_counter("grower.component_connect_base_depth_tasks");
        return options;
    }
    const auto* stage = depth_stage_or_null(config, stage_index);
    const int depth_increment = stage != nullptr && stage->component_connect_ffb_depth_increment >= 0
        ? stage->component_connect_ffb_depth_increment
        : config.component_connect_ffb_depth_increment;
    const int configured_max_depth = stage != nullptr && stage->component_connect_ffb_max_depth > 0
        ? stage->component_connect_ffb_max_depth
        : config.component_connect_ffb_max_depth;
    const int base_depth = std::max(0, options.max_depth);
    const int max_depth = std::max(base_depth, configured_max_depth);
    options.max_depth = std::min(max_depth,
                                 base_depth + std::max(0, depth_increment) * std::max(1, pair_unknown_failures));
    context.diagnostics().add_counter("grower.component_connect_adaptive_ffb_tasks");
    if (pair_unknown_failures > 0) {
        context.diagnostics().add_counter("grower.component_connect_unknown_depth_retry_tasks");
        set_max_diagnostic(context,
                           "grower.component_connect_pair_unknown_failures_max",
                           static_cast<double>(pair_unknown_failures));
    }
    set_max_diagnostic(context,
                       "grower.component_connect_adaptive_ffb_depth_max",
                       static_cast<double>(options.max_depth));
    return options;
}

void record_grower_ffb_failure(StageContext& context,
                               const FindFreeBoxResult& result) {
    context.diagnostics().add_counter("grower.ffb_failures");
    context.diagnostics().add_counter("grower.ffb_fail_code." + std::to_string(result.fail_code));
    if (result.seed_collision) {
        context.diagnostics().add_counter("grower.ffb_seed_collision");
    }
    if (result.hit_unknown_depth_cap) {
        context.diagnostics().add_counter("grower.ffb_unknown_depth_cap");
    }
    if (result.hit_reserved_depth_cap) {
        context.diagnostics().add_counter("grower.ffb_reserved_depth_cap");
    }
    if (result.deadline_reached) {
        context.diagnostics().add_counter("grower.ffb_deadline_reached");
    }
}

void record_worker_oracle_counters(StageContext& context,
                                   const OracleCounters& counters) {
    context.diagnostics().add_counter("grower.worker_oracle.node_validations", counters.node_validations);
    context.diagnostics().add_counter("grower.worker_oracle.materializations", counters.materializations);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_source_incremental_state",
                                      counters.materialization_source_incremental_state);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_reused_endpoint_cache",
                                      counters.materialization_reused_endpoint_cache);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_reused_external_evidence",
                                      counters.materialization_reused_external_evidence);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_reused_shared_endpoint_cache",
                                      counters.materialization_reused_shared_endpoint_cache);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_stored_shared_endpoint_cache",
                                      counters.materialization_stored_shared_endpoint_cache);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_reused_cached_envelope",
                                      counters.materialization_reused_cached_envelope);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_endpoint_time_us",
                                      counters.materialization_endpoint_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_endpoint_wall_time_us",
                                      counters.materialization_endpoint_wall_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.validate_node_total_time_us",
                                      counters.validate_node_total_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.validate_node_preamble_time_us",
                                      counters.validate_node_preamble_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.validate_node_endpoint_path_time_us",
                                      counters.validate_node_endpoint_path_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.validate_node_classify_time_us",
                                      counters.validate_node_classify_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.validate_node_overhead_time_us",
                                      counters.validate_node_overhead_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_envelope_time_us",
                                      counters.materialization_envelope_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_cache_lookup_time_us",
                                      counters.materialization_cache_lookup_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_cache_read_time_us",
                                      counters.materialization_cache_read_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_external_lookup_time_us",
                                      counters.materialization_external_lookup_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_external_read_time_us",
                                      counters.materialization_external_read_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_envelope_compute_time_us",
                                      counters.materialization_envelope_compute_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_envelope_read_time_us",
                                      counters.materialization_envelope_read_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_envelope_collision_time_us",
                                      counters.materialization_envelope_collision_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_candidate_dirty_count",
                                      counters.materialization_candidate_dirty_count);
    context.diagnostics().add_counter("grower.worker_oracle.materialization_predh_rebuild_count",
                                      counters.materialization_predh_rebuild_count);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_evaluations", counters.scoring_evaluations);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_changed_dim_inferred",
                                      counters.scoring_changed_dim_inferred);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_source_incremental_state",
                                      counters.scoring_source_incremental_state);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_reused_endpoint_cache",
                                      counters.scoring_reused_endpoint_cache);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_reused_external_evidence",
                                      counters.scoring_reused_external_evidence);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_endpoint_time_us",
                                      counters.scoring_endpoint_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_envelope_time_us",
                                      counters.scoring_envelope_time_us);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_candidate_dirty_count",
                                      counters.scoring_candidate_dirty_count);
    context.diagnostics().add_counter("grower.worker_oracle.scoring_predh_rebuild_count",
                                      counters.scoring_predh_rebuild_count);
}

void record_committed_box_stats(StageContext& context,
                                const BoxNode& box) {
    context.diagnostics().add_counter("grower.committed_box_volume_sum", box.volume);
    set_max_diagnostic(context, "grower.committed_box_volume_max", box.volume);
    if (box.safety_status == BoxSafetyStatus::CertifiedFree) {
        context.diagnostics().add_counter("grower.certified_boxes_committed");
        context.diagnostics().add_counter("grower.certified_box_volume_sum", box.volume);
    } else if (box.safety_status == BoxSafetyStatus::ProvisionalFree) {
        context.diagnostics().add_counter("grower.provisional_boxes_committed");
        context.diagnostics().add_counter("grower.provisional_box_volume_sum", box.volume);
    }
    if (box.strict_audit_required) {
        context.diagnostics().add_counter("grower.strict_audit_required_boxes");
        context.diagnostics().add_counter("grower.strict_audit_required_volume_sum", box.volume);
    }
}

void finalize_result(GrowerResult& result, double adjacency_tol) {
    result.adjacency = compute_adjacency(result.boxes, adjacency_tol);
    auto islands = find_islands(result.adjacency);
    result.adjacency_islands = static_cast<int>(islands.size());
    result.all_connected = islands.size() <= 1;
    result.adjacency_largest_island = 0;
    for (const auto& island : islands) {
        result.adjacency_largest_island = std::max(result.adjacency_largest_island, static_cast<int>(island.size()));
    }
}

OracleNodeId find_leaf_containing(BoxOracle& oracle, const Eigen::Ref<const Eigen::VectorXd>& q) {
    if (q.size() != oracle.n_dims() || !oracle.contains_point(oracle.root_node(), q)) {
        return kInvalidOracleNodeId;
    }
    OracleNodeId node = oracle.root_node();
    while (!oracle.is_leaf(node)) {
        node = oracle.child_containing_point(node, q);
        if (node == kInvalidOracleNodeId) {
            break;
        }
    }
    return node;
}

void write_json_string(std::ostream& out, std::string_view value) {
    out << '"';
    for (char ch : value) {
        switch (ch) {
            case '"': out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\b': out << "\\b"; break;
            case '\f': out << "\\f"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20) {
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(static_cast<unsigned char>(ch))
                        << std::dec << std::setfill(' ');
                } else {
                    out << ch;
                }
        }
    }
    out << '"';
}

void write_json_vector(std::ostream& out, const Eigen::Ref<const Eigen::VectorXd>& vector) {
    out << '[';
    for (int index = 0; index < vector.size(); ++index) {
        if (index > 0) out << ',';
        out << std::setprecision(17) << vector[index];
    }
    out << ']';
}

void write_json_intervals(std::ostream& out, const std::vector<Interval>& intervals) {
    out << '[';
    for (std::size_t index = 0; index < intervals.size(); ++index) {
        if (index > 0) out << ',';
        out << '[' << std::setprecision(17) << intervals[index].lo << ',' << intervals[index].hi << ']';
    }
    out << ']';
}

void write_json_face(std::ostream& out, const GrowTraceFace& face) {
    out << '{'
        << "\"valid\":" << (face.valid ? "true" : "false")
        << ",\"rank\":" << face.rank
        << ",\"selected\":" << (face.selected ? "true" : "false")
        << ",\"seed_covered\":" << (face.seed_covered ? "true" : "false")
        << ",\"parent_index\":" << face.parent_index
        << ",\"parent_box_id\":" << face.parent_box_id
        << ",\"dim\":" << face.dim
        << ",\"side\":" << face.side
        << ",\"face_value\":" << std::setprecision(17) << face.face_value
        << ",\"score\":" << std::setprecision(17) << face.score
        << ",\"scanned_boxes\":" << face.scanned_boxes
        << ",\"scanned_faces\":" << face.scanned_faces
        << '}';
}

void write_json_faces(std::ostream& out, const std::vector<GrowTraceFace>& faces) {
    out << '[';
    for (std::size_t index = 0; index < faces.size(); ++index) {
        if (index > 0) out << ',';
        write_json_face(out, faces[index]);
    }
    out << ']';
}

const GrowTraceFace* trace_face_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr && worker_result->selected_face.valid) {
        return &worker_result->selected_face;
    }
    if (task != nullptr && task->selected_face.valid) {
        return &task->selected_face;
    }
    return nullptr;
}

int trace_task_id_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr) return worker_result->task_id;
    return task != nullptr ? task->task_id : -1;
}

int trace_iteration_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr) return worker_result->iteration;
    return task != nullptr ? task->iteration : -1;
}

int trace_target_root_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr) return worker_result->target_root_id;
    return task != nullptr ? task->target_root_id : -1;
}

const char* trace_target_type_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr) return grow_target_type_str(worker_result->target_type);
    return task != nullptr ? grow_target_type_str(task->target_type) : "";
}

const Eigen::VectorXd* trace_target_from(const GrowTask* task, const GrowWorkerResult* worker_result) {
    if (worker_result != nullptr && worker_result->target.size() > 0) return &worker_result->target;
    if (task != nullptr && task->target.size() > 0) return &task->target;
    return nullptr;
}

}  // namespace

RrtGrower::RrtGrower(BoxOracle& oracle, GrowerConfig config)
    : oracle_(oracle), config_(std::move(config)), rng_(config_.rng_seed) {}

void RrtGrower::open_trace() {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    trace_event_count_ = 0;
    trace_opened_ = false;
    if (trace_file_.is_open()) {
        trace_file_.close();
    }
    if (!config_.trace_enabled || config_.trace_path.empty()) {
        return;
    }
    const std::filesystem::path path(config_.trace_path);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    trace_file_.open(path, std::ios::out | std::ios::trunc);
    trace_opened_ = trace_file_.is_open();
}

void RrtGrower::close_trace() {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    if (trace_file_.is_open()) {
        trace_file_.flush();
        trace_file_.close();
    }
    trace_opened_ = false;
}

bool RrtGrower::trace_enabled() const {
    return config_.trace_enabled && trace_opened_ &&
           (config_.trace_max_events <= 0 ||
            trace_event_count_ < static_cast<std::uint64_t>(config_.trace_max_events));
}

void RrtGrower::write_trace_event(const std::string& event,
                                  const std::function<void(std::ostream&)>& write_fields) const {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    if (!config_.trace_enabled || !trace_opened_ || !trace_file_.is_open()) {
        return;
    }
    if (config_.trace_max_events > 0 &&
        trace_event_count_ >= static_cast<std::uint64_t>(config_.trace_max_events)) {
        return;
    }
    trace_file_ << "{\"event_index\":" << trace_event_count_
                << ",\"event\":";
    write_json_string(trace_file_, event);
    write_fields(trace_file_);
    trace_file_ << "}\n";
    trace_event_count_ += 1;
}

void RrtGrower::trace_root_seed(int iteration,
                                int root_id,
                                const Eigen::Ref<const Eigen::VectorXd>& seed) const {
    write_trace_event("root_seed", [&](std::ostream& out) {
        out << ",\"iteration\":" << iteration
            << ",\"worker_id\":-1"
            << ",\"root_id\":" << root_id
            << ",\"seed\":";
        write_json_vector(out, seed);
    });
}

void RrtGrower::trace_task_plan(const GrowTask& task) const {
    write_trace_event("rrt_sampled_target", [&](std::ostream& out) {
        out << ",\"task_id\":" << task.task_id
            << ",\"iteration\":" << task.iteration
            << ",\"worker_id\":-1"
            << ",\"source_root_id\":" << task.source_root_id
            << ",\"root_id\":" << task.root_id
            << ",\"target_root_id\":" << task.target_root_id
            << ",\"intertree_goal_bias\":" << (task.intertree_goal_bias ? "true" : "false")
            << ",\"component_connect_target\":" << (task.component_connect_target ? "true" : "false")
            << ",\"component_pair_unknown_failures\":" << task.component_pair_unknown_failures
            << ",\"component_connect_staged_target\":" << (task.component_connect_staged_target ? "true" : "false")
            << ",\"component_connect_gap_sq\":" << std::setprecision(17) << task.component_connect_gap_sq
            << ",\"target_type\":";
        write_json_string(out, grow_target_type_str(task.target_type));
        out << ",\"target\":";
        if (task.target.size() > 0) {
            write_json_vector(out, task.target);
        } else {
            out << "[]";
        }
    });
    if (task.selected_face.valid) {
        if (!task.face_candidates.empty()) {
            write_trace_event("nearest_face_candidates", [&](std::ostream& out) {
                out << ",\"task_id\":" << task.task_id
                    << ",\"iteration\":" << task.iteration
                    << ",\"worker_id\":-1"
                    << ",\"selected_rank\":" << task.selected_face.rank
                    << ",\"candidate_count\":" << task.face_candidates.size()
                    << ",\"faces\":";
                write_json_faces(out, task.face_candidates);
            });
        }
        write_trace_event("nearest_face_candidate", [&](std::ostream& out) {
            out << ",\"task_id\":" << task.task_id
                << ",\"iteration\":" << task.iteration
                << ",\"worker_id\":-1"
                << ",\"selected\":true"
                << ",\"face\":";
            write_json_face(out, task.selected_face);
        });
        write_trace_event("selected_face", [&](std::ostream& out) {
            out << ",\"task_id\":" << task.task_id
                << ",\"iteration\":" << task.iteration
                << ",\"worker_id\":-1"
                << ",\"seed\":";
            write_json_vector(out, task.seed);
            out << ",\"target\":";
            if (task.target.size() > 0) {
                write_json_vector(out, task.target);
            } else {
                out << "[]";
            }
            out << ",\"face\":";
            write_json_face(out, task.selected_face);
        });
    }
    write_trace_event("rrt_seed", [&](std::ostream& out) {
        out << ",\"task_id\":" << task.task_id
            << ",\"iteration\":" << task.iteration
            << ",\"worker_id\":-1"
            << ",\"parent_box_id\":" << task.parent_box_id
            << ",\"root_id\":" << task.root_id
            << ",\"target_root_id\":" << task.target_root_id
            << ",\"component_connect_target\":" << (task.component_connect_target ? "true" : "false")
            << ",\"component_pair_unknown_failures\":" << task.component_pair_unknown_failures
            << ",\"component_connect_staged_target\":" << (task.component_connect_staged_target ? "true" : "false")
            << ",\"seed\":";
        write_json_vector(out, task.seed);
    });
}

void RrtGrower::trace_ffb_result(const std::string& event,
                                 const Eigen::Ref<const Eigen::VectorXd>& seed,
                                 const FindFreeBoxResult& ffb_result,
                                 int parent_box_id,
                                 int root_id,
                                 const GrowTask* task,
                                 const GrowWorkerResult* worker_result,
                                 int worker_id,
                                 int ffb_depth) const {
    const bool component_connect_target = worker_result != nullptr
        ? worker_result->component_connect_target
        : (task != nullptr && task->component_connect_target);
    const int source_root_id = worker_result != nullptr
        ? worker_result->source_root_id
        : (task != nullptr ? task->source_root_id : -1);
    write_trace_event(event, [&](std::ostream& out) {
        out << ",\"task_id\":" << trace_task_id_from(task, worker_result)
            << ",\"iteration\":" << trace_iteration_from(task, worker_result)
            << ",\"worker_id\":" << worker_id
            << ",\"parent_box_id\":" << parent_box_id
            << ",\"root_id\":" << root_id
            << ",\"source_root_id\":" << source_root_id
            << ",\"target_root_id\":" << trace_target_root_from(task, worker_result)
            << ",\"ffb_depth\":" << ffb_depth
            << ",\"component_connect_target\":" << (component_connect_target ? "true" : "false")
            << ",\"component_pair_unknown_failures\":"
            << (worker_result != nullptr ? worker_result->component_pair_unknown_failures : (task != nullptr ? task->component_pair_unknown_failures : 0))
            << ",\"component_connect_staged_target\":"
            << ((worker_result != nullptr ? worker_result->component_connect_staged_target : (task != nullptr && task->component_connect_staged_target)) ? "true" : "false")
            << ",\"component_connect_gap_sq\":" << std::setprecision(17)
            << (worker_result != nullptr ? worker_result->component_connect_gap_sq : (task != nullptr ? task->component_connect_gap_sq : 0.0))
            << ",\"found\":" << (ffb_result.found ? "true" : "false")
            << ",\"node\":" << ffb_result.node
            << ",\"fail_code\":" << ffb_result.fail_code
            << ",\"splits\":" << ffb_result.splits
            << ",\"decisions\":" << ffb_result.decisions
            << ",\"seed_collision\":" << (ffb_result.seed_collision ? "true" : "false")
            << ",\"hit_reserved_depth_cap\":" << (ffb_result.hit_reserved_depth_cap ? "true" : "false")
            << ",\"hit_unknown_depth_cap\":" << (ffb_result.hit_unknown_depth_cap ? "true" : "false")
            << ",\"deadline_reached\":" << (ffb_result.deadline_reached ? "true" : "false")
            << ",\"total_ms\":" << std::setprecision(17) << ffb_result.total_ms
            << ",\"target_type\":";
        write_json_string(out, trace_target_type_from(task, worker_result));
        out << ",\"seed\":";
        write_json_vector(out, seed);
        out << ",\"target\":";
        if (const auto* target = trace_target_from(task, worker_result)) {
            write_json_vector(out, *target);
        } else {
            out << "[]";
        }
        if (!ffb_result.intervals.empty()) {
            out << ",\"intervals\":";
            write_json_intervals(out, ffb_result.intervals);
        }
        if (const auto* face = trace_face_from(task, worker_result)) {
            out << ",\"selected_face\":";
            write_json_face(out, *face);
        }
    });
}

void RrtGrower::trace_box_added(const BoxNode& box,
                                const GrowTask* task,
                                const GrowWorkerResult* worker_result,
                                int worker_id) const {
    write_trace_event("box_added", [&](std::ostream& out) {
        out << ",\"task_id\":" << trace_task_id_from(task, worker_result)
            << ",\"iteration\":" << trace_iteration_from(task, worker_result)
            << ",\"worker_id\":" << worker_id
            << ",\"box_id\":" << box.id
            << ",\"parent_box_id\":" << box.parent_box_id
            << ",\"root_id\":" << box.root_id
            << ",\"tree_id\":" << box.tree_id
            << ",\"target_root_id\":" << trace_target_root_from(task, worker_result)
            << ",\"volume\":" << std::setprecision(17) << box.volume
            << ",\"seed\":";
        write_json_vector(out, box.seed_config);
        out << ",\"intervals\":";
        write_json_intervals(out, box.joint_intervals);
        if (const auto* face = trace_face_from(task, worker_result)) {
            out << ",\"selected_face\":";
            write_json_face(out, *face);
        }
    });
}

void RrtGrower::trace_box_rejected(const std::string& reason,
                                   const Eigen::Ref<const Eigen::VectorXd>& seed,
                                   int parent_box_id,
                                   int root_id,
                                   const GrowTask* task,
                                   const GrowWorkerResult* worker_result,
                                   int worker_id,
                                   const FindFreeBoxResult* ffb_result) const {
    write_trace_event("box_rejected", [&](std::ostream& out) {
        out << ",\"task_id\":" << trace_task_id_from(task, worker_result)
            << ",\"iteration\":" << trace_iteration_from(task, worker_result)
            << ",\"worker_id\":" << worker_id
            << ",\"parent_box_id\":" << parent_box_id
            << ",\"root_id\":" << root_id
            << ",\"reason\":";
        write_json_string(out, reason);
        out << ",\"seed\":";
        write_json_vector(out, seed);
        if (ffb_result != nullptr) {
            out << ",\"ffb_node\":" << ffb_result->node
                << ",\"ffb_fail_code\":" << ffb_result->fail_code;
        }
        if (const auto* face = trace_face_from(task, worker_result)) {
            out << ",\"selected_face\":";
            write_json_face(out, *face);
        }
    });
}

GrowerResult RrtGrower::grow(const std::vector<Eigen::VectorXd>& seeds) {
    RuntimeConfig runtime;
    runtime.mode = config_.n_threads > 1 ? ExecutionMode::Parallel : ExecutionMode::Inline;
    runtime.n_threads = config_.n_threads;
    runtime.batch_size = config_.task_batch_size;
    runtime.parallel_threshold = config_.parallel_threshold;
    runtime.deterministic_reduce = config_.deterministic_reduce;
    StageContext context(runtime, Deadline::after_ms(config_.timeout_ms));
    return grow(seeds, context);
}

GrowerResult RrtGrower::grow_from_existing(const std::vector<BoxNode>& initial_boxes,
                                           const std::vector<Eigen::VectorXd>& seeds,
                                           StageContext& context) {
    initial_boxes_ = initial_boxes;
    GrowerResult result = grow(seeds, context);
    initial_boxes_.clear();
    return result;
}

std::vector<Eigen::VectorXd> RrtGrower::select_initial_roots(const std::vector<Eigen::VectorXd>& seeds,
                                                             StageContext& context) {
    const auto root = oracle_.native_root_hull();
    std::vector<Eigen::VectorXd> selected;
    std::vector<OracleNodeId> selected_leaves;

    auto try_add_root = [&](const Eigen::VectorXd& candidate,
                            bool user_seed,
                            bool enforce_distance) {
        if (candidate.size() != static_cast<int>(root.size()) ||
            !oracle_.contains_point(oracle_.root_node(), candidate)) {
            context.diagnostics().add_counter(user_seed
                ? "grower.root_seed_user_invalid"
                : "grower.root_seed_candidate_invalid");
            return false;
        }
        if (oracle_.point_in_collision(candidate)) {
            context.diagnostics().add_counter(user_seed
                ? "grower.root_seed_user_collision"
                : "grower.root_seed_candidate_collision");
            return false;
        }
        if (enforce_distance && config_.root_seed_min_normalized_linf > 0.0) {
            for (const auto& existing : selected) {
                if (normalized_linf_distance(root, existing, candidate) < config_.root_seed_min_normalized_linf) {
                    context.diagnostics().add_counter("grower.root_seed_min_distance_rejected");
                    return false;
                }
            }
        }
        const OracleNodeId leaf = find_leaf_containing(oracle_, candidate);
        if (config_.root_seed_max_lca_depth >= 0 && leaf >= 0) {
            for (OracleNodeId selected_leaf : selected_leaves) {
                if (selected_leaf < 0) continue;
                const int ancestor_depth = common_ancestor_depth(oracle_, leaf, selected_leaf);
                if (ancestor_depth < 0) {
                    context.diagnostics().add_counter("grower.root_seed_lca_unavailable");
                    continue;
                }
                if (ancestor_depth > config_.root_seed_max_lca_depth) {
                    context.diagnostics().add_counter("grower.root_seed_lca_rejected");
                    return false;
                }
            }
        }
        selected.push_back(candidate);
        selected_leaves.push_back(leaf);
        return true;
    };

    if (config_.root_seed_include_user_seeds) {
        for (const auto& seed : seeds) {
            try_add_root(seed, true, false);
        }
    }

    int target_count = static_cast<int>(selected.size()) + std::max(0, config_.extra_random_roots);
    if (target_count <= 0) {
        target_count = 1;
    }

    const int candidates_per_round = std::max(1, config_.root_seed_candidate_count);
    int attempts = 0;
    int empty_rounds = 0;
    while (static_cast<int>(selected.size()) < target_count && empty_rounds < 4) {
        Eigen::VectorXd best_candidate;
        OracleNodeId best_leaf = kInvalidOracleNodeId;
        double best_score = -1.0;
        bool found = false;
        for (int candidate_index = 0; candidate_index < candidates_per_round; ++candidate_index) {
            attempts += 1;
            Eigen::VectorXd candidate = sample_uniform();
            if (oracle_.point_in_collision(candidate)) {
                context.diagnostics().add_counter("grower.root_seed_candidate_collision");
                continue;
            }
            double min_distance = selected.empty() ? 1.0 : std::numeric_limits<double>::infinity();
            for (const auto& existing : selected) {
                min_distance = std::min(min_distance, normalized_linf_distance(root, existing, candidate));
            }
            if (!selected.empty() && min_distance < config_.root_seed_min_normalized_linf) {
                context.diagnostics().add_counter("grower.root_seed_min_distance_rejected");
                continue;
            }
            const OracleNodeId leaf = find_leaf_containing(oracle_, candidate);
            bool lca_ok = true;
            if (config_.root_seed_max_lca_depth >= 0 && leaf >= 0) {
                for (OracleNodeId selected_leaf : selected_leaves) {
                    if (selected_leaf < 0) continue;
                    const int ancestor_depth = common_ancestor_depth(oracle_, leaf, selected_leaf);
                    if (ancestor_depth < 0) {
                        context.diagnostics().add_counter("grower.root_seed_lca_unavailable");
                        continue;
                    }
                    if (ancestor_depth > config_.root_seed_max_lca_depth) {
                        context.diagnostics().add_counter("grower.root_seed_lca_rejected");
                        lca_ok = false;
                        break;
                    }
                }
            }
            if (!lca_ok) {
                continue;
            }
            if (min_distance > best_score) {
                best_score = min_distance;
                best_candidate = std::move(candidate);
                best_leaf = leaf;
                found = true;
            }
        }
        if (!found) {
            empty_rounds += 1;
            continue;
        }
        selected.push_back(std::move(best_candidate));
        selected_leaves.push_back(best_leaf);
        empty_rounds = 0;
    }

    if (selected.empty()) {
        selected.push_back(sample_uniform());
    }

    double min_pair_distance = selected.size() <= 1 ? 0.0 : std::numeric_limits<double>::infinity();
    for (int outer = 0; outer < static_cast<int>(selected.size()); ++outer) {
        for (int inner = outer + 1; inner < static_cast<int>(selected.size()); ++inner) {
            min_pair_distance = std::min(min_pair_distance,
                                         normalized_linf_distance(root,
                                                                  selected[static_cast<std::size_t>(outer)],
                                                                  selected[static_cast<std::size_t>(inner)]));
        }
    }
    context.diagnostics().set_value("grower.root_seed_attempts", static_cast<double>(attempts));
    context.diagnostics().set_value("grower.root_seeds_target_count", static_cast<double>(target_count));
    context.diagnostics().set_value("grower.root_seeds_final_count", static_cast<double>(selected.size()));
    context.diagnostics().set_value("grower.root_seeds_min_normalized_linf", min_pair_distance);
    if (config_.intertree_goal_bias > 0.5 || config_.rrt_goal_bias > 0.5) {
        context.diagnostics().set_value("grower.goal_bias_high_warning", 1.0);
    }
    return selected;
}

GrowerResult RrtGrower::grow(const std::vector<Eigen::VectorXd>& seeds,
                             StageContext& context) {
    using Clock = std::chrono::steady_clock;
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.grow");
    const auto t0 = Clock::now();
    const auto deadline = t0 + std::chrono::duration_cast<Clock::duration>(
                                 std::chrono::duration<double, std::milli>(config_.timeout_ms));
    GrowerResult result;
    FindFreeBoxService ffb(oracle_);
    open_trace();
    const bool has_initial_boxes = !initial_boxes_.empty();
    if (has_initial_boxes) {
        result.boxes = initial_boxes_;
        next_box_id_ = 0;
        int max_root_id = -1;
        for (const BoxNode& box : result.boxes) {
            next_box_id_ = std::max(next_box_id_, box.id + 1);
            max_root_id = std::max(max_root_id, box.root_id);
        }
        result.n_roots = static_cast<int>(group_boxes_by_root(result.boxes).roots.size());
        context.diagnostics().set_value("grower.initial_boxes", static_cast<double>(result.boxes.size()));
        context.diagnostics().set_value("grower.initial_roots", static_cast<double>(result.n_roots));
        context.diagnostics().set_value("grower.initial_next_box_id", static_cast<double>(next_box_id_));
        context.diagnostics().set_value("grower.initial_root_id_base", static_cast<double>(max_root_id + 1));
    } else {
        next_box_id_ = 0;
    }
    random_anchor_targets_.clear();
    component_parent_failures_.clear();
    failure_cooling_.clear();
    if (config_.boundary_epsilon > config_.adjacency_tolerance) {
        context.diagnostics().set_value("grower.invalid_boundary_epsilon", 1.0);
    }
    context.diagnostics().set_value("grower.executor_threads", static_cast<double>(context.executor().n_threads()));
    context.diagnostics().set_value("grower.config_threads", static_cast<double>(config_.n_threads));

    int active_depth_stage_index = -2;
    FindFreeBoxOptions active_ffb_options = config_.find_free_box;
    auto refresh_depth_stage = [&](int box_count) {
        const int next_stage = select_depth_stage_index(config_, box_count);
        active_ffb_options = staged_ffb_options(config_, next_stage);
        if (next_stage != active_depth_stage_index) {
            if (active_depth_stage_index != -2) {
                context.diagnostics().add_counter("grower.depth_stage_switches");
                set_max_diagnostic(context,
                                   "grower.depth_stage_box_count_at_switch_max",
                                   static_cast<double>(box_count));
            }
            active_depth_stage_index = next_stage;
        }
        context.diagnostics().set_value("grower.depth_stage_index", static_cast<double>(active_depth_stage_index));
        context.diagnostics().set_value("grower.depth_stage_depth", static_cast<double>(active_ffb_options.max_depth));
        set_max_diagnostic(context,
                           "grower.depth_stage_depth_max",
                           static_cast<double>(active_ffb_options.max_depth));
    };

    std::vector<Eigen::VectorXd> roots = select_initial_roots(seeds, context);
    const int n_anchor_targets = std::max(0, config_.random_anchor_targets);
    random_anchor_targets_.reserve(config_.fixed_anchor_targets.size() + static_cast<std::size_t>(n_anchor_targets));
    std::vector<Eigen::VectorXd> anchor_reference_points = roots;
    if (anchor_reference_points.empty()) {
        anchor_reference_points = seeds;
    }
    std::vector<OracleNodeId> anchor_reference_leaves;
    anchor_reference_leaves.reserve(anchor_reference_points.size() + static_cast<std::size_t>(n_anchor_targets));
    for (const auto& point : anchor_reference_points) {
        anchor_reference_leaves.push_back(find_leaf_containing(oracle_, point));
    }
    for (const Eigen::VectorXd& anchor : config_.fixed_anchor_targets) {
        if (anchor.size() != static_cast<int>(oracle_.native_root_hull().size())) {
            context.diagnostics().add_counter("grower.fixed_anchor_target_invalid");
            continue;
        }
        if (oracle_.point_in_collision(anchor)) {
            context.diagnostics().add_counter("grower.fixed_anchor_target_collision");
            continue;
        }
        const Eigen::VectorXd clipped_anchor = clip_to_root_intervals(anchor, oracle_.native_root_hull());
        const double clip_delta = (clipped_anchor - anchor).cwiseAbs().maxCoeff();
        if (clip_delta > 1e-12) {
            context.diagnostics().add_counter("grower.fixed_anchor_target_clipped_to_root");
            set_max_diagnostic(context,
                               "grower.fixed_anchor_target_clip_delta_max",
                               clip_delta);
        }
        if (oracle_.point_in_collision(clipped_anchor)) {
            context.diagnostics().add_counter("grower.fixed_anchor_target_clipped_collision");
            continue;
        }
        random_anchor_targets_.push_back(clipped_anchor);
        if (oracle_.contains_point(oracle_.root_node(), clipped_anchor)) {
            anchor_reference_points.push_back(clipped_anchor);
            anchor_reference_leaves.push_back(find_leaf_containing(oracle_, clipped_anchor));
        }
        context.diagnostics().add_counter("grower.fixed_anchor_targets");
    }
    const int anchor_candidates_per_round = std::max(1, config_.anchor_target_candidate_count);
    for (int anchor_index = 0; anchor_index < n_anchor_targets; ++anchor_index) {
        Eigen::VectorXd best_anchor;
        OracleNodeId best_leaf = kInvalidOracleNodeId;
        int best_lca_depth = std::numeric_limits<int>::max();
        double best_distance = -1.0;
        bool found_anchor = false;
        for (int candidate_index = 0; candidate_index < anchor_candidates_per_round; ++candidate_index) {
            Eigen::VectorXd candidate = sample_uniform();
            if (oracle_.point_in_collision(candidate)) {
                context.diagnostics().add_counter("grower.anchor_target_candidate_collision");
                continue;
            }
            const OracleNodeId leaf = find_leaf_containing(oracle_, candidate);
            int max_lca_depth = 0;
            bool lca_available = true;
            for (OracleNodeId reference_leaf : anchor_reference_leaves) {
                if (leaf < 0 || reference_leaf < 0) {
                    lca_available = false;
                    continue;
                }
                const int ancestor_depth = common_ancestor_depth(oracle_, leaf, reference_leaf);
                if (ancestor_depth < 0) {
                    lca_available = false;
                    continue;
                }
                max_lca_depth = std::max(max_lca_depth, ancestor_depth);
            }
            if (!lca_available) {
                context.diagnostics().add_counter("grower.anchor_target_lca_unavailable");
            }
            if (config_.anchor_target_max_lca_depth >= 0 &&
                lca_available &&
                max_lca_depth > config_.anchor_target_max_lca_depth) {
                context.diagnostics().add_counter("grower.anchor_target_lca_rejected");
                continue;
            }
            double min_distance = anchor_reference_points.empty() ? 1.0 : std::numeric_limits<double>::infinity();
            const auto native_root = oracle_.native_root_hull();
            for (const auto& reference : anchor_reference_points) {
                min_distance = std::min(min_distance,
                                        normalized_linf_distance(native_root, reference, candidate));
            }
            if (!found_anchor ||
                max_lca_depth < best_lca_depth ||
                (max_lca_depth == best_lca_depth && min_distance > best_distance)) {
                best_anchor = std::move(candidate);
                best_leaf = leaf;
                best_lca_depth = max_lca_depth;
                best_distance = min_distance;
                found_anchor = true;
            }
        }
        if (!found_anchor) {
            best_anchor = sample_uniform();
            best_leaf = find_leaf_containing(oracle_, best_anchor);
            best_lca_depth = -1;
            best_distance = 0.0;
            context.diagnostics().add_counter("grower.anchor_target_fallback_uniform");
        }
        random_anchor_targets_.push_back(best_anchor);
        anchor_reference_points.push_back(best_anchor);
        anchor_reference_leaves.push_back(best_leaf);
        set_max_diagnostic(context,
                           "grower.anchor_target_lca_depth_max",
                           static_cast<double>(best_lca_depth));
        if (best_lca_depth >= 0) {
            const double previous_min = context.diagnostics().value("grower.anchor_target_lca_depth_min");
            context.diagnostics().set_value(
                "grower.anchor_target_lca_depth_min",
                previous_min == 0.0 && anchor_index == 0
                    ? static_cast<double>(best_lca_depth)
                    : std::min(previous_min, static_cast<double>(best_lca_depth)));
        }
        set_max_diagnostic(context,
                           "grower.anchor_target_min_distance_max",
                           best_distance);
    }
    if (!random_anchor_targets_.empty()) {
        context.diagnostics().set_value("grower.random_anchor_targets",
                                        static_cast<double>(random_anchor_targets_.size()));
    }
    const FindFreeBoxOptions root_ffb_options = config_.find_free_box;
    context.diagnostics().set_value("grower.depth_stage_root_depth", static_cast<double>(root_ffb_options.max_depth));
    int root_id_base = 0;
    if (has_initial_boxes) {
        for (const BoxNode& box : result.boxes) {
            root_id_base = std::max(root_id_base, box.root_id + 1);
        }
    }
    for (int i = 0; i < static_cast<int>(roots.size()) && static_cast<int>(result.boxes.size()) < config_.max_boxes; ++i) {
        if (context.should_stop()) break;
        if (has_initial_boxes && point_covered_by_existing_box(result.boxes, roots[static_cast<std::size_t>(i)])) {
            context.diagnostics().add_counter("grower.initial_root_seed_already_covered");
            continue;
        }
        refresh_depth_stage(static_cast<int>(result.boxes.size()));
        GrowTask root_task;
        root_task.task_id = -1;
        root_task.iteration = i;
        root_task.seed = roots[static_cast<std::size_t>(i)];
        root_task.target = root_task.seed;
        root_task.target_type = GrowTargetType::RootSeed;
        root_task.parent_box_id = -1;
        root_task.root_id = root_id_base + i;
        trace_root_seed(i, root_task.root_id, root_task.seed);
        const int id = create_box(root_task.seed, -1, root_task.root_id, result.boxes, ffb, context, &root_ffb_options, &root_task);
        if (id >= 0) {
            result.n_roots += 1;
            result.n_ffb_success += 1;
        } else {
            result.n_ffb_fail += 1;
        }
    }

    if (config_.frontwave_bootstrap_boxes > 0 && !result.boxes.empty() &&
        static_cast<int>(result.boxes.size()) < config_.max_boxes) {
        ScopedStageTimer bootstrap_timer(context.diagnostics(), "grower.rrt.frontwave_bootstrap");
        FindFreeBoxOptions bootstrap_options = config_.find_free_box;
        if (config_.frontwave_bootstrap_depth > 0) {
            bootstrap_options.max_depth = config_.frontwave_bootstrap_depth;
        }
        const int bootstrap_target = std::min(config_.max_boxes,
                                              std::max(static_cast<int>(result.boxes.size()),
                                                       config_.frontwave_bootstrap_boxes));
        const int samples_per_box = std::max(1, config_.frontwave_bootstrap_boundary_samples);
        std::queue<int> bootstrap_frontier;
        for (const BoxNode& box : result.boxes) {
            bootstrap_frontier.push(box.id);
        }
        std::uniform_real_distribution<double> bootstrap_u01(0.0, 1.0);
        int bootstrap_misses = 0;
        while (!bootstrap_frontier.empty() &&
               static_cast<int>(result.boxes.size()) < bootstrap_target &&
               bootstrap_misses < config_.max_consecutive_miss) {
            if (context.should_stop()) {
                break;
            }
            if (config_.timeout_ms > 0.0 && Clock::now() >= deadline) {
                break;
            }
            const int parent_id = bootstrap_frontier.front();
            bootstrap_frontier.pop();
            auto parent_it = std::find_if(result.boxes.begin(), result.boxes.end(), [&](const BoxNode& candidate) {
                return candidate.id == parent_id;
            });
            if (parent_it == result.boxes.end()) {
                continue;
            }
            const BoxNode parent = *parent_it;
            const auto root_intervals = oracle_.native_root_hull();
            struct BootstrapFace { int dim = -1; int side = 0; };
            std::vector<BootstrapFace> faces;
            faces.reserve(static_cast<std::size_t>(2 * parent.n_dims()));
            for (int dim = 0; dim < parent.n_dims(); ++dim) {
                if (parent.joint_intervals[static_cast<std::size_t>(dim)].lo >
                    root_intervals[static_cast<std::size_t>(dim)].lo + config_.boundary_epsilon) {
                    faces.push_back({dim, 0});
                }
                if (parent.joint_intervals[static_cast<std::size_t>(dim)].hi <
                    root_intervals[static_cast<std::size_t>(dim)].hi - config_.boundary_epsilon) {
                    faces.push_back({dim, 1});
                }
            }
            std::shuffle(faces.begin(), faces.end(), rng_);
            const int n_faces = std::min(samples_per_box, static_cast<int>(faces.size()));
            for (int face_index = 0; face_index < n_faces &&
                 static_cast<int>(result.boxes.size()) < bootstrap_target; ++face_index) {
                const BootstrapFace& face = faces[static_cast<std::size_t>(face_index)];
                Eigen::VectorXd seed(parent.n_dims());
                for (int dim = 0; dim < parent.n_dims(); ++dim) {
                    if (dim == face.dim) {
                        seed[dim] = face.side == 1
                            ? parent.joint_intervals[static_cast<std::size_t>(dim)].hi + config_.boundary_epsilon
                            : parent.joint_intervals[static_cast<std::size_t>(dim)].lo - config_.boundary_epsilon;
                    } else {
                        seed[dim] = parent.joint_intervals[static_cast<std::size_t>(dim)].lo +
                                    bootstrap_u01(rng_) *
                                    parent.joint_intervals[static_cast<std::size_t>(dim)].width();
                    }
                    seed[dim] = std::clamp(seed[dim],
                                           root_intervals[static_cast<std::size_t>(dim)].lo,
                                           root_intervals[static_cast<std::size_t>(dim)].hi);
                }
                if (point_covered_by_existing_box(result.boxes, seed)) {
                    context.diagnostics().add_counter("grower.frontwave_bootstrap_seed_covered");
                    continue;
                }
                GrowTask bootstrap_task;
                bootstrap_task.task_id = -2;
                bootstrap_task.iteration = static_cast<int>(result.boxes.size());
                bootstrap_task.seed = seed;
                bootstrap_task.target = seed;
                bootstrap_task.target_type = GrowTargetType::Unexplored;
                bootstrap_task.parent_box_id = parent.id;
                bootstrap_task.root_id = parent.root_id;
                const int id = create_box(seed,
                                          parent.id,
                                          parent.root_id,
                                          result.boxes,
                                          ffb,
                                          context,
                                          &bootstrap_options,
                                          &bootstrap_task);
                context.diagnostics().add_counter("grower.frontwave_bootstrap_attempts");
                if (id >= 0) {
                    bootstrap_frontier.push(id);
                    result.n_ffb_success += 1;
                    bootstrap_misses = 0;
                    context.diagnostics().add_counter("grower.frontwave_bootstrap_added");
                } else {
                    result.n_ffb_fail += 1;
                    bootstrap_misses += 1;
                    context.diagnostics().add_counter("grower.frontwave_bootstrap_failures");
                }
            }
        }
        context.diagnostics().set_value("grower.frontwave_bootstrap_box_count",
                                        static_cast<double>(result.boxes.size()));
        context.diagnostics().set_value("grower.frontwave_bootstrap_depth",
                                        static_cast<double>(bootstrap_options.max_depth));
    }

    int consecutive_miss = 0;
    int connected_at_count = -1;
    Clock::time_point connected_at_time = t0;
    int next_task_id = 0;
    int loop_iteration = 0;
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    while (static_cast<int>(result.boxes.size()) < config_.max_boxes && consecutive_miss < config_.max_consecutive_miss) {
        if (context.should_stop()) {
            break;
        }
        loop_iteration += 1;
        if (config_.timeout_ms > 0.0 && Clock::now() >= deadline) {
            break;
        }
        if (result.boxes.empty()) {
            break;
        }
        refresh_depth_stage(static_cast<int>(result.boxes.size()));
        if (config_.connect_mode && connected(result.boxes)) {
            if (connected_at_count < 0) {
                connected_at_count = static_cast<int>(result.boxes.size());
                connected_at_time = Clock::now();
                context.diagnostics().set_value("grower.connected_at_box_count", static_cast<double>(connected_at_count));
            }
            const int box_count = static_cast<int>(result.boxes.size());
            const int quality_min_boxes = std::max(0, config_.quality_min_connected_boxes);
            const bool quality_floor_reached = quality_min_boxes == 0 || box_count >= quality_min_boxes;
            if (config_.post_connect_time_budget_ms > 0.0) {
                const double connected_elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - connected_at_time).count();
                context.diagnostics().set_value("grower.post_connect_elapsed_ms", connected_elapsed_ms);
                if (connected_elapsed_ms >= config_.post_connect_time_budget_ms) {
                    context.diagnostics().set_value("grower.stop_post_connect_time_budget", 1.0);
                    break;
                }
            }
            if (quality_floor_reached) {
                context.diagnostics().set_value("grower.quality_min_connected_boxes_reached", 1.0);
            }
            if (config_.stop_after_connect && quality_floor_reached) {
                break;
            }
            if (config_.post_connect_extra_boxes > 0 &&
                box_count >= connected_at_count + config_.post_connect_extra_boxes &&
                quality_floor_reached) {
                break;
            }
        }

        const int batch_size = resolve_task_batch_size(result.boxes, context);
        const RootGroups active_groups = group_boxes_by_root(result.boxes);
        const bool use_task_path = batch_size > 1 ||
                                   (config_.expand_all_roots_per_sample && active_groups.roots.size() > 1);
        if (use_task_path) {
            const int remaining = config_.max_boxes - static_cast<int>(result.boxes.size());
            const int requested_tasks = std::max(1, std::min(batch_size, remaining));
            auto tasks = make_growth_tasks(result.boxes,
                                           roots,
                                           next_task_id,
                                           requested_tasks,
                                           active_ffb_options,
                                           context);
            if (tasks.empty()) {
                result.n_ffb_fail += 1;
                consecutive_miss += 1;
                continue;
            }
            if (static_cast<int>(tasks.size()) > remaining) {
                tasks.resize(static_cast<std::size_t>(remaining));
                context.diagnostics().add_counter("grower.task_truncated_to_remaining");
            }
            next_task_id += static_cast<int>(tasks.size());

            bool batch_success = false;
            int batch_fail = 0;
            auto worker_results = run_worker_ffb_tasks(tasks, active_ffb_options, active_depth_stage_index, context);
            if (!worker_results.empty()) {
                for (auto& worker_result : worker_results) {
                    if (context.should_stop() || static_cast<int>(result.boxes.size()) >= config_.max_boxes) {
                        break;
                    }
                    const int id = worker_result.accepted_by_worker
                        ? commit_box(worker_result.seed,
                                     std::move(worker_result.free_box),
                                     worker_result.parent_box_id,
                                     worker_result.root_id,
                                     result.boxes,
                                     context,
                                     &worker_result)
                        : -1;
                    if (id >= 0) {
                        result.n_ffb_success += 1;
                        batch_success = true;
                        if (worker_result.component_connect_target) {
                            context.diagnostics().add_counter("grower.component_connect_successes");
                            component_parent_failures_.erase(worker_result.parent_box_id);
                            record_component_connect_result(worker_result.source_root_id,
                                                            worker_result.target_root_id,
                                                            true,
                                                            nullptr,
                                                            context);
                            const int chain_added = grow_component_connect_chain(result.boxes,
                                                                                 ffb,
                                                                                 active_ffb_options,
                                                                                 active_depth_stage_index,
                                                                                 worker_result.source_root_id,
                                                                                 context);
                            result.n_ffb_success += chain_added;
                        }
                    } else {
                        if (!worker_result.free_box.found) {
                            worker_result.free_box.node = worker_result.domain_root_node;
                            record_failure_cooling(worker_result.free_box,
                                                   worker_result.domain_root_node,
                                                   worker_result.ffb_depth,
                                                   static_cast<int>(result.boxes.size()),
                                                   context);
                        }
                        result.n_ffb_fail += 1;
                        batch_fail += 1;
                        if (worker_result.component_connect_target) {
                            context.diagnostics().add_counter("grower.component_connect_failures");
                            const int failures = ++component_parent_failures_[worker_result.parent_box_id];
                            set_max_diagnostic(context,
                                               "grower.component_connect_parent_failure_max",
                                               static_cast<double>(failures));
                            record_component_connect_result(worker_result.source_root_id,
                                                            worker_result.target_root_id,
                                                            false,
                                                            &worker_result.free_box,
                                                            context);
                        }
                    }
                }
            } else {
                for (const auto& task : tasks) {
                    if (context.should_stop() || static_cast<int>(result.boxes.size()) >= config_.max_boxes) {
                        break;
                    }
                    FindFreeBoxOptions task_options;
                    const FindFreeBoxOptions* override_options = nullptr;
                    if (task.component_connect_target) {
                        task_options = component_connect_ffb_options(config_,
                                                                    context,
                                                                    active_ffb_options,
                                                                    active_depth_stage_index,
                                                                    task.component_pair_unknown_failures);
                        override_options = &task_options;
                    } else {
                        override_options = &active_ffb_options;
                    }
                    FindFreeBoxResult observed_result;
                    const int id = create_box(task.seed,
                                              task.parent_box_id,
                                              task.root_id,
                                              result.boxes,
                                              ffb,
                                              context,
                                              override_options,
                                              &task,
                                              -1,
                                              &observed_result);
                    if (id >= 0) {
                        result.n_ffb_success += 1;
                        batch_success = true;
                        if (task.component_connect_target) {
                            context.diagnostics().add_counter("grower.component_connect_successes");
                            component_parent_failures_.erase(task.parent_box_id);
                            record_component_connect_result(task.source_root_id,
                                                            task.target_root_id,
                                                            true,
                                                            nullptr,
                                                            context);
                            const int chain_added = grow_component_connect_chain(result.boxes,
                                                                                 ffb,
                                                                                 active_ffb_options,
                                                                                 active_depth_stage_index,
                                                                                 task.source_root_id,
                                                                                 context);
                            result.n_ffb_success += chain_added;
                        }
                    } else {
                        result.n_ffb_fail += 1;
                        batch_fail += 1;
                        if (task.component_connect_target) {
                            context.diagnostics().add_counter("grower.component_connect_failures");
                            const int failures = ++component_parent_failures_[task.parent_box_id];
                            set_max_diagnostic(context,
                                               "grower.component_connect_parent_failure_max",
                                               static_cast<double>(failures));
                            record_component_connect_result(task.source_root_id,
                                                            task.target_root_id,
                                                            false,
                                                            observed_result.found || observed_result.fail_code != 0 ? &observed_result : nullptr,
                                                            context);
                        }
                    }
                }
            }
            consecutive_miss = batch_success ? 0 : consecutive_miss + std::max(1, batch_fail);
            continue;
        }

        Eigen::VectorXd seed;
        Eigen::VectorXd target;
        GrowTargetType target_type = GrowTargetType::Unknown;
        GrowTraceFace selected_face;
        std::vector<GrowTraceFace> face_candidates;
        int parent_box_id = -1;
        int root_id = -1;
        int target_root_id = -1;
        int component_pair_unknown_failures = 0;
        bool component_connect_staged_target = false;
        double component_connect_gap_sq = 0.0;
        bool component_connect_attempt = false;
        if (config_.connect_mode && roots.size() > 1 && u01(rng_) < config_.component_connect_prob) {
            component_connect_attempt = make_component_connect_seed(result.boxes,
                                                                    seed,
                                                                    target,
                                                                    parent_box_id,
                                                                    root_id,
                                                                    target_root_id,
                                                                    component_pair_unknown_failures,
                                                                    component_connect_staged_target,
                                                                    component_connect_gap_sq,
                                                                    context);
            if (component_connect_attempt) {
                context.diagnostics().add_counter("grower.component_connect_attempts");
                context.diagnostics().add_counter("grower.component_connect_target_tasks");
                target_type = GrowTargetType::ComponentConnect;
            }
        }
        if (!component_connect_attempt) {
            if (roots.size() > 1 && u01(rng_) < config_.rrt_goal_bias) {
                std::uniform_int_distribution<int> pick(0, static_cast<int>(roots.size()) - 1);
                target_root_id = pick(rng_);
                target = roots[static_cast<std::size_t>(target_root_id)];
                target_type = GrowTargetType::QueryRoot;
            } else if (u01(rng_) < config_.unexplored_sample_prob) {
                target = sample_unexplored();
                target_type = GrowTargetType::Unexplored;
            } else {
                target = sample_uniform();
                target_type = GrowTargetType::Uniform;
            }

            if (!make_frontier_seed(result.boxes,
                                    target,
                                    seed,
                                    parent_box_id,
                                    root_id,
                                    &context,
                                    &selected_face,
                                    &face_candidates)) {
                result.n_ffb_fail += 1;
                consecutive_miss += 1;
                continue;
            }
        }

        GrowTask trace_task;
        trace_task.task_id = next_task_id++;
        trace_task.iteration = loop_iteration;
        trace_task.seed = seed;
        trace_task.target = target;
        trace_task.target_type = target_type;
        trace_task.parent_box_id = parent_box_id;
        trace_task.root_id = root_id;
        trace_task.source_root_id = component_connect_attempt ? root_id : -1;
        trace_task.target_root_id = target_root_id;
        trace_task.intertree_goal_bias = target_root_id >= 0;
        trace_task.component_connect_target = component_connect_attempt;
        trace_task.component_pair_unknown_failures = component_pair_unknown_failures;
        trace_task.component_connect_staged_target = component_connect_staged_target;
        trace_task.component_connect_gap_sq = component_connect_gap_sq;
        trace_task.selected_face = selected_face;
        trace_task.face_candidates = std::move(face_candidates);
        trace_task.ffb_depth = active_ffb_options.max_depth;
        trace_task_plan(trace_task);

        FindFreeBoxOptions component_options;
        const FindFreeBoxOptions* override_options = nullptr;
        if (component_connect_attempt) {
            component_options = component_connect_ffb_options(config_,
                                                              context,
                                                              active_ffb_options,
                                                              active_depth_stage_index,
                                                              trace_task.component_pair_unknown_failures);
            override_options = &component_options;
        } else {
            override_options = &active_ffb_options;
        }
        FindFreeBoxResult ffb_result_for_pair;
        const int id = create_box(seed, parent_box_id, root_id, result.boxes, ffb, context, override_options, &trace_task, -1, &ffb_result_for_pair);
        if (id >= 0) {
            result.n_ffb_success += 1;
            if (component_connect_attempt) {
                context.diagnostics().add_counter("grower.component_connect_successes");
                component_parent_failures_.erase(parent_box_id);
                record_component_connect_result(trace_task.source_root_id >= 0 ? trace_task.source_root_id : trace_task.root_id,
                                                trace_task.target_root_id,
                                                true,
                                                nullptr,
                                                context);
                const int chain_added = grow_component_connect_chain(result.boxes,
                                                                     ffb,
                                                                     active_ffb_options,
                                                                     active_depth_stage_index,
                                                                     trace_task.source_root_id >= 0 ? trace_task.source_root_id : trace_task.root_id,
                                                                     context);
                result.n_ffb_success += chain_added;
            }
            consecutive_miss = 0;
        } else {
            result.n_ffb_fail += 1;
            if (component_connect_attempt) {
                context.diagnostics().add_counter("grower.component_connect_failures");
                const int failures = ++component_parent_failures_[parent_box_id];
                set_max_diagnostic(context,
                                   "grower.component_connect_parent_failure_max",
                                   static_cast<double>(failures));
                record_component_connect_result(trace_task.source_root_id >= 0 ? trace_task.source_root_id : trace_task.root_id,
                                                trace_task.target_root_id,
                                                false,
                                                ffb_result_for_pair.found || ffb_result_for_pair.fail_code != 0 ? &ffb_result_for_pair : nullptr,
                                                context);
            }
            consecutive_miss += 1;
        }
    }

    finalize_result(result, config_.adjacency_tolerance);
    result.build_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    close_trace();
    return result;
}

int RrtGrower::create_box(const Eigen::VectorXd& seed,
                          int parent_box_id,
                          int root_id,
                          std::vector<BoxNode>& boxes,
                          FindFreeBoxService& ffb,
                          StageContext& context,
                          const FindFreeBoxOptions* override_options,
                          const GrowTask* trace_task,
                          int worker_id,
                          FindFreeBoxResult* observed_result) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.create_box");
    if (parent_box_id >= 0 && point_covered_by_existing_box(boxes, seed)) {
        context.diagnostics().add_counter("grower.seed_already_covered");
        trace_box_rejected("seed_already_covered", seed, parent_box_id, root_id, trace_task, nullptr, worker_id);
        return -1;
    }
    const FindFreeBoxOptions& options = override_options != nullptr ? *override_options : config_.find_free_box;
    OracleNodeId domain_node = kInvalidOracleNodeId;
    if (seed_in_failure_cooling(seed,
                                options.max_depth,
                                static_cast<int>(boxes.size()),
                                context,
                                &domain_node)) {
        trace_box_rejected("failure_cooling", seed, parent_box_id, root_id, trace_task, nullptr, worker_id);
        return -1;
    }
    auto ffb_result = ffb.find(seed,
                               context,
                               options);
    if (observed_result != nullptr) {
        *observed_result = ffb_result;
    }
    if (!ffb_result.found) {
        record_grower_ffb_failure(context, ffb_result);
        trace_ffb_result("ffb_fail", seed, ffb_result, parent_box_id, root_id, trace_task, nullptr, worker_id, options.max_depth);
        record_failure_cooling(ffb_result,
                               domain_node,
                               options.max_depth,
                               static_cast<int>(boxes.size()),
                               context);
        return -1;
    }
    trace_ffb_result("ffb_success", seed, ffb_result, parent_box_id, root_id, trace_task, nullptr, worker_id, options.max_depth);
    if (!intervals_contain_point(ffb_result.intervals, seed, config_.adjacency_tolerance)) {
        context.diagnostics().add_counter("grower.ffb_result_seed_miss");
        set_max_diagnostic(context, "grower.ffb_result_seed_miss_gap_max",
                           intervals_point_gap(ffb_result.intervals, seed));
        trace_box_rejected("ffb_result_seed_miss", seed, parent_box_id, root_id, trace_task, nullptr, worker_id, &ffb_result);
        return -1;
    }
    if (!allow_box_commit(oracle_, ffb_result, config_.commit_policy, context)) {
        trace_box_rejected("commit_policy_rejected", seed, parent_box_id, root_id, trace_task, nullptr, worker_id, &ffb_result);
        return -1;
    }
    const int box_id = next_box_id_++;
    BoxNode box;
    box.id = box_id;
    box.joint_intervals = std::move(ffb_result.intervals);
    box.seed_config = seed;
    box.tree_id = ffb_result.node;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id;
    box.safety_status = ffb_result.validation_detail.safety_status;
    box.strict_audit_required = ffb_result.validation_detail.strict_audit_required;
    box.compute_volume();

    if (parent_box_id >= 0) {
        auto parent_it = std::find_if(boxes.begin(), boxes.end(), [&](const BoxNode& candidate) {
            return candidate.id == parent_box_id;
        });
        if (parent_it != boxes.end() && box_contains_box_exact(*parent_it, box)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            context.diagnostics().add_counter("grower.child_contained_in_parent");
            if (seed_parent_gap > config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", seed_parent_gap);
            }
            context.diagnostics().add_counter("grower.rejected_contained_child");
            trace_box_rejected("contained_child", seed, parent_box_id, root_id, trace_task, nullptr, worker_id, &ffb_result);
            return -1;
        }
        if (parent_it != boxes.end() && !boxes_connected(*parent_it, box, config_.adjacency_tolerance)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            const double gap = std::sqrt(box_gap_squared(*parent_it, box));
            context.diagnostics().add_counter("grower.rejected_disconnected");
            context.diagnostics().add_counter("grower.rejected_disconnected_gap_sum", gap);
            set_max_diagnostic(context, "grower.rejected_disconnected_gap_max", gap);
            if (seed_parent_gap <= config_.boundary_epsilon + config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", gap);
            }
            trace_box_rejected("disconnected_child", seed, parent_box_id, root_id, trace_task, nullptr, worker_id, &ffb_result);
            return -1;
        }
    }
    oracle_.reserve_node(ffb_result.node, box.id);
    record_failure_cooling_success(ffb_result.node, context);
    boxes.push_back(std::move(box));
    record_committed_box_stats(context, boxes.back());
    trace_box_added(boxes.back(), trace_task, nullptr, worker_id);
    return box_id;
}

int RrtGrower::commit_box(const Eigen::VectorXd& seed,
                          FindFreeBoxResult ffb_result,
                          int parent_box_id,
                          int root_id,
                          std::vector<BoxNode>& boxes,
                          StageContext& context,
                          const GrowWorkerResult* trace_result) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.commit_box");
    if (!ffb_result.found) {
        trace_box_rejected("ffb_not_found", seed, parent_box_id, root_id, nullptr, trace_result,
                           trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
        return -1;
    }
    if (parent_box_id >= 0 && point_covered_by_existing_box(boxes, seed)) {
        context.diagnostics().add_counter("grower.seed_already_covered");
        trace_box_rejected("seed_already_covered", seed, parent_box_id, root_id, nullptr, trace_result,
                           trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
        return -1;
    }
    if (!intervals_contain_point(ffb_result.intervals, seed, config_.adjacency_tolerance)) {
        context.diagnostics().add_counter("grower.ffb_result_seed_miss");
        set_max_diagnostic(context, "grower.ffb_result_seed_miss_gap_max",
                           intervals_point_gap(ffb_result.intervals, seed));
        trace_box_rejected("ffb_result_seed_miss", seed, parent_box_id, root_id, nullptr, trace_result,
                           trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
        return -1;
    }
    if (!allow_box_commit(oracle_, ffb_result, config_.commit_policy, context)) {
        trace_box_rejected("commit_policy_rejected", seed, parent_box_id, root_id, nullptr, trace_result,
                           trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
        return -1;
    }
    const int box_id = next_box_id_++;
    BoxNode box;
    box.id = box_id;
    box.joint_intervals = std::move(ffb_result.intervals);
    box.seed_config = seed;
    box.tree_id = ffb_result.node;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id;
    box.safety_status = ffb_result.validation_detail.safety_status;
    box.strict_audit_required = ffb_result.validation_detail.strict_audit_required;
    box.compute_volume();

    if (parent_box_id >= 0) {
        auto parent_it = std::find_if(boxes.begin(), boxes.end(), [&](const BoxNode& candidate) {
            return candidate.id == parent_box_id;
        });
        if (parent_it != boxes.end() && box_contains_box_exact(*parent_it, box)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            context.diagnostics().add_counter("grower.child_contained_in_parent");
            if (seed_parent_gap > config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", seed_parent_gap);
            }
            context.diagnostics().add_counter("grower.rejected_contained_child");
            trace_box_rejected("contained_child", seed, parent_box_id, root_id, nullptr, trace_result,
                               trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
            return -1;
        }
        if (parent_it != boxes.end() && !boxes_connected(*parent_it, box, config_.adjacency_tolerance)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            const double gap = std::sqrt(box_gap_squared(*parent_it, box));
            context.diagnostics().add_counter("grower.rejected_disconnected");
            context.diagnostics().add_counter("grower.rejected_disconnected_gap_sum", gap);
            set_max_diagnostic(context, "grower.rejected_disconnected_gap_max", gap);
            if (seed_parent_gap <= config_.boundary_epsilon + config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", gap);
            }
            trace_box_rejected("disconnected_child", seed, parent_box_id, root_id, nullptr, trace_result,
                               trace_result != nullptr ? trace_result->worker_id : -1, &ffb_result);
            return -1;
        }
    }
    oracle_.reserve_node(ffb_result.node, box.id);
    record_failure_cooling_success(ffb_result.node, context);
    const int id = box.id;
    boxes.push_back(std::move(box));
    record_committed_box_stats(context, boxes.back());
    return id;
}

std::vector<GrowWorkerResult> RrtGrower::run_worker_ffb_tasks(const std::vector<GrowTask>& tasks,
                                                              const FindFreeBoxOptions& base_options,
                                                              int depth_stage_index,
                                                              StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.run_worker_ffb_tasks");
    if (!config_.worker_local_ffb) {
        context.diagnostics().add_counter("grower.worker_ffb_disabled");
        return {};
    }
    if (tasks.empty()) {
        context.diagnostics().add_counter("grower.worker_ffb_empty_tasks");
        return {};
    }
    if (context.executor().n_threads() <= 1) {
        context.diagnostics().add_counter("grower.worker_ffb_inline_executor");
        return {};
    }
    std::vector<std::unique_ptr<BoxOracleSession>> sessions(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        if (tasks[i].domain_root_node < 0) {
            context.diagnostics().add_counter("grower.worker_ffb_missing_domain");
            return {};
        }
        OracleSessionConfig session_config;
        session_config.worker_id = static_cast<int>(i);
        session_config.read_only = false;
        session_config.domain_root = tasks[i].domain_root_node;
        sessions[i] = oracle_.make_session(session_config);
        if (!sessions[i]) {
            context.diagnostics().add_counter("grower.worker_ffb_non_session_oracle");
            return {};
        }
    }
    context.diagnostics().add_counter("grower.worker_ffb_batches");
    context.diagnostics().add_counter("grower.worker_ffb_tasks", static_cast<double>(tasks.size()));

    std::vector<GrowWorkerResult> results(tasks.size());
    context.executor().parallel_for(0, static_cast<int>(tasks.size()), [&](int index) {
        const int worker_id = current_worker_id();
        ScopedStageTimer task_timer(context.diagnostics(), "grower.rrt.worker_ffb_task");
        const auto& task = tasks[static_cast<std::size_t>(index)];
        auto& session = sessions[static_cast<std::size_t>(index)];
        FindFreeBoxService worker_ffb(session->oracle());
        FindFreeBoxOptions task_options = base_options;
        if (task.component_connect_target) {
            task_options = component_connect_ffb_options(config_,
                                                         context,
                                                         base_options,
                                                         depth_stage_index,
                                                         task.component_pair_unknown_failures);
        }
        auto ffb_result = worker_ffb.find(task.seed, context, task_options);
        record_worker_oracle_counters(context, session->oracle().counters());
        if (!ffb_result.found) {
            record_grower_ffb_failure(context, ffb_result);
        }

        GrowWorkerResult worker_result;
        worker_result.task_id = task.task_id;
        worker_result.iteration = task.iteration;
        worker_result.worker_id = worker_id;
        worker_result.accepted_by_worker = ffb_result.found;
        worker_result.seed = task.seed;
        worker_result.target = task.target;
        worker_result.target_type = task.target_type;
        worker_result.free_box = std::move(ffb_result);
        worker_result.parent_box_id = task.parent_box_id;
        worker_result.root_id = task.root_id;
        worker_result.source_root_id = task.source_root_id;
        worker_result.target_root_id = task.target_root_id;
        worker_result.intertree_goal_bias = task.intertree_goal_bias;
        worker_result.component_connect_target = task.component_connect_target;
        worker_result.component_pair_unknown_failures = task.component_pair_unknown_failures;
        worker_result.component_connect_staged_target = task.component_connect_staged_target;
        worker_result.component_connect_gap_sq = task.component_connect_gap_sq;
        worker_result.domain_root_node = task.domain_root_node;
        worker_result.ffb_depth = task_options.max_depth;
        worker_result.selected_face = task.selected_face;
        worker_result.face_candidates = task.face_candidates;
        trace_ffb_result(worker_result.accepted_by_worker ? "ffb_success" : "ffb_fail",
                         worker_result.seed,
                         worker_result.free_box,
                         worker_result.parent_box_id,
                         worker_result.root_id,
                         nullptr,
                         &worker_result,
                         worker_id,
                         task_options.max_depth);
        results[static_cast<std::size_t>(index)] = std::move(worker_result);
    });

    for (std::size_t i = 0; i < results.size(); ++i) {
        if (!results[i].accepted_by_worker) {
            continue;
        }
        if (!sessions[i]->commit()) {
            results[i].accepted_by_worker = false;
            context.diagnostics().add_counter("grower.worker_ffb_commit_failures");
            continue;
        }
        const OracleNodeId master_node = sessions[i]->map_node_to_master(results[i].free_box.node);
        if (master_node < 0) {
            results[i].accepted_by_worker = false;
            context.diagnostics().add_counter("grower.worker_ffb_remap_failures");
            continue;
        }
        results[i].free_box.node = master_node;
        context.diagnostics().add_counter("grower.worker_ffb_commits");
    }
    return results;
}

int RrtGrower::resolve_task_batch_size(const std::vector<BoxNode>& boxes,
                                       StageContext& context) const {
    if (boxes.empty() || context.executor().n_threads() <= 1) {
        return 1;
    }
    const int batch_size = config_.task_batch_size > 1
        ? config_.task_batch_size
        : context.runtime().batch_size;
    if (batch_size <= 1) {
        return 1;
    }
    const int threshold = config_.parallel_threshold > 0
        ? config_.parallel_threshold
        : context.runtime().parallel_threshold;
    if (threshold > 0 && static_cast<int>(boxes.size()) < threshold) {
        return 1;
    }
    return batch_size;
}

std::vector<GrowTask> RrtGrower::make_growth_tasks(const std::vector<BoxNode>& boxes,
                                                   const std::vector<Eigen::VectorXd>& roots,
                                                   int first_task_id,
                                                   int n_tasks,
                                                   const FindFreeBoxOptions& base_options,
                                                   StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.make_growth_tasks");
    if (n_tasks <= 0 || boxes.empty()) {
        return {};
    }

    const RootGroups active_groups = group_boxes_by_root(boxes);
    if (active_groups.roots.empty()) {
        return {};
    }

    struct TaskRequest {
        Eigen::VectorXd target;
        Eigen::VectorXd seed;
        GrowTargetType target_type = GrowTargetType::Unknown;
        int parent_box_id = -1;
        int source_root_id = -1;
        int root_id = -1;
        int target_root_id = -1;
        int iteration = -1;
        bool has_seed = false;
        bool intertree = false;
        bool component_connect = false;
        int component_pair_unknown_failures = 0;
        bool component_connect_staged_target = false;
        double component_connect_gap_sq = 0.0;
        GrowTraceFace selected_face;
        std::vector<GrowTraceFace> face_candidates;
    };

    struct CachedComponentConnectSeed {
        bool resolved = false;
        bool found = false;
        Eigen::VectorXd seed;
        Eigen::VectorXd target;
        int parent_box_id = -1;
        int root_id = -1;
        int target_root_id = -1;
        int pair_unknown_failures = 0;
        bool staged_target = false;
        double component_gap_sq = std::numeric_limits<double>::infinity();
        GrowTraceFace selected_face;
        std::vector<GrowTraceFace> face_candidates;
    };

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<TaskRequest> requests;
    const std::size_t roots_per_sample = config_.expand_all_roots_per_sample
        ? active_groups.roots.size()
        : std::size_t{1};
    requests.reserve(static_cast<std::size_t>(n_tasks) * roots_per_sample);

    auto effective_intertree_goal_bias = [&](int sample_index) {
        double bias = std::clamp(config_.intertree_goal_bias, 0.0, 1.0);
        if (bias <= 0.5) {
            return bias;
        }
        const int period = std::max(1, config_.high_goal_bias_pulse_period);
        const bool pulse = period <= 1 || ((first_task_id + sample_index) % period == 0);
        if (pulse) {
            context.diagnostics().add_counter("grower.goal_bias_high_pulse_tasks");
            return bias;
        }
        context.diagnostics().add_counter("grower.goal_bias_high_capped_tasks");
        return std::min(std::clamp(config_.sustained_goal_bias_cap, 0.0, 0.5), bias);
    };

    auto sample_target = [&](int source_root_id,
                             int sample_index,
                             int& target_root_id,
                             bool& intertree,
                             GrowTargetType& target_type) {
        intertree = false;
        target_root_id = -1;
        if (source_root_id >= 0 && active_groups.roots.size() > 1 &&
            u01(rng_) < effective_intertree_goal_bias(sample_index)) {
            std::vector<int> candidates;
            candidates.reserve(active_groups.roots.size() - 1);
            for (int candidate_root : active_groups.roots) {
                if (candidate_root != source_root_id) {
                    candidates.push_back(candidate_root);
                }
            }
            if (!candidates.empty()) {
                std::uniform_int_distribution<int> pick(0, static_cast<int>(candidates.size()) - 1);
                target_root_id = candidates[static_cast<std::size_t>(pick(rng_))];
                intertree = true;
                target_type = GrowTargetType::IntertreeRoot;
                if (target_root_id >= 0 && target_root_id < static_cast<int>(roots.size())) {
                    return roots[static_cast<std::size_t>(target_root_id)];
                }
                const auto group_it = active_groups.by_root.find(target_root_id);
                if (group_it != active_groups.by_root.end() && !group_it->second.empty()) {
                    return boxes[static_cast<std::size_t>(group_it->second.front())].center();
                }
            }
        }
        if (roots.size() > 1 && u01(rng_) < config_.rrt_goal_bias) {
            std::uniform_int_distribution<int> pick_root_seed(0, static_cast<int>(roots.size()) - 1);
            target_root_id = pick_root_seed(rng_);
            target_type = GrowTargetType::QueryRoot;
            return roots[static_cast<std::size_t>(target_root_id)];
        }
        if (!random_anchor_targets_.empty() &&
            u01(rng_) < std::clamp(config_.anchor_target_prob, 0.0, 1.0)) {
            std::uniform_int_distribution<int> pick_anchor(0, static_cast<int>(random_anchor_targets_.size()) - 1);
            target_type = GrowTargetType::Uniform;
            context.diagnostics().add_counter("grower.target_category.anchor");
            return random_anchor_targets_[static_cast<std::size_t>(pick_anchor(rng_))];
        }
        if (u01(rng_) < config_.unexplored_sample_prob) {
            target_type = GrowTargetType::Unexplored;
            return sample_unexplored();
        }
        target_type = GrowTargetType::Uniform;
        return sample_uniform();
    };

    // One-shot categorical over target categories using explicit configured
    // probabilities. This replaces the legacy sequence of independent
    // Bernoulli gates while keeping each category's probability directly under
    // user control. If the configured mass sums to less than 1, the remainder
    // is assigned to pure-uniform sampling.
    auto choose_target_category = [&](int source_root_id, int sample_index) -> GrowTargetType {
        const bool roots_multi = active_groups.roots.size() > 1;
        const bool can_component = config_.connect_mode && source_root_id >= 0 &&
                                   roots_multi && config_.component_connect_prob > 0.0;
        const bool can_intertree = source_root_id >= 0 && roots_multi;
        const bool can_rrt = roots.size() > 1;
        (void)sample_index;
        double p_cc = can_component ? std::clamp(config_.component_connect_prob, 0.0, 1.0) : 0.0;
        double p_inter = can_intertree ? std::clamp(config_.intertree_goal_bias, 0.0, 1.0) : 0.0;
        double p_rrt = can_rrt ? std::clamp(config_.rrt_goal_bias, 0.0, 1.0) : 0.0;
        double p_unexp = std::clamp(config_.unexplored_sample_prob, 0.0, 1.0);
        double p_uniform = std::clamp(config_.sample_uniform_prob, 0.0, 1.0);
        const double assigned = p_cc + p_inter + p_rrt + p_unexp + p_uniform;
        if (assigned < 1.0) {
            p_uniform += 1.0 - assigned;
        }
        const double r = u01(rng_);
        double acc = p_cc;
        if (r < acc) {
            return GrowTargetType::ComponentConnect;
        }
        acc += p_inter;
        if (r < acc) {
            return GrowTargetType::IntertreeRoot;
        }
        acc += p_rrt;
        if (r < acc) {
            return GrowTargetType::QueryRoot;
        }
        acc += p_unexp;
        if (r < acc) {
            return GrowTargetType::Unexplored;
        }
        return GrowTargetType::Uniform;
    };

    // Build a growth target for an already-selected category (categorical mode).
    // Mirrors the branches of sample_target but is driven by an explicit
    // category rather than a Bernoulli cascade; unavailable directed branches
    // gracefully fall back to a uniform sample.
    auto build_target_for_category = [&](int source_root_id,
                                         GrowTargetType category,
                                         int& target_root_id,
                                         bool& intertree,
                                         GrowTargetType& target_type) -> Eigen::VectorXd {
        intertree = false;
        target_root_id = -1;
        if (category == GrowTargetType::IntertreeRoot && source_root_id >= 0 &&
            active_groups.roots.size() > 1) {
            std::vector<int> candidates;
            candidates.reserve(active_groups.roots.size() - 1);
            for (int candidate_root : active_groups.roots) {
                if (candidate_root != source_root_id) {
                    candidates.push_back(candidate_root);
                }
            }
            if (!candidates.empty()) {
                std::uniform_int_distribution<int> pick(0, static_cast<int>(candidates.size()) - 1);
                target_root_id = candidates[static_cast<std::size_t>(pick(rng_))];
                intertree = true;
                target_type = GrowTargetType::IntertreeRoot;
                if (target_root_id >= 0 && target_root_id < static_cast<int>(roots.size())) {
                    return roots[static_cast<std::size_t>(target_root_id)];
                }
                const auto group_it = active_groups.by_root.find(target_root_id);
                if (group_it != active_groups.by_root.end() && !group_it->second.empty()) {
                    return boxes[static_cast<std::size_t>(group_it->second.front())].center();
                }
            }
        }
        if (category == GrowTargetType::QueryRoot && roots.size() > 1) {
            std::uniform_int_distribution<int> pick_root_seed(0, static_cast<int>(roots.size()) - 1);
            target_root_id = pick_root_seed(rng_);
            target_type = GrowTargetType::QueryRoot;
            return roots[static_cast<std::size_t>(target_root_id)];
        }
        if (category == GrowTargetType::Unexplored) {
            target_type = GrowTargetType::Unexplored;
            return sample_unexplored();
        }
        target_type = GrowTargetType::Uniform;
        return sample_uniform();
    };

    RootComponentGraph component_graph;
    const void* component_graph_ptr = nullptr;
    if (config_.connect_mode && active_groups.roots.size() > 1 && config_.component_connect_prob > 0.0) {
        component_graph = build_root_component_graph(boxes,
                                                     config_.adjacency_tolerance,
                                                     config_.component_connect_island_aware);
        component_graph_ptr = &component_graph;
        context.diagnostics().set_value("grower.component_connect_components",
                                        static_cast<double>(component_graph.components.size()));
        set_max_diagnostic(context,
                           "grower.component_connect_connected_root_pairs_max",
                           static_cast<double>(component_graph.connected_cross_root_pairs));
    }

    const bool use_component_connect_seed_cache =
        config_.expand_all_roots_per_sample &&
        config_.component_connect_candidate_limit <= 1 &&
        config_.connect_mode &&
        active_groups.roots.size() > 1 &&
        config_.component_connect_prob > 0.0;
    std::unordered_map<int, CachedComponentConnectSeed> component_connect_seed_cache;
    if (use_component_connect_seed_cache) {
        component_connect_seed_cache.reserve(active_groups.roots.size());
    }

    auto make_request = [&](int source_root_id, int sample_index, const Eigen::VectorXd* shared_anchor_target = nullptr) {
        TaskRequest request;
        request.source_root_id = source_root_id;
        request.iteration = first_task_id + sample_index;
        if (shared_anchor_target != nullptr) {
            request.target = *shared_anchor_target;
            request.target_type = GrowTargetType::Uniform;
            context.diagnostics().add_counter("grower.target_category.shared_anchor");
            return request;
        }
        GrowTargetType chosen_category = GrowTargetType::Uniform;
        bool want_component_connect = false;
        if (config_.sample_categorical_allocation) {
            chosen_category = choose_target_category(source_root_id, sample_index);
            want_component_connect = (chosen_category == GrowTargetType::ComponentConnect);
        } else {
            want_component_connect = config_.connect_mode && source_root_id >= 0 &&
                                     active_groups.roots.size() > 1 &&
                                     config_.component_connect_prob > 0.0 &&
                                     u01(rng_) < config_.component_connect_prob;
        }
        if (want_component_connect && config_.connect_mode && source_root_id >= 0 &&
            active_groups.roots.size() > 1 && config_.component_connect_prob > 0.0) {
            bool found_component_connect_seed = false;
            if (use_component_connect_seed_cache) {
                auto [cache_it, inserted] = component_connect_seed_cache.try_emplace(source_root_id);
                (void)inserted;
                CachedComponentConnectSeed& cached = cache_it->second;
                if (!cached.resolved) {
                    cached.found = make_component_connect_seed_for_root(boxes,
                                                                        source_root_id,
                                                                        cached.seed,
                                                                        cached.target,
                                                                        cached.parent_box_id,
                                                                        cached.root_id,
                                                                        cached.target_root_id,
                                                                        cached.pair_unknown_failures,
                                                                        cached.staged_target,
                                                                        cached.component_gap_sq,
                                                                        &cached.selected_face,
                                                                        &cached.face_candidates,
                                                                        context,
                                                                        component_graph_ptr);
                    cached.resolved = true;
                    context.diagnostics().add_counter("grower.component_connect_seed_cache_misses");
                } else {
                    context.diagnostics().add_counter("grower.component_connect_seed_cache_hits");
                }
                if (cached.found) {
                    request.seed = cached.seed;
                    request.target = cached.target;
                    request.parent_box_id = cached.parent_box_id;
                    request.root_id = cached.root_id;
                    request.target_root_id = cached.target_root_id;
                    request.component_pair_unknown_failures = cached.pair_unknown_failures;
                    request.component_connect_staged_target = cached.staged_target;
                    request.component_connect_gap_sq = cached.component_gap_sq;
                    request.selected_face = cached.selected_face;
                    request.face_candidates = cached.face_candidates;
                    found_component_connect_seed = true;
                }
            } else {
                found_component_connect_seed = make_component_connect_seed_for_root(boxes,
                                                                                   source_root_id,
                                                                                   request.seed,
                                                                                   request.target,
                                                                                   request.parent_box_id,
                                                                                   request.root_id,
                                                                                   request.target_root_id,
                                                                                   request.component_pair_unknown_failures,
                                                                                   request.component_connect_staged_target,
                                                                                   request.component_connect_gap_sq,
                                                                                   &request.selected_face,
                                                                                   &request.face_candidates,
                                                                                   context,
                                                                                   component_graph_ptr);
            }
            if (found_component_connect_seed) {
                request.has_seed = true;
                request.intertree = true;
                request.component_connect = true;
                request.target_type = GrowTargetType::ComponentConnect;
                context.diagnostics().add_counter("grower.component_connect_attempts");
                context.diagnostics().add_counter("grower.component_connect_target_tasks");
                return request;
            }
            context.diagnostics().add_counter("grower.component_connect_target_no_candidate");
        }
        if (config_.sample_categorical_allocation) {
            request.target = build_target_for_category(source_root_id,
                                                       chosen_category,
                                                       request.target_root_id,
                                                       request.intertree,
                                                       request.target_type);
        } else {
            request.target = sample_target(source_root_id,
                                           sample_index,
                                           request.target_root_id,
                                           request.intertree,
                                           request.target_type);
        }
        return request;
    };

    {
        ScopedStageTimer request_timer(context.diagnostics(), "grower.rrt.make_growth_tasks.request_generation");
        for (int sample_index = 0; sample_index < n_tasks; ++sample_index) {
            const Eigen::VectorXd* shared_anchor_target = nullptr;
            if (config_.expand_all_roots_per_sample &&
                active_groups.roots.size() > 1 &&
                !random_anchor_targets_.empty() &&
                u01(rng_) < std::clamp(config_.anchor_target_prob, 0.0, 1.0)) {
                std::uniform_int_distribution<int> pick_anchor(0, static_cast<int>(random_anchor_targets_.size()) - 1);
                shared_anchor_target = &random_anchor_targets_[static_cast<std::size_t>(pick_anchor(rng_))];
            }
            if (config_.expand_all_roots_per_sample) {
                for (int source_root_id : active_groups.roots) {
                    requests.push_back(make_request(source_root_id, sample_index, shared_anchor_target));
                }
            } else {
                requests.push_back(make_request(-1, sample_index));
            }
        }

        if (config_.anchor_wave_targets_per_batch > 0 &&
            config_.expand_all_roots_per_sample &&
            active_groups.roots.size() > 1 &&
            !random_anchor_targets_.empty()) {
            const int wave_targets = std::min(std::max(0, config_.anchor_wave_targets_per_batch),
                                              static_cast<int>(random_anchor_targets_.size()));
            std::unordered_set<int> selected_anchor_indices;
            selected_anchor_indices.reserve(static_cast<std::size_t>(wave_targets));
            for (int wave_index = 0; wave_index < wave_targets; ++wave_index) {
                int anchor_index = -1;
                if (wave_targets >= static_cast<int>(random_anchor_targets_.size())) {
                    anchor_index = wave_index;
                } else {
                    std::uniform_int_distribution<int> pick_anchor(0, static_cast<int>(random_anchor_targets_.size()) - 1);
                    for (int attempt = 0; attempt < 8; ++attempt) {
                        const int candidate = pick_anchor(rng_);
                        if (selected_anchor_indices.insert(candidate).second) {
                            anchor_index = candidate;
                            break;
                        }
                    }
                    if (anchor_index < 0) {
                        for (int candidate = 0; candidate < static_cast<int>(random_anchor_targets_.size()); ++candidate) {
                            if (selected_anchor_indices.insert(candidate).second) {
                                anchor_index = candidate;
                                break;
                            }
                        }
                    }
                }
                if (anchor_index < 0 || anchor_index >= static_cast<int>(random_anchor_targets_.size())) {
                    continue;
                }
                const Eigen::VectorXd& anchor = random_anchor_targets_[static_cast<std::size_t>(anchor_index)];
                for (int source_root_id : active_groups.roots) {
                    requests.push_back(make_request(source_root_id, n_tasks + wave_index, &anchor));
                    context.diagnostics().add_counter("grower.anchor_wave_root_tasks");
                }
                context.diagnostics().add_counter("grower.anchor_wave_targets");
            }
        }
    }

    context.diagnostics().add_counter("grower.growth_target_samples", static_cast<double>(n_tasks));
    context.diagnostics().add_counter("grower.growth_tasks_planned", static_cast<double>(requests.size()));
    if (config_.expand_all_roots_per_sample && active_groups.roots.size() > 1) {
        context.diagnostics().add_counter("grower.all_root_sample_batches");
        context.diagnostics().add_counter("grower.all_root_sample_root_attempts", static_cast<double>(requests.size()));
    }
    int intertree_requests = 0;
    int component_connect_requests = 0;
    int query_root_requests = 0;
    int unexplored_requests = 0;
    int uniform_requests = 0;
    for (const auto& request : requests) {
        if (request.intertree && !request.component_connect) {
            intertree_requests += 1;
        }
        switch (request.target_type) {
        case GrowTargetType::ComponentConnect:
            component_connect_requests += 1;
            break;
        case GrowTargetType::QueryRoot:
            query_root_requests += 1;
            break;
        case GrowTargetType::Unexplored:
            unexplored_requests += 1;
            break;
        case GrowTargetType::Uniform:
            uniform_requests += 1;
            break;
        default:
            break;
        }
    }
    if (intertree_requests > 0) {
        context.diagnostics().add_counter("grower.intertree_goal_bias_tasks", static_cast<double>(intertree_requests));
    }
    if (component_connect_requests > 0) {
        context.diagnostics().add_counter("grower.target_category.component_connect",
                                          static_cast<double>(component_connect_requests));
    }
    if (query_root_requests > 0) {
        context.diagnostics().add_counter("grower.target_category.query_root",
                                          static_cast<double>(query_root_requests));
    }
    if (unexplored_requests > 0) {
        context.diagnostics().add_counter("grower.target_category.unexplored",
                                          static_cast<double>(unexplored_requests));
    }
    if (uniform_requests > 0) {
        context.diagnostics().add_counter("grower.target_category.uniform",
                                          static_cast<double>(uniform_requests));
    }

    std::vector<GrowTask> tasks(requests.size());
    {
        ScopedStageTimer seed_timer(context.diagnostics(), "grower.rrt.make_growth_tasks.frontier_seed_selection");
        context.executor().parallel_for(0, static_cast<int>(requests.size()), [&](int task_index) {
            if (context.should_stop()) {
                return;
            }
            const auto& request = requests[static_cast<std::size_t>(task_index)];
            Eigen::VectorXd seed;
            int parent_box_id = -1;
            int root_id = -1;
            GrowTraceFace selected_face = request.selected_face;
            std::vector<GrowTraceFace> face_candidates = request.face_candidates;
            bool made_seed = request.has_seed;
            if (request.has_seed) {
                seed = request.seed;
                parent_box_id = request.parent_box_id;
                root_id = request.root_id;
            } else {
                const bool target_seed = request.source_root_id >= 0
                    ? make_frontier_seed_for_root(boxes,
                                                  request.source_root_id,
                                                  request.target,
                                                  seed,
                                                  parent_box_id,
                                                  root_id,
                                                  nullptr,
                                                  &selected_face,
                                                  &face_candidates)
                    : make_frontier_seed(boxes,
                                         request.target,
                                         seed,
                                         parent_box_id,
                                         root_id,
                                         nullptr,
                                         &selected_face,
                                         &face_candidates);
                if (!target_seed) {
                    return;
                }
                made_seed = true;
            }
            if (!made_seed) {
                return;
            }
            GrowTask task;
            task.task_id = first_task_id + task_index;
            task.iteration = request.iteration;
            task.seed = std::move(seed);
            task.target = request.target;
            task.target_type = request.target_type;
            task.parent_box_id = parent_box_id;
            task.root_id = root_id;
            task.source_root_id = request.source_root_id;
            task.target_root_id = request.target_root_id;
            task.intertree_goal_bias = request.intertree;
            task.component_connect_target = request.component_connect;
            task.component_pair_unknown_failures = request.component_pair_unknown_failures;
            task.component_connect_staged_target = request.component_connect_staged_target;
            task.component_connect_gap_sq = request.component_connect_gap_sq;
            task.selected_face = selected_face;
            task.face_candidates = std::move(face_candidates);
            tasks[static_cast<std::size_t>(task_index)] = std::move(task);
        });
    }

    std::vector<GrowTask> out;
    std::unordered_set<OracleNodeId> used_domains;
    const bool require_worker_domain = config_.worker_local_ffb && context.executor().n_threads() > 1;
    int skipped_frontier = 0;
    out.reserve(tasks.size());
    {
        ScopedStageTimer filter_timer(context.diagnostics(), "grower.rrt.make_growth_tasks.filter_tasks");
        for (auto& task : tasks) {
            if (task.task_id < 0 || task.parent_box_id < 0) {
                skipped_frontier += 1;
                continue;
            }
            if (seed_covered_by_frontier_cache(boxes, task.seed, &context)) {
                context.diagnostics().add_counter("grower.seed_already_covered");
                continue;
            }
            const OracleNodeId domain_root = find_leaf_containing(oracle_, task.seed);
            if (node_in_failure_cooling(domain_root,
                                        base_options.max_depth,
                                        static_cast<int>(boxes.size()),
                                        context)) {
                context.diagnostics().add_counter("grower.task_skipped_failure_cooling");
                if (config_.coverage_first_stop_loss) {
                    context.diagnostics().add_counter("grower.hard_frontier_task_skips");
                }
                continue;
            }
            if (domain_root >= 0 && !oracle_.is_reserved(domain_root) &&
                used_domains.find(domain_root) == used_domains.end()) {
                task.domain_root_node = domain_root;
                task.ffb_depth = base_options.max_depth;
                used_domains.insert(domain_root);
            } else if (domain_root >= 0 && oracle_.is_reserved(domain_root)) {
                context.diagnostics().add_counter("grower.task_reserved_domain");
            } else if (domain_root >= 0) {
                context.diagnostics().add_counter("grower.task_duplicate_domain");
            }
            if (require_worker_domain && task.domain_root_node < 0) {
                context.diagnostics().add_counter("grower.task_skipped_no_worker_domain");
                continue;
            }
            trace_task_plan(task);
            out.push_back(std::move(task));
        }
    }
    if (skipped_frontier > 0) {
        context.diagnostics().add_counter("grower.frontier_no_uncovered_seed", skipped_frontier);
        if (config_.coverage_first_stop_loss) {
            context.diagnostics().add_counter("grower.hard_frontier_no_uncovered_seed", skipped_frontier);
        }
    }
    return out;
}

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
    const auto root = oracle_.native_root_hull();
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

Eigen::VectorXd RrtGrower::staged_component_target(const BoxNode& parent,
                                                   const Eigen::Ref<const Eigen::VectorXd>& target,
                                                   bool& staged,
                                                   double& normalized_linf) const {
    staged = false;
    normalized_linf = 0.0;
    if (!config_.component_connect_staged_growth || config_.component_connect_stage_normalized_linf <= 0.0) {
        return target;
    }
    const auto root = oracle_.native_root_hull();
    if (target.size() != static_cast<int>(root.size())) {
        return target;
    }
    const Eigen::VectorXd parent_center = parent.center();
    normalized_linf = normalized_linf_distance(root, parent_center, target);
    const double stage = std::max(1e-6, config_.component_connect_stage_normalized_linf);
    if (normalized_linf <= stage) {
        return target;
    }
    Eigen::VectorXd staged_target = parent_center + (stage / normalized_linf) * (target - parent_center);
    for (int dim = 0; dim < staged_target.size(); ++dim) {
        staged_target[dim] = std::clamp(staged_target[dim],
                                        root[static_cast<std::size_t>(dim)].lo,
                                        root[static_cast<std::size_t>(dim)].hi);
    }
    staged = true;
    return staged_target;
}

int RrtGrower::component_pair_unknown_failures(int source_root_id,
                                               int target_root_id) const {
    if (source_root_id < 0 || target_root_id < 0 || source_root_id == target_root_id) {
        return 0;
    }
    const auto it = component_pair_unknown_failures_.find(component_pair_key(source_root_id, target_root_id));
    return it == component_pair_unknown_failures_.end() ? 0 : it->second;
}

void RrtGrower::record_component_connect_result(int source_root_id,
                                                int target_root_id,
                                                bool success,
                                                const FindFreeBoxResult* ffb_result,
                                                StageContext& context) {
    if (source_root_id < 0 || target_root_id < 0 || source_root_id == target_root_id) {
        return;
    }
    const std::uint64_t key = component_pair_key(source_root_id, target_root_id);
    if (success) {
        component_pair_unknown_failures_.erase(key);
        context.diagnostics().add_counter("grower.component_connect_pair_successes");
        return;
    }
    if (ffb_result != nullptr && ffb_result->hit_unknown_depth_cap) {
        const int failures = ++component_pair_unknown_failures_[key];
        context.diagnostics().add_counter("grower.component_connect_pair_unknown_failures");
        set_max_diagnostic(context,
                           "grower.component_connect_pair_unknown_failures_max",
                           static_cast<double>(failures));
    }
}

int RrtGrower::grow_component_connect_chain(std::vector<BoxNode>& boxes,
                                            FindFreeBoxService& ffb,
                                            const FindFreeBoxOptions& base_options,
                                            int depth_stage_index,
                                            int source_root_id,
                                            StageContext& context) {
    const int max_steps = std::max(0, config_.component_connect_chain_steps);
    if (max_steps <= 0 || source_root_id < 0 || boxes.empty()) {
        return 0;
    }
    const int max_added = config_.component_connect_chain_max_boxes > 0
        ? std::min(max_steps, config_.component_connect_chain_max_boxes)
        : max_steps;
    int added = 0;
    int failures = 0;
    for (int step = 0;
         step < max_steps && added < max_added &&
         static_cast<int>(boxes.size()) < config_.max_boxes &&
         !context.should_stop();
         ++step) {
        if (connected(boxes)) {
            context.diagnostics().add_counter("grower.component_connect_chain_connected_stop");
            break;
        }

        Eigen::VectorXd seed;
        Eigen::VectorXd target;
        int parent_box_id = -1;
        int root_id = -1;
        int target_root_id = -1;
        int pair_unknown_failures = 0;
        bool staged_target = false;
        double component_gap_sq = 0.0;
        GrowTraceFace selected_face;
        std::vector<GrowTraceFace> face_candidates;
        if (!make_component_connect_seed_for_root(boxes,
                                                  source_root_id,
                                                  seed,
                                                  target,
                                                  parent_box_id,
                                                  root_id,
                                                  target_root_id,
                                                  pair_unknown_failures,
                                                  staged_target,
                                                  component_gap_sq,
                                                  &selected_face,
                                                  &face_candidates,
                                                  context,
                                                  nullptr)) {
            context.diagnostics().add_counter("grower.component_connect_chain_no_seed");
            break;
        }

        GrowTask task;
        task.task_id = -1;
        task.iteration = step;
        task.seed = seed;
        task.target = target;
        task.target_type = GrowTargetType::ComponentConnect;
        task.parent_box_id = parent_box_id;
        task.root_id = root_id;
        task.source_root_id = source_root_id;
        task.target_root_id = target_root_id;
        task.intertree_goal_bias = true;
        task.component_connect_target = true;
        task.component_pair_unknown_failures = pair_unknown_failures;
        task.component_connect_staged_target = staged_target;
        task.component_connect_gap_sq = component_gap_sq;
        task.selected_face = selected_face;
        task.face_candidates = std::move(face_candidates);

        FindFreeBoxOptions task_options = component_connect_ffb_options(config_,
                                                                        context,
                                                                        base_options,
                                                                        depth_stage_index,
                                                                        pair_unknown_failures);
        task.ffb_depth = task_options.max_depth;
        FindFreeBoxResult observed_result;
        context.diagnostics().add_counter("grower.component_connect_chain_attempts");
        const int id = create_box(seed,
                                  parent_box_id,
                                  root_id,
                                  boxes,
                                  ffb,
                                  context,
                                  &task_options,
                                  &task,
                                  -1,
                                  &observed_result);
        if (id >= 0) {
            added += 1;
            failures = 0;
            source_root_id = root_id;
            context.diagnostics().add_counter("grower.component_connect_chain_added");
            context.diagnostics().add_counter("grower.component_connect_successes");
            component_parent_failures_.erase(parent_box_id);
            record_component_connect_result(source_root_id,
                                            target_root_id,
                                            true,
                                            nullptr,
                                            context);
            continue;
        }

        failures += 1;
        context.diagnostics().add_counter("grower.component_connect_chain_failures");
        const int parent_failures = ++component_parent_failures_[parent_box_id];
        set_max_diagnostic(context,
                           "grower.component_connect_parent_failure_max",
                           static_cast<double>(parent_failures));
        record_component_connect_result(source_root_id,
                                        target_root_id,
                                        false,
                                        observed_result.found || observed_result.fail_code != 0 ? &observed_result : nullptr,
                                        context);
        if (failures >= 2) {
            context.diagnostics().add_counter("grower.component_connect_chain_failure_stop");
            break;
        }
    }
    if (added > 0) {
        set_max_diagnostic(context,
                           "grower.component_connect_chain_added_max",
                           static_cast<double>(added));
    }
    return added;
}

bool RrtGrower::make_component_connect_seed(const std::vector<BoxNode>& boxes,
                                            Eigen::VectorXd& seed,
                                            Eigen::VectorXd& target,
                                            int& parent_box_id,
                                            int& root_id,
                                            int& target_root_id,
                                            int& pair_unknown_failures,
                                            bool& staged_target,
                                            double& component_gap_sq,
                                            StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.make_component_connect_seed");
    target_root_id = -1;
    pair_unknown_failures = 0;
    staged_target = false;
    component_gap_sq = std::numeric_limits<double>::infinity();
    if (boxes.size() < 2) {
        return false;
    }
    struct Candidate {
        int parent = -1;
        int target = -1;
        int parent_failures = std::numeric_limits<int>::max();
        int pair_unknown_failures = 0;
        bool staged = false;
        double gap_sq = std::numeric_limits<double>::infinity();
        double face_score = std::numeric_limits<double>::infinity();
        double center_sq = std::numeric_limits<double>::infinity();
        Eigen::VectorXd target_point;
    };

    const RootComponentGraph component_graph = build_root_component_graph(
        boxes,
        config_.adjacency_tolerance,
        config_.component_connect_island_aware);
    context.diagnostics().set_value("grower.component_connect_components",
                                    static_cast<double>(component_graph.components.size()));
    set_max_diagnostic(context,
                       "grower.component_connect_connected_root_pairs_max",
                       static_cast<double>(component_graph.connected_cross_root_pairs));
    if (component_graph.components.size() < 2) {
        context.diagnostics().add_counter("grower.component_connect_all_roots_already_connected");
        return false;
    }
    std::vector<Candidate> candidates;

    auto consider_directed = [&](int parent_index,
                                 int target_index,
                                 double gap_sq) {
        const auto& parent_box = boxes[static_cast<std::size_t>(parent_index)];
        const auto& target_box = boxes[static_cast<std::size_t>(target_index)];
        const int failures = [&]() {
            const auto it = component_parent_failures_.find(parent_box.id);
            return it == component_parent_failures_.end() ? 0 : it->second;
        }();
        if (config_.component_connect_max_parent_failures > 0 &&
            failures >= config_.component_connect_max_parent_failures) {
            context.diagnostics().add_counter("grower.component_connect_parent_skipped");
            return;
        }
        Eigen::VectorXd target_point = closest_point_in_box(target_box, parent_box.center());
        if ((target_point - parent_box.center()).cwiseAbs().maxCoeff() <= 1e-12) {
            target_point = target_box.center();
        }
        bool staged = false;
        double staged_distance = 0.0;
        target_point = staged_component_target(parent_box, target_point, staged, staged_distance);
        double face_score = std::numeric_limits<double>::infinity();
        if (!best_uncovered_directed_face_score(boxes, parent_box, target_point, face_score, &context)) {
            context.diagnostics().add_counter("grower.component_connect_parent_closed_frontier");
            return;
        }
        const int pair_failures = component_pair_unknown_failures(parent_box.root_id, target_box.root_id);
        const double center_sq = (parent_box.center() - target_box.center()).squaredNorm();
        if (staged) {
            context.diagnostics().add_counter("grower.component_connect_staged_targets");
            set_max_diagnostic(context,
                               "grower.component_connect_stage_distance_max",
                               staged_distance);
        }
        candidates.push_back({parent_index,
                              target_index,
                              failures,
                              pair_failures,
                              staged,
                              gap_sq,
                              face_score,
                              center_sq,
                              std::move(target_point)});
    };

    for (int lhs_component = 0; lhs_component < static_cast<int>(component_graph.components.size()); ++lhs_component) {
        for (int rhs_component = lhs_component + 1; rhs_component < static_cast<int>(component_graph.components.size()); ++rhs_component) {
            const auto& lhs_group = component_graph.components[static_cast<std::size_t>(lhs_component)];
            const auto& rhs_group = component_graph.components[static_cast<std::size_t>(rhs_component)];
            const double component_gap_sq = interval_bounds_gap_squared(lhs_group.bounds, rhs_group.bounds);
            for (int lhs_index : lhs_group.indices) {
                const auto& lhs = boxes[static_cast<std::size_t>(lhs_index)];
                for (int rhs_index : rhs_group.indices) {
                    const auto& rhs = boxes[static_cast<std::size_t>(rhs_index)];
                    const double gap_sq = box_gap_squared(lhs, rhs);
                    const double ranked_gap_sq = std::min(gap_sq, component_gap_sq);
                    if (lhs_group.indices.size() <= rhs_group.indices.size()) {
                        consider_directed(lhs_index, rhs_index, ranked_gap_sq);
                        consider_directed(rhs_index, lhs_index, ranked_gap_sq);
                    } else {
                        consider_directed(rhs_index, lhs_index, ranked_gap_sq);
                        consider_directed(lhs_index, rhs_index, ranked_gap_sq);
                    }
                }
            }
        }
    }

    if (candidates.empty()) {
        context.diagnostics().add_counter("grower.component_connect_no_candidate");
        return false;
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        if (std::abs(lhs.gap_sq - rhs.gap_sq) > 1e-18) return lhs.gap_sq < rhs.gap_sq;
        if (std::abs(lhs.face_score - rhs.face_score) > 1e-18) return lhs.face_score < rhs.face_score;
        if (lhs.parent_failures != rhs.parent_failures) return lhs.parent_failures < rhs.parent_failures;
        if (lhs.pair_unknown_failures != rhs.pair_unknown_failures) return lhs.pair_unknown_failures > rhs.pair_unknown_failures;
        return lhs.center_sq < rhs.center_sq;
    });
    const int choice_limit = std::max(1, std::min(config_.component_connect_candidate_limit,
                                                 static_cast<int>(candidates.size())));
    int choice_index = 0;
    if (choice_limit > 1) {
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        choice_index = std::min(choice_limit - 1,
                                static_cast<int>(u01(rng_) * u01(rng_) * choice_limit));
        set_max_diagnostic(context,
                           "grower.component_connect_candidate_rank_max",
                           static_cast<double>(choice_index));
    }
    const Candidate& best = candidates[static_cast<std::size_t>(choice_index)];
    const auto& parent = boxes[static_cast<std::size_t>(best.parent)];
    const auto& target_box = boxes[static_cast<std::size_t>(best.target)];
    if (best.parent_failures > 0) {
        context.diagnostics().add_counter("grower.component_connect_retried_parent");
    }
    if (!make_frontier_seed_from_parent(boxes,
                                        best.parent,
                                        best.target_point,
                                        seed,
                                        parent_box_id,
                                        root_id,
                                        true,
                                        nullptr,
                                        nullptr,
                                        &context)) {
        context.diagnostics().add_counter("grower.component_connect_no_frontier_seed");
        const int failures = ++component_parent_failures_[parent.id];
        set_max_diagnostic(context,
                           "grower.component_connect_parent_failure_max",
                           static_cast<double>(failures));
        return false;
    }
    target = best.target_point;
    target_root_id = target_box.root_id;
    pair_unknown_failures = best.pair_unknown_failures;
    staged_target = best.staged;
    component_gap_sq = best.gap_sq;
    return true;
}

bool RrtGrower::make_component_connect_seed_for_root(const std::vector<BoxNode>& boxes,
                                                     int source_root_id,
                                                     Eigen::VectorXd& seed,
                                                     Eigen::VectorXd& target,
                                                     int& parent_box_id,
                                                     int& root_id,
                                                     int& target_root_id,
                                                     int& pair_unknown_failures,
                                                     bool& staged_target,
                                                     double& component_gap_sq,
                                                     GrowTraceFace* face,
                                                     std::vector<GrowTraceFace>* face_candidates,
                                                     StageContext& context,
                                                     const void* component_graph_override) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.rrt.make_component_connect_seed_for_root");
    target_root_id = -1;
    pair_unknown_failures = 0;
    staged_target = false;
    component_gap_sq = std::numeric_limits<double>::infinity();

    RootComponentGraph local_component_graph;
    const auto* component_graph_ptr = static_cast<const RootComponentGraph*>(component_graph_override);
    if (component_graph_ptr == nullptr) {
        local_component_graph = build_root_component_graph(boxes,
                                                           config_.adjacency_tolerance,
                                                           config_.component_connect_island_aware);
        component_graph_ptr = &local_component_graph;
        context.diagnostics().set_value("grower.component_connect_components",
                                        static_cast<double>(component_graph_ptr->components.size()));
        set_max_diagnostic(context,
                           "grower.component_connect_connected_root_pairs_max",
                           static_cast<double>(component_graph_ptr->connected_cross_root_pairs));
    }
    const RootComponentGraph& component_graph = *component_graph_ptr;
    const auto source_it = component_graph.groups.by_root.find(source_root_id);
    const auto source_component_it = component_graph.root_to_component.find(source_root_id);
    if (source_it == component_graph.groups.by_root.end() ||
        source_component_it == component_graph.root_to_component.end() ||
        component_graph.components.size() < 2) {
        context.diagnostics().add_counter("grower.component_connect_no_candidate");
        return false;
    }

    struct TargetSummary {
        int component = -1;
        std::vector<int> roots;
        std::vector<int> indices;
        std::vector<Interval> bounds;
        Eigen::VectorXd center;
        int root_order_gap = std::numeric_limits<int>::max();
        double gap_sq = std::numeric_limits<double>::infinity();
        double center_sq = std::numeric_limits<double>::infinity();
    };
    struct Candidate {
        int parent = -1;
        int target_summary = -1;
        int target_root = -1;
        int parent_failures = std::numeric_limits<int>::max();
        int pair_unknown_failures = 0;
        int root_order_gap = std::numeric_limits<int>::max();
        bool staged = false;
        Eigen::VectorXd target_point;
        double target_gap_sq = std::numeric_limits<double>::infinity();
        double face_score = std::numeric_limits<double>::infinity();
        double center_sq = std::numeric_limits<double>::infinity();
    };

    const int source_component = source_component_it->second;
    const RootComponent& source_summary = component_graph.components[static_cast<std::size_t>(source_component)];
    const auto& source_bounds = source_summary.bounds;
    const Eigen::VectorXd& source_center = source_summary.center;
    const int neighbor_window = std::max(1, config_.component_connect_neighbor_root_window);
    std::vector<TargetSummary> targets;
    targets.reserve(component_graph.components.size() - 1);
    for (const RootComponent& component : component_graph.components) {
        if (component.id == source_component || component.indices.empty()) {
            continue;
        }
        TargetSummary target;
        target.component = component.id;
        target.roots = component.roots;
        target.indices = component.indices;
        target.bounds = component.bounds;
        target.center = component.center;
        for (int candidate_root : target.roots) {
            if (candidate_root == source_root_id) {
                continue;
            }
            target.root_order_gap = std::min(target.root_order_gap, std::abs(candidate_root - source_root_id));
        }
        target.gap_sq = interval_bounds_gap_squared(source_bounds, target.bounds);
        target.center_sq = (source_center - target.center).squaredNorm();
        if (config_.component_connect_neighbor_root_bias && target.root_order_gap <= neighbor_window) {
            context.diagnostics().add_counter("grower.component_connect_neighbor_root_targets");
        }
        targets.push_back(std::move(target));
    }
    if (targets.empty()) {
        context.diagnostics().add_counter("grower.component_connect_no_candidate");
        return false;
    }
    auto target_less = [&](const TargetSummary& lhs, const TargetSummary& rhs) {
        if (config_.component_connect_neighbor_root_bias) {
            const bool lhs_neighbor = lhs.root_order_gap <= neighbor_window;
            const bool rhs_neighbor = rhs.root_order_gap <= neighbor_window;
            if (lhs_neighbor != rhs_neighbor) return lhs_neighbor;
            if (lhs_neighbor && lhs.root_order_gap != rhs.root_order_gap) {
                return lhs.root_order_gap < rhs.root_order_gap;
            }
        }
        if (std::abs(lhs.gap_sq - rhs.gap_sq) > 1e-18) return lhs.gap_sq < rhs.gap_sq;
        return lhs.center_sq < rhs.center_sq;
    };
    std::sort(targets.begin(), targets.end(), target_less);
    const int target_limit = std::max(1, std::min(config_.component_connect_candidate_limit,
                                                 static_cast<int>(targets.size())));
    set_max_diagnostic(context,
                       "grower.component_connect_target_roots_considered_max",
                       static_cast<double>(target_limit));

    std::vector<Candidate> coarse_candidates;
    for (int parent_index : source_it->second) {
        const auto& parent = boxes[static_cast<std::size_t>(parent_index)];
        const auto failure_it = component_parent_failures_.find(parent.id);
        const int failures = failure_it == component_parent_failures_.end() ? 0 : failure_it->second;
        if (config_.component_connect_max_parent_failures > 0 &&
            failures >= config_.component_connect_max_parent_failures) {
            context.diagnostics().add_counter("grower.component_connect_parent_skipped");
            continue;
        }
        const Eigen::VectorXd parent_center = parent.center();
        for (int target_index = 0; target_index < target_limit; ++target_index) {
            const auto& target_summary = targets[static_cast<std::size_t>(target_index)];
            Eigen::VectorXd target_point = closest_point_in_intervals(target_summary.bounds, parent_center);
            if ((target_point - parent_center).cwiseAbs().maxCoeff() <= 1e-12) {
                target_point = target_summary.center;
            }
            bool staged = false;
            double staged_distance = 0.0;
            target_point = staged_component_target(parent, target_point, staged, staged_distance);
            double face_score = std::numeric_limits<double>::infinity();
            if (!best_uncovered_directed_face_score(boxes, parent, target_point, face_score, &context)) {
                context.diagnostics().add_counter("grower.component_connect_parent_closed_frontier");
                continue;
            }
            int pair_failures = 0;
            int target_root_guess = target_summary.roots.empty() ? -1 : target_summary.roots.front();
            for (int candidate_target_root : target_summary.roots) {
                const int candidate_failures = component_pair_unknown_failures(source_root_id, candidate_target_root);
                if (candidate_failures > pair_failures) {
                    pair_failures = candidate_failures;
                    target_root_guess = candidate_target_root;
                }
            }
            coarse_candidates.push_back({parent_index,
                                         target_index,
                                         target_root_guess,
                                         failures,
                                         pair_failures,
                                         target_summary.root_order_gap,
                                         staged,
                                         std::move(target_point),
                                         target_summary.gap_sq,
                                         face_score,
                                         (parent_center - target_summary.center).squaredNorm()});
            if (staged) {
                context.diagnostics().add_counter("grower.component_connect_staged_targets");
                set_max_diagnostic(context,
                                   "grower.component_connect_stage_distance_max",
                                   staged_distance);
            }
        }
    }

    if (coarse_candidates.empty()) {
        context.diagnostics().add_counter("grower.component_connect_no_candidate");
        return false;
    }
    auto candidate_less = [&](const Candidate& lhs, const Candidate& rhs) {
        if (config_.component_connect_neighbor_root_bias) {
            const bool lhs_neighbor = lhs.root_order_gap <= neighbor_window;
            const bool rhs_neighbor = rhs.root_order_gap <= neighbor_window;
            if (lhs_neighbor != rhs_neighbor) return lhs_neighbor;
            if (lhs_neighbor && lhs.root_order_gap != rhs.root_order_gap) {
                return lhs.root_order_gap < rhs.root_order_gap;
            }
        }
        if (std::abs(lhs.target_gap_sq - rhs.target_gap_sq) > 1e-18) return lhs.target_gap_sq < rhs.target_gap_sq;
        if (std::abs(lhs.face_score - rhs.face_score) > 1e-18) return lhs.face_score < rhs.face_score;
        if (lhs.parent_failures != rhs.parent_failures) return lhs.parent_failures < rhs.parent_failures;
        if (lhs.pair_unknown_failures != rhs.pair_unknown_failures) return lhs.pair_unknown_failures > rhs.pair_unknown_failures;
        return lhs.center_sq < rhs.center_sq;
    };
    std::sort(coarse_candidates.begin(), coarse_candidates.end(), candidate_less);

    std::vector<Candidate> candidates;
    if (config_.component_connect_staged_growth) {
        const int coarse_limit = std::min(static_cast<int>(coarse_candidates.size()),
                                         std::max(4, std::max(1, config_.component_connect_candidate_limit) * 2));
        candidates.reserve(static_cast<std::size_t>(coarse_limit));
        for (int coarse_index = 0; coarse_index < coarse_limit; ++coarse_index) {
            candidates.push_back(coarse_candidates[static_cast<std::size_t>(coarse_index)]);
        }
        context.diagnostics().add_counter("grower.component_connect_component_target_tasks");
        set_max_diagnostic(context,
                           "grower.component_connect_component_target_limit_max",
                           static_cast<double>(coarse_limit));
    } else {
        const int refine_target = std::max(8, std::max(1, config_.component_connect_candidate_limit) * 4);
        const int refine_limit = std::min(static_cast<int>(coarse_candidates.size()), refine_target);
        context.diagnostics().add_counter("grower.component_connect_actual_target_refine_calls");
        set_max_diagnostic(context,
                           "grower.component_connect_actual_target_refine_limit_max",
                           static_cast<double>(refine_limit));

        candidates.reserve(static_cast<std::size_t>(refine_limit));
        int target_boxes_scanned = 0;
        for (int coarse_index = 0; coarse_index < refine_limit; ++coarse_index) {
            const Candidate& coarse = coarse_candidates[static_cast<std::size_t>(coarse_index)];
            const auto& parent = boxes[static_cast<std::size_t>(coarse.parent)];
            const Eigen::VectorXd parent_center = parent.center();
            Candidate best = coarse;
            bool found_actual_target = false;
            if (coarse.target_summary >= 0 && coarse.target_summary < static_cast<int>(targets.size())) {
                const auto& target_summary = targets[static_cast<std::size_t>(coarse.target_summary)];
                for (int target_box_index : target_summary.indices) {
                    const auto& target_box = boxes[static_cast<std::size_t>(target_box_index)];
                    Eigen::VectorXd target_point = closest_point_in_box(target_box, parent_center);
                    if ((target_point - parent_center).cwiseAbs().maxCoeff() <= 1e-12) {
                        target_point = target_box.center();
                    }
                    bool staged = false;
                    double staged_distance = 0.0;
                    target_point = staged_component_target(parent, target_point, staged, staged_distance);
                    double face_score = std::numeric_limits<double>::infinity();
                    if (!best_uncovered_directed_face_score(boxes, parent, target_point, face_score, &context)) {
                        continue;
                    }
                    target_boxes_scanned += 1;
                    const double target_gap_sq = box_gap_squared(parent, target_box);
                    const double center_sq = (parent_center - target_box.center()).squaredNorm();
                    if (!found_actual_target ||
                        target_gap_sq < best.target_gap_sq - 1e-18 ||
                        (std::abs(target_gap_sq - best.target_gap_sq) <= 1e-18 &&
                         (face_score < best.face_score - 1e-18 ||
                          (std::abs(face_score - best.face_score) <= 1e-18 && center_sq < best.center_sq)))) {
                        best.target_point = std::move(target_point);
                        best.target_gap_sq = target_gap_sq;
                        best.face_score = face_score;
                        best.center_sq = center_sq;
                        best.target_root = target_box.root_id;
                        best.pair_unknown_failures = component_pair_unknown_failures(parent.root_id, target_box.root_id);
                        best.staged = staged;
                        found_actual_target = true;
                        if (staged) {
                            context.diagnostics().add_counter("grower.component_connect_staged_targets");
                            set_max_diagnostic(context,
                                               "grower.component_connect_stage_distance_max",
                                               staged_distance);
                        }
                    }
                }
            }
            if (found_actual_target) {
                context.diagnostics().add_counter("grower.component_connect_actual_target_refined");
            }
            candidates.push_back(std::move(best));
        }
        set_max_diagnostic(context,
                           "grower.component_connect_actual_target_boxes_scanned_max",
                           static_cast<double>(target_boxes_scanned));
    }
    std::sort(candidates.begin(), candidates.end(), candidate_less);

    const int choice_limit = std::max(1, std::min(config_.component_connect_candidate_limit,
                                                 static_cast<int>(candidates.size())));
    int choice_index = 0;
    if (choice_limit > 1) {
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        choice_index = std::min(choice_limit - 1,
                                static_cast<int>(u01(rng_) * u01(rng_) * choice_limit));
        set_max_diagnostic(context,
                           "grower.component_connect_candidate_rank_max",
                           static_cast<double>(choice_index));
    }
    const Candidate& best = candidates[static_cast<std::size_t>(choice_index)];
    const auto& parent = boxes[static_cast<std::size_t>(best.parent)];
    if (config_.component_connect_neighbor_root_bias && best.root_order_gap <= neighbor_window) {
        context.diagnostics().add_counter("grower.component_connect_neighbor_root_selected");
        set_max_diagnostic(context,
                           "grower.component_connect_neighbor_root_gap_max",
                           static_cast<double>(best.root_order_gap));
    }
    if (best.parent_failures > 0) {
        context.diagnostics().add_counter("grower.component_connect_retried_parent");
    }
    if (best.pair_unknown_failures > 0) {
        context.diagnostics().add_counter("grower.component_connect_retried_unknown_pair");
    }
    GrowTraceFace selected_face;
    GrowTraceFace* selected_face_out = face != nullptr ? face : &selected_face;
    if (!make_frontier_seed_from_parent(boxes,
                                        best.parent,
                                        best.target_point,
                                        seed,
                                        parent_box_id,
                                        root_id,
                                        config_.component_connect_require_target_direction,
                                        selected_face_out,
                                        face_candidates,
                                        &context)) {
        context.diagnostics().add_counter("grower.component_connect_no_frontier_seed");
        const int failures = ++component_parent_failures_[parent.id];
        set_max_diagnostic(context,
                           "grower.component_connect_parent_failure_max",
                           static_cast<double>(failures));
        return false;
    }
    const double lateral_prob = std::clamp(config_.component_connect_lateral_sample_prob, 0.0, 1.0);
    if (lateral_prob > 0.0 && selected_face_out->valid) {
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        if (u01(rng_) < lateral_prob) {
            const auto root = oracle_.native_root_hull();
            const double seed_epsilon = std::max(config_.boundary_epsilon, 0.25 * config_.adjacency_tolerance);
            const int attempts = std::max(1, config_.component_connect_lateral_sample_attempts);
            bool applied = false;
            for (int attempt = 0; attempt < attempts; ++attempt) {
                Eigen::VectorXd candidate_seed = seed;
                for (int dim = 0; dim < parent.n_dims(); ++dim) {
                    if (dim == selected_face_out->dim) {
                        continue;
                    }
                    const double lo = parent.joint_intervals[static_cast<std::size_t>(dim)].lo;
                    const double hi = parent.joint_intervals[static_cast<std::size_t>(dim)].hi;
                    const double safe_lo = lo + seed_epsilon;
                    const double safe_hi = hi - seed_epsilon;
                    if (safe_lo <= safe_hi) {
                        std::uniform_real_distribution<double> coord(safe_lo, safe_hi);
                        candidate_seed[dim] = coord(rng_);
                    } else {
                        candidate_seed[dim] = 0.5 * (lo + hi);
                    }
                    candidate_seed[dim] = std::clamp(candidate_seed[dim],
                                                     root[static_cast<std::size_t>(dim)].lo,
                                                     root[static_cast<std::size_t>(dim)].hi);
                }
                if (!seed_covered_by_frontier_cache(boxes, candidate_seed, &context)) {
                    seed = std::move(candidate_seed);
                    context.diagnostics().add_counter("grower.component_connect_lateral_seed");
                    set_max_diagnostic(context,
                                       "grower.component_connect_lateral_attempt_max",
                                       static_cast<double>(attempt + 1));
                    applied = true;
                    break;
                }
                context.diagnostics().add_counter("grower.component_connect_lateral_seed_covered");
            }
            if (!applied) {
                context.diagnostics().add_counter("grower.component_connect_lateral_seed_fallback_direct");
            }
        }
    }
    target = best.target_point;
    target_root_id = best.target_root;
    pair_unknown_failures = best.pair_unknown_failures;
    staged_target = best.staged;
    component_gap_sq = best.target_gap_sq;
    return true;
}

bool RrtGrower::node_in_failure_cooling(OracleNodeId node,
                                        int active_depth,
                                        int box_count,
                                        StageContext& context) {
    if (!hard_frontier_stop_loss_enabled() || node < 0) {
        return false;
    }
    auto it = failure_cooling_.find(node);
    if (it == failure_cooling_.end()) {
        return false;
    }
    auto& entry = it->second;
    if (entry.cool_until_box_count <= box_count) {
        if (entry.cool_until_box_count > 0) {
            context.diagnostics().add_counter("grower.failure_cooling_expired");
        }
        failure_cooling_.erase(it);
        return false;
    }
    if (config_.failure_cooling_retry_on_depth_raise && active_depth > entry.max_failed_depth) {
        context.diagnostics().add_counter("grower.failure_cooling_retries_after_stage_raise");
        return false;
    }
    context.diagnostics().add_counter("grower.failure_cooling_hits");
    context.diagnostics().add_counter("grower.failure_cooling_skips");
    if (config_.coverage_first_stop_loss) {
        context.diagnostics().add_counter("grower.hard_frontier_stop_loss_hits");
        context.diagnostics().add_counter("grower.hard_frontier_stop_loss_skips");
        set_max_diagnostic(context,
                           "grower.hard_frontier_remaining_horizon_max",
                           static_cast<double>(entry.cool_until_box_count - box_count));
    }
    set_max_diagnostic(context,
                       "grower.failure_cooling_remaining_horizon_max",
                       static_cast<double>(entry.cool_until_box_count - box_count));
    return true;
}

bool RrtGrower::seed_in_failure_cooling(const Eigen::Ref<const Eigen::VectorXd>& seed,
                                        int active_depth,
                                        int box_count,
                                        StageContext& context,
                                        OracleNodeId* domain_node) {
    OracleNodeId node = kInvalidOracleNodeId;
    if (hard_frontier_stop_loss_enabled()) {
        node = find_leaf_containing(oracle_, seed);
    }
    if (domain_node != nullptr) {
        *domain_node = node;
    }
    return node_in_failure_cooling(node, active_depth, box_count, context);
}

void RrtGrower::record_failure_cooling(const FindFreeBoxResult& result,
                                       OracleNodeId fallback_node,
                                       int active_depth,
                                       int box_count,
                                       StageContext& context) {
    if (!hard_frontier_stop_loss_enabled() || active_depth < config_.failure_cooling_min_depth) {
        return;
    }
    const bool eligible = config_.failure_cooling_unknown_only
        ? result.hit_unknown_depth_cap
        : (result.hit_unknown_depth_cap || result.seed_collision || result.fail_code == 3 || result.fail_code == 6);
    if (!eligible) {
        return;
    }
    const OracleNodeId node = result.node >= 0 ? result.node : fallback_node;
    if (node < 0) {
        return;
    }
    auto& entry = failure_cooling_[node];
    entry.fail_count += 1;
    entry.last_failed_box_count = box_count;
    entry.max_failed_depth = std::max(entry.max_failed_depth, active_depth);
    context.diagnostics().add_counter("grower.failure_cooling_recorded_failures");
    if (config_.coverage_first_stop_loss) {
        context.diagnostics().add_counter("grower.hard_frontier_recorded_failures");
        if (result.hit_unknown_depth_cap) {
            context.diagnostics().add_counter("grower.hard_frontier_unknown_depth_cap_failures");
        }
        set_max_diagnostic(context,
                           "grower.hard_frontier_depth_max",
                           static_cast<double>(active_depth));
        set_max_diagnostic(context,
                           "grower.hard_frontier_node_count_max",
                           static_cast<double>(failure_cooling_.size()));
        set_max_diagnostic(context,
                           "grower.hard_frontier_fail_count_max",
                           static_cast<double>(entry.fail_count));
    }
    set_max_diagnostic(context,
                       "grower.failure_cooling_node_count_max",
                       static_cast<double>(failure_cooling_.size()));
    set_max_diagnostic(context,
                       "grower.failure_cooling_fail_count_max",
                       static_cast<double>(entry.fail_count));
    const int threshold = hard_frontier_failure_threshold();
    if (entry.fail_count >= threshold) {
        const int horizon = hard_frontier_box_horizon();
        entry.cool_until_box_count = std::max(entry.cool_until_box_count, box_count + horizon);
        entry.cooled_at_depth = active_depth;
        context.diagnostics().add_counter("grower.failure_cooling_activated");
        if (config_.coverage_first_stop_loss) {
            context.diagnostics().add_counter("grower.hard_frontier_activated");
            set_max_diagnostic(context,
                               "grower.hard_frontier_cool_until_box_count_max",
                               static_cast<double>(entry.cool_until_box_count));
        }
        set_max_diagnostic(context,
                           "grower.failure_cooling_cool_until_box_count_max",
                           static_cast<double>(entry.cool_until_box_count));
    }
}

void RrtGrower::record_failure_cooling_success(OracleNodeId node,
                                               StageContext& context) {
    if (!hard_frontier_stop_loss_enabled() || node < 0) {
        return;
    }
    if (failure_cooling_.erase(node) > 0) {
        context.diagnostics().add_counter("grower.failure_cooling_success_clears");
        if (config_.coverage_first_stop_loss) {
            context.diagnostics().add_counter("grower.hard_frontier_success_clears");
        }
    }
}

bool RrtGrower::hard_frontier_stop_loss_enabled() const {
    return config_.failure_cooling_enabled || config_.coverage_first_stop_loss;
}

int RrtGrower::hard_frontier_failure_threshold() const {
    if (config_.coverage_first_stop_loss) {
        return std::max(1, config_.hard_frontier_failure_threshold);
    }
    return std::max(1, config_.failure_cooling_threshold);
}

int RrtGrower::hard_frontier_box_horizon() const {
    if (config_.coverage_first_stop_loss) {
        return std::max(1, config_.hard_frontier_box_horizon);
    }
    return std::max(1, config_.failure_cooling_box_horizon);
}

Eigen::VectorXd RrtGrower::sample_uniform() {
    const auto root = oracle_.native_root_hull();
    Eigen::VectorXd q(static_cast<int>(root.size()));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int dim = 0; dim < static_cast<int>(root.size()); ++dim) {
        q[dim] = root[static_cast<std::size_t>(dim)].lo +
                 u01(rng_) * root[static_cast<std::size_t>(dim)].width();
    }
    return q;
}

Eigen::VectorXd RrtGrower::sample_unexplored() {
    const OracleNodeId node = oracle_.select_unexplored_node();
    std::vector<Interval> intervals;
    if (node >= 0) {
        auto copies = oracle_.native_interval_copies_for_node(node, oracle_.node_intervals(node));
        if (!copies.empty()) {
            std::uniform_int_distribution<std::size_t> pick(0, copies.size() - 1);
            intervals = std::move(copies[pick(rng_)]);
        }
    }
    if (intervals.empty()) {
        intervals = oracle_.native_root_hull();
    }
    Eigen::VectorXd q(static_cast<int>(intervals.size()));
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int dim = 0; dim < static_cast<int>(intervals.size()); ++dim) {
        q[dim] = intervals[static_cast<std::size_t>(dim)].lo +
                 u01(rng_) * intervals[static_cast<std::size_t>(dim)].width();
    }
    return q;
}

bool RrtGrower::prepare_frontier_seed_with_memory(const std::vector<BoxNode>& boxes,
                                                  const BoxNode& parent,
                                                  const Eigen::VectorXd& target,
                                                  int face_dim,
                                                  int side,
                                                  Eigen::VectorXd& seed,
                                                  StageContext* context) const {
    const auto root = oracle_.native_root_hull();
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
    const auto root = oracle_.native_root_hull();
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
        std::sort(ordered.begin(), ordered.end(), [](const FaceCandidate& lhs, const FaceCandidate& rhs) {
            if (std::abs(lhs.score - rhs.score) > 1e-18) {
                return lhs.score < rhs.score;
            }
            if (lhs.parent_index != rhs.parent_index) {
                return lhs.parent_index < rhs.parent_index;
            }
            if (lhs.dim != rhs.dim) {
                return lhs.dim < rhs.dim;
            }
            return lhs.side < rhs.side;
        });
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
    const auto root = oracle_.native_root_hull();
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
    std::sort(candidates.begin(), candidates.end(), [](const FaceCandidate& lhs, const FaceCandidate& rhs) {
        if (std::abs(lhs.score - rhs.score) > 1e-18) {
            return lhs.score < rhs.score;
        }
        if (lhs.dim != rhs.dim) {
            return lhs.dim < rhs.dim;
        }
        return lhs.side < rhs.side;
    });

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

bool RrtGrower::connected(const std::vector<BoxNode>& boxes) const {
    if (boxes.size() <= 1) {
        return true;
    }
    return find_islands(compute_adjacency(boxes, config_.adjacency_tolerance)).size() <= 1;
}

FrontwaveGrower::FrontwaveGrower(BoxOracle& oracle, GrowerConfig config)
    : oracle_(oracle), config_(std::move(config)), rng_(config_.rng_seed) {}

GrowerResult FrontwaveGrower::grow(const std::vector<Eigen::VectorXd>& seeds) {
    RuntimeConfig runtime;
    runtime.mode = config_.n_threads > 1 ? ExecutionMode::Parallel : ExecutionMode::Inline;
    runtime.n_threads = config_.n_threads;
    runtime.batch_size = config_.task_batch_size;
    runtime.parallel_threshold = config_.parallel_threshold;
    runtime.deterministic_reduce = config_.deterministic_reduce;
    StageContext context(runtime, Deadline::after_ms(config_.timeout_ms));
    return grow(seeds, context);
}

GrowerResult FrontwaveGrower::grow(const std::vector<Eigen::VectorXd>& seeds,
                                   StageContext& context) {
    using Clock = std::chrono::steady_clock;
    ScopedStageTimer function_timer(context.diagnostics(), "grower.frontwave.grow");
    const auto t0 = Clock::now();
    const auto deadline = t0 + std::chrono::duration_cast<Clock::duration>(
                                 std::chrono::duration<double, std::milli>(config_.timeout_ms));
    GrowerResult result;
    FindFreeBoxService ffb(oracle_);
    next_box_id_ = 0;
    if (config_.boundary_epsilon > config_.adjacency_tolerance) {
        context.diagnostics().set_value("grower.invalid_boundary_epsilon", 1.0);
    }
    std::queue<int> frontier;

    for (int i = 0; i < static_cast<int>(seeds.size()) && static_cast<int>(result.boxes.size()) < config_.max_boxes; ++i) {
        if (context.should_stop()) break;
        const int id = create_box(seeds[static_cast<std::size_t>(i)], -1, i, result.boxes, ffb, context);
        if (id >= 0) {
            frontier.push(id);
            result.n_roots += 1;
            result.n_ffb_success += 1;
        } else {
            result.n_ffb_fail += 1;
        }
    }

    std::vector<GrowerConfig::FrontwaveStage> stages = config_.frontwave_stages;
    if (stages.empty()) {
        stages.push_back({config_.max_boxes});
    }
    int stage_index = 0;
    int consecutive_miss = 0;
    while (!frontier.empty() && static_cast<int>(result.boxes.size()) < config_.max_boxes &&
           consecutive_miss < config_.max_consecutive_miss) {
        if (context.should_stop()) {
            break;
        }
        if (config_.timeout_ms > 0.0 && Clock::now() >= deadline) {
            break;
        }
        while (stage_index + 1 < static_cast<int>(stages.size()) &&
               static_cast<int>(result.boxes.size()) >= stages[static_cast<std::size_t>(stage_index)].box_limit) {
            stage_index += 1;
        }
        if (stages[static_cast<std::size_t>(stage_index)].box_limit > 0 &&
            static_cast<int>(result.boxes.size()) >= stages[static_cast<std::size_t>(stage_index)].box_limit &&
            stage_index + 1 >= static_cast<int>(stages.size())) {
            break;
        }

        const int current_id = frontier.front();
        frontier.pop();
        auto it = std::find_if(result.boxes.begin(), result.boxes.end(), [&](const BoxNode& box) {
            return box.id == current_id;
        });
        if (it == result.boxes.end()) {
            continue;
        }
        const Eigen::VectorXd* bias = seeds.empty() ? nullptr : &seeds.back();
        const auto boundaries = boundary_seeds(*it, bias);
        bool handled_batch = false;
        if (context.executor().n_threads() > 1 && boundaries.size() > 1) {
            std::vector<GrowTask> tasks;
            std::unordered_set<OracleNodeId> used_domains;
            tasks.reserve(boundaries.size());
            for (int task_index = 0; task_index < static_cast<int>(boundaries.size()); ++task_index) {
                const auto& boundary = boundaries[static_cast<std::size_t>(task_index)];
                GrowTask task;
                task.task_id = task_index;
                task.seed = boundary.q;
                task.parent_box_id = boundary.parent_box_id;
                task.root_id = boundary.root_id;
                if (point_covered_by_existing_box(result.boxes, task.seed)) {
                    context.diagnostics().add_counter("grower.seed_already_covered");
                    continue;
                }
                const OracleNodeId domain_root = find_leaf_containing(oracle_, task.seed);
                if (domain_root >= 0 && !oracle_.is_reserved(domain_root) &&
                    used_domains.find(domain_root) == used_domains.end()) {
                    task.domain_root_node = domain_root;
                    used_domains.insert(domain_root);
                }
                tasks.push_back(std::move(task));
            }
            auto worker_results = run_worker_ffb_tasks(tasks, context);
            if (!worker_results.empty()) {
                handled_batch = true;
                for (auto& worker_result : worker_results) {
                    if (static_cast<int>(result.boxes.size()) >= config_.max_boxes) {
                        break;
                    }
                    if (context.should_stop()) break;
                    const int id = worker_result.accepted_by_worker
                        ? commit_box(worker_result.seed,
                                     std::move(worker_result.free_box),
                                     worker_result.parent_box_id,
                                     worker_result.root_id,
                                     result.boxes,
                                     context)
                        : -1;
                    if (id >= 0) {
                        frontier.push(id);
                        result.n_ffb_success += 1;
                        consecutive_miss = 0;
                    } else {
                        result.n_ffb_fail += 1;
                        consecutive_miss += 1;
                    }
                }
            }
        }
        if (handled_batch) {
            continue;
        }
        for (const auto& boundary : boundaries) {
            if (static_cast<int>(result.boxes.size()) >= config_.max_boxes) {
                break;
            }
            if (context.should_stop()) break;
            const int id = create_box(boundary.q, boundary.parent_box_id, boundary.root_id, result.boxes, ffb, context);
            if (id >= 0) {
                frontier.push(id);
                result.n_ffb_success += 1;
                consecutive_miss = 0;
            } else {
                result.n_ffb_fail += 1;
                consecutive_miss += 1;
            }
        }
    }

    finalize_result(result, config_.adjacency_tolerance);
    result.build_time_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    return result;
}

int FrontwaveGrower::create_box(const Eigen::VectorXd& seed,
                                int parent_box_id,
                                int root_id,
                                std::vector<BoxNode>& boxes,
                                FindFreeBoxService& ffb,
                                StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.frontwave.create_box");
    if (parent_box_id >= 0 && point_covered_by_existing_box(boxes, seed)) {
        context.diagnostics().add_counter("grower.seed_already_covered");
        return -1;
    }
    auto ffb_result = ffb.find(seed, context, config_.find_free_box);
    if (!ffb_result.found) {
        return -1;
    }
    if (!intervals_contain_point(ffb_result.intervals, seed, config_.adjacency_tolerance)) {
        context.diagnostics().add_counter("grower.ffb_result_seed_miss");
        set_max_diagnostic(context, "grower.ffb_result_seed_miss_gap_max",
                           intervals_point_gap(ffb_result.intervals, seed));
        return -1;
    }
    if (!allow_box_commit(oracle_, ffb_result, config_.commit_policy, context)) {
        return -1;
    }
    BoxNode box;
    box.id = next_box_id_++;
    box.joint_intervals = std::move(ffb_result.intervals);
    box.seed_config = seed;
    box.tree_id = ffb_result.node;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id;
    box.safety_status = ffb_result.validation_detail.safety_status;
    box.strict_audit_required = ffb_result.validation_detail.strict_audit_required;
    box.compute_volume();
    if (parent_box_id >= 0) {
        auto parent_it = std::find_if(boxes.begin(), boxes.end(), [&](const BoxNode& candidate) {
            return candidate.id == parent_box_id;
        });
        if (parent_it != boxes.end() && box_contains_box_exact(*parent_it, box)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            context.diagnostics().add_counter("grower.child_contained_in_parent");
            if (seed_parent_gap > config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", seed_parent_gap);
            }
            context.diagnostics().add_counter("grower.rejected_contained_child");
            return -1;
        }
        if (parent_it != boxes.end() && !boxes_connected(*parent_it, box, config_.adjacency_tolerance)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            const double gap = std::sqrt(box_gap_squared(*parent_it, box));
            context.diagnostics().add_counter("grower.rejected_disconnected");
            context.diagnostics().add_counter("grower.rejected_disconnected_gap_sum", gap);
            set_max_diagnostic(context, "grower.rejected_disconnected_gap_max", gap);
            if (seed_parent_gap <= config_.boundary_epsilon + config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", gap);
            }
            return -1;
        }
    }
    oracle_.reserve_node(ffb_result.node, box.id);
    const int id = box.id;
    boxes.push_back(std::move(box));
    record_committed_box_stats(context, boxes.back());
    return id;
}

int FrontwaveGrower::commit_box(const Eigen::VectorXd& seed,
                                FindFreeBoxResult ffb_result,
                                int parent_box_id,
                                int root_id,
                                std::vector<BoxNode>& boxes,
                                StageContext& context) {
    ScopedStageTimer function_timer(context.diagnostics(), "grower.frontwave.commit_box");
    if (!ffb_result.found) {
        return -1;
    }
    if (parent_box_id >= 0 && point_covered_by_existing_box(boxes, seed)) {
        context.diagnostics().add_counter("grower.seed_already_covered");
        return -1;
    }
    if (!intervals_contain_point(ffb_result.intervals, seed, config_.adjacency_tolerance)) {
        context.diagnostics().add_counter("grower.ffb_result_seed_miss");
        set_max_diagnostic(context, "grower.ffb_result_seed_miss_gap_max",
                           intervals_point_gap(ffb_result.intervals, seed));
        return -1;
    }
    if (!allow_box_commit(oracle_, ffb_result, config_.commit_policy, context)) {
        return -1;
    }
    BoxNode box;
    box.id = next_box_id_++;
    box.joint_intervals = std::move(ffb_result.intervals);
    box.seed_config = seed;
    box.tree_id = ffb_result.node;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id;
    box.safety_status = ffb_result.validation_detail.safety_status;
    box.strict_audit_required = ffb_result.validation_detail.strict_audit_required;
    box.compute_volume();
    if (parent_box_id >= 0) {
        auto parent_it = std::find_if(boxes.begin(), boxes.end(), [&](const BoxNode& candidate) {
            return candidate.id == parent_box_id;
        });
        if (parent_it != boxes.end() && box_contains_box_exact(*parent_it, box)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            context.diagnostics().add_counter("grower.child_contained_in_parent");
            if (seed_parent_gap > config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", seed_parent_gap);
            }
            context.diagnostics().add_counter("grower.rejected_contained_child");
            return -1;
        }
        if (parent_it != boxes.end() && !boxes_connected(*parent_it, box, config_.adjacency_tolerance)) {
            const double seed_parent_gap = box_point_gap(*parent_it, seed);
            const double gap = std::sqrt(box_gap_squared(*parent_it, box));
            context.diagnostics().add_counter("grower.rejected_disconnected");
            context.diagnostics().add_counter("grower.rejected_disconnected_gap_sum", gap);
            set_max_diagnostic(context, "grower.rejected_disconnected_gap_max", gap);
            if (seed_parent_gap <= config_.boundary_epsilon + config_.adjacency_tolerance) {
                context.diagnostics().add_counter("grower.connected_invariant_violation");
                set_max_diagnostic(context, "grower.connected_invariant_gap_max", gap);
            }
            return -1;
        }
    }
    oracle_.reserve_node(ffb_result.node, box.id);
    const int id = box.id;
    boxes.push_back(std::move(box));
    record_committed_box_stats(context, boxes.back());
    return id;
}

std::vector<GrowWorkerResult> FrontwaveGrower::run_worker_ffb_tasks(const std::vector<GrowTask>& tasks,
                                                                    StageContext& context) {
    if (!config_.worker_local_ffb || tasks.empty() || context.executor().n_threads() <= 1) {
        return {};
    }
    std::vector<std::unique_ptr<BoxOracleSession>> sessions(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        if (tasks[i].domain_root_node < 0) {
            return {};
        }
        OracleSessionConfig session_config;
        session_config.worker_id = static_cast<int>(i);
        session_config.read_only = false;
        session_config.domain_root = tasks[i].domain_root_node;
        sessions[i] = oracle_.make_session(session_config);
        if (!sessions[i]) {
            return {};
        }
    }
    context.diagnostics().add_counter("frontwave.worker_ffb_batches");
    context.diagnostics().add_counter("frontwave.worker_ffb_tasks", static_cast<double>(tasks.size()));

    std::vector<GrowWorkerResult> results(tasks.size());
    context.executor().parallel_for(0, static_cast<int>(tasks.size()), [&](int index) {
        const auto& task = tasks[static_cast<std::size_t>(index)];
        auto& session = sessions[static_cast<std::size_t>(index)];
        FindFreeBoxService worker_ffb(session->oracle());
        auto ffb_result = worker_ffb.find(task.seed, context, config_.find_free_box);

        GrowWorkerResult worker_result;
        worker_result.task_id = task.task_id;
        worker_result.accepted_by_worker = ffb_result.found;
        worker_result.seed = task.seed;
        worker_result.free_box = std::move(ffb_result);
        worker_result.parent_box_id = task.parent_box_id;
        worker_result.root_id = task.root_id;
        results[static_cast<std::size_t>(index)] = std::move(worker_result);
    });

    for (std::size_t i = 0; i < results.size(); ++i) {
        if (!results[i].accepted_by_worker) {
            continue;
        }
        if (!sessions[i]->commit()) {
            results[i].accepted_by_worker = false;
            context.diagnostics().add_counter("frontwave.worker_ffb_commit_failures");
            continue;
        }
        const OracleNodeId master_node = sessions[i]->map_node_to_master(results[i].free_box.node);
        if (master_node < 0) {
            results[i].accepted_by_worker = false;
            context.diagnostics().add_counter("frontwave.worker_ffb_remap_failures");
            continue;
        }
        results[i].free_box.node = master_node;
        context.diagnostics().add_counter("frontwave.worker_ffb_commits");
    }
    return results;
}

std::vector<FrontwaveGrower::BoundarySeed> FrontwaveGrower::boundary_seeds(const BoxNode& box,
                                                                            const Eigen::VectorXd* bias_target) {
    std::vector<BoundarySeed> seeds;
    const auto root = oracle_.native_root_hull();
    const int nd = box.n_dims();
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    struct Face { int dim; int side; double priority; };
    std::vector<Face> faces;
    for (int dim = 0; dim < nd; ++dim) {
        if (box.joint_intervals[dim].lo > root[static_cast<std::size_t>(dim)].lo + config_.boundary_epsilon) {
            faces.push_back({dim, 0, 0.0});
        }
        if (box.joint_intervals[dim].hi < root[static_cast<std::size_t>(dim)].hi - config_.boundary_epsilon) {
            faces.push_back({dim, 1, 0.0});
        }
    }
    if (bias_target != nullptr) {
        const Eigen::VectorXd center = box.center();
        for (auto& face : faces) {
            face.priority = (face.side == 1 ? 1.0 : -1.0) * ((*bias_target)[face.dim] - center[face.dim]);
        }
        std::sort(faces.begin(), faces.end(), [](const Face& lhs, const Face& rhs) {
            return lhs.priority > rhs.priority;
        });
    } else {
        std::shuffle(faces.begin(), faces.end(), rng_);
    }
    const int n_faces = std::min(config_.n_boundary_samples, static_cast<int>(faces.size()));
    for (int i = 0; i < n_faces; ++i) {
        const Face& face = faces[static_cast<std::size_t>(i)];
        Eigen::VectorXd q(nd);
        for (int dim = 0; dim < nd; ++dim) {
            if (dim == face.dim) {
                q[dim] = face.side == 1
                    ? box.joint_intervals[dim].hi + config_.boundary_epsilon
                    : box.joint_intervals[dim].lo - config_.boundary_epsilon;
            } else {
                q[dim] = box.joint_intervals[dim].lo +
                         u01(rng_) * box.joint_intervals[dim].width();
            }
            q[dim] = std::clamp(q[dim], root[static_cast<std::size_t>(dim)].lo, root[static_cast<std::size_t>(dim)].hi);
        }
        seeds.push_back({q, box.id, box.root_id});
    }
    return seeds;
}

std::unique_ptr<IGrower> make_grower(BoxOracle& oracle, const GrowerConfig& config) {
    if (config.mode == GrowerConfig::Mode::Frontwave) {
        return std::make_unique<FrontwaveGrower>(oracle, config);
    }
    return std::make_unique<RrtGrower>(oracle, config);
}

}  // namespace rbf
