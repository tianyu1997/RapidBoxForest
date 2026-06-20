#include "grower_frontier_helpers.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace rbf::grower_frontier {

bool WorseFaceCandidateFirst::operator()(const FaceCandidate& lhs, const FaceCandidate& rhs) const {
    return face_candidate_less(lhs, rhs);
}

bool face_candidate_less(const FaceCandidate& lhs, const FaceCandidate& rhs) {
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

std::string frontier_seed_cache_key(const Eigen::Ref<const Eigen::VectorXd>& seed) {
    std::ostringstream out;
    out << seed.size() << ':' << std::setprecision(17);
    for (int dim = 0; dim < seed.size(); ++dim) {
        out << seed[dim] << ',';
    }
    return out.str();
}

}  // namespace rbf::grower_frontier
