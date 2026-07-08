#pragma once

#include <SBF/grower.h>

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace rbf::grower_frontier {

struct FaceCandidate {
    int parent_index = -1;
    int dim = -1;
    int side = 0;
    double score = std::numeric_limits<double>::infinity();
};

struct WorseFaceCandidateFirst {
    bool operator()(const FaceCandidate& lhs, const FaceCandidate& rhs) const;
};

bool face_candidate_less(const FaceCandidate& lhs, const FaceCandidate& rhs);

bool can_step_outside_face(const BoxNode& box,
                           const std::vector<Interval>& root,
                           int dim,
                           int side,
                           double epsilon);

double face_seed_score(const BoxNode& box,
                       const std::vector<Interval>& root,
                       const Eigen::Ref<const Eigen::VectorXd>& target,
                       int face_dim,
                       int side,
                       double epsilon);

Eigen::VectorXd make_face_seed(const BoxNode& box,
                               const std::vector<Interval>& root,
                               const Eigen::Ref<const Eigen::VectorXd>& target,
                               int face_dim,
                               int side,
                               double epsilon);

std::uint64_t frontier_face_memory_key(int parent_box_id, int face_dim, int side);

std::uint64_t frontier_face_total_bins(int nd, int face_dim, int bins_per_dim);

std::uint64_t frontier_face_bin_for_seed(const BoxNode& box,
                                         const Eigen::Ref<const Eigen::VectorXd>& seed,
                                         int face_dim,
                                         int bins_per_dim);

Eigen::VectorXd make_face_seed_for_bin(const BoxNode& box,
                                       const std::vector<Interval>& root,
                                       int face_dim,
                                       int side,
                                       double epsilon,
                                       int bins_per_dim,
                                       std::uint64_t bin_code);

int frontier_face_attempt_budget(const GrowerConfig& config,
                                 const BoxNode& box,
                                 const std::vector<Interval>& root,
                                 int face_dim);

GrowTraceFace make_trace_face(const std::vector<BoxNode>& boxes,
                              const FaceCandidate& candidate,
                              int scanned_boxes,
                              int scanned_faces,
                              int rank,
                              bool selected,
                              bool seed_covered);

std::string frontier_seed_cache_key(const Eigen::Ref<const Eigen::VectorXd>& seed);

}  // namespace rbf::grower_frontier
