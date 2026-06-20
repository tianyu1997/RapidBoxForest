#pragma once

#include "planning_forest_obb.h"
#include "planning_forest_obb_candidate.h"

#include <Eigen/Core>

#include <vector>

namespace rbf {

bool obb_generators_within_domain(const Eigen::VectorXd& center,
                                  const Eigen::MatrixXd& generators,
                                  const std::vector<Interval>& domain,
                                  double tol);

Eigen::VectorXd obb_domain_reference(const std::vector<Interval>& domain);

Eigen::VectorXd obb_joint_scales(const std::vector<Interval>& domain);

Eigen::VectorXd obb_to_scaled(const Eigen::VectorXd& q,
                              const Eigen::VectorXd& ref,
                              const Eigen::VectorXd& scales);

bool obb_orthonormalize_columns(Eigen::MatrixXd& matrix);

std::vector<int> obb_low_risk_joint_order(const Robot& robot, int dims);

bool obb_make_candidate_from_scaled(const Eigen::VectorXd& center_y,
                                    const Eigen::MatrixXd& basis_y,
                                    const Eigen::VectorXd& radii_y,
                                    const std::vector<Interval>& domain,
                                    ObbPortalCandidate& candidate);

bool obb_fit_scaled_path_with_basis(const std::vector<Eigen::VectorXd>& path,
                                    const std::vector<Interval>& domain,
                                    const Eigen::MatrixXd& basis_y,
                                    double lateral_radius,
                                    double longitudinal_margin,
                                    ObbPortalCandidate& candidate,
                                    ObbPortalValidationStats& stats);

std::vector<Eigen::MatrixXd> obb_orientation_candidates(const Robot& robot,
                                                        const std::vector<Eigen::VectorXd>& path,
                                                        const std::vector<Interval>& domain,
                                                        ObbPortalValidationStats& stats,
                                                        bool primary_only = false);

}  // namespace rbf
