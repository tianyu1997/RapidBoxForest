#include "planning_forest_obb_geometry.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rbf {

bool obb_generators_within_domain(const Eigen::VectorXd& center,
                                  const Eigen::MatrixXd& generators,
                                  const std::vector<Interval>& domain,
                                  double tol) {
    const int dims = static_cast<int>(center.size());
    if (static_cast<int>(domain.size()) != dims || generators.rows() != dims) {
        return false;
    }
    for (int dim = 0; dim < dims; ++dim) {
        double radius = 0.0;
        for (int var = 0; var < generators.cols(); ++var) {
            radius += std::abs(generators(dim, var));
        }
        if (center[dim] - radius < domain[static_cast<std::size_t>(dim)].lo - tol ||
            center[dim] + radius > domain[static_cast<std::size_t>(dim)].hi + tol) {
            return false;
        }
    }
    return true;
}

Eigen::VectorXd obb_domain_reference(const std::vector<Interval>& domain) {
    Eigen::VectorXd ref(static_cast<int>(domain.size()));
    for (int dim = 0; dim < ref.size(); ++dim) {
        ref[dim] = domain[static_cast<std::size_t>(dim)].center();
    }
    return ref;
}

Eigen::VectorXd obb_joint_scales(const std::vector<Interval>& domain) {
    Eigen::VectorXd scales(static_cast<int>(domain.size()));
    for (int dim = 0; dim < scales.size(); ++dim) {
        const double width = domain[static_cast<std::size_t>(dim)].width();
        scales[dim] = width > 1e-12 ? 1.0 / width : 1.0;
    }
    return scales;
}

Eigen::VectorXd obb_to_scaled(const Eigen::VectorXd& q,
                              const Eigen::VectorXd& ref,
                              const Eigen::VectorXd& scales) {
    return (q - ref).cwiseProduct(scales);
}

bool obb_orthonormalize_columns(Eigen::MatrixXd& matrix) {
    const int dims = static_cast<int>(matrix.rows());
    int cols = 0;
    for (int input = 0; input < matrix.cols() && cols < dims; ++input) {
        Eigen::VectorXd candidate = matrix.col(input);
        for (int col = 0; col < cols; ++col) {
            candidate -= matrix.col(col) * matrix.col(col).dot(candidate);
        }
        const double norm = candidate.norm();
        if (norm > 1e-10) {
            matrix.col(cols++) = candidate / norm;
        }
    }
    for (int dim = 0; dim < dims && cols < dims; ++dim) {
        Eigen::VectorXd candidate = Eigen::VectorXd::Zero(dims);
        candidate[dim] = 1.0;
        for (int col = 0; col < cols; ++col) {
            candidate -= matrix.col(col) * matrix.col(col).dot(candidate);
        }
        const double norm = candidate.norm();
        if (norm > 1e-10) {
            matrix.col(cols++) = candidate / norm;
        }
    }
    return cols == dims;
}

std::vector<int> obb_low_risk_joint_order(const Robot& robot, int dims) {
    std::vector<std::pair<double, int>> scored;
    scored.reserve(static_cast<std::size_t>(dims));
    for (int joint = 0; joint < dims; ++joint) {
        double sensitivity = 1e-6;
        for (int link = joint; link < robot.n_joints(); ++link) {
            const auto& dh = robot.dh_params()[static_cast<std::size_t>(link)];
            sensitivity += std::abs(dh.a) + 0.25 * std::abs(dh.d) + 1e-3;
        }
        scored.emplace_back(sensitivity, joint);
    }
    std::sort(scored.begin(), scored.end());
    std::vector<int> order;
    order.reserve(scored.size());
    for (const auto& item : scored) {
        order.push_back(item.second);
    }
    return order;
}

bool obb_make_candidate_from_scaled(const Eigen::VectorXd& center_y,
                                    const Eigen::MatrixXd& basis_y,
                                    const Eigen::VectorXd& radii_y,
                                    const std::vector<Interval>& domain,
                                    ObbPortalCandidate& candidate) {
    const int dims = static_cast<int>(center_y.size());
    if (dims <= 0 ||
        basis_y.rows() != dims ||
        basis_y.cols() != dims ||
        radii_y.size() != dims) {
        return false;
    }
    const Eigen::VectorXd ref = obb_domain_reference(domain);
    const Eigen::VectorXd scales = obb_joint_scales(domain);
    candidate.center_y = center_y;
    candidate.basis_y = basis_y;
    candidate.radii_y = radii_y;
    candidate.center_q.resize(dims);
    for (int dim = 0; dim < dims; ++dim) {
        candidate.center_q[dim] = ref[dim] + center_y[dim] / scales[dim];
    }
    candidate.generators_q = Eigen::MatrixXd::Zero(dims, dims);
    for (int row = 0; row < dims; ++row) {
        for (int col = 0; col < dims; ++col) {
            candidate.generators_q(row, col) = basis_y(row, col) * radii_y[col] / scales[row];
        }
    }
    if (!obb_generators_within_domain(candidate.center_q,
                                      candidate.generators_q,
                                      domain,
                                      1e-10)) {
        return false;
    }
    double score = 0.0;
    for (int col = 0; col < dims; ++col) {
        score += std::log(std::max(1e-14, radii_y[col]));
    }
    candidate.score = score;
    return true;
}

bool obb_fit_scaled_path_with_basis(const std::vector<Eigen::VectorXd>& path,
                                    const std::vector<Interval>& domain,
                                    const Eigen::MatrixXd& basis_y,
                                    double lateral_radius,
                                    double longitudinal_margin,
                                    ObbPortalCandidate& candidate,
                                    ObbPortalValidationStats& stats) {
    const int dims = static_cast<int>(path.front().size());
    const Eigen::VectorXd ref = obb_domain_reference(domain);
    const Eigen::VectorXd scales = obb_joint_scales(domain);
    std::vector<Eigen::VectorXd> scaled_path;
    scaled_path.reserve(path.size());
    for (const auto& waypoint : path) {
        if (waypoint.size() != dims) {
            ++stats.degenerate_rejects;
            return false;
        }
        scaled_path.push_back(obb_to_scaled(waypoint, ref, scales));
    }
    Eigen::VectorXd min_proj = Eigen::VectorXd::Constant(dims, std::numeric_limits<double>::infinity());
    Eigen::VectorXd max_proj = Eigen::VectorXd::Constant(dims, -std::numeric_limits<double>::infinity());
    for (const auto& y : scaled_path) {
        const Eigen::VectorXd z = basis_y.transpose() * y;
        for (int col = 0; col < dims; ++col) {
            min_proj[col] = std::min(min_proj[col], z[col]);
            max_proj[col] = std::max(max_proj[col], z[col]);
        }
    }
    Eigen::VectorXd center_proj = 0.5 * (min_proj + max_proj);
    Eigen::VectorXd radii_y = 0.5 * (max_proj - min_proj);
    radii_y[0] += std::max(0.0, longitudinal_margin);
    const double lateral = std::max(0.0, lateral_radius);
    for (int col = 1; col < dims; ++col) {
        radii_y[col] += lateral;
    }
    for (int col = 0; col < dims; ++col) {
        const double min_radius = (col == 0 || lateral > 0.0) ? 1e-8 : 0.0;
        radii_y[col] = std::max(radii_y[col], min_radius);
    }
    const Eigen::VectorXd center_y = basis_y * center_proj;
    if (!obb_make_candidate_from_scaled(center_y, basis_y, radii_y, domain, candidate)) {
        ++stats.joint_limit_rejects;
        return false;
    }
    stats.longitudinal_radius = std::max(stats.longitudinal_radius, radii_y[0]);
    stats.lateral_radius = std::max(stats.lateral_radius, lateral);
    stats.variables = dims;
    return true;
}

std::vector<Eigen::MatrixXd> obb_orientation_candidates(const Robot& robot,
                                                        const std::vector<Eigen::VectorXd>& path,
                                                        const std::vector<Interval>& domain,
                                                        ObbPortalValidationStats& stats,
                                                        bool primary_only) {
    std::vector<Eigen::MatrixXd> candidates;
    const int dims = static_cast<int>(path.front().size());
    const Eigen::VectorXd ref = obb_domain_reference(domain);
    const Eigen::VectorXd scales = obb_joint_scales(domain);
    std::vector<Eigen::VectorXd> scaled_path;
    scaled_path.reserve(path.size());
    for (const auto& waypoint : path) {
        scaled_path.push_back(obb_to_scaled(waypoint, ref, scales));
    }

    Eigen::VectorXd tangent = scaled_path.back() - scaled_path.front();
    if (tangent.norm() <= 1e-12) {
        ++stats.degenerate_rejects;
        return candidates;
    }
    tangent.normalize();

    {
        Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(dims, dims);
        basis.col(0) = tangent;
        const auto risk_order = obb_low_risk_joint_order(robot, dims);
        int col = 1;
        for (int joint : risk_order) {
            if (col >= dims) {
                break;
            }
            basis.col(col) = Eigen::VectorXd::Unit(dims, joint);
            ++col;
        }
        if (obb_orthonormalize_columns(basis)) {
            candidates.push_back(basis);
        }
    }
    if (primary_only) {
        stats.candidates += static_cast<int>(candidates.size());
        return candidates;
    }

    for (int preferred_axis = 0; preferred_axis < dims; ++preferred_axis) {
        Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(dims, dims);
        basis.col(0) = tangent;
        int col = 1;
        basis.col(col++) = Eigen::VectorXd::Unit(dims, preferred_axis);
        for (int axis = 0; axis < dims && col < dims; ++axis) {
            if (axis == preferred_axis) {
                continue;
            }
            basis.col(col++) = Eigen::VectorXd::Unit(dims, axis);
        }
        if (obb_orthonormalize_columns(basis)) {
            candidates.push_back(basis);
        }
    }

    if (path.size() >= 3U) {
        Eigen::VectorXd mean = Eigen::VectorXd::Zero(dims);
        for (const auto& y : scaled_path) {
            mean += y;
        }
        mean /= static_cast<double>(scaled_path.size());
        Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(dims, dims);
        for (const auto& y : scaled_path) {
            const Eigen::VectorXd d = y - mean;
            cov.noalias() += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(cov);
        if (solver.info() == Eigen::Success) {
            Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(dims, dims);
            for (int col = 0; col < dims; ++col) {
                basis.col(col) = solver.eigenvectors().col(dims - 1 - col);
            }
            if (basis.col(0).dot(tangent) < 0.0) {
                basis.col(0) *= -1.0;
            }
            if (obb_orthonormalize_columns(basis)) {
                candidates.push_back(basis);
            }
        }
    }

    {
        Eigen::MatrixXd basis = Eigen::MatrixXd::Identity(dims, dims);
        candidates.push_back(basis);
    }
    stats.candidates += static_cast<int>(candidates.size());
    return candidates;
}

}  // namespace rbf
