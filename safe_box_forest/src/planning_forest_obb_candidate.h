#pragma once

#include <Eigen/Core>

#include <limits>

namespace rbf {

struct ObbPortalCandidate {
    Eigen::VectorXd center_q;
    Eigen::VectorXd center_y;
    Eigen::MatrixXd basis_y;
    Eigen::VectorXd radii_y;
    Eigen::MatrixXd generators_q;
    double score = -std::numeric_limits<double>::infinity();
};

}  // namespace rbf
