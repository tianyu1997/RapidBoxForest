#pragma once

#include "planning_forest_obb.h"

#include <sbf/envelope/envelope_collision.h>

#include <Eigen/Core>

#include <vector>

namespace rbf {

struct ObbEndpointZonotope {
    detail::Vec3 center{};
    std::vector<detail::Vec3> generators;
    detail::Vec3 remainder{};
};

struct ObbLinkZonotopeHull {
    ObbEndpointZonotope proximal;
    ObbEndpointZonotope distal;
    double radius = 0.0;
    float aabb[6] = {};
};

Eigen::MatrixXd obb_compress_generator_columns(const Eigen::MatrixXd& generators,
                                               double column_norm_tol = 1e-12);

std::vector<ObbLinkZonotopeHull> obb_compute_link_zonotopes(
    const Robot& robot,
    const Eigen::VectorXd& center,
    const Eigen::MatrixXd& generators,
    int n_vars);

void obb_compute_link_aabb(ObbLinkZonotopeHull& link, double pad);

bool obb_zonotope_link_separates_obstacle(const ObbLinkZonotopeHull& link,
                                          const float* obstacle,
                                          double pad,
                                          ObbPortalValidationStats& stats);

}  // namespace rbf
