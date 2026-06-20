#pragma once

#include "planning_forest_obb_sampled.h"

#include <sbf/envelope/envelope_collision.h>

#include <vector>

namespace rbf {

double obb_endpoint_lipschitz_error(const Robot& robot,
                                    int frame_index,
                                    const std::vector<double>& joint_deviation);

std::vector<detail::Vec3> obb_sample_frame_positions(const Robot& robot,
                                                     const Eigen::VectorXd& q);

}  // namespace rbf
