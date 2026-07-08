#pragma once

#include <SBF/scene_types.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace rbf::python_binding {

inline std::vector<int> active_link_map_vec(const rbf::Robot& robot) {
    const int* map = robot.active_link_map();
    if (map == nullptr) return {};
    return std::vector<int>(map, map + robot.n_active_links());
}

inline std::vector<double> active_link_radii_vec(const rbf::Robot& robot) {
    const double* radii = robot.active_link_radii();
    if (radii == nullptr) return {};
    return std::vector<double>(radii, radii + robot.n_active_links());
}

inline Eigen::VectorXd eigen_vector_from_list(const std::vector<double>& values) {
    Eigen::VectorXd vector(static_cast<Eigen::Index>(values.size()));
    for (std::size_t i = 0; i < values.size(); ++i) {
        vector[static_cast<Eigen::Index>(i)] = values[i];
    }
    return vector;
}

inline std::vector<Eigen::VectorXd> eigen_vectors_from_lists(
    const std::vector<std::vector<double>>& values) {
    std::vector<Eigen::VectorXd> result;
    result.reserve(values.size());
    for (const auto& item : values) {
        result.push_back(eigen_vector_from_list(item));
    }
    return result;
}

inline std::vector<double> vector_to_list(const Eigen::VectorXd& values) {
    std::vector<double> result(static_cast<std::size_t>(values.size()));
    for (Eigen::Index i = 0; i < values.size(); ++i) {
        result[static_cast<std::size_t>(i)] = values[i];
    }
    return result;
}

inline std::vector<std::vector<double>> eigen_path_to_lists(
    const std::vector<Eigen::VectorXd>& path) {
    std::vector<std::vector<double>> result;
    result.reserve(path.size());
    for (const auto& waypoint : path) {
        result.push_back(vector_to_list(waypoint));
    }
    return result;
}

inline std::vector<rbf::Interval> intervals_from_pairs(
    const std::vector<std::vector<double>>& pairs) {
    std::vector<rbf::Interval> intervals;
    intervals.reserve(pairs.size());
    for (const auto& pair : pairs) {
        if (pair.size() != 2) {
            throw std::invalid_argument("each interval pair must have exactly two entries");
        }
        if (pair[1] < pair[0]) {
            throw std::invalid_argument("interval upper bound must be >= lower bound");
        }
        intervals.push_back(rbf::Interval{pair[0], pair[1]});
    }
    return intervals;
}

inline py::list interval_pairs_to_python(const std::vector<rbf::Interval>& intervals) {
    py::list result;
    for (const auto& interval : intervals) {
        py::list pair;
        pair.append(interval.lo);
        pair.append(interval.hi);
        result.append(std::move(pair));
    }
    return result;
}

inline bool intervals_contain_point_pybind(const std::vector<rbf::Interval>& intervals,
                                           const Eigen::Ref<const Eigen::VectorXd>& point,
                                           double tolerance) {
    if (intervals.size() != static_cast<std::size_t>(point.size())) {
        return false;
    }
    for (Eigen::Index dim = 0; dim < point.size(); ++dim) {
        const auto& interval = intervals[static_cast<std::size_t>(dim)];
        const double value = point[dim];
        if (value < interval.lo - tolerance || value > interval.hi + tolerance) {
            return false;
        }
    }
    return true;
}

inline bool interval_boxes_connected_pybind(const std::vector<rbf::Interval>& lhs,
                                            const std::vector<rbf::Interval>& rhs,
                                            double tolerance) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    int shared_dims = 0;
    int overlap_dims = 0;
    for (std::size_t dim = 0; dim < lhs.size(); ++dim) {
        const double overlap_lo = std::max(lhs[dim].lo, rhs[dim].lo);
        const double overlap_hi = std::min(lhs[dim].hi, rhs[dim].hi);
        if (overlap_hi < overlap_lo - tolerance) {
            return false;
        }
        if (overlap_hi - overlap_lo < tolerance) {
            shared_dims += 1;
        } else {
            overlap_dims += 1;
        }
    }
    return shared_dims >= 1 || overlap_dims == static_cast<int>(lhs.size());
}

inline std::vector<Eigen::VectorXd> densify_path_pybind(
    const std::vector<Eigen::VectorXd>& waypoints,
    double step) {
    std::vector<Eigen::VectorXd> samples;
    if (waypoints.empty()) {
        return samples;
    }
    if (waypoints.size() == 1) {
        samples.push_back(waypoints.front());
        return samples;
    }
    const double effective_step = std::max(step, 1e-12);
    for (std::size_t index = 1; index < waypoints.size(); ++index) {
        const Eigen::VectorXd& a = waypoints[index - 1];
        const Eigen::VectorXd& b = waypoints[index];
        const double length = (b - a).norm();
        const int n = std::max(1, static_cast<int>(std::ceil(length / effective_step)));
        for (int sample = 0; sample < n; ++sample) {
            const double u = static_cast<double>(sample) / static_cast<double>(n);
            samples.push_back((1.0 - u) * a + u * b);
        }
    }
    samples.push_back(waypoints.back());
    return samples;
}

} // namespace rbf::python_binding
