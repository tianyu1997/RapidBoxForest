#include <LECTDatabase/sbf/scene.h>

#include <sbf/core/fk_state.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace rbf {
namespace {

void mat4_mul_point(const double lhs[16], const double rhs[16], double out[16]) {
    for (int row = 0; row < 3; ++row) {
        const double* lrow = lhs + row * 4;
        for (int col = 0; col < 3; ++col) {
            out[row * 4 + col] = lrow[0] * rhs[col] + lrow[1] * rhs[4 + col] + lrow[2] * rhs[8 + col];
        }
        out[row * 4 + 3] = lrow[0] * rhs[3] + lrow[1] * rhs[7] + lrow[2] * rhs[11] + lrow[3];
    }
    out[12] = 0.0;
    out[13] = 0.0;
    out[14] = 0.0;
    out[15] = 1.0;
}

void build_dh_point(double alpha, double a, double ct, double st, double d, double out[16]) {
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    out[0] = ct;
    out[1] = -st;
    out[2] = 0.0;
    out[3] = a;
    out[4] = st * ca;
    out[5] = ct * ca;
    out[6] = -sa;
    out[7] = -d * sa;
    out[8] = st * sa;
    out[9] = ct * sa;
    out[10] = ca;
    out[11] = d * ca;
    out[12] = 0.0;
    out[13] = 0.0;
    out[14] = 0.0;
    out[15] = 1.0;
}

bool segment_hits_inflated_aabb(const double origin[3],
                                const double dir[3],
                                const float* obstacle,
                                double radius,
                                double tolerance) {
    const double effective_radius = std::max(0.0, radius - std::max(0.0, tolerance));
    const double lo[3] = {static_cast<double>(obstacle[0]) - effective_radius,
                          static_cast<double>(obstacle[1]) - effective_radius,
                          static_cast<double>(obstacle[2]) - effective_radius};
    const double hi[3] = {static_cast<double>(obstacle[3]) + effective_radius,
                          static_cast<double>(obstacle[4]) + effective_radius,
                          static_cast<double>(obstacle[5]) + effective_radius};
    double enter = 0.0;
    double exit = 1.0;
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(dir[axis]) < 1e-15) {
            if (origin[axis] < lo[axis] || origin[axis] > hi[axis]) {
                return false;
            }
            continue;
        }
        const double inv = 1.0 / dir[axis];
        double t0 = (lo[axis] - origin[axis]) * inv;
        double t1 = (hi[axis] - origin[axis]) * inv;
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        enter = std::max(enter, t0);
        exit = std::min(exit, t1);
        if (enter > exit) {
            return false;
        }
    }
    return true;
}

}  // namespace

bool aabbs_collide_obstacles(const float* link_aabbs,
                             int n_slots,
                             const float* obstacle_aabbs,
                             int n_obstacles) {
    for (int slot = 0; slot < n_slots; ++slot) {
        const float* link = link_aabbs + slot * 6;
        for (int obs = 0; obs < n_obstacles; ++obs) {
            const float* obstacle = obstacle_aabbs + obs * 6;
            constexpr float eps = 1e-10f;
            if (link[3] < obstacle[0] - eps || link[0] > obstacle[3] + eps) continue;
            if (link[4] < obstacle[1] - eps || link[1] > obstacle[4] + eps) continue;
            if (link[5] < obstacle[2] - eps || link[2] > obstacle[5] + eps) continue;
            return true;
        }
    }
    return false;
}

Scene::Scene(std::vector<Obstacle> obstacles) : obstacles_(std::move(obstacles)) {
    repack();
}

void Scene::set_obstacles(std::vector<Obstacle> obstacles) {
    obstacles_ = std::move(obstacles);
    repack();
}

void Scene::set_obstacles(const Obstacle* obstacles, int n_obstacles) {
    if (n_obstacles < 0) {
        throw std::invalid_argument("LECTDatabase Scene obstacle count must be non-negative");
    }
    if (n_obstacles == 0) {
        clear();
        return;
    }
    if (obstacles == nullptr) {
        throw std::invalid_argument("LECTDatabase Scene obstacle pointer must not be null when count is positive");
    }
    obstacles_.assign(obstacles, obstacles + n_obstacles);
    repack();
}

void Scene::add_obstacle(const Obstacle& obstacle) {
    obstacles_.push_back(obstacle);
    repack();
}

bool Scene::remove_obstacle_at(int index, Obstacle* removed) {
    if (index < 0 || index >= static_cast<int>(obstacles_.size())) {
        return false;
    }
    const auto offset = static_cast<std::ptrdiff_t>(index);
    if (removed != nullptr) {
        *removed = obstacles_[static_cast<std::size_t>(index)];
    }
    obstacles_.erase(obstacles_.begin() + offset);
    repack();
    return true;
}

void Scene::clear() {
    obstacles_.clear();
    obstacle_aabbs_.clear();
}

void Scene::repack() {
    obstacle_aabbs_.resize(obstacles_.size() * 6);
    for (std::size_t i = 0; i < obstacles_.size(); ++i) {
        const auto& bounds = obstacles_[i].bounds;
        float* out = obstacle_aabbs_.data() + i * 6;
        out[0] = bounds[0];
        out[1] = bounds[1];
        out[2] = bounds[2];
        out[3] = bounds[3];
        out[4] = bounds[4];
        out[5] = bounds[5];
    }
}

CollisionChecker::CollisionChecker(const Robot& robot, Scene scene)
    : robot_(&robot), scene_(std::move(scene)) {}

void CollisionChecker::set_obstacles(const Obstacle* obstacles, int n_obstacles) {
    scene_.set_obstacles(obstacles, n_obstacles);
}

const Robot& CollisionChecker::robot() const {
    if (robot_ == nullptr) {
        throw std::logic_error("LECTDatabase CollisionChecker has no robot");
    }
    return *robot_;
}

bool CollisionChecker::check_config(const Eigen::Ref<const Eigen::VectorXd>& q) const {
    const Robot& model = robot();
    if (scene_.empty()) {
        return false;
    }
    if (q.size() != model.n_joints()) {
        throw std::invalid_argument("LECTDatabase CollisionChecker q dimension mismatch");
    }

    const int n = model.n_joints();
    const auto& dh = model.dh_params();
    double prefix[MAX_TF][16];
    std::memset(prefix[0], 0, sizeof(prefix[0]));
    prefix[0][0] = prefix[0][5] = prefix[0][10] = prefix[0][15] = 1.0;

    double joint_tf[16];
    for (int i = 0; i < n; ++i) {
        const double theta = (dh[i].joint_type == 0) ? q[i] + dh[i].theta : dh[i].theta;
        const double d_value = (dh[i].joint_type == 0) ? dh[i].d : q[i] + dh[i].d;
        build_dh_point(dh[i].alpha, dh[i].a, std::cos(theta), std::sin(theta), d_value, joint_tf);
        mat4_mul_point(prefix[i], joint_tf, prefix[i + 1]);
    }
    if (model.has_tool()) {
        const auto& tool = *model.tool_frame();
        build_dh_point(tool.alpha, tool.a, std::cos(tool.theta), std::sin(tool.theta), tool.d, joint_tf);
        mat4_mul_point(prefix[n], joint_tf, prefix[n + 1]);
    }

    const int n_active = model.n_active_links();
    const int* active_map = model.active_link_map();
    const double* radii = model.active_link_radii();
    for (int active = 0; active < n_active; ++active) {
        const int link = active_map[active];
        const double origin[3] = {prefix[link][3], prefix[link][7], prefix[link][11]};
        const double target[3] = {prefix[link + 1][3], prefix[link + 1][7], prefix[link + 1][11]};
        const double dir[3] = {target[0] - origin[0], target[1] - origin[1], target[2] - origin[2]};
        const double radius = radii != nullptr ? radii[active] : 0.0;
        for (int obs = 0; obs < scene_.n_obstacles(); ++obs) {
            if (segment_hits_inflated_aabb(origin,
                                           dir,
                                           scene_.obstacle_aabbs_data() + obs * 6,
                                           radius,
                                           collision_tolerance_)) {
                return true;
            }
        }
    }
    return false;
}

bool CollisionChecker::check_box(const std::vector<Interval>& intervals) const {
    const Robot& model = robot();
    if (scene_.empty()) {
        return false;
    }
    if (static_cast<int>(intervals.size()) != model.n_joints()) {
        throw std::invalid_argument("LECTDatabase CollisionChecker interval dimension mismatch");
    }
    FKState state = compute_fk_full(model, intervals);
    const int n_active = model.n_active_links();
    std::vector<float> link_aabbs(static_cast<std::size_t>(n_active) * 6);
    extract_link_aabbs(state, model.active_link_map(), n_active, link_aabbs.data(), model.active_link_radii());
    return aabbs_collide_obstacles(link_aabbs.data(), n_active, scene_.obstacle_aabbs_data(), scene_.n_obstacles());
}

bool CollisionChecker::check_segment(const Eigen::Ref<const Eigen::VectorXd>& a,
                                     const Eigen::Ref<const Eigen::VectorXd>& b,
                                     int resolution) const {
    if (resolution <= 0) {
        return check_config(a) || check_config(b);
    }
    const Eigen::VectorXd diff = b - a;
    for (int i = 0; i <= resolution; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(resolution);
        Eigen::VectorXd q = a + t * diff;
        if (check_config(q)) {
            return true;
        }
    }
    return false;
}

bool CollisionChecker::check_segment_interior(const Eigen::Ref<const Eigen::VectorXd>& a,
                                              const Eigen::Ref<const Eigen::VectorXd>& b,
                                              int resolution) const {
    if (resolution <= 1) {
        return false;
    }
    const Eigen::VectorXd diff = b - a;
    for (int i = 1; i < resolution; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(resolution);
        Eigen::VectorXd q = a + t * diff;
        if (check_config(q)) {
            return true;
        }
    }
    return false;
}

}  // namespace rbf
