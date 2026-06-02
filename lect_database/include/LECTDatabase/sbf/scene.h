#pragma once

#include <link_interval_envelope/robot.h>
#include <link_interval_envelope/types.h>

#include <Eigen/Core>

#include <utility>
#include <vector>

namespace rbf {

class Scene {
public:
    Scene() = default;
    explicit Scene(std::vector<Obstacle> obstacles);

    void set_obstacles(std::vector<Obstacle> obstacles);
    void set_obstacles(const Obstacle* obstacles, int n_obstacles);
    void add_obstacle(const Obstacle& obstacle);
    bool remove_obstacle_at(int index, Obstacle* removed = nullptr);
    void clear();

    bool empty() const { return obstacles_.empty(); }
    int n_obstacles() const { return static_cast<int>(obstacles_.size()); }
    const std::vector<Obstacle>& obstacles() const { return obstacles_; }
    const std::vector<float>& obstacle_aabbs() const { return obstacle_aabbs_; }
    const float* obstacle_aabbs_data() const { return obstacle_aabbs_.empty() ? nullptr : obstacle_aabbs_.data(); }

private:
    void repack();

    std::vector<Obstacle> obstacles_;
    std::vector<float> obstacle_aabbs_;
};

bool aabbs_collide_obstacles(const float* link_aabbs,
                             int n_slots,
                             const float* obstacle_aabbs,
                             int n_obstacles);

class CollisionChecker {
public:
    CollisionChecker() = default;
    CollisionChecker(const Robot& robot, Scene scene = {});

    void set_scene(Scene scene) { scene_ = std::move(scene); }
    void set_obstacles(const Obstacle* obstacles, int n_obstacles);
    void set_collision_tolerance(double tolerance) { collision_tolerance_ = tolerance > 0.0 ? tolerance : 0.0; }
    double collision_tolerance() const { return collision_tolerance_; }
    const Robot& robot() const;
    const Scene& scene() const { return scene_; }

    bool check_config(const Eigen::Ref<const Eigen::VectorXd>& q) const;
    bool check_box(const std::vector<Interval>& intervals) const;
    bool check_segment(const Eigen::Ref<const Eigen::VectorXd>& a,
                       const Eigen::Ref<const Eigen::VectorXd>& b,
                       int resolution = 16) const;
    bool check_segment_interior(const Eigen::Ref<const Eigen::VectorXd>& a,
                                const Eigen::Ref<const Eigen::VectorXd>& b,
                                int resolution = 16) const;

private:
    const Robot* robot_ = nullptr;
    Scene scene_;
    double collision_tolerance_ = 0.0;
};

}  // namespace rbf
