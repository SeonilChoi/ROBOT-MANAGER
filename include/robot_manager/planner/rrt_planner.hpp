#ifndef PLANNER_RRT_PLANNER_HPP_
#define PLANNER_RRT_PLANNER_HPP_

#include "robot_manager/core/planner.hpp"

#include <cmath>
#include <functional>
#include <mutex>
#include <utility>
#include <vector>

namespace micros {

/** Optional collision check: true if in collision. If not set, no collision checking is performed. */
using ConfigCollisionFn = std::function<bool(const joint_state_t&, const obstacle_state_t&)>;
/** Optional segment collision check: true if segment a->b is in collision. */
using SegmentCollisionFn = std::function<bool(const joint_state_t&, const joint_state_t&, const obstacle_state_t&)>;

class RrtPlanner : public Planner {
public:
    explicit RrtPlanner()
    : Planner() {
        planner_thread_ = std::thread([this]() { this->run(); });
    }

    ~RrtPlanner() {
        request_stop();
        if (planner_thread_.joinable()) {
            planner_thread_.join();
        }
    }

    void eval(double progress, joint_state_t& joint_command) override;

    /** Set optional collision checker. If unset, all configs/segments are treated as collision-free. */
    void set_collision_checker(ConfigCollisionFn config_fn, SegmentCollisionFn segment_fn);

    /** Set joint limits (rad). Defaults to [-pi, pi] per joint if not set. */
    void set_joint_limits(const double* min_positions, const double* max_positions, uint8_t num_joints);

private:
    bool generate_trajectory(const robot_state_t& current_state, const robot_state_t& target_state, const obstacle_state_t& obstacle_state) override;

    /** Time-parameterized waypoints: (normalized time in [0,1], joint_state). */
    using Trajectory = std::vector<std::pair<double, joint_state_t>>;

    std::mutex trajectory_mutex_;
    
    Trajectory trajectory_;

    ConfigCollisionFn config_collision_;
    
    SegmentCollisionFn segment_collision_;

    static constexpr size_t kMaxIterations = 5000;
    
    static constexpr double kStepSize = 0.05; // [cm]
    
    static constexpr double kGoalBias = 0.1;
    
    static constexpr double kGoalThreshold = 0.05; // [cm]

    double joint_min_[MAX_JOINT_SIZE];
    
    double joint_max_[MAX_JOINT_SIZE];
    
    uint8_t joint_limits_size_{0};
    
    bool joint_limits_set_{false};
};

}

#endif // PLANNER_RRT_PLANNER_HPP_
