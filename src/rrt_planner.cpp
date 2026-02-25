#include "robot_manager/planner/rrt_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace micros {

namespace {

constexpr double kPi = 3.14159265358979323846;

double joint_distance(const joint_state_t& a, const joint_state_t& b)
{
    uint8_t n = std::min(a.number_of_joints, b.number_of_joints);
    double sum = 0.0;
    for (uint8_t i = 0; i < n; ++i) {
        double d = a.position[i] - b.position[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

void copy_joint_state_metadata(const joint_state_t& from, joint_state_t& to)
{
    to.number_of_joints = from.number_of_joints;
    for (uint8_t i = 0; i < from.number_of_joints; ++i) {
        to.target_id[i] = from.target_id[i];
        to.controller_idx[i] = from.controller_idx[i];
    }
}

void steer(const joint_state_t& from, const joint_state_t& toward, double step_size, joint_state_t& out)
{
    copy_joint_state_metadata(from, out);
    uint8_t n = from.number_of_joints;
    double d = joint_distance(from, toward);
    if (d <= step_size || d < 1e-9) {
        for (uint8_t i = 0; i < n; ++i)
            out.position[i] = toward.position[i];
        return;
    }
    double scale = step_size / d;
    for (uint8_t i = 0; i < n; ++i)
        out.position[i] = from.position[i] + scale * (toward.position[i] - from.position[i]);
}

void interpolate(const joint_state_t& a, const joint_state_t& b, double t, joint_state_t& out)
{
    copy_joint_state_metadata(a, out);
    uint8_t n = a.number_of_joints;
    for (uint8_t i = 0; i < n; ++i)
        out.position[i] = a.position[i] + t * (b.position[i] - a.position[i]);
}

} // namespace

void RrtPlanner::set_collision_checker(ConfigCollisionFn config_fn, SegmentCollisionFn segment_fn)
{
    config_collision_ = std::move(config_fn);
    segment_collision_ = std::move(segment_fn);
}

void RrtPlanner::set_joint_limits(const double* min_positions, const double* max_positions, uint8_t num_joints)
{
    joint_limits_size_ = std::min(num_joints, static_cast<uint8_t>(MAX_JOINT_SIZE));
    for (uint8_t i = 0; i < joint_limits_size_; ++i) {
        joint_min_[i] = min_positions[i];
        joint_max_[i] = max_positions[i];
    }
    joint_limits_set_ = true;
}

void RrtPlanner::plan(const robot_state_t& current_state, const robot_state_t& target_state, const obstacle_state_t& obstacle_state)
{
    if (is_running_.load(std::memory_order_relaxed)) {
        return;
    }

    current_state_.store(current_state, std::memory_order_relaxed);
    target_state_.store(target_state, std::memory_order_relaxed);
    obstacle_state_.store(obstacle_state, std::memory_order_relaxed);

    is_planned_.store(false, std::memory_order_relaxed);
    notify_work_ready();
}

bool RrtPlanner::generate_trajectory(const robot_state_t& current_state, const robot_state_t& target_state, const obstacle_state_t& obstacle_state)
{
    const joint_state_t& start = current_state.joint_state;
    const joint_state_t& goal = target_state.joint_state;
    uint8_t n = start.number_of_joints;
    if (n == 0 || n != goal.number_of_joints) {
        std::lock_guard lock(trajectory_mutex_);
        trajectory_.clear();
        return false;
    }

    std::vector<joint_state_t> nodes;
    std::vector<size_t> parent;
    nodes.push_back(start);
    parent.push_back(0);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> goal_bias_dist(0.0, 1.0);

    double min_pos[MAX_JOINT_SIZE], max_pos[MAX_JOINT_SIZE];
    if (joint_limits_set_ && joint_limits_size_ >= n) {
        for (uint8_t i = 0; i < n; ++i) {
            min_pos[i] = joint_min_[i];
            max_pos[i] = joint_max_[i];
        }
    } else {
        for (uint8_t i = 0; i < n; ++i) {
            min_pos[i] = -kPi;
            max_pos[i] = kPi;
        }
    }

    auto in_collision = [this, &obstacle_state](const joint_state_t& q) -> bool {
        if (config_collision_)
            return config_collision_(q, obstacle_state);
        return false;
    };
    auto segment_collision = [this, &obstacle_state](const joint_state_t& a, const joint_state_t& b) -> bool {
        if (segment_collision_) {
            return segment_collision_(a, b, obstacle_state);
        }
        constexpr int kSteps = 10;
        joint_state_t interp;
        for (int s = 1; s < kSteps; ++s) {
            double t = static_cast<double>(s) / kSteps;
            interpolate(a, b, t, interp);
            if (in_collision(interp))
                return true;
        }
        return false;
    };

    for (size_t iter = 0; iter < kMaxIterations; ++iter) {
        joint_state_t sample;
        copy_joint_state_metadata(start, sample);

        if (goal_bias_dist(gen) < kGoalBias) {
            for (uint8_t i = 0; i < n; ++i)
                sample.position[i] = goal.position[i];
        } else {
            std::uniform_real_distribution<> pos_dist(0.0, 1.0);
            for (uint8_t i = 0; i < n; ++i)
                sample.position[i] = min_pos[i] + pos_dist(gen) * (max_pos[i] - min_pos[i]);
        }

        size_t nearest = 0;
        double nearest_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < nodes.size(); ++i) {
            double d = joint_distance(nodes[i], sample);
            if (d < nearest_dist) {
                nearest_dist = d;
                nearest = i;
            }
        }

        joint_state_t new_node;
        steer(nodes[nearest], sample, kStepSize, new_node);

        if (in_collision(new_node))
            continue;
        if (segment_collision(nodes[nearest], new_node))
            continue;

        nodes.push_back(new_node);
        parent.push_back(nearest);

        if (joint_distance(new_node, goal) <= kGoalThreshold) {
            std::vector<size_t> path;
            for (size_t idx = nodes.size() - 1; ; idx = parent[idx]) {
                path.push_back(idx);
                if (idx == 0) break;
            }
            std::reverse(path.begin(), path.end());

            double total_len = 0.0;
            for (size_t i = 1; i < path.size(); ++i)
                total_len += joint_distance(nodes[path[i - 1]], nodes[path[i]]]);
            if (total_len < 1e-9) total_len = 1.0;

            Trajectory traj;
            double cum = 0.0;
            for (size_t i = 0; i < path.size(); ++i) {
                double t = (i == 0) ? 0.0 : (cum / total_len);
                if (i > 0)
                    cum += joint_distance(nodes[path[i - 1]], nodes[path[i]]]);
                joint_state_t js = nodes[path[i]];
                copy_joint_state_metadata(start, js);
                for (uint8_t j = 0; j < n; ++j) js.velocity[j] = 0.0;
                for (uint8_t j = 0; j < n; ++j) js.torque[j] = 0.0;
                traj.emplace_back(t, js);
            }
            traj.front().first = 0.0;
            traj.back().first = 1.0;

            std::lock_guard lock(trajectory_mutex_);
            trajectory_ = std::move(traj);
            return true;
        }
    }

    std::lock_guard lock(trajectory_mutex_);
    trajectory_.clear();
    return false;
}

void RrtPlanner::eval(double progress, joint_state_t& joint_command)
{
    if (!is_planned_.load(std::memory_order_acquire)) {
        return;
    }

    std::lock_guard lock(trajectory_mutex_);
    if (trajectory_.empty()) {
        return;
    }
    if (trajectory_.size() == 1) {
        joint_command = trajectory_[0].second;
        return;
    }

    progress = std::max(0.0, std::min(1.0, progress));
    size_t i = 0;
    while (i + 1 < trajectory_.size() && trajectory_[i + 1].first <= progress)
        ++i;

    if (i + 1 >= trajectory_.size()) {
        joint_command = trajectory_.back().second;
        return;
    }

    double t0 = trajectory_[i].first;
    double t1 = trajectory_[i + 1].first;
    double t = (t1 - t0) > 1e-9 ? (progress - t0) / (t1 - t0) : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    interpolate(trajectory_[i].second, trajectory_[i + 1].second, t, joint_command);
}

}
