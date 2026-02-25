#ifndef CORE_PLANNER_HPP_
#define CORE_PLANNER_HPP_

#include "robot_manager/types.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace micros {

class Planner {
public:
    explicit Planner() = default;

    virtual ~Planner() = default;

    virtual void eval(double progress, joint_state_t& joint_command) = 0;

    void plan(const robot_state_t& current_state, const robot_state_t& target_state, const obstacle_state_t& obstacle_state) {
        if (is_running_.load(std::memory_order_relaxed)) {
            return;
        }
    
        current_state_.store(current_state, std::memory_order_relaxed);
        target_state_.store(target_state, std::memory_order_relaxed);
        obstacle_state_.store(obstacle_state, std::memory_order_relaxed);
    
        is_planned_.store(false, std::memory_order_relaxed);
        notify_work_ready();
    }
    
    /** Call before joining the planner thread (e.g. in derived destructor). */
    void request_stop() {
        std::lock_guard lock(mutex_);
        stop_requested_.store(true, std::memory_order_relaxed);
        cv_.notify_one();
    }

    bool is_planned() const { return is_planned_.load(std::memory_order_relaxed); }

protected:
    /** Returns true if a trajectory was successfully generated. */
    virtual bool generate_trajectory(const robot_state_t& current_state, const robot_state_t& target_state, const obstacle_state_t& obstacle_state) = 0;
    
    void run() {
        while (true) {
            {
                std::unique_lock lock(mutex_);
                cv_.wait(lock, [this] { 
                    return stop_requested_.load(std::memory_order_relaxed) || is_running_.load(std::memory_order_relaxed);
                });
            }
        }

        if (stop_requested_.load(std::memory_order_relaxed)) {
            break;
        }

        robot_state_t current = current_state_.load(std::memory_order_relaxed);
        robot_state_t target = target_state_.load(std::memory_order_relaxed);
        obstacle_state_t obstacle = obstacle_state_.load(std::memory_order_relaxed);

        {
            std::lock_guard lock(mutex_);
            is_running_.store(false, std::memory_order_relaxed);
        }

        bool success = generate_trajectory(current, target, obstacle);
        is_planned_.store(success, std::memory_order_release);
    }

    /** Call from derived plan() after storing state and setting is_planned_ = false. */
    void notify_work_ready() {
        std::lock_guard lock(mutex_);
        is_running_.store(true, std::memory_order_relaxed);
        cv_.notify_one();
    }

    std::thread planner_thread_;
    
    std::mutex mutex_;
    
    std::condition_variable cv_;

    std::atomic<bool> is_planned_{false};

    std::atomic<bool> is_running_{false};
    
    std::atomic<bool> stop_requested_{false};

    std::atomic<robot_state_t> current_state_{};
    
    std::atomic<robot_state_t> target_state_{};
    
    std::atomic<obstacle_state_t> obstacle_state_{};
};

}

#endif // CORE_PLANNER_HPP_
