#include <stdexcept>

#include "robot_manager/scheduler/fsm_scheduler.hpp"

micros::FsmScheduler::FsmScheduler(double dt)
: Scheduler(dt) {}

void micros::FsmScheduler::reset()
{
    t_ = 0.0;
}

void micros::FsmScheduler::step()
{
    t_ += dt_;
}

bool micros::FsmScheduler::tick(const fsm_action_t& action, fsm_state_t& next_state)
{
    T_ = action.duration;
    double t = T_ != 0.0 ? t_ + dt_ : 0.0;

    next_state.state = transition_table[current_state_.state][static_cast<uint8_t>(action.action)];
    next_state.progress = progress_raw(t);
    
    if (next_state.state == State::INVALID) throw std::runtime_error("Invalid state transition.");

    if (next_state.state != current_state_.state) {
        current_state_ = next_state;
        return true;
    }

    if (next_state.progress >= 1.0) {
        t_ = 0.0;
        if (next_state.state == State::HOMING) {
            current_state_.state = State::STOPPED;
            current_state_.progress = 0.0;
        }
        return true;
    }
    
    return false;
}

double micros::FsmScheduler::progress_raw(double t)
{
    if (T_ == 0.0) return 0.0;
    return t / T_;
}