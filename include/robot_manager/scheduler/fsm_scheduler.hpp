#ifndef SCHEDULER_FSM_SCHEDULER_HPP_
#define SCHEDULER_FSM_SCHEDULER_HPP_

#include "robot_manager/core/scheduler.hpp"

namespace micros {

class FsmScheduler : public Scheduler {
public:
    FsmScheduler(double dt)
    : Scheduler(dt) {}

    ~FsmScheduler() override = default;

    void reset() override;

    void step() override;

    bool tick(const fsm_action_t& action, fsm_state_t& next_state) override;

private:
    double progress_raw(double t) override;

    fsm_state_t current_state_;
};

}
#endif // SCHEDULER_FSM_SCHEDULER_HPP_