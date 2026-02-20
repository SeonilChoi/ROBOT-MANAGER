#ifndef SCHEDULER_FSM_SCHEDULER_HPP_
#define SCHEDULER_FSM_SCHEDULER_HPP_

#include "robot_manager/core/scheduler.hpp"

namespace micros {

class FsmScheduler : public Scheduler {
public:
    FsmScheduler(double T, double dt)
    : Scheduler(T, dt) {}

    ~FsmScheduler() override = default;

    void reset() override;

    void step(Action action) override;

    void get_state() override;

private:
    fsm_state_t current_state_;
};

}
#endif // SCHEDULER_FSM_SCHEDULER_HPP_