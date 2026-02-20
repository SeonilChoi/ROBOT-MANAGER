#ifndef CORE_SCHEDULER_HPP_
#define CORE_SCHEDULER_HPP_

namespace micros {

class Scheduler {
public:
    explicit Scheduler(double dt)
    : dt_(dt) {}

    virtual ~Scheduler() = default;

    virtual void reset() = 0;

    virtual bool step(const fsm_action_t& action, fsm_state_t& next_state) = 0;

protected:
    virtual double progress_raw(double t) = 0;

    const double dt_;

    double T_{0.0};

    double t_{0.0};
};

}
#endif // CORE_SCHEDULER_HPP_