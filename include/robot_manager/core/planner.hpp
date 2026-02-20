#ifndef CORE_PLANNER_HPP_
#define CORE_PLANNER_HPP_

namespace micros {

class Planner {
public:
    explicit Planner() = default;

    virtual ~Planner() = default;

    virtual void reset(const robot_state_t& current_state, const ) = 0;

    virtual void eval(double progress) = 0;
};

} // namespace micros
#endif // CORE_PLANNER_HPP_