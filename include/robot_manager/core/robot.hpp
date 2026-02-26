#ifndef CORE_ROBOT_HPP_
#define CORE_ROBOT_HPP_

namespace micros {

class Robot {
public:
    explicit Robot(const robot_config_t& config)
    : id_(config.id)
    , number_of_joints_(config.number_of_joints)
    , controller_idxs_(config.controller_idxs)
    , scheduler_type_(config.scheduler_type)
    , planner_type_(config.planner_type) {}

    virtual ~Robot() = default;

    virtual void initialize() = 0;

    virtual void control(joint_state_t& joint_command) = 0;

    virtual void update(const joint_state_t& joint_state) = 0;

    uint8_t id() const { return id_; }

    uint8_t number_of_joints() const { return number_of_joints_; }

    const uint8_t* controller_idxs() const { return controller_idxs_; }

protected:
    const uint8_t id_;

    const uint8_t number_of_joints_;

    const uint8_t controller_idxs_[MAX_JOINT_SIZE]{};

    const SchedulerType scheduler_type_;

    const PlannerType planner_type_;
};

}
#endif // CORE_ROBOT_HPP_