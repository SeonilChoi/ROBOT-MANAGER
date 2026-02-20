#ifndef CORE_ROBOT_CONTROLLER_HPP_
#define CORE_ROBOT_CONTROLLER_HPP_

namespace micros {

class RobotController {
public:
    explicit RobotController(const robot_config_t& config)
    : id_(config.id)
    , number_of_joints_(config.number_of_joints)
    , controller_idxs_(config.controller_idxs) {}

    virtual ~RobotController() = default;

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
};

}
#endif // CORE_ROBOT_CONTROLLER_HPP_