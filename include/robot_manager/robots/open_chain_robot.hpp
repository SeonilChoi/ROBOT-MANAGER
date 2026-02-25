#ifndef ROBOTS_OPEN_CHAIN_ROBOT_CONTROLLER_HPP_
#define ROBOTS_OPEN_CHAIN_ROBOT_CONTROLLER_HPP_

#include "robot_manager/core/robot_controller.hpp"
#include "robot_manager/scheduler/fsm_scheduler.hpp"

namespace micros {

class OpenChainRobotController : public RobotController {
public:
    OpenChainRobotController(const robot_config_t& config);
    
    ~OpenChainRobotController() override = default;

    void initialize() override;

    void control(joint_state_t& joint_command) override;

    void update(const joint_state_t& joint_state) override;

private:
    FsmScheduler fsm_scheduler_{0.01};
};

} // namespace micros
#endif // ROBOTS_OPEN_CHAIN_ROBOT_CONTROLLER_HPP_