#include "robot_manager/robots/open_chain_robot_controller.hpp"

micros::OpenChainRobotController::OpenChainRobotController(const robot_config_t& config)
: RobotController(config) {}

void micros::OpenChainRobotController::initialize()
{

}

void micros::OpenChainRobotController::set_action(const fsm_action_t& action)
{
    fsm_state_t state{};
    bool is_event = fsm_scheduler_.step(action, state);

    if (is_event) {
        if (state.progress >= 1.0) {
            if (state.state == State::HOMING) {
                is_home_ = true;
            } else if (state.state == State::OPERATING) {
                count_++;
                if (count_ == 10) {
                    count_ = 0;
                    is_finished_ = true;
                }
            }
        }
    }
}

void micros::OpenChainRobotController::control(joint_state_t& joint_command)
{
    
}

void micros::OpenChainRobotController::update(const joint_state_t& joint_state)
{

}