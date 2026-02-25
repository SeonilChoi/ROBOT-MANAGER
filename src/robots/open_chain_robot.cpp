#include "robot_manager/robots/open_chain_robot_controller.hpp"

micros::OpenChainRobotController::OpenChainRobotController(const robot_config_t& config)
: RobotController(config) {}

void micros::OpenChainRobotController::initialize()
{
    current_state_.id = id_;
}

void micros::OpenChainRobotController::set_action(const fsm_action_t& action)
{
    fsm_state_t state{};
    bool is_event = fsm_scheduler_.tick(action, state);
    current_progress_ = state.progress;

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
        } else if (state.progress != 0.0) {
            planner_.plan(current_state_, target_state_, obstacle_state_);
        }
    }
}

void micros::OpenChainRobotController::control(joint_state_t& joint_command)
{
    if (planner_.is_planned()) {
        planner_.eval(current_progress_, joint_command);
        fsm_scheduler_.step();
    }
}

void micros::OpenChainRobotController::update(const joint_state_t& joint_state)
{
    get_pose(joint_state, current_state_.pose);
    get_twist(joint_state, current_state_.twist);
    get_wrench(joint_state, current_state_.wrench);
    current_state_.joint_state = joint_state;
}

void micros::OpenChainRobotController::get_pose(const joint_state_t& joint_state, pose_t& pose)
{
    forward_kinematics(joint_state.position, joint_state.number_of_joints, pose.position, pose.orientation);
}

void micros::OpenChainRobotController::get_twist(const joint_state_t& joint_state, twist_t& twist)
{
    forward_jacobian();
}

void micros::OpenChainRobotController::get_wrench(const joint_state_t& joint_state, wrench_t& wrench)
{
    inverse_dynamics();
}