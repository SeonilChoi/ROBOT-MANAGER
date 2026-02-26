#include "robot_manager/robots/little_reader.hpp"

void micros::LittleReader::initialize()
{
    switch (scheduler_type_) {
    case SchedulerType::FSM: {
        scheduler_ = std::make_unique<FsmScheduler>(0.01);
        break;
    } default: {
        throw std::runtime_error("Unsupported scheduler type: " + std::to_string(scheduler_type_));
    }
    }

    switch (planner_type_) {
    case PlannerType::RRT: {
        planner_ = std::make_unique<RrtPlanner>();
        break;
    } default: {
        throw std::runtime_error("Unsupported planner type: " + std::to_string(planner_type_));
    }
    }
    planner_->run();
}

void micros::LittleReader::control(joint_state_t& joint_command)
{
    if (!is_home_) {
        home();
    } else if (!is_moved_) {
        move();
    } else if (!is_stopped_) {
        stop();
    }

    if (planner_->is_planned()) {
        planner_->eval(current_fsm_state_.progress, joint_command);
        scheduler_->step();
    }
}

void micros::LittleReader::update(const joint_state_t& joint_state)
{

}

void micros::LittleReader::home()
{
    bool is_event = scheduler_->tick(fsm_action_t{Action::HOME, 10.0}, current_fsm_state_);

    if (is_event) {
        if (current_fsm_state_.progress == 1.0) {
            count_++;
        } else if (current_fsm_state_.progress != 0.0) {
            joint_state_t current_joint_state{}, target_joint_state{};
            if (count_ == 0) {
                current_joint_state.number_of_joints = 2;
                current_joint_state.position[0] = current_robot_state_.joint_state.position[0];
                current_joint_state.position[1] = current_robot_state_.joint_state.position[1];
                target_joint_state.number_of_joints = 2;
                target_joint_state.position[0] = home_robot_state_.joint_state.position[0];
                target_joint_state.position[1] = home_robot_state_.joint_state.position[1];
            } else if (count_ == 1) {
                current_joint_state.number_of_joints = 2;
                current_joint_state.position[0] = current_robot_state_.joint_state.position[2];
                current_joint_state.position[1] = current_robot_state_.joint_state.position[3];
                target_joint_state.number_of_joints = 2;
                target_joint_state.position[0] = home_robot_state_.joint_state.position[2];
                target_joint_state.position[1] = home_robot_state_.joint_state.position[3];
            }
            planner_->plan(current_joint_state, target_joint_state, obstacle_state_);
        }
    }

    if (count_ >= max_home_cycles_) {
        is_home_ = true;
    }
}

void micros::LittleReader::move()
{
    bool is_event = scheduler_->tick(fsm_action_t{Action::MOVE, 10.0}, current_fsm_state_);

    if (is_event) {
        if (current_fsm_state_.progress == 1.0) {
            count_++;
        } else if (current_fsm_state_.progress != 0.0) {
            joint_state_t current_joint_state{}, target_joint_state{};
            if (count_ == 0) {

            }
        }
    }

    if (count_ >= max_cycles_) {
        is_moved_ = true;
    }
}

void micros::LittleReader::stop()
{
    bool is_event = scheduler_->tick(fsm_action_t{Action::STOP, 0.0}, current_fsm_state_);

    if (is_event) {
        is_stopped_ = true;
        if (is_operating_) is_home_ =false;
    }
}