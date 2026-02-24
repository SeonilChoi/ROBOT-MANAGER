#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "robot_manager/robot_manager.hpp"

micros::RobotManager::RobotManager(const std::string& config_file)
{
    load_configurations(config_file);
    initialize_robot_manager();
}

void micros::RobotManager::load_configurations(const std::string& config_file)
{
    YAML::Node root = YAML::LoadFile(config_file);
    if (!root) {
        throw std::runtime_error("Failed to load configuration file.");
    }

    uint8_t robot_size = root["number_of_robots"].as<uint8_t>();

    YAML::Node robots = root["robots"];
    if (!robots || !robots.IsSequence()) {
        throw std::runtime_error("Invalid robots configuration.");
    }

    robots_.reserve(robot_size);

    for (const auto& r : robots) {
        robot_config_t r_cfg{};
        r_cfg.id = r["id"].as<uint8_t>();

        robots_.push_back(std::make_unique<RobotController>(r_cfg));
    }
}

void micros::RobotManager::initialize_robot_manager()
{
    for (auto& robot : robots_) {
        robot->initialize();
    }
}

void micros::RobotManager::control(motor_state_t* cmds, uint8_t& size)
{   
    if (!is_home_) {
        is_home_ = set_all_robot_home();
    } else if (!is_finished_) {
        is_finished_ = set_all_robot_move();
    } else {
        set_all_robot_home();
    }

    for (auto& robot : robots_) {
        joint_state_t joint_command{};
        robot->control(joint_command);
        for (uint8_t i = 0; i < joint_command.number_of_joints; ++i) {
            cmds[size].target_id[0] = joint_command.target_id[i];
            cmds[size].number_of_targets = 1;
            cmds[size].controller_idx = joint_command.controller_idx[i];
            cmds[size].position = joint_command.position[i];
            cmds[size].velocity = joint_command.velocity[i];
            cmds[size].torque = joint_command.torque[i];
            size++;
        }
    }
}

void micros::RobotManager::update(const motor_state_t* states, const uint8_t size)
{
    for (auto& robot : robots_) {
        joint_state_t joint_state{};
        const uint8_t number_of_joints = robot->number_of_joints();
        const uint8_t* controller_idxs = robot->controller_idxs();

        joint_state.number_of_joints = number_of_joints;
        for (uint8_t i = 0; i < number_of_joints; ++i) {
            joint_state.controller_idx[i] = controller_idxs[i];
            joint_state.position[i] = states[controller_idxs[i]].position;
            joint_state.velocity[i] = states[controller_idxs[i]].velocity;
            joint_state.torque[i] = states[controller_idxs[i]].torque;
        }

        robot->update(joint_state);
    }
}

bool micros::RobotManager::set_all_robot_home()
{
    for (uint8_t i = 0; i < robots_.size(); ++i) {
        if (robots_[i]->is_home()) {
            robots_[i]->set_action(fsm_action_t{Action::STOP, 0.0});
        } else if (i == robot_idx_) {
            robots_[i]->set_action(fsm_action_t{Action::HOME, 10.0});
        } else {
            robots_[i]->set_action(fsm_action_t{Action::STOP, 0.0});
        }
    }

    if (robots_[robot_idx_]->is_home()) robot_idx_++;

    if (robot_idx_ == robots_.size()) {
        robot_idx_ = 0;
        return true;
    }
    return false;
}

bool micros::RobotManager::set_all_robot_move()
{
    uint8_t count = 0;
    for (auto& robot : robots_) {
        if (robot->is_finished()) {
            robot->set_action(fsm_action_t{Action::STOP, 0.0});
            count++;
        } else {
            robot->set_action(fsm_action_t{Action::MOVE, 10.0});
        }
    }

    if (count == robots_.size()) {
        return true;
    }
    return false;
}