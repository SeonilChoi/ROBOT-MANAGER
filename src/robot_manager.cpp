#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "robot_manager/robot_manager.hpp"

micros::RobotManager::RobotManager(const std::string& config_file)
{
    load_configurations(config_file);
    initialize();
}

void micros::RobotManager::load_configurations(const std::string& config_file)
{
    YAML::Node root = YAML::LoadFile(config_file);
    if (!root) {
        throw std::runtime_error("Failed to load configuration file.");
    }

    YAML::Node robot = root["robot"];
    if (!robot || !robot.IsMap()) {
        throw std::runtime_error("Invalid robot configuration.");
    }

    robot_config_t r_cfg{};
    r_cfg.id = robot["id"].as<uint8_t>();
    r_cfg.number_of_joints = robot["number_of_joints"].as<uint8_t>();
    for (uint8_t i = 0; i < r_cfg.number_of_joints; ++i) {
        r_cfg.controller_idxs[i] = robot["controller_idxs"][i].as<uint8_t>();
    }
    r_cfg.scheduler_type = to_scheduler_type(robot["scheduler_type"].as<std::string>());
    r_cfg.planner_type = to_planner_type(robot["planner_type"].as<std::string>());

    switch (to_robot_type(robot["type"].as<std::string>())) {
    case RobotType::LittleReader: {
        robot_ = std::make_unique<LittleReader>(r_cfg);
        break;
    }
    default: {
        throw std::runtime_error("Unsupported robot type: " + robot["type"].as<std::string>());
    }
    }
}

void micros::RobotManager::initialize()
{
    robot_->initialize();
}

void micros::RobotManager::control(motor_state_t* cmds, uint8_t& size)
{   
    joint_state_t joint_command{};
    robot_->control(joint_command);

    for (uint8_t i = 0; i < joint_command.number_of_joints; ++i) {
        cmds[size].number_of_targets = 1;
        cmds[size].target_id[0]      = joint_command.target_id[i];
        
        cmds[size].controller_idx    = joint_command.controller_idx[i];
        cmds[size].position          = joint_command.position[i];
        cmds[size].velocity          = joint_command.velocity[i];
        cmds[size].torque            = joint_command.torque[i];
        size++;
    }
}

void micros::RobotManager::update(const motor_state_t* states, const uint8_t size)
{
    joint_state_t joint_state{};
    const uint8_t number_of_joints = robot_->number_of_joints();
    const uint8_t* controller_idxs = robot_->controller_idxs();

    joint_state.number_of_joints = number_of_joints;
    for (uint8_t i = 0; i < number_of_joints; ++i) {
        joint_state.controller_idx[i] = controller_idxs[i];
        joint_state.position[i] = states[controller_idxs[i]].position;
        joint_state.velocity[i] = states[controller_idxs[i]].velocity;
        joint_state.torque[i] = states[controller_idxs[i]].torque;
    }
    robot_->update(joint_state);
}

void micros::RobotManager::set_robot_control_state(const uint8_t cs)
{
    if (cs == 0) { // STOP
        robot_->setup_stop();
    } else if (cs == 1) { // MOVE
        robot_->setup_move();
    } else if (cs == 2) { // HOME
        robot_->setup_home();
    } else if (cs == 3) { // OPERATING
        robot_->setup_operating();
    }
}