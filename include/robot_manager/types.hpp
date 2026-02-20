#ifndef ROBOT_MANAGER_TYPES_HPP_
#define ROBOT_MANAGER_TYPES_HPP_

#include <cstdint>

namespace micros {

inline constexpr uint8_t MAX_JOINT_SIZE = 16; // Maximum number of joints.

enum class State {
    STOPPED,
    OPERATING,
    HOMING,
    INVALID
};

enum class Action {
    STOP,
    MOVE,
    HOME
};

struct fsm_state_t {
    State state{State::STOPPED};
    double progress{0.0};
};

struct fsm_action_t {
    Action action{Action::STOP};
    double duration{0.0};
};

struct robot_config_t {

};

struct position_t {
    double x;
    double y;
    double z;
};

struct orientation_t {
    double roll;
    double pitch;
    double yaw;
};

struct pose_t {
    position_t position;
    orientation_t orientation;
}

struct twist_t {
    position_t linear;
    orientation_t angular;
}

struct wrench_t {
    position_t force;
    orientation_t torque;
}

struct joint_state_t {
    uint8_t number_of_joints;
    uint8_t target_id[MAX_JOINT_SIZE];
    uint8_t controller_idx[MAX_JOINT_SIZE];
    double position[MAX_JOINT_SIZE];
    double velocity[MAX_JOINT_SIZE];
    double torque[MAX_JOINT_SIZE];
};

struct robot_state_t {
    uint8_t id; // Robot ID.
    pose_t pose;
    twist_t twist;
    wrench_t wrench;
    joint_state_t joint_state;
};

constexpr std::array<std::array<State, 3>, 3> transition_table{{
    {State::STOPPED, State::OPERATING, State::HOMING},
    {State::STOPPED, State::OPERATING, State::INVALID},
    {State::STOPPED, State::HOMING,    State::INVALID}
}};

}
#endif // ROBOT_MANAGER_TYPES_HPP_