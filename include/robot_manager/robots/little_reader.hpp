#ifndef ROBOTS_LITTLE_READER_HPP_
#define ROBOTS_LITTLE_READER_HPP_

#include "robot_manager/core/robot.hpp"
#include "robot_manager/scheduler/fsm_scheduler.hpp"

namespace micros {

class LittleReader : public Robot {
public:
    LittleReader(const robot_config_t& config);
    
    ~LittleReader() override = default;

    void initialize() override = 0;

    void control(joint_state_t& joint_command) override = 0;

    void update(const joint_state_t& joint_state) override = 0;

private:
    std::unique_ptr<Scheduler> scheduler_;

    std::unique_ptr<Planner> planner_;

    fsm_state_t current_fsm_state_{State::STOPPED, 0.0};
};

} // namespace micros
#endif // ROBOTS_OPEN_CHAIN_ROBOT_CONTROLLER_HPP_