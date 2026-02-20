#ifndef ROBOT_MANAGER_ROBOT_MANAGER_HPP_
#define ROBOT_MANAGER_ROBOT_MANAGER_HPP_

#include <string>
#include <memory>
#include <vector>

namespace micros {

class RobotManager {
public:
    RobotManager(const std::string& config_file);

    ~RobotManager() = default;

    void control(motor_state_t* cmds, uint8_t& size);

    void update(const motor_state_t* states, const uint8_t size);

private:
    void load_configurations(const std::string& config_file);

    void initialize_robot_manager();

    std::vector<std::unique_ptr<RobotController>> controllers_;
};

} // namespace micros
#endif // ROBOT_MANAGER_ROBOT_MANAGER_HPP_