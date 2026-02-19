#ifndef ROBOT_MANAGER_ROBOT_MANAGER_HPP_
#define ROBOT_MANAGER_ROBOT_MANAGER_HPP_

namespace micros {

class RobotManager {
public:
    RobotManager(const std::string& config_file);

    ~RobotManager() = default;

private:
    void load_configurations(const std::string& config_file);

    void initialize();
};

} // namespace micros
#endif // ROBOT_MANAGER_ROBOT_MANAGER_HPP_