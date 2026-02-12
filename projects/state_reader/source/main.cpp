#include <string>
#include <iostream>
#include <franka/robot.h>
#include <franka/robot_state.h>

std::string mode_to_str(franka::RobotMode m) {
    switch (m) {
        case franka::RobotMode::kOther:
            return "kOther";
        case franka::RobotMode::kIdle:
            return "kIdle";
        case franka::RobotMode::kMove:
            return "kMove";
        case franka::RobotMode::kGuiding:
            return "kGuiding";
        case franka::RobotMode::kReflex:
            return "kReflex";
        case franka::RobotMode::kUserStopped:
            return "kUserStopped";
        case franka::RobotMode::kAutomaticErrorRecovery:
            return "kAutomaticErrorRecover";
        default:
            return "INVALID";
    }
}

int main() {
    std::string hostname = "172.16.0.2";
    try {
        franka::Robot panda1{hostname};
        franka::RobotState state = panda1.readOnce();
        std::cout << "Mode: " << mode_to_str(state.robot_mode) << "\n";
    } catch (std::exception &e) {
        std::cerr << "Error: " << e.what();
    } 
}
