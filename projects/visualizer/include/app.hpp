#ifndef APP_HPP
#define APP_HPP

#include <atomic>
#include <fstream>
#include <franka/robot.h>
#include <thread>

namespace Asclepius {

class App {
public:
    App(std::ofstream &m_log_file);
    App(App &&) = delete; 
    App(const App &) = delete; 
    App &operator=(App &&) = delete;
    App &operator=(const App &) = delete;
    ~App();

    void initialize();
private:
    void run();
    
    std::thread m_app_main_thread;
    std::shared_ptr<franka::Robot> m_robot1;
    std::shared_ptr<franka::Robot> m_robot2;
    std::atomic<double> m_robot_state_poses[2][7];
    std::atomic<double> m_robot_command_poses[2][7];
    std::atomic_bool m_running;
    std::ofstream &m_log_file; 
}; // class App

}; // namespace Asclepius

#endif//APP_HPP
