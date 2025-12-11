#ifndef APP_HPP
#define APP_HPP

#include <atomic>
#include <fstream>
#include <franka/robot.h>

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
    void run();
private:
    std::ofstream &m_log_file; 
    franka::Robot m_robot1;
    franka::Robot m_robot2;
    std::atomic<double> m_robot_poses[2][7];
    std::atomic_bool m_running;
}; // class App

}; // namespace Asclepius

#endif//APP_HPP
