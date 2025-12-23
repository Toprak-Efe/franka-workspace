#include <app.hpp>
#include <cpptrace/basic.hpp>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <fstream>
#include <config.hpp>
#include <memory>

Asclepius::App::App(std::ofstream &log_file) :
    m_log_file(log_file)
{
    m_robot1 = std::make_shared<franka::Robot>(g_configuration["ROBOT1_HOSTNAME"]);
    m_robot2 = std::make_shared<franka::Robot>(g_configuration["ROBOT2_HOSTNAME"]);
}

Asclepius::App::~App() {
    m_robot1->stop(); 
    m_robot2->stop(); 
    m_app_main_thread.join();
}

void Asclepius::App::initialize() {
    m_robot1->read([&](const franka::RobotState &state) {
        for (size_t i = 0; i < 7; i++) {
            m_robot_state_poses[0][i] = state.q[i];
        }
        return m_running ? false : true;
    }); 
    m_robot2->read([&](const franka::RobotState &state) {
        for (size_t i = 0; i < 7; i++) {
            m_robot_state_poses[1][i] = state.q[i];
        }
        return m_running ? false : true;
    });
}


void Asclepius::App::run() {
    while (m_running) {
         
    }
}

