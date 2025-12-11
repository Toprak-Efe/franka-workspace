#include <atomic>
#include <cpptrace/basic.hpp>
#include <app.hpp>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <fstream>
#include <ios>
#include <menu.hpp>
#include <filesystem>
#include <config.hpp>

Asclepius::App::App(std::ofstream &log_file) :
    m_log_file(log_file),
    m_robot1(g_configuration["ROBOT1_HOSTNAME"]),
    m_robot2(g_configuration["ROBOT2_HOSTNAME"])
{
    m_robot1.read([&](const franka::RobotState &state) {
        for (size_t i = 0; i < 7; i++) {
            m_robot_poses[0][i] = state.q[i];
        }
        return m_running ? false : true;
    }); 
    m_robot2.read([&](const franka::RobotState &state) {
        for (size_t i = 0; i < 7; i++) {
            m_robot_poses[1][i] = state.q[i];
        }
        return m_running ? false : true;
    }); 
}

Asclepius::App::~App() {
    m_robot1.stop(); 
    m_robot2.stop(); 
}

void Asclepius::App::initialize() {

}


std::vector<std::string> getNetIfaces() {
    std::vector<std::string> out(1);
    std::filesystem::path network_ifaces_path("/sys/class/net/");
    if (!std::filesystem::is_directory(network_ifaces_path)) return out;
    for (const auto &dir : std::filesystem::directory_iterator{network_ifaces_path}) {
        std::string entry = dir.path().stem();
        std::ifstream type_file{dir.path() / "type", std::ios_base::in};
        int network_iface_type = -1;
        type_file >> network_iface_type;
        type_file.close();
        if (network_iface_type < 0) continue;
        if (std::filesystem::exists(dir.path() / "wireless")) {
            entry = "󰖩 " + entry;
        } else {
            switch (network_iface_type) {
                case 1:
                    entry = "󰈀 " + entry;
                    break;
                case 2:
                    entry = "󰈀 " + entry;
                    break;
                case 772:
                    entry = " " + entry;
                    break;
                default:
                    continue;
            }
        }
        out.emplace_back(entry);
    }
    return out;
}

void Asclepius::App::run() {

    while (m_running) {
         
    }
}

