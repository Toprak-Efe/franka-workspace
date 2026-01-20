#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <iostream>
#include <chrono>
#include <cpptrace/basic.hpp>
#include <exception>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <fstream>
#include <memory>
#include <eigen3/Eigen/Dense>

#include "../include/app.hpp"
#include "../include/config.hpp"

#include <HD/hd.h>
#include <thread>
#include <fmt/format.h>

HDCallbackCode HDCALLBACK haptic_callback(void *data) {
    using namespace Asclepius;
    system_data_t *system_data = (system_data_t *) data;
    std::array<HDdouble, 16> transform;
    hdGetDoublev(HD_CURRENT_TRANSFORM, (HDdouble *) transform.data());
    std::array<HDdouble, 3> linear_velocity;
    std::array<HDdouble, 3> angular_velocity;
    hdGetDoublev(HD_CURRENT_VELOCITY, linear_velocity.data());
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, angular_velocity.data());
    system_data->haptic_state.add({
       .transform{transform},
       .linear_velocity{linear_velocity[0], linear_velocity[1], linear_velocity[2]},
       .angular_velocity{angular_velocity[0], angular_velocity[1], angular_velocity[2]}
    });
    return HD_CALLBACK_DONE;
}

Asclepius::App::App(std::ofstream &log_file) :
    m_log_file(log_file)
{
}

Asclepius::App::~App() {
    for (auto robot : m_robots)
        robot->stop();
    m_app_main_thread.join();
}

void Asclepius::App::initialize() {
    using namespace Asclepius;
    for (size_t idx = 0; idx < 2; idx++) {
        _initialize_haptic(idx);
        _initialize_robot(idx);
    }
    run();
}

void Asclepius::App::_initialize_robot(size_t idx) {
    try {
        std::string hostname = g_configuration[fmt::format("ROBOT{:d}_HOSTNAME", idx)];
        m_robots[idx] = std::make_shared<franka::Robot>(hostname);
        auto &robot = m_robots[idx];

        franka::Model robot_model = robot->loadModel(); 
        franka::RobotState initial_state = robot->readOnce();
        
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d position_d(initial_transform.translation());
        Eigen::Quaterniond orientation_d(initial_transform.rotation());

        std::thread robot_thread{[&](){
             
        }};

        robot->setCollisionBehavior({{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

        robot->control([&](const franka::RobotState &f_state, franka::Duration dur) {
            Asclepius::RobotState a_robot_state{
                .transform{f_state.EE_T_K},
                .joint_velocities{f_state.dq},
                .joint_positions{f_state.q},
                .joint_contacts{f_state.joint_contact}
            };
            system_data[idx].robot_state.add(a_robot_state);
            return franka::CartesianPose{};
        });
    } catch (std::exception &e) {
        std::cout << e.what() << "\n"; 
    }
}
    
void Asclepius::App::_initialize_haptic(size_t idx) {
    try {
        std::string hostname = fmt::format("HAPTIC{:d}_DEVICENAME", idx);
        HDstring haptic_devicename = g_configuration[hostname].c_str();
        HHD device_id = hdInitDevice(haptic_devicename);
        hdMakeCurrentDevice(device_id);
        if (!hdIsEnabled(HD_FORCE_OUTPUT)) {
            hdEnable(HD_FORCE_OUTPUT);
        }

        hdScheduleAsynchronous(::haptic_callback,
            &system_data[idx],
            HD_DEFAULT_SCHEDULER_PRIORITY
        );
    } catch (std::exception &e) {
        std::cout << e.what() << "\n";
    }
}


void Asclepius::App::run() {
    using namespace std::chrono_literals;
    
    RobotState robot_state;
    RobotCommand robot_command;
    HapticState haptic_state;
    HapticCommand haptic_command;

    while (m_running) {
        std::this_thread::sleep_for(10ms); // 100 Hz 
        for (system_data_t &data : system_data) {
            data.robot_state.get(robot_state);
            data.haptic_state.get(haptic_state);

            // calculate commands 
            
            data.haptic_command.add(haptic_command);
            data.robot_command.add(robot_command);
        }
    }
}

