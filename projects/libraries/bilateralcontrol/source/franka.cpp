#include "tlib/control/spatial.hpp"
#include <bilateralcontrol/common.hpp>
#include <bilateralcontrol/franka.hpp>
#include <chrono>
#include <exception>
#include <franka/robot.h>
#include <stdexcept>
#include <stop_token>

Asclepius::FrankaDevice::FrankaDevice(const std::string &device_name, const std::string &host_name,
                                      SignalPort<Asclepius::WrenchDisplacement> *sensor_port,
                                      SignalPort<Twist> *command_port)
    : DeviceInterface<Asclepius::WrenchDisplacement, Twist>(sensor_port, command_port),
      state_(State::Idle), flock_("asclepius", device_name) {}

void Asclepius::FrankaDevice::start() {
  auto expected = State::Idle;
  if (!state_.compare_exchange_strong(expected, State::Running)) {
    throw std::logic_error{"Franka: cannot run when not idle."};
  }

  worker_ = std::jthread([&](std::stop_token) {
    try {
      franka::Robot robot{host_name_};
      robot.setCollisionBehavior(
          {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
          {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
          {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
          {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
          {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
          {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
      robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
      robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
      robot.control([&](const franka::RobotState &state,
                        franka::Duration duration) -> franka::CartesianVelocities {
        auto timestamp = std::chrono::steady_clock::now();

        std::array<double, 6> force = state.O_F_ext_hat_K;
        std::array<double, 6> pose = Eigen::Map<std::array<double, 6>>(
            Eigen::Map<Eigen::Matrix4d>(state.EE_T_K) * Eigen::Vector4d{{0.0, 0.0, 0.0, 1.0}});
        Asclepius::WrenchDisplacement sensor{Wrench{force}, Displacement{pose}};
      });

    } catch (std::exception &e) {
    }
  });
}
