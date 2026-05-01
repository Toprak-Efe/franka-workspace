#include "bilateralcontrol/common.hpp"
#include "toml11/find.hpp"
#include <bilateralcontrol/haptic.hpp>
#include <bilateralcontrol/shutdown.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <pwd.h>
#include <realtimeloop/monoloop.hpp>
#include <stop_token>
#include <string>
#include <sys/types.h>
#include <thread>
#include <toml.hpp>
#include <unistd.h>

void signal_handler(int signo) { Asclepius::ShutdownCoordinator::instance().shutdown(); }

int main(int argc, char *argv[]) {
  using namespace std::chrono_literals;

  signal(SIGINT, &signal_handler);
  signal(SIGTERM, &signal_handler);

  try {
    const char *home_env = ::getenv("HOME");
    if (home_env == nullptr) {
      struct passwd *pw = ::getpwuid(::getuid());
      home_env = pw->pw_dir;
    }
    std::filesystem::path config_path{std::filesystem::path(std::string{home_env}) / ".config" /
                                      "asclepius" / "haptic_test.toml"};
    auto cfg = toml::parse(config_path);
    auto cfg_general = toml::find(cfg, "general");
    auto cfg_parameters = toml::find(cfg, "parameters");

    std::string device_name = toml::find<std::string>(cfg_general, "device_name");

    double dampening = toml::find<double>(cfg_parameters, "dampening");
    double mass = toml::find<double>(cfg_parameters, "mass");
    double amplitude = toml::find<double>(cfg_parameters, "amplitude");
    double spring = toml::find<double>(cfg_parameters, "spring");
    double tolerance = toml::find<double>(cfg_parameters, "tolerance");

    struct {
      SignalPort<Asclepius::TwistDisplacement, 1> sensor_port{"Vm"};
      SignalPort<Wrench> command_port{"Fm"};
    } Ports;

    Asclepius::HapticDevice device{device_name, &Ports.sensor_port, &Ports.command_port};
    device.start();
    std::this_thread::sleep_for(20ms);

    std::jthread controller{[&](std::stop_token st) {
      SignalPort<Twist> acceleration_record{"Am"};
      bool init = false;
      Displacement initial_pose{};
      Asclepius::Monoloop<1'000'000> loop{
          st, [&]() {
            auto stamp = std::chrono::steady_clock::now();
            Asclepius::TwistDisplacement state = Ports.sensor_port.sample();
            Displacement pose = state.get<1>();
            Twist twist = state.get<0>();

            if (!init) {
              init = true;
              initial_pose = state.get<1>();
              Wrench command{stamp};
              Ports.command_port.push(command);
              return;
            }

            Displacement dpose = (pose - initial_pose);
            Wrench wall_force = static_cast<Wrench>(dpose * -spring) *
                                Wrench{std::array<double, 6>{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};
            Wrench dampen_force = static_cast<Wrench>(-dampening * twist) *
                                  Wrench{std::array<double, 6>{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}};

            Wrench command =
                //       static_cast<Wrench>(spring * (initial_pose - pose));
                //       static_cast<Wrench>(-amplitude * (mass * acc + dampening * twist));
                dpose.vec().z() < 0.0 ? amplitude * (wall_force + dampen_force) : Wrench{};

            for (double &x : command.vec()) {
              if (x < tolerance && -tolerance > x) {
                x = 0;
              }
            }

            command.stamp() = stamp;
            Ports.command_port.push(command);
          }};
    }};

    Asclepius::ShutdownCoordinator::instance().await_shutdown();
  } catch (std::exception &e) {
    std::cerr << "Exiting main due to exception: " << e.what();
  }
}
