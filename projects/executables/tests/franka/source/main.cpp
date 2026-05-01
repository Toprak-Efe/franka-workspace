#include "bilateralcontrol/common.hpp"
#include "realtimeloop/monoloop.hpp"
#include <bilateralcontrol/franka.hpp>
#include <bilateralcontrol/shutdown.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <franka/exception.h>
#include <iostream>
#include <pwd.h>
#include <stdexcept>
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
                                      "asclepius" / "franka_test.toml"};
    auto config = toml::parse(config_path);
    auto config_general = config["general"];
    auto config_parameters = config["parameters"];
    const std::string device_name = toml::find<std::string>(config_general, "device_name");
    const std::string device_ip = toml::find<std::string>(config_general, "device_ip");
    double amplitude = toml::find<double>(config_parameters, "amplitude");
    double frequency = toml::find<double>(config_parameters, "frequency");

    struct {
      SignalPort<Asclepius::WrenchDisplacement> sensor_port{"FPm"};
      SignalPort<Twist> command_port{"Vm"};
    } Ports;

    Asclepius::FrankaDevice device{device_name, device_ip, &Ports.sensor_port, &Ports.command_port};
    device.start();
    std::this_thread::sleep_for(80ms);

    double time = 0.0;
    std::jthread controller([&](std::stop_token st) {
      Asclepius::Monoloop<1'000'000> loop(st, [&]() {
        time += 0.001;
        double v_x = ::cos(time * M_PI * 2.0 * frequency) * amplitude;
        Twist command{Twist::Vector6{v_x, 0.0, 0.0, 0.0, 0.0, 0.0},
                      std::chrono::steady_clock::now()};
        // command = Twist{{0.0}, stamp};
        Ports.command_port.push(command);
      });
    });

    Asclepius::ShutdownCoordinator::instance().await_shutdown();

    device.stop();
    controller.request_stop();
    if (controller.joinable()) {
      controller.join();
    }
  } catch (franka::Exception &e) {
    std::cerr << "Caugh Franka exception! Exiting with error: " << e.what();
  } catch (std::exception &e) {
    std::cerr << "Caugh exception! Exiting with error: " << e.what();
  } catch (...) {
    auto e = std::current_exception();
    std::cerr << "Caught unknown exception! Exiting.";
  }
}
