#include <bilateralcontrol/common.hpp>
#include <bilateralcontrol/franka.hpp>
#include <bilateralcontrol/haptic.hpp>
#include <bilateralcontrol/shutdown.hpp>
#include <csignal>
#include <iostream>
#include <pwd.h>
#include <toml.hpp>

struct FrankaPorts {
  FrankaPorts() = delete;
  FrankaPorts(const std::string &channel_prefix)
      : sensor_port(channel_prefix + "/Fm_Pm"), command_port(channel_prefix + "/Vm") {};
  SignalPort<Asclepius::WrenchDisplacement> sensor_port;
  SignalPort<Twist> command_port;
}; // struct FrankaPorts

struct HapticPorts {
  HapticPorts() = delete;
  HapticPorts(const std::string &channel_prefix)
      : sensor_port(channel_prefix + "/Vm_Pm"), command_port(channel_prefix + "/Fm") {};
  SignalPort<Asclepius::TwistDisplacement> sensor_port;
  SignalPort<Wrench> command_port;
}; // struct HapticPorts

void signal_handler(int signo) { Asclepius::ShutdownCoordinator::instance().shutdown(); }

int main() {
  signal(SIGINT, &signal_handler);
  signal(SIGTERM, &signal_handler);

  try {
    const char *home_env = ::getenv("HOME");
    if (home_env == nullptr) {
      struct passwd *pw = ::getpwuid(::getuid());
      home_env = pw->pw_dir;
    }
    std::filesystem::path config_path{std::filesystem::path(std::string{home_env}) / ".config" /
                                      "asclepius" / "asclepius.toml"};
    auto config = toml::parse(config_path);
    const std::string left_haptic_name = config["left"]["haptic_device"].as_string();
    const std::string left_franka_host = config["left"]["franka_host"].as_string();
    const std::string right_haptic_name = config["right"]["haptic_device"].as_string();
    const std::string right_franka_host = config["right"]["franka_host"].as_string();




  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what();
  }
}
