#include <bilateralcontrol/haptic.hpp>
#include <bilateralcontrol/shutdown.hpp>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
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
  signal(SIGINT, &signal_handler);
  signal(SIGTERM, &signal_handler);

  const char *home_env = ::getenv("HOME");
  if (home_env == nullptr) {
    struct passwd *pw = ::getpwuid(::getuid());
    home_env = pw->pw_dir;
  }
  std::filesystem::path config_path{std::filesystem::path(std::string{home_env}) / ".config" / "asclepius" / "haptic_test.toml"};
  auto config = toml::parse(config_path);
  const std::string device_name = toml::find<std::string>(config, "device_name");

  struct {
    SignalPort<Asclepius::TwistDisplacement> sensor_port{"Vm"};
    SignalPort<Wrench> command_port{"Fm"};
  } Ports;

  Asclepius::HapticDevice device{device_name, &Ports.sensor_port, &Ports.command_port};
  device.start();
  std::jthread controller{[&](std::stop_token st) {
    Asclepius::Monoloop<1'000'000> loop{
        st, [&]() {
          auto stamp = std::chrono::steady_clock::now();
          Wrench command{std::array<double, 6>{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, stamp};
          Ports.command_port.push(command);
        }};
  }};
  Asclepius::ShutdownCoordinator::instance().await_shutdown();
  device.stop();
  controller.request_stop();
  if (controller.joinable()) {
    controller.join();
  }
}
