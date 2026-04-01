#include <bilateralcontrol/haptic.hpp>
#include <bilateralcontrol/shutdown.hpp>
#include <chrono>
#include <csignal>
#include <realtimeloop/monoloop.hpp>
#include <stop_token>
#include <thread>
#include <toml.hpp>

void signal_handler(int signo) { Asclepius::ShutdownCoordinator::instance().shutdown(); }

int main(int argc, char *argv[]) {
  signal(SIGINT, &signal_handler);
  signal(SIGTERM, &signal_handler);

  auto config = toml::parse("~/.config/asclepius/haptic_test.toml");
  const std::string device_name = toml::find<std::string>(config, "device_name");

  struct {
    SignalPort<Asclepius::TwistDisplacement> sensor_port{"master/sensors"};
    SignalPort<Wrench> command_port{"master/commands"};
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
