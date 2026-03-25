#include <snippets/control/physics.hpp>
#include <HD/hd.h>
#include <bilateralcontrol/haptic.hpp>
#include <bilateralcontrol/shutdown.hpp>
#include <csignal>
#include <realtimeloop/monoloop.hpp>
#include <stop_token>
#include <thread>

void signal_handler(int signo) { Asclepius::ShutdownCoordinator::get().shutdown(); }

int main(int argc, char *argv[]) {
  signal(SIGINT, &signal_handler);
  signal(SIGTERM, &signal_handler);

  const char *device_name = "HAPTIC1_DEVICENAME";
  Asclepius::HapticServoLoop<true> haptic_loop{device_name};
  Asclepius::ShutdownCoordinator &coordinator = Asclepius::ShutdownCoordinator::get();
  haptic_loop.start(coordinator);

  std::jthread millisecond_thread = std::jthread([&](std::stop_token stoken) {
    Asclepius::Monoloop<std::function<void()>, 1'000'000> millisecond_loop{
        stoken, [&]() {
          auto &buffer = haptic_loop.getBuffer();

          Asclepius::TwistDisplacement td_out;
          buffer.tdbuff.get(td_out);
        }};
  });

  return 0;
}
