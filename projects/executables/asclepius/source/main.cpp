#include "../include/config.hpp"
#include <bilateralcontrol/shutdown.hpp>
#include <bilateralcontrol/synclogger.hpp>
#include <bilateralcontrol/system.hpp>
#include <csignal>
#include <exception>

using namespace Asclepius;

void signal_handler(int) { ShutdownCoordinator::get().shutdown(); }

int main() {
  signal(SIGINT, signal_handler);

  constexpr bool TelemetryOn{true};
  System<TelemetryOn> controller{g_configuration["ROBOT1_HOSTNAME"],
                                 g_configuration["HAPTIC1_DEVICENAME"]};
  controller.start();
  ShutdownCoordinator::get().await_shutdown();
  controller.stop();

  auto excptr = ShutdownCoordinator::get().get_shutdown_exception();
  if (!excptr)
    return 0;
  try {
    std::rethrow_exception(*excptr);
  } catch (std::exception &e) {
    SyncLogger::get().info("Exception thrown, exiting: %s\n", e.what());
    return 1;
  }
}
