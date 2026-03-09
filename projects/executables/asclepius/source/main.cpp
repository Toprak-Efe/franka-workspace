#include <config.hpp>
#include <control.hpp>
#include <csignal>
#include <cstdio>
#include <logging.hpp>
#include <shutdown.hpp>
#include <unistd.h>

using namespace Asclepius;

void signal_handler(int) { g_shutdown_coordinator.shutdown(); }

int main() {
  signal(SIGINT, signal_handler);

  try {
    BilateralControl<true> controller{g_configuration["ROBOT1_HOSTNAME"],
                                      g_configuration["HAPTIC1_DEVICENAME"]};
    controller.start();

    g_shutdown_coordinator.await_shutdown();
  } catch (std::exception &e) {
    auto &log = Logger::get();
    log.error("Received exception {}, exiting.", e.what());
    return 1;
  }
}
