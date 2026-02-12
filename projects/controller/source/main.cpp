#include "fmt/core.h"
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <HDU/hduError.h>
#include <config.hpp>
#include <csignal>
#include <exception.hpp>
#include <pipeline.hpp>
#include <shutdown.hpp>

void signal_handler(int) { Asclepius::g_shutdown_coordinator.shutdown(); }

int main() {

  struct {
    std::unique_ptr<Asclepius::Pipeline> left_pipeline;
    std::unique_ptr<Asclepius::Pipeline> right_pipeline;
  } g_pipelines;

  std::string left_franka_name(Asclepius::g_configuration["ROBOT1_HOSTNAME"]);
  std::string left_haptic_name(
      Asclepius::g_configuration["HAPTIC1_DEVICENAME"]);
  std::string right_franka_name(Asclepius::g_configuration["ROBOT2_HOSTNAME"]);
  std::string right_haptic_name(
      Asclepius::g_configuration["HAPTIC2_DEVICENAME"]);

  signal(SIGINT, signal_handler);

  try {
    g_pipelines.left_pipeline = std::make_unique<Asclepius::Pipeline>(
        left_franka_name, left_haptic_name);
    g_pipelines.right_pipeline = std::make_unique<Asclepius::Pipeline>(
        right_franka_name, right_haptic_name);

    HDErrorInfo error; 
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        throw Asclepius::AsclepiusException("Failed to start haptics scheduler.");
    }

    g_pipelines.left_pipeline->start();
    g_pipelines.right_pipeline->start();

    Asclepius::g_shutdown_coordinator.await_shutdown();

    hdStopScheduler();

    g_pipelines.left_pipeline->stop();
    g_pipelines.right_pipeline->stop();
    
  } catch (Asclepius::AsclepiusException &error) {
    fmt::print("Asclepius Exception: {}", error.what());
  }

  return 0;
}
