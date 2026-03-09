#include <HD/hd.h>
#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <HDU/hduError.h>

#include <array>
#include <thread>
#include <atomic>
#include <csignal>
#include <cmath>

using namespace std::chrono_literals;

volatile std::atomic_bool g_run{true};

void signal_handler(int signal) {
    g_run.store(false, std::memory_order_relaxed);
}

typedef std::array<HDdouble, 3> vec3;

vec3 haptic_force_field(const vec3 &pos) {
    return {1.0, std::cos(pos[1]*0.1)*0.6, 0.0};
}

HDCallbackCode HDCALLBACK haptic_handler(void *data) {
    HHD hHD = hdGetCurrentDevice();

    hdBeginFrame(hHD);
    vec3 pos;
    hdGetDoublev(HD_CURRENT_POSITION, pos.data());
    vec3 force = haptic_force_field(pos);
    hdSetDoublev(HD_CURRENT_FORCE, force.data());
    hdEndFrame(hHD);
    
    if (g_run.load(std::memory_order_relaxed)) {
        return HD_CALLBACK_CONTINUE;
    } else {
        return HD_CALLBACK_DONE;
    }
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTSTP, signal_handler);
    std::signal(SIGQUIT, signal_handler);

    HDErrorInfo error;

    HHD hhd = hdInitDevice("Left Haptic");
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to initialize device.");
        return -1;
    }

    hdEnable(HD_FORCE_OUTPUT); 
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Failed to start schedule.");
        return -1;
    }
    
    HDSchedulerHandle scheduler_handle = hdScheduleAsynchronous(
            haptic_handler, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    while (g_run) {
        std::this_thread::sleep_for(100ms);
    }
    hdWaitForCompletion(scheduler_handle, HD_WAIT_CHECK_STATUS);

    hdStopScheduler();
    hdUnschedule(scheduler_handle);
    hdDisableDevice(hhd); 

    return 0;
}
