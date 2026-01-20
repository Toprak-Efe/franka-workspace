#include <HD/hd.h>
#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>

#include <array>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

volatile std::atomic_bool g_run{true};

void signal_handler() {
    g_run.store(false, std::memory_order_relaxed);
}

typedef std::array<HDdouble, 3> vec3;

vec3 haptic_force_field(const vec3 &pos) {
    return {0.01, 0.0, 0.0};
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
    using namespace std::chrono_literals;
    HHD device_id = hdInitDevice("Right Device");
    hdEnable(HD_FORCE_OUTPUT); 
    HDSchedulerHandle scheduler_handle = hdScheduleAsynchronous(haptic_handler, (void *)0, HD_DEFAULT_SCHEDULER_PRIORITY); 
    hdStartScheduler();

    while (g_run) {
        std::this_thread::sleep_for(1s);
    }

    hdUnschedule(scheduler_handle);
    hdStopScheduler();
    hdDisableDevice(device_id); 
    return 0;
}
