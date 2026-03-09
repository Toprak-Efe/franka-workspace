#pragma once

#include <HD/hd.h>
#include <string>

HDCallbackCode HDCALLBACK haptic_callback(void *buffer);

namespace Asclepius {

struct HapticBuffer {}; // struct HapticBuffer

template <bool TelemetryEnabled> class HapticServoLoop {
public:
    HapticServoLoop(const std::string &devicename) {

    }
    void start() {}
    void stop() {}
private:
}; // class HapticServoLoop;

}; // namespace Asclepius
