#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/robot_state.h>

#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/screen.hpp>
#include <ftxui/screen/string.hpp>

#define ROBOT_HOSTNAME "127.0.0.1"

int main(int argc, char *argv[]) {
    using namespace ftxui;

    std::

    Element interface;
    auto screen = Screen::Create(
        Dimension::Full(),
        Dimension::Fit(interface)
    );

    try {
        franka::Robot robot(ROBOT_HOSTNAME);
        robot.read([&](const franka::RobotState &state) {

            return false; 
        });
    } catch (franka::Exception &e) {
        
    }
    return 0;
}
