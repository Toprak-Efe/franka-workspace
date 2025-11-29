#include <atomic>
#include <cpptrace/basic.hpp>
#include <app.hpp>
#include <menu.hpp>

App::App() {

}

App::~App() {

}

void App::run(std::ofstream &log_file) {
    std::vector<std::string> menu{
        "󰭆 Robots",
        "󰈆 Exit"
    };

    std::atomic_bool running(true);
    while (running) {
        int selected = menu::select(menu);
        switch (selected) {
            case 0:
                break;
            case 1:
                running = false;
                break;
            default:
                log_file << "ERROR: Received unexpected value.\n" << cpptrace::generate_trace().to_string();
                exit(1);
                break;
        }
    }
}

