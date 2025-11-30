#include <algorithm>
#include <atomic>
#include <cpptrace/basic.hpp>
#include <app.hpp>
#include <cstdio>
#include <fstream>
#include <ios>
#include <menu.hpp>
#include <filesystem>
#include <ranges>

App::App() {

}

App::~App() {

}

std::vector<std::string> getNetIfaces() {
    std::vector<std::string> out(1);
    std::filesystem::path network_ifaces_path("/sys/class/net/");
    if (!std::filesystem::is_directory(network_ifaces_path)) return out;
    for (const auto &dir : std::filesystem::directory_iterator{network_ifaces_path}) {
        std::string entry = dir.path().stem();
        std::ifstream type_file{dir.path() / "type", std::ios_base::in};
        int network_iface_type = -1;
        type_file >> network_iface_type;
        type_file.close();
        if (network_iface_type < 0) continue;
        if (std::filesystem::exists(dir.path() / "wireless")) {
            entry = "󰖩 " + entry;
        } else {
            switch (network_iface_type) {
                case 1:
                    entry = "󰈀 " + entry;
                    break;
                case 2:
                    entry = "󰈀 " + entry;
                    break;
                case 772:
                    entry = " " + entry;
                    break;
                default:
                    continue;
            }
        }
        out.emplace_back(entry);
    }
    return out;
}

void App::run(std::ofstream &log_file) {
    std::vector<std::string> menu{
        "󰭆 Robots",
        "󱂇 Network",
        "󰈆 Exit"
    };

    std::atomic_bool running(true);
    while (running) {
        int selected = menu::select(menu);
        switch (selected) {
            case 0:
                break;
            case 1:
                {
                    auto netIfaces = getNetIfaces();
                    menu::display(netIfaces);
                }
                break;
            case 2:
                running = false;
                break;
            default:
                log_file << "ERROR: Received unexpected value.\n" << cpptrace::generate_trace().to_string();
                exit(1);
        }
    }
}

