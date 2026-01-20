#include "../include/app.hpp"
#include <fstream>
#include <iostream>
#include <ios>

int main() {
    std::ofstream log_file("/tmp/franka.log", std::ios_base::out);
    Asclepius::App app(log_file);
    app.initialize();
    return 0;
}
