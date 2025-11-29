#include <fstream>
#include <app.hpp>

int main(int argc, char *argv[]) {
    std::ofstream log_file; 
    log_file.open("/tmp/robot.log", std::ios_base::out | std::ios_base::app);
    
    App application;
    application.run(log_file);

    if (log_file.is_open()) {
        log_file.close();
    }
    exit(0);
}
