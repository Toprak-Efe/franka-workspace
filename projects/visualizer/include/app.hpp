#ifndef APP_HPP
#define APP_HPP

#include <fstream>

class App {
public:
    App();
    App(App &&) = default;
    App(const App &) = default;
    App &operator=(App &&) = default;
    App &operator=(const App &) = default;
    ~App();

    void run(std::ofstream &log_file);
private:

};

#endif//APP_HPP
