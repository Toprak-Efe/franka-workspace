#pragma once 

#include <mutex>
#include <string>
#include <unordered_map>

namespace Asclepius {

class Configuration {
public:
    Configuration();
    ~Configuration();
    const std::string &operator[](const std::string &index);
private:
    std::unordered_map <std::string, std::string> m_config;
}; // class Configuration

inline Configuration g_configuration;

}; // namespace Asclepius

