#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>
#include <unordered_map>

namespace Asclepius {

class Configuration {
public:
    Configuration();
    ~Configuration();
    std::string operator[](std::string_view index);
private:
    std::unordered_map <std::string_view, std::string> m_config;
}; // class Configuration

inline Configuration g_configuration;

}; // namespace Asclepius

#endif//CONFIG_HPP
