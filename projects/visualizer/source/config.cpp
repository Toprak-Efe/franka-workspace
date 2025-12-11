#include <config.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

Asclepius::Configuration::Configuration() {
    std::ifstream f("config.json");
    json config = json::parse(f);
    f.close();

    // TO-DO

    return;
}

Asclepius::Configuration::~Configuration() {
    return;
}

std::string Asclepius::Configuration::operator[](std::string_view index) {
    if (m_config.find(index) == m_config.end()) {
        return "";
    }
    return m_config.at(index);
}
