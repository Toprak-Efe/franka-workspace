#include "../include/config.hpp"
#include <array>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string_view>
#include <utility>

using json = nlohmann::json;

Asclepius::Configuration::Configuration() {
  using namespace std::literals;
  constexpr const std::array<std::pair<std::string_view, std::string_view>, 5> defaults{{
      {"ROBOT1_HOSTNAME"sv, "172.16.1.2"sv},
      {"ROBOT2_HOSTNAME"sv, "172.16.0.2"sv},
      {"HAPTIC1_DEVICENAME"sv, "Left Device"sv},
      {"HAPTIC2_DEVICENAME"sv, "Right Device"sv},
      {"LOGFILE"sv, "/tmp/asclepius.log"sv},
  }};
  for (const auto &[key, value] : defaults)
    m_config.emplace(key, value);

  std::ifstream f("config.json");
  if (f.fail()) {
    return;
  }
  json config = json::parse(f);
  if (config.is_discarded()) {
    return;
  }
  for (const auto &[key, value] : m_config) {
    if (config.contains(key)) {
      if (config[key].type() == nlohmann::detail::value_t::string)
        m_config[key] = config[key];
    }
  }
  f.close();
  return;
}

Asclepius::Configuration::~Configuration() { return; }

const std::string &Asclepius::Configuration::operator[](const std::string &index) {
  if (m_config.find(index) == m_config.end()) {
    throw std::invalid_argument("Invalid look-up from configuration.");
  }
  return m_config.at(index);
}
