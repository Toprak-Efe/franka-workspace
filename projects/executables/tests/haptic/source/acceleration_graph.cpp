#include "matplot/freestanding/plot.h"
#include <bilateralcontrol/common.hpp>
#include <csignal>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <matplot/matplot.h>
#include <span>
#include <tlib/control/spatial.hpp>

std::vector<std::byte> get_latest_telemetry_bytes_from(const std::filesystem::path &folder) {
  namespace fs = std::filesystem;
  fs::path latest_file;

  for (const auto &entry : fs::directory_iterator(folder)) {
    if (!entry.is_regular_file())
      continue;
    if (latest_file.empty() || entry.last_write_time() > fs::last_write_time(latest_file)) {
      latest_file = entry.path();
    }
  }

  auto size = fs::file_size(latest_file);
  std::vector<std::byte> out(size);
  std::ifstream f(latest_file, std::ios::binary);
  f.read(reinterpret_cast<char *>(out.data()), size);
  return out;
}

template <typename T> std::vector<T> reconstruct_series_from(const std::vector<std::byte> &bytes) {
  const size_t count = bytes.size() / T::CanonicalSize;
  std::vector<T> reconstruction;
  reconstruction.reserve(count);

  std::span all(bytes);
  for (size_t i = 0; i < count; i++) {
    T signal;
    T::serial_load(all.subspan(i * T::CanonicalSize, T::CanonicalSize), signal);
    reconstruction.emplace_back(signal);
  }
  return reconstruction;
}

template <typename T>
std::vector<T> get_telemetry_series_from(const std::filesystem::path &telemetry_file) {
  std::vector<std::byte> bytes = get_latest_telemetry_bytes_from(telemetry_file);
  std::vector<T> telemetry_series = reconstruct_series_from<T>(bytes);
  return telemetry_series;
}

int main(int argc, char *argv[]) {
  namespace fs = std::filesystem;
  const static fs::path signals_base_folder = fs::temp_directory_path() / "tlibtelemetry";
  const fs::path am_folder(signals_base_folder / "Am");
  const fs::path fm_folder(signals_base_folder / "Fm");
  const fs::path vm_folder(signals_base_folder / "Vm");

  std::vector<double> fx, ax, vx, dx;

  { // Extract needed data to x,y,z
    std::vector<Wrench> commands_series = get_telemetry_series_from<Wrench>(fm_folder);
    std::vector<Asclepius::TwistDisplacement> sensor_series =
        get_telemetry_series_from<Asclepius::TwistDisplacement>(vm_folder);
    std::vector<Twist> acceleration_series = get_telemetry_series_from<Twist>(am_folder);

    fx.reserve(commands_series.size());
    vx.reserve(sensor_series.size());
    dx.reserve(sensor_series.size());
    ax.reserve(acceleration_series.size());

    for (const Asclepius::TwistDisplacement &td : sensor_series) {
      vx.emplace_back(td.get<0>().linear().x());
    }
    for (const Asclepius::TwistDisplacement &td : sensor_series) {
      dx.emplace_back(td.get<1>().linear().x());
    }
    for (const Wrench &f : commands_series) {
      fx.emplace_back(f.linear().x());
    }
    for (const Twist &a : acceleration_series) {
      ax.emplace_back(a.linear().x());
    }
  }

  matplot::subplot(4, 1, 0);
  matplot::plot(dx);
  matplot::subplot(4, 1, 1);
  matplot::plot(vx);
  matplot::subplot(4, 1, 2);
  matplot::plot(ax);
  matplot::subplot(4, 1, 3);
  matplot::plot(fx);

  matplot::save("image.svg");

  return 0;
}
