#include <array>
#include <cmath>
#include <cstddef>
#include <matplot/matplot.h>
#include <tlib/control/biquad.hpp>
#include <vector>

struct Waveform {
  double phase;
  double amplitude;
  double frequency;
}; // struct Waveform

template <size_t C> auto get_frequencies(double df) -> std::array<Waveform, C> {
  std::array<Waveform, C> out;
  for (size_t i = 0; i < C; i++) {
    Waveform wform;
    wform.phase = matplot::rand(0.0, 2 * matplot::pi);
    wform.amplitude = matplot::rand(0.1, 1.0);
    wform.frequency = static_cast<double>(i) * df;
    out.at(i) = wform;
  }
  return out;
}

int main(int argc, char *argv[]) {
  constexpr size_t NumberOfPoints = 4096;
  constexpr size_t NumberOfFrequencies = 64;
  constexpr double DeltaFrequency = 20.0;
  constexpr double DeltaTime = NumberOfPoints/DeltaFrequency;

  auto waveforms = get_frequencies<NumberOfFrequencies>(DeltaFrequency);

  std::vector<double> t;
  for (size_t i = 0; i < NumberOfPoints; i++)
    t.emplace_back(i);

  std::vector<double> x;
  for (size_t i = 0; i < NumberOfPoints; i++) {
    double x_out = 0;
    for (auto w : waveforms) {
      x_out += w.amplitude * std::sin(w.frequency * (i * DeltaTime) + w.phase);
    }
    x.emplace_back(x_out);
  }

  Biquad<double> low_pass(0.00782021, 0.01564042, 0.00782021, -1.73472577, 0.7660066);

  std::vector<double> y;
  for (double x : x) {
    y.emplace_back(low_pass(x));
  }

  auto p = matplot::plot(t, y, t, x);
  p[1]->line_style("--");
  p[0]->line_width(2);
  matplot::show();
}
