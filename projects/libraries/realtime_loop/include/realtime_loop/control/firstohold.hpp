#pragma once

#include <chrono>
#include <ranges>

namespace Asclepius::Control {

using Timestamp = std::chrono::time_point<std::chrono::steady_clock>;
template <typename T>
concept Timestamped = requires(T a) {
  { a.timestamp } -> std::same_as<Timestamp &>;
  { a.data } -> std::ranges::range;
};

template <Timestamped T> class FirstOrderHold {
public:
  FirstOrderHold(double period) : m_period{period} {}
  void push(const T &sample) {
    m_samples[0] = m_samples[1];
    m_samples[1] = sample;
  }
  T sample() {
    Timestamp curr_time = std::chrono::steady_clock::now();
    Timestamp &prev_time = m_samples[1].timestamp;
    Timestamp &last_time = m_samples[0].timestamp;

    decltype(T::data) data = m_samples[0].data;
    for (auto &&[curr_sample, prev_sample, last_sample] :
         std::ranges::views::zip(data, m_samples[1].data, m_samples[0].data)) {
      curr_sample = (curr_time - prev_time) * (prev_sample - last_sample) / (prev_time - last_time);
    }
    return {.data{data}, .timestamp{curr_time}};
  }

private:
  double m_period;
  std::array<T, 2> m_samples;
};

}; // namespace Asclepius::Control

