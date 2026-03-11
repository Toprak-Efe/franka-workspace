#pragma once

#include <type_traits>

namespace Asclepius::Control {

template <typename T>
requires requires(T &t) {
  { std::is_arithmetic<T>() };
  { std::is_copy_constructible<T>() };
  { std::is_default_constructible<T>() };
}
class ImpedanceFilter {
public:
  ImpedanceFilter() = delete;

  /**
   * @desc Initializes the impedance filter with the parameters entered.
   * @param m_v The virtual mass/inertia of the system.
   * @param k_v The dampening constant for the system.
   * @param T_s sampling period.
   */
  ImpedanceFilter(double m_v, double k_v, double T_s) : m_v{m_v}, k_v{k_v}, T_s{T_s} {
    b0 = 2 * m_v / T_s + k_v;
    b1 = 2 * m_v / T_s - k_v;
  }

  /**
   * @desc Runs the next velocity sample through the filter and returns the force/torque.
   * @param x1 Input linear/angular velocity.
   * @param x2 Input force/torque.
   * @return y Output force/torque.
   */
  T sample(T x1, T x2) {
    T y = b0 * x1 - b1 * x1_prev + x2 + x2_prev - y_prev;
    x1_prev = x1;
    x2_prev = x2;
    y_prev = y;
    return y;
  }
  /**
   * @desc Sets the state of the filter to zero.
   */
  void reset() {
    x1_prev = T{};
    x2_prev = T{};
    y_prev = T{};
  }

private:
  // states
  T x1_prev; // v_m
  T x2_prev; // f_s
  T y_prev;

  // constants
  double b0; // 2*m_v/T + k_v
  double b1; // 2*m_v/T - k_v

  // parameters
  double m_v; // virtual mass
  double k_v; // viscosity constant
  double T_s; // sampling period
}; // class ImpedanceFilter;

}; // namespace Asclepius::Control
