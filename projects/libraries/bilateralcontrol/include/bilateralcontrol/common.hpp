#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Map.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <snippets/control/nthhold.hpp>
#include <snippets/control/physics.hpp>
#include <snippets/memory/triplebuffer.hpp>
#include <utility>

namespace Asclepius {

class CouplingMap {
public:
  CouplingMap();
  CouplingMap(const CouplingMap &) = default;
  CouplingMap(const Eigen::Quaterniond &rotation, double scale);

  CouplingMap T() { return CouplingMap(m_rotation.inverse(), 1.0 / m_scale); }

  template <typename Tag> SpatialVector<Tag> operator*(const SpatialVector<Tag> &v) {
    return SpatialVector<Tag>(m_rotation * v.linear(), m_rotation * v.angular()) * m_scale;
  }

  void update(const Eigen::Quaterniond &rotation, double scale);

private:
  Eigen::Quaterniond m_rotation;
  double m_scale{1.0};
}; // class CouplingMap

class BilateralController {
public:
  BilateralController() = delete;
  BilateralController(double T, double mass_v, double k_v, double b_v, double c1_p = 0,
                      double c2_p = 0);
  void update(double T, double mass_v, double k_v, double b_v, double c1_p = 0, double c2_p = 0);

  std::pair<Wrench, Twist> operator()(const Twist &v_m, const Wrench &f_s,
                                      const Displacement &p_m = Displacement(),
                                      const Displacement &p_s = Displacement());

private:
  double T;
  CouplingMap R;
  Admittance h_22;
  Impedance h11_1;
  Impedance h11_2;
  Stiffness c1, c2;

private:
  Twist v_m_prev;
  Wrench f_s_prev;
  Displacement p_m_prev, p_s_prev;
}; // class BilateralController

using TwistBuffer = TripleBuffer<Twist>;
struct TwistDisplacement {
  Twist twist;
  Displacement displacement;
};
using TwistDisplacementBuffer = TripleBuffer<TwistDisplacement>;

using WrenchBuffer = TripleBuffer<Wrench>;
struct WrenchDisplacement {
  Wrench wrench;
  Displacement displacement;
};
using WrenchDisplacementBuffer = TripleBuffer<WrenchDisplacement>;

struct WrenchTwistDisplacementBuffer {
  WrenchBuffer wbuff;
  TwistDisplacementBuffer tdbuff;
}; // WrenchTwistDisplacementBuffer

struct TwistWrenchDisplacementBuffer {
  TwistBuffer tbuff;
  WrenchDisplacementBuffer wdbuff;
}; // TwistWrenchDisplacementBuffer

}; // namespace Asclepius
