#include <bilateralcontrol/common.hpp>
#include <eigen3/Eigen/src/Core/Matrix.h>

Asclepius::CouplingMap::CouplingMap() : m_rotation(Eigen::Quaterniond::Identity()), m_scale{1.0} {}

Asclepius::CouplingMap::CouplingMap(const Eigen::Quaterniond &rotation, double scale)
    : m_rotation(rotation), m_scale(scale) {
  assert(m_scale != 0.0 && "A scale of zero is invalid.");
}

void Asclepius::CouplingMap::update(const Eigen::Quaterniond &rotation, double scale) {
  assert(m_scale != 0.0 && "A scale of zero is invalid.");
  m_rotation = rotation;
  m_scale = scale;
}

Asclepius::BilateralController::BilateralController(double T, double m_v, double k_v, double b_v,
                                                    double c1, double c2) {} 

void Asclepius::BilateralController::update(double T, double m_v, double k_v, double b_v, double c1,
                                            double c2) {}

std::pair<Wrench, Twist> Asclepius::BilateralController::operator()(
    const Twist &v_m, const Wrench &f_s, const Displacement &p_m,
    const Displacement &p_s) {

}
