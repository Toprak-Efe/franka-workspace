#pragma once

#include <array>

namespace Asclepius {

typedef struct {
    // states 
    double x1_prev; // v_m
    double x2_prev; // f_s
    double y_prev;

    // constants 
    double b0; // 2*m_v/T + k_v
    double b1; // 2*m_v/T - k_v
    
    // parameters 
    double m_v; // virtual mass
    double k_v; // viscosity constant
    double T;   // sampling period
} impedance_filter_t;

/**
 * @desc Initializes the impedance filter with the parameters entered.
 * @param filter A pointer to an impedance filter. 
 * @param m_v The virtual mass/inertia of the system.
 * @param k_v The dampening constant for the system.
 * @param T sampling period.
 */
void init_impedance_filter(impedance_filter_t *filter, double m_v, double k_v, double T);

/**
 * @desc Runs the next velocity sample through the filter and returns the force/torque.
 * @param filter A pointer to the impedance filter.
 * @param x1 Input linear/angular velocity.
 * @param x2 Input force/torque.
 * @return y Output force/torque.
 */
double sample_impedance_filter(impedance_filter_t *filter, double x1, double x2);

/**
 * @desc Sets the state of the filter to zero.
 * @param filter A pointer to the impedance filter.
 */
void reset_impedance_filter(impedance_filter_t *filter);

}; // namespace Asclepius
