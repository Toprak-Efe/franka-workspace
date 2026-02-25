#include <filter.hpp>

void Asclepius::init_impedance_filter(impedance_filter_t *filter, double m_v, double k_v, double T) {
    filter->T = T;
    filter->m_v = m_v;
    filter->k_v = k_v;
    filter->b0 = 2*m_v/T + k_v;
    filter->b1 = 2*m_v/T - k_v;
    filter->x1_prev = 0;
    filter->x2_prev = 0;
    filter->y_prev = 0;
}

double Asclepius::sample_impedance_filter(impedance_filter_t *filter, double x1, double x2) {
    double y = filter->b0*x1 - filter->b1*filter->x1_prev + x2 + filter->x2_prev - filter->y_prev;
    filter->x1_prev = x1;
    filter->x2_prev = x2;
    filter->y_prev = y;
    return y;
}

void Asclepius::reset_impedance_filter(impedance_filter_t *filter) {
    filter->x1_prev = 0;
    filter->x2_prev = 0;
    filter->y_prev = 0;
}

