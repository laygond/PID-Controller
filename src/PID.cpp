#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients, errors, and set point
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

double PID::Update(double e) {
  /**
   * Updates for PID errors based on difference error e.
   */
  d_error = e - p_error;
  p_error = e;
  i_error += e;

  return -Kp*p_error - Ki*i_error - Kd*d_error;
}
