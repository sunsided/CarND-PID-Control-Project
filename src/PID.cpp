#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() noexcept
    : p_error(0.0), i_error(0.0), d_error(0.0) {
    Init(0.0, 0.0, 0.0);
}

void PID::Init(const double Kp_, const double Ki_, const double Kd_) noexcept {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
}

void PID::UpdateError(const double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() const {
    return -Kp * p_error
           -Ki * i_error
           -Kd * d_error;
}