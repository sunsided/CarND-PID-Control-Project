#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() noexcept {
    Init(0.0, 0.0, 0.0);
}

void PID::Init(const double Kp_, const double Ki_, const double Kd_) noexcept {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    ResetError();
}

void PID::UpdateError(const double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    total_error += cte * cte;
    count += 1.0;
}

void PID::ResetError() {
    p_error = 0;
    i_error = 0;
    d_error = 0;
    total_error = 0.0;
    count = 0.0;
};

double PID::TotalError() const {
    if (count == 0) return 0.0;
    return total_error / count;
}

double PID::Calculate() const {
    return -Kp * p_error
           -Ki * i_error
           -Kd * d_error;
}