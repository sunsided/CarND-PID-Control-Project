#ifndef PID_H
#define PID_H

#include <array>

class PID {
public:
    /**
     * Constructor
     */
    PID() noexcept;

    /**
     * Destructor.
     */
    virtual ~PID() noexcept = default;

    /**
     * Initialize PID.
     * @param (Kp_, Ki_, Kd_) The initial PID coefficients
     */
    void Init(double Kp_, double Ki_, double Kd_) noexcept;

    /**
     * Initialize PID.
     * @param (Kp_, Ki_, Kd_) The initial PID coefficients
     */
    void Init(const std::array<double, 3>& params) noexcept {
        Init(params.at(0), params.at(1), params.at(2));
    }

    /**
     * Update the PID error variables given cross track error.
     * @param cte The current cross track error
     */
    void UpdateError(double cte);

    /**
     * Calculate the total PID error.
     * @output The total PID error
     */
    double TotalError() const;

    /**
     * Returns the step count (number of measurements)
     * @output The number of measurements.
     */
    inline double StepCount() const { return count;}

    /**
    * Resets the total error.
    */
    void ResetError();

    /**
     * Calculate the PID output.
     * @output The PID output.
     */
    double Calculate() const;

private:
    double p_error;
    double i_error;
    double d_error;

    double Kp;
    double Ki;
    double Kd;

    double count;
    double total_error;
};

#endif  // PID_H