#ifndef PID_TWIDDLE_H
#define PID_TWIDDLE_H

#include <array>
#include <chrono>

class PID;

enum class TwiddleState {
    Initialize,
    FiddleUp,
    MeasureFiddleUp,
    FiddleDown,
    MeasureFiddleDown,
    MoveOn
};

/**
 * An implementation of coordinate ascent to find optimal PID tuning parameters.
 */
class Twiddle {
public:
    explicit Twiddle(PID& pid,
            const std::array<double, 3>& initial,
            const std::array<double, 3>& initial_steps = {1.0, 1.0, 1.0},
            double lambda = 0.1,
            double delay_secs=1.0,
            double initial_error = std::numeric_limits<double>::quiet_NaN()) noexcept;
    ~Twiddle() noexcept = default;

    void update();

private:
    void next_param();
    void update_controller(bool improved);

private:
    const double _lambda;
    PID* _pid;
    std::array<double, 3> _p;
    std::array<double, 3> _dp;
    double _best_error;
    size_t _i;
    TwiddleState _state;
    const double _delay_secs;
    bool _need_update;
    std::chrono::high_resolution_clock::time_point _last_update;
};

#endif //PID_TWIDDLE_H
