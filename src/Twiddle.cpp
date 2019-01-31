#include <cmath>
#include <cassert>
#include <iostream>
#include <random>
#include "PID.h"
#include "Twiddle.h"

Twiddle::Twiddle(PID& pid, const std::array<double, 3>& initial, const std::array<double, 3>& initial_steps,
        double lambda, double delay_secs,
        double pid_initial_error) noexcept
    : _lambda{lambda}, _pid{&pid},
    _p{initial}, _dp{initial_steps},
    _best_error{pid_initial_error},
    _i{0}, _state{TwiddleState::Initialize},
    _delay_secs{delay_secs}, _need_update{true}
{
    assert(lambda > 0);
    assert(lambda <= 0.5);
    pid.Init(initial);

    _last_update = std::chrono::high_resolution_clock::now();

    const auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_int_distribution<int> distribution(0, _dp.size() - 1);

    _i = static_cast<size_t>(distribution(generator));
    if (!std::isnan(_best_error)) {
        _state = TwiddleState::FiddleUp;
    }
}

void Twiddle::next_param() {
    if (++_i >= _dp.size()) {
        _i = 0;
    }
    _state = TwiddleState::FiddleUp;
    _need_update = true;
}

void Twiddle::update_controller(bool improved) {
    _pid->Init(_p);
    if (improved) {
        std::cout << ":)   ";
    }
    else {
        std::cout << "  :( ";
    }
    std::cout << "Parameters at error=" << _best_error
              << ", Kp=" << _p.at(0) << " (+/- " << _dp.at(0) << ")"
              << ", Ki=" << _p.at(1) << " (+/- " << _dp.at(1) << ")"
              << ", Kd=" << _p.at(2) << " (+/- " << _dp.at(2) << ")"
              << std::endl;
}

void Twiddle::update() {
    // Sample some data ...
    const auto now = std::chrono::high_resolution_clock::now();
    const auto time_delta = now - _last_update;
    const auto delay_secs = std::chrono::duration_cast<std::chrono::seconds>(time_delta).count();
    if (delay_secs >= _delay_secs) {
        _last_update = now;
        _need_update = true;
    }

    if (!_need_update) return;
    _need_update = false;

    // Run the algorithm
    auto &pid = *_pid;
    const auto total_error = pid.TotalError();
    switch(_state) {
        case TwiddleState::Initialize: {
            std::cout << "Fiddle initialized." << std::endl;
            _best_error = total_error;
            _state = TwiddleState::FiddleUp;
            _need_update = true;
            break;
        }

        case TwiddleState::FiddleUp: {
            _p[_i] += _dp[_i];
            std::cout << "Increasing param #" << _i << " to " << _p[_i] << "." << std::endl;
            _state = TwiddleState::MeasureFiddleUp;
            break;
        }

        case TwiddleState::MeasureFiddleUp: {
            if (total_error < _best_error) {
                _best_error = total_error;
                std::cout << "Increasing param #" << _i << " to " << _p[_i] << " succeeded." << std::endl;

                update_controller(true);
                // At this point, we succeeded and may extend the search area more.
                _dp[_i] *= (1.0 + _lambda);
                next_param();
                break;
            }

            _state = TwiddleState::FiddleDown;
            _need_update = true;
            break;
        }

        case TwiddleState::FiddleDown: {
            // We need to subtract twice, because we incremented in FiddleUp.
            _p[_i] -= 2 * _dp[_i];
            std::cout << "Decreasing param #" << _i << " to " << _p[_i] << "." << std::endl;
            _state = TwiddleState::MeasureFiddleDown;
            break;
        }

        case TwiddleState::MeasureFiddleDown: {
            if (total_error < _best_error) {
                _best_error = total_error;
                std::cout << "Decreasing param #" << _i << " to " << _p[_i] << " succeeded." << std::endl;

                update_controller(true);
                // At this point, we succeeded and may extend the search area more.
                _dp[_i] *= (1.0 + _lambda);
                next_param();
                break;
            }

            _state = TwiddleState::MoveOn;
            _need_update = true;
            break;
        }

        case TwiddleState::MoveOn: {
            std::cout << "Nothing worked for param #" << _i << ", error still at " << total_error << "; resetting to " << _p[_i] << "." << std::endl;
            _p[_i] += _dp[_i];
            update_controller(false);
            // At this point, nothing helped; undo the subtracted value and refine the search area.
            _dp[_i] *= (1.0 - _lambda);

            next_param();
            break;
        }
    }

     pid.ResetError();
}