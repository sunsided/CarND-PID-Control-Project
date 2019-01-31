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
    _delay_secs{delay_secs}, _need_update{true}, _scale{1.0}
{
    assert(lambda > 0);
    assert(lambda <= 0.5);
    pid.Init(initial);

    _last_update = std::chrono::high_resolution_clock::now();

    const auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    _generator = std::default_random_engine{seed};
    _index_distribution = std::uniform_int_distribution<int>{0, static_cast<int>(_dp.size() - 1)};
    _param_distribution = std::normal_distribution<double>{1, 0.1};

    _i = static_cast<size_t>(_index_distribution(_generator));
}

void Twiddle::next_param() {
    const auto old_index = _i;
    while (old_index == (_i = static_cast<size_t>(_index_distribution(_generator)))) ;
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
            std::cout << "Stochastic Twiddle warming up ..." << std::endl;
            if (std::isnan(_best_error) || total_error < _best_error) {
                _best_error = total_error;
            }
            _state = TwiddleState::FiddleUp;
            break;
        }

        case TwiddleState::FiddleUp: {
            _scale = _param_distribution(_generator);
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
                _dp[_i] *= (1.0 + _lambda * _scale);
                next_param();
                break;
            }

            std::cout << "Increasing failed for param #" << _i << ", error at " << total_error << "." << std::endl;
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
                _dp[_i] *= (1.0 + _lambda * _scale);
                next_param();
                break;
            }

            std::cout << "Decreasing failed for param #" << _i << ", error at " << total_error << "." << std::endl;
            _state = TwiddleState::MoveOn;
            _need_update = true;
            break;
        }

        case TwiddleState::MoveOn: {
            _p[_i] += _dp[_i];
            std::cout << "Nothing worked for param #" << _i << ", error still at " << total_error << "; resetting to " << _p[_i] << "." << std::endl;
            update_controller(false);
            // At this point, nothing helped; undo the subtracted value and refine the search area.
            _dp[_i] *= (1.0 - _lambda * _scale);

            next_param();
            break;
        }
    }

     pid.ResetError();
}