#include <cmath>
#include <iostream>
#include <string>
#include <uWS/uWS.h>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_last_of(']');
    if (found_null != string::npos) {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

inline constexpr double normalize_angle(const double angle, const double min_angle=-25, const double max_angle=25) {
    // Normalize to 0..1 range, then to -1..1.
    const auto normalized = (angle - min_angle) / (max_angle - min_angle);
    const auto converted = 2.0 * normalized - 1.0;
    return std::max(-1.0, std::min(1.0, converted));
}

inline constexpr double denormalize_angle(const double angle, const double min_angle=-25, const double max_angle=25) {
    // Normalize to 0..1 range, then to min..max.
    const auto normalized = 0.5 * (1.0 + angle);
    const auto converted = (normalized * (max_angle - min_angle)) + min_angle;
    return std::max(min_angle, std::min(max_angle, converted));
}

int main() {
    uWS::Hub h;

#if false
    auto pid_initial_error = std::numeric_limits<double>::quiet_NaN();
    std::array<double, 3> pid_params{0.1, 0, 0.1};
    std::array<double, 3> pid_param_steps{0.1, 0.05, 0.1};
#else
    auto pid_initial_error = std::numeric_limits<double>::quiet_NaN();
    std::array<double, 3> pid_params{0.002288822303194195, 0.0000254892438216043462, 0.174542321715073213};
    std::array<double, 3> pid_param_steps{1.7984689635948094e-3, 1.4959913862067697e-5, 1.9937038494087163e-2};
#endif
    const auto twiddle_lambda = 0.2;
    const auto twiddle_delay = 45;
    const auto car_throttle = 0.3;

    std::cout.precision(17);

    PID pid;
    Twiddle twiddle{pid, pid_params, pid_param_steps, twiddle_lambda, twiddle_delay, pid_initial_error};

    h.onMessage([&pid, &twiddle, &car_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(string(data).substr(0, length));
            if (s.empty()) {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                return;
            }

            const auto j = json::parse(s);
            string event = j[0].get<string>();
            if (event != "telemetry") return;

            // j[1] is the data JSON object
            const auto cte = std::stod(j[1]["cte"].get<string>());
            [[maybe_unused]] const auto speed = std::stod(j[1]["speed"].get<string>());
            const auto angle = std::stod(j[1]["steering_angle"].get<string>());
            [[maybe_unused]] const auto normalized_angle = normalize_angle(angle);

            pid.UpdateError(cte);
            //twiddle.update();

            const double raw_steer_value = pid.Calculate();
            const double steer_value = std::max(-1.0, std::min(1.0, raw_steer_value));
            const auto new_angle = denormalize_angle(steer_value);

            // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = new_angle;
            msgJson["throttle"] = car_throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end websocket message if
    }); // end h.onMessage

    h.onConnection([&pid](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
        pid.ResetError();
    });

    h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}