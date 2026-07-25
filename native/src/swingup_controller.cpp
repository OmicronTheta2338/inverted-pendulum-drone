#include "swingup_controller.h"

#include <cmath>

namespace armbalance {

double SwingUpController::wrap_pi(double a) {
    constexpr double kPi = 3.141592653589793;
    constexpr double kTwoPi = 6.283185307179586;
    while (a > kPi) a -= kTwoPi;
    while (a < -kPi) a += kTwoPi;
    return a;
}

SwingUpOutput SwingUpController::update(double theta, double theta_dot,
        double s, double v, double h_dev, double vy, double dt) {
    constexpr double kPi = 3.141592653589793;
    SwingUpOutput out;

    // gate on the up-frame state (theta_up = pi - theta_down; rate flips sign)
    const double theta_up = wrap_pi(kPi - theta);
    const double rate_up = -theta_dot;
    if (!balancing_ &&
        std::fabs(theta_up) < cfg_.catch_angle &&
        std::fabs(rate_up) < cfg_.catch_rate) {
        balancing_ = true;
    }

    if (balancing_) {
        // LQR drives lateral; vertical falls back to pure altitude-hold (pump role is done)
        Measurement m;
        m.theta = theta_up;
        m.theta_dot = rate_up;
        m.s = s;
        m.v = v;
        out.a_lat = balance_.update(m, dt);
        out.a_up = clampd(-cfg_.k_h * h_dev - cfg_.k_hv * vy, -cfg_.a_max_up, cfg_.a_max_up);
    } else {
        // 2-D energy pump: aim the pivot accel opposite the bob's tangential velocity
        const double e = 0.5 * cfg_.L * cfg_.L * theta_dot * theta_dot
                         - cfg_.g * cfg_.L * std::cos(theta);
        const double e_err = e - e_top_;               // < 0 below the top's energy
        double s_sign = last_sign_;
        if (std::fabs(theta_dot) > cfg_.rate_eps) {
            s_sign = sgn(theta_dot);
            last_sign_ = s_sign;
        }
        const double mag = cfg_.pump_gain * e_err * s_sign;
        out.a_lat = clampd(mag * std::cos(theta) - cfg_.k_s * s - cfg_.k_v * v,
                           -cfg_.a_max_lat, cfg_.a_max_lat);
        out.a_up = clampd(mag * std::sin(theta) - cfg_.k_h * h_dev - cfg_.k_hv * vy,
                          -cfg_.a_max_up, cfg_.a_max_up);
    }

    out.balancing = balancing_;
    return out;
}

}  // namespace armbalance
