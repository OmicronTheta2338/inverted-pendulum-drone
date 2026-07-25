#include "swingup2_controller.h"

#include "swingup2_traj_data.h"   // AUTO-GENERATED nominal U*, X*, gains K (armbalance::swingup2_traj)

#include <cmath>

namespace armbalance {

double SwingUp2Controller::wrap_pi(double a) {
    constexpr double kPi = 3.141592653589793;
    constexpr double kTwoPi = 6.283185307179586;
    while (a > kPi) a -= kTwoPi;
    while (a < -kPi) a += kTwoPi;
    return a;
}

SwingUp2Output SwingUp2Controller::update(double th1, double th1d, double th2, double th2d,
        double s, double v, double h_dev, double vy, double dt) {
    constexpr double kPi = 3.141592653589793;
    using namespace swingup2_traj;   // N, DT, U[N][2], X[N+1][8], K[N][16]
    SwingUp2Output out;

    // up-frame state for the gate + the LQR catch (theta_up = pi - theta_down; rate flips sign)
    const double th1u = wrap_pi(kPi - th1), th2u = wrap_pi(kPi - th2);
    const double r1u = -th1d, r2u = -th2d;

    // gate: both links inside the catch region -> hand off. Also force the handoff once the
    // trajectory has run out, so a lagging real stack still ends on the balance LQR (not a frozen
    // terminal command). Whichever fires first latches balancing_.
    if (!balancing_) {
        const bool in_gate =
            std::fabs(th1u) < cfg_.catch_angle && std::fabs(th2u) < cfg_.catch_angle &&
            std::fabs(r1u) < cfg_.catch_rate && std::fabs(r2u) < cfg_.catch_rate;
        if (in_gate || t_ >= N * DT) balancing_ = true;
    }

    if (balancing_) {
        Measurement2 m;
        m.theta1 = th1u; m.theta1_dot = r1u;
        m.theta2 = th2u; m.theta2_dot = r2u;
        m.s = s; m.v = v;
        out.a_lat = balance_.update(m, dt);
        out.a_up = clampd(-cfg_.k_h * h_dev - cfg_.k_hv * vy, -cfg_.a_max_up, cfg_.a_max_up);
        out.balancing = true;
        return out;
    }

    // --- replay + TVLQR:  u = U*[k] + K[k] * (x_meas - X*[k]),  angle rows wrapped ---
    int k = (int)std::lround(t_ / DT);
    if (k < 0) k = 0;
    if (k > N - 1) k = N - 1;
    const double xm[8] = {th1, th1d, th2, th2d, s, v, h_dev, vy};
    double dx[8];
    for (int i = 0; i < 8; ++i) dx[i] = xm[i] - X[k][i];
    dx[0] = wrap_pi(dx[0]);
    dx[2] = wrap_pi(dx[2]);
    double u0 = U[k][0], u1 = U[k][1];
    for (int i = 0; i < 8; ++i) {
        u0 += K[k][i] * dx[i];       // K row 0 -> a_lat
        u1 += K[k][8 + i] * dx[i];   // K row 1 -> a_up
    }
    out.a_lat = clampd(u0, -cfg_.a_max_lat, cfg_.a_max_lat);
    out.a_up = clampd(u1, -cfg_.a_max_up, cfg_.a_max_up);
    out.balancing = false;
    t_ += dt;
    return out;
}

}  // namespace armbalance
