#include "arm_balance_controller.h"

namespace armbalance {

double ArmBalanceController::update(const Measurement& m, double dt) {
    // ---- 1. estimator: light per-channel low-pass (prime on the first sample
    //         so we don't ring in from zero) ----
    if (!seeded_) {
        e_ = m;
        seeded_ = true;
    } else {
        e_.theta     = lpf(e_.theta,     m.theta,     cfg_.fc_theta,     dt);
        e_.theta_dot = lpf(e_.theta_dot, m.theta_dot, cfg_.fc_theta_dot, dt);
        e_.s         = lpf(e_.s,         m.s,         cfg_.fc_s,         dt);
        e_.v         = lpf(e_.v,         m.v,         cfg_.fc_v,         dt);
    }

    const double s_err = e_.s - cfg_.s_ref;

    // ---- 2. control law ----
    double u;
    if (cfg_.law == Law::LQR) {
        // u = -K x   (K entries are negative -> +theta lean commands +accel)
        u = -(cfg_.k_theta    * e_.theta
            + cfg_.k_thetadot * e_.theta_dot
            + cfg_.k_s        * s_err
            + cfg_.k_v        * e_.v);
    } else {
        // PID: inner angle PD (fast) + outer cart PD (slow) + optional cart I.
        const double a_bal = cfg_.pid_kp_th * e_.theta + cfg_.pid_kd_th * e_.theta_dot;
        const double a_pos = -cfg_.pid_kp_s * s_err   - cfg_.pid_kd_s * e_.v
                             - cfg_.pid_ki_s * cart_i_;
        u = a_bal + a_pos;
    }

    // ---- 3. saturate ----
    const double u_sat = clampd(u, -cfg_.a_max, cfg_.a_max);
    saturated_ = (u_sat != u);

    // ---- 4. PID cart-integral update with anti-windup (freeze while railed) ----
    if (cfg_.law == Law::PID && cfg_.pid_ki_s > 0.0 && !saturated_) {
        cart_i_ = clampd(cart_i_ + s_err * dt, -cfg_.pid_i_max, cfg_.pid_i_max);
    }

    last_u_ = u_sat;
    return u_sat;
}

}  // namespace armbalance
