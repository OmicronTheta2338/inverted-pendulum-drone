#include "three_arm_balance_controller.h"

namespace armbalance {

double ThreeArmBalanceController::update(const Measurement3 &m, double dt) {
    if (!seeded_) {
        e_ = m;               // prime the filters with the first sample
        seeded_ = true;
    } else {
        e_.theta1     = lpf(e_.theta1,     m.theta1,     cfg_.fc_theta,     dt);
        e_.theta1_dot = lpf(e_.theta1_dot, m.theta1_dot, cfg_.fc_theta_dot, dt);
        e_.theta2     = lpf(e_.theta2,     m.theta2,     cfg_.fc_theta,     dt);
        e_.theta2_dot = lpf(e_.theta2_dot, m.theta2_dot, cfg_.fc_theta_dot, dt);
        e_.theta3     = lpf(e_.theta3,     m.theta3,     cfg_.fc_theta,     dt);
        e_.theta3_dot = lpf(e_.theta3_dot, m.theta3_dot, cfg_.fc_theta_dot, dt);
        e_.s          = lpf(e_.s,          m.s,          cfg_.fc_s,         dt);
        e_.v          = lpf(e_.v,          m.v,          cfg_.fc_v,         dt);
    }

    // MIMO full-state feedback: u = -K x  (single input, eight states).
    double u = -(cfg_.k1  * e_.theta1
               + cfg_.k1d * e_.theta1_dot
               + cfg_.k2  * e_.theta2
               + cfg_.k2d * e_.theta2_dot
               + cfg_.k3  * e_.theta3
               + cfg_.k3d * e_.theta3_dot
               + cfg_.ks  * (e_.s - cfg_.s_ref)
               + cfg_.kv  * e_.v);

    const double clamped = clampd(u, -cfg_.a_max, cfg_.a_max);
    saturated_ = (clamped != u);
    return clamped;
}

}  // namespace armbalance
