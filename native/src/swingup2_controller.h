// DOUBLE-pendulum SWING-UP controller -- PORTABLE CORE (no Godot / ArduPilot).
//
// Replaces the earlier scalar energy-pump (which provably flails on the double: pivot-translation
// forcing excites both the fundamental and the internal links-opposing mode, and scalar energy
// shaping can't select one). Instead we REPLAY an iLQR-optimized swing-up trajectory with
// time-varying LQR (TVLQR) tracking:
//
//     u(t) = U*(t) + K(t) * ( x_measured - X*(t) )        clamped to the accel rails
//
// The nominal U*, X* and gains K live in the generated header swingup2_traj_data.h (from
// native/tools/swingup2_traj.py). x = [th1,w1,th2,w2, xc,vx,yc,vy] with angles from straight DOWN
// (absolute). Open-loop replay of X* is unstable near the top (fastest pole ~7.2 rad/s), so the
// TVLQR feedback is REQUIRED. Once both links are up + slow (inside the catch gate) -- or the
// trajectory has run out -- we hand off to the 6-state TwoArmBalanceController LQR to hold upright.
#ifndef SWINGUP2_CONTROLLER_H
#define SWINGUP2_CONTROLLER_H

#include "two_arm_balance_controller.h"

namespace armbalance {

struct SwingUp2Config {
    double L1 = 0.7, L2 = 0.6, g = 9.8;
    double m1 = 0.1, m2 = 0.1;      // bob masses [kg] (informational; the trajectory bakes them in)

    double pump_gain = 3.0;         // DEAD (kept for GDExtension config compat; unused by TVLQR replay)
    double p_eps = 1e-3;            // DEAD
    double a_max_lat = 12.0, a_max_up = 9.8;
    double k_s = 1.0, k_v = 2.0;    // DEAD (centering now baked into the trajectory + K(t))
    double k_h = 2.0, k_hv = 4.0;   // altitude hold AFTER the catch

    // catch gate (both links); angle from UP + rate. The double basin is tiny (~6-8 deg), keep tight.
    double catch_angle = 0.1396;    // 8 deg [rad]
    double catch_rate = 0.6981;     // 40 deg/s [rad/s]

    Config2 balance;                // the 6-state LQR used after the catch
};

struct SwingUp2Output { double a_lat = 0.0; double a_up = 0.0; bool balancing = false; };

class SwingUp2Controller {
public:
    void configure(const SwingUp2Config& c) {
        cfg_ = c;
        balance_.configure(c.balance);
        reset();
    }
    void reset() { balancing_ = false; t_ = 0.0; balance_.reset(); }
    bool balancing() const { return balancing_; }

    // theta1, theta2 from straight DOWN [rad] (absolute), their rates; cart s,v; altitude
    // deviation h_dev, vertical velocity vy; dt. Returns {a_lat, a_up, balancing}.
    SwingUp2Output update(double th1, double th1d, double th2, double th2d,
                          double s, double v, double h_dev, double vy, double dt);

private:
    static double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
    static double wrap_pi(double a);

    SwingUp2Config cfg_{};
    bool balancing_ = false;
    double t_ = 0.0;                // elapsed replay time -> trajectory index
    TwoArmBalanceController balance_;
};

}  // namespace armbalance

#endif  // SWINGUP2_CONTROLLER_H
