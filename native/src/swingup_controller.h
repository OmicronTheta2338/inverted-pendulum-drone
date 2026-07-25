// Single-arm ENERGY-SWINGUP controller -- PORTABLE CORE (no Godot, no ArduPilot).
//
// Swings the pendulum up from HANGING to inverted by 2-D energy shaping, then hands off to the
// ArmBalanceController LQR once the arrival is inside its (measured) region of attraction. Like the
// balance core, this is plain C++ intended to compile unchanged onto the flight controller (H743).
//
// PLANT / INPUT: the drone is a cart whose LATERAL and VERTICAL accelerations are the control inputs
// (ModeAccel realises them). theta is the arm angle from straight DOWN (0 = hanging, pi = upright),
// +ve toward +X. The pump rate is
//
//     de/dt = -L*thd*(a_x*cos(theta) + a_y*sin(theta))     e = 0.5*L^2*thd^2 - g*L*cos(theta)
//
// so we aim the pivot accel opposite the bob's tangential velocity: a = k*(e-e_top)*sign(thd)*(cos,sin).
// a_x has authority at bottom/top, a_y (vertical) at horizontal -> no cart-pole dead zone. Centering
// (lateral) and altitude-hold (vertical) terms keep the pivot home. Output accels are clamped to the
// ModeAccel rails (|a_lat|<=a_max_lat, |a_up|<=g).
#ifndef SWINGUP_CONTROLLER_H
#define SWINGUP_CONTROLLER_H

#include "arm_balance_controller.h"

namespace armbalance {

struct SwingUpConfig {
    double L = 0.7;              // arm length [m]
    double g = 9.8;             // gravity [m/s^2]

    // --- energy pump ---
    double pump_gain = 1.0;     // k; gentle (=1) arrives slow (catchable); aggressive overshoots fast
    double rate_eps = 0.05;     // |thd| below this -> hold last sign (sign(thd) ill-defined)
    double a_max_lat = 12.0;    // lateral authority [m/s^2] (full lat_stick)
    double a_max_up = 9.8;      // vertical authority [m/s^2]; |a_up| <= g (ModeAccel up range)

    // --- drift regulation (steals authority from the pump) ---
    double k_s = 1.0, k_v = 2.0;    // lateral centering (position, velocity)
    double k_h = 2.0, k_hv = 4.0;   // altitude hold (deviation, vertical velocity)

    // --- catch gate (hand off to the LQR); measured basin box, angle from UP + rate ---
    double catch_angle = 0.34906585;   // 20 deg [rad]
    double catch_rate = 1.74532925;    // 100 deg/s [rad/s]

    Config balance;             // the LQR sub-config used after the catch (defaults = single-arm gains)
};

struct SwingUpOutput {
    double a_lat = 0.0;         // commanded lateral accel [m/s^2]
    double a_up = 0.0;          // commanded net vertical accel [m/s^2] (0 = hover)
    bool balancing = false;     // false = still pumping, true = LQR has the pole
};

class SwingUpController {
public:
    void configure(const SwingUpConfig& c) {
        cfg_ = c;
        e_top_ = c.g * c.L;
        balance_.configure(c.balance);
        reset();
    }
    const SwingUpConfig& config() const { return cfg_; }

    void reset() {
        balancing_ = false;
        last_sign_ = 1.0;
        balance_.reset();
    }
    bool balancing() const { return balancing_; }

    // theta from straight DOWN [rad], theta_dot [rad/s]; lateral cart s [m], v [m/s];
    // altitude deviation h_dev = h - h_ref [m], vertical velocity vy [m/s]; dt [s].
    SwingUpOutput update(double theta, double theta_dot,
                         double s, double v, double h_dev, double vy, double dt);

private:
    static double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
    static double sgn(double x) { return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0); }
    static double wrap_pi(double a);

    SwingUpConfig cfg_{};
    double e_top_ = 6.86;
    bool balancing_ = false;
    double last_sign_ = 1.0;
    ArmBalanceController balance_;
};

}  // namespace armbalance

#endif  // SWINGUP_CONTROLLER_H
