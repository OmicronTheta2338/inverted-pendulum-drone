// Single-arm inverted-pendulum balance controller -- PORTABLE CORE.
//
// This file has NO Godot and NO ArduPilot dependencies. It is plain C++ and is
// the artifact intended to compile, unchanged, onto the flight controller
// (STM32H743). The Godot test scene and the real drone only differ in how they
// feed it measurements and consume its output; the control law is identical.
//
// PLANT (see native/tools/compute_lqr.py and Balance-Controller-Design.md):
//   The drone is treated as a CART whose lateral acceleration is the control
//   input (a separate layer -- ArduPilot ModeAccel on the real vehicle -- makes
//   the realised acceleration track the command). One rigid arm hinges on it and
//   swings in the lateral-vertical plane. Linearised about upright:
//
//       theta_ddot = (g/L) theta - (1/L) u
//       s_ddot     = u
//
//   with state x = [theta, theta_dot, s, v] and control u = commanded lateral
//   acceleration [m/s^2]. Because u is an acceleration, the law is independent
//   of the drone and arm masses.
//
// I/O CONTRACT:
//   input  : Measurement (arm angle & rate, cart position & velocity), all in
//            SI, arm angle measured from straight-UP, +ve tipping toward +X.
//   output : commanded cart lateral acceleration [m/s^2], clamped to +-a_max.
//
#ifndef ARM_BALANCE_CONTROLLER_H
#define ARM_BALANCE_CONTROLLER_H

namespace armbalance {

// Sensed state handed to the controller each tick (already in engineering units;
// on the real vehicle theta is reconstructed from the arm encoder + IMU roll,
// theta_dot from an arm gyro, s/v from the position/velocity estimate).
struct Measurement {
    double theta = 0.0;      // arm angle from straight up [rad]  (0 = balanced)
    double theta_dot = 0.0;  // arm angular rate [rad/s]
    double s = 0.0;          // cart lateral position [m]
    double v = 0.0;          // cart lateral velocity [m/s]
};

enum class Law {
    LQR,   // full-state u = -K x  (primary)
    PID,   // inner angle PD + outer cart PD (+ optional cart integral); bring-up
};

struct Config {
    // --- plant (informational; the law uses the gains below, not these) ---
    double L = 0.6;          // arm length [m]
    double g = 9.8;          // gravity [m/s^2]

    // --- LQR full-state gain K (from compute_lqr.py, Q=diag(400,10,2,4), R=1) ---
    // Control is u = -(k_theta*theta + k_thetadot*theta_dot
    //                  + k_s*(s - s_ref) + k_v*v). K entries are negative, so the
    // effective feedback on a +theta lean is +43.9*theta (accelerate the cart
    // under the bob). Same K also stabilises the 0.5 m binding arm.
    double k_theta     = -43.913959;
    double k_thetadot  = -10.177313;
    double k_s         =  -1.414214;
    double k_v         =  -3.720995;
    double s_ref       =   0.0;      // cart station-keeping target [m]

    // --- output saturation ---
    double a_max = 12.0;             // |a_cmd| clamp [m/s^2] (rail headroom)

    // --- estimator: first-order low-pass per channel, cutoff in Hz ---
    // Kept HIGH so filter group delay stays small: on an unstable plant, added
    // latency -- not noise -- is what erodes the margin against the RHP pole.
    // fc <= 0 disables filtering on that channel (pass-through).
    double fc_theta     = 30.0;
    double fc_theta_dot = 25.0;
    double fc_s         = 15.0;
    double fc_v         = 15.0;

    // --- control law selection ---
    Law law = Law::LQR;

    // --- PID alternative (effective POSITIVE gains; see Balance doc appendix) ---
    // u = kp_th*theta + kd_th*theta_dot - kp_s*(s-s_ref) - kd_s*v - ki_s*Integral.
    // Defaults mirror the LQR gains so PID == LQR unless ki_s is used.
    double pid_kp_th = 43.913959;
    double pid_kd_th = 10.177313;
    double pid_kp_s  =  1.414214;
    double pid_kd_s  =  3.720995;
    double pid_ki_s  =  0.0;         // cart-position integral (steady bias reject)
    double pid_i_max =  2.0;         // integral clamp [m/s^2] (anti-windup)
};

class ArmBalanceController {
public:
    void configure(const Config& c) { cfg_ = c; reset(); }
    const Config& config() const { return cfg_; }

    void reset() {
        seeded_ = false;
        e_ = Measurement{};
        cart_i_ = 0.0;
        saturated_ = false;
        last_u_ = 0.0;
    }

    // Advance one control tick; returns commanded cart lateral accel [m/s^2].
    double update(const Measurement& m, double dt);

    // Introspection for logging / tests.
    Measurement estimate() const { return e_; }
    bool last_saturated() const { return saturated_; }

private:
    static double lpf(double prev, double x, double fc, double dt) {
        if (fc <= 0.0 || dt <= 0.0) return x;          // pass-through
        const double a = (kTwoPi * fc * dt) / (1.0 + kTwoPi * fc * dt);
        return prev + a * (x - prev);
    }
    static double clampd(double x, double lo, double hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    }
    static constexpr double kTwoPi = 6.283185307179586;

    Config cfg_{};
    Measurement e_{};      // filtered (estimated) state
    bool seeded_ = false;  // filter states primed with first sample
    double cart_i_ = 0.0;  // PID cart-position integral
    bool saturated_ = false;
    double last_u_ = 0.0;
};

}  // namespace armbalance

#endif  // ARM_BALANCE_CONTROLLER_H
