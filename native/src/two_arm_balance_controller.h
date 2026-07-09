// DOUBLE INVERTED PENDULUM on a cart -- PORTABLE CONTROLLER CORE.
//
// A generic 6-state, single-input LQR (u = -K x). Here it stabilises a double
// inverted pendulum: link1 (L1) hinged to an acceleration-commanded cart, link2
// (L2) hinged to link1's TIP, both held upright by the single cart acceleration u.
// (Same core shape as the single-arm controller, just 6 states / one input.)
//
// PLANT (linearised about upright; point masses m1,m2; absolute angles from up):
//     M [th1'';th2''] = G [th1;th2] + H u
//     M = [[(m1+m2)L1, m2 L2],[L1, L2]], G = diag((m1+m2)g, g), H = [-(m1+m2);-1]
//   state x = [theta1, theta1_dot, theta2, theta2_dot, s, v], u = cart accel.
//
// Both unstable modes are controllable by the one input (the serial chain couples
// well), so unlike two parallel arms this is robustly stabilisable. Gains are still
// larger than the single arm and the loop is more latency-sensitive (fastest
// unstable pole ~7.2 rad/s). See native/tools/compute_lqr2.py and section 9 of
// Balance-Controller-Design.md.
//
#ifndef TWO_ARM_BALANCE_CONTROLLER_H
#define TWO_ARM_BALANCE_CONTROLLER_H

namespace armbalance {

struct Measurement2 {
    double theta1 = 0.0;      // link1 angle from up [rad]
    double theta1_dot = 0.0;  // link1 rate [rad/s]
    double theta2 = 0.0;      // link2 angle from up [rad] (absolute, not relative)
    double theta2_dot = 0.0;  // link2 rate [rad/s]
    double s = 0.0;           // cart position [m]
    double v = 0.0;           // cart velocity [m/s]
};

struct Config2 {
    // --- plant (informational; the law uses the gains below) ---
    double L1 = 0.7;          // link1 length [m]
    double L2 = 0.6;          // link2 length [m]
    double g  = 9.8;

    // --- MIMO-LQR gain K (1x6) from compute_lqr2.py, Q=diag(400,10,400,10,2,4), R=1 ---
    // u = -(k1*theta1 + k1d*theta1_dot + k2*theta2 + k2d*theta2_dot
    //       + ks*(s - s_ref) + kv*v).
    double k1  = -213.542773;
    double k1d =  -14.504648;
    double k2  =  260.829154;
    double k2d =   45.149116;
    double ks  =    1.414214;
    double kv  =    4.525040;
    double s_ref = 0.0;

    // --- output saturation ---
    double a_max = 12.0;

    // --- estimator: first-order low-pass per channel, cutoff [Hz] (<=0 = off) ---
    double fc_theta     = 30.0;
    double fc_theta_dot = 25.0;
    double fc_s         = 15.0;
    double fc_v         = 15.0;
};

class TwoArmBalanceController {
public:
    void configure(const Config2 &c) { cfg_ = c; reset(); }
    const Config2 &config() const { return cfg_; }

    void reset() {
        seeded_ = false;
        e_ = Measurement2{};
        saturated_ = false;
    }

    // Advance one control tick; returns commanded cart lateral accel [m/s^2].
    double update(const Measurement2 &m, double dt);

    Measurement2 estimate() const { return e_; }
    bool last_saturated() const { return saturated_; }

private:
    static double lpf(double prev, double x, double fc, double dt) {
        if (fc <= 0.0 || dt <= 0.0) return x;
        const double a = (kTwoPi * fc * dt) / (1.0 + kTwoPi * fc * dt);
        return prev + a * (x - prev);
    }
    static double clampd(double x, double lo, double hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    }
    static constexpr double kTwoPi = 6.283185307179586;

    Config2 cfg_{};
    Measurement2 e_{};
    bool seeded_ = false;
    bool saturated_ = false;
};

}  // namespace armbalance

#endif  // TWO_ARM_BALANCE_CONTROLLER_H
