// TRIPLE INVERTED PENDULUM on a cart -- PORTABLE CONTROLLER CORE.
//
// A generic 8-state, single-input LQR (u = -K x). Here it stabilises a SERIES triple
// inverted pendulum: link1 (L1) hinged to an acceleration-commanded cart, link2 (L2)
// hinged to link1's TIP, link3 (L3) hinged to link2's TIP, all held upright by the one
// cart acceleration u. Same core shape as the single/double controllers -- just 8 states.
//
// PLANT (linearised about upright; point masses m1,m2,m3; absolute angles from up):
//     Mt th'' = Gt th + Ht u
//   let mu_j = sum_{k>=j} m_k (mass at/below joint j);
//     Mt_jk = L_j L_k mu_max(j,k),  Gt = diag( g L_j mu_j ),  Ht_j = -L_j mu_j.
//   state x = [theta1, w1, theta2, w2, theta3, w3, s, v],  u = cart accel.
//
// The two equal 0.6 m links stay controllable (rank 8) because in SERIES they sit at
// different chain positions -- their mode shapes differ, so one input can see all three
// modes (unlike two PARALLEL equal arms, which are uncontrollable). The catch: the gains
// are ~3-8x the double and the fastest unstable pole is ~9.9 rad/s (1/p ~ 101 ms), so
// the loop is far more latency- AND saturation-sensitive -- ~0.4 deg on link2 alone
// commands the full +-12 rail. See native/tools/compute_lqr3.py (K + closed-loop check).
//
#ifndef THREE_ARM_BALANCE_CONTROLLER_H
#define THREE_ARM_BALANCE_CONTROLLER_H

namespace armbalance {

struct Measurement3 {
    double theta1 = 0.0;      // link1 angle from up [rad]
    double theta1_dot = 0.0;  // link1 rate [rad/s]
    double theta2 = 0.0;      // link2 angle from up [rad] (absolute)
    double theta2_dot = 0.0;  // link2 rate [rad/s]
    double theta3 = 0.0;      // link3 angle from up [rad] (absolute)
    double theta3_dot = 0.0;  // link3 rate [rad/s]
    double s = 0.0;           // cart position [m]
    double v = 0.0;           // cart velocity [m/s]
};

struct Config3 {
    // --- plant (informational; the law uses the gains below) ---
    double L1 = 0.7;          // link1 length [m]
    double L2 = 0.6;          // link2 length [m]
    double L3 = 0.6;          // link3 length [m]
    double g  = 9.8;

    // --- MIMO-LQR gain K (1x8) from compute_lqr3.py, Q=diag(400,10,400,10,400,10,2,4),
    //     R=1. u = -(k1*th1 + k1d*w1 + k2*th2 + k2d*w2 + k3*th3 + k3d*w3
    //             + ks*(s - s_ref) + kv*v). Closed-loop poles all LHP (verified in tool).
    double k1  =  -601.361783;
    double k1d =   -32.818475;
    double k2  =  1639.755052;
    double k2d =   149.665587;
    double k3  = -1127.057775;
    double k3d =  -173.762737;
    double ks  =    -1.414214;
    double kv  =    -5.173150;
    double s_ref = 0.0;

    // --- output saturation ---
    double a_max = 12.0;

    // --- estimator: first-order low-pass per channel, cutoff [Hz] (<=0 = off) ---
    // Kept HIGH so group delay stays small -- on this fast RHP pole, added latency (not
    // noise) erodes the margin. But the big k2/k3 amplify sensor noise hard, so these
    // cutoffs are the primary tuning knob for the triple (chatter vs delay).
    double fc_theta     = 30.0;
    double fc_theta_dot = 25.0;
    double fc_s         = 15.0;
    double fc_v         = 15.0;
};

class ThreeArmBalanceController {
public:
    void configure(const Config3 &c) { cfg_ = c; reset(); }
    const Config3 &config() const { return cfg_; }

    void reset() {
        seeded_ = false;
        e_ = Measurement3{};
        saturated_ = false;
    }

    // Advance one control tick; returns commanded cart lateral accel [m/s^2].
    double update(const Measurement3 &m, double dt);

    Measurement3 estimate() const { return e_; }
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

    Config3 cfg_{};
    Measurement3 e_{};
    bool seeded_ = false;
    bool saturated_ = false;
};

}  // namespace armbalance

#endif  // THREE_ARM_BALANCE_CONTROLLER_H
