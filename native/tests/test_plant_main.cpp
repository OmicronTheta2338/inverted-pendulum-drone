// Pure-C++ validation harness for the arm-balance controller.
//
// Simulates the SAME plant the Godot scene does -- a nonlinear planar cart-pole
// whose cart acceleration is the (actuator-lagged, first-order) realised command
// -- driven by the portable controller, with Gaussian sensor noise and a fixed
// measurement latency. Runs a no-noise baseline plus a Monte-Carlo seed sweep and
// declares PASS/FAIL on the 15 s balance goal. This proves the control law
// independently of Godot / GDExtension.
//
// Build & run:
//   g++ -O2 -std=c++17 -I../src test_plant_main.cpp ../src/arm_balance_controller.cpp -o test_plant
//   ./test_plant
//
#include "arm_balance_controller.h"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

using armbalance::ArmBalanceController;
using armbalance::Config;
using armbalance::Measurement;

namespace {

constexpr double kDt      = 1.0 / 400.0;          // 400 Hz physics tick [s]
constexpr double kT       = 15.0;                 // balance duration [s]
constexpr int    kSteps   = (int)(kT / kDt + 0.5);
constexpr double kG       = 9.8;
constexpr double kL       = 0.6;

// Actuator model = inner-loop (ModeAccel) reference model H_ref(s) = 1/(tau_cl*s + 1).
// The spec parameter is the effective delay tau_cl; "1% of a step by 40 ms" is just the
// derived settling tail (= tau_cl*ln(100)), not the phase-relevant number.
constexpr double kActTauMs = 8.0;                 // tau_cl (effective delay tau_d) [ms]
constexpr double kActTau   = kActTauMs / 1000.0;  // [s]

// PASS thresholds.
constexpr double kMaxAngleDeg   = 15.0;   // never exceed this during the run
constexpr double kFinalAngleDeg = 5.0;    // upright & settled at t = 15 s

struct NoiseCfg {
    double sigma_theta     = 0.2 * M_PI / 180.0;   // 0.2 deg  [rad]
    double sigma_theta_dot = 1.0 * M_PI / 180.0;   // 1 deg/s  [rad/s]
    double sigma_s         = 0.005;                // 5 mm     [m]
    double sigma_v         = 0.02;                 // 2 cm/s   [m/s]
    double latency_s       = 0.0075;               // 3 ticks @ 400 Hz [s]
    bool   enabled         = true;
};

struct TrialResult {
    double max_angle_deg   = 0.0;
    double final_angle_deg = 0.0;
    double rms_cmd         = 0.0;
    double max_cmd         = 0.0;
    bool   saturated       = false;
    bool   passed          = false;
};

// Nonlinear planar cart-pole, cart acceleration = a_real (the realised, lagged
// command). theta from straight-up, +ve tipping toward +X:
//     theta_ddot = (g sin theta - a_real cos theta) / L
TrialResult run_trial(unsigned seed, const NoiseCfg& nz,
                      double theta0_deg, double kick_rad_s, bool verbose = false) {
    ArmBalanceController ctrl;
    Config cfg;                       // defaults = the shipped LQR gains
    cfg.L = kL; cfg.g = kG;
    ctrl.configure(cfg);

    // true plant state
    double theta = theta0_deg * M_PI / 180.0, theta_dot = 0.0;
    double s = 0.0, v = 0.0;
    double a_real = 0.0;

    std::mt19937 rng(seed);
    std::normal_distribution<double> N(0.0, 1.0);
    const double act_alpha = 1.0 - std::exp(-kDt / kActTau);

    const int lat_n = std::max(0, (int)std::lround(nz.latency_s / kDt));
    std::vector<Measurement> delay((size_t)lat_n + 1);
    int widx = 0;

    TrialResult r;
    double sum_cmd2 = 0.0;
    const int kick_step = (int)(7.5 / kDt);   // mid-run disturbance

    for (int i = 0; i < kSteps; ++i) {
        // --- sample measurement (+ noise), push through the delay buffer ---
        Measurement meas{theta, theta_dot, s, v};
        if (nz.enabled) {
            meas.theta     += nz.sigma_theta     * N(rng);
            meas.theta_dot += nz.sigma_theta_dot * N(rng);
            meas.s         += nz.sigma_s         * N(rng);
            meas.v         += nz.sigma_v         * N(rng);
        }
        delay[(size_t)widx] = meas;
        const int ridx = (widx - lat_n + (lat_n + 1)) % (lat_n + 1);
        const Measurement delayed = delay[(size_t)ridx];
        widx = (widx + 1) % (lat_n + 1);

        // --- controller ---
        const double a_cmd = ctrl.update(delayed, kDt);
        sum_cmd2 += a_cmd * a_cmd;
        r.max_cmd = std::max(r.max_cmd, std::fabs(a_cmd));
        if (ctrl.last_saturated()) r.saturated = true;

        // --- actuator first-order lag: realised accel chases the command ---
        a_real += act_alpha * (a_cmd - a_real);

        // --- integrate plant (semi-implicit Euler) ---
        const double theta_ddot = (kG * std::sin(theta) - a_real * std::cos(theta)) / kL;
        theta_dot += theta_ddot * kDt;
        theta     += theta_dot * kDt;
        v += a_real * kDt;      // cart acceleration = realised command
        s += v * kDt;

        // mid-run disturbance kick to the arm rate
        if (i == kick_step) theta_dot += kick_rad_s;

        const double ang_deg = std::fabs(theta) * 180.0 / M_PI;
        r.max_angle_deg = std::max(r.max_angle_deg, ang_deg);

        if (verbose && (i % 400 == 0)) {
            printf("  t=%5.2f  theta=%+6.2f deg  s=%+5.2f m  a_cmd=%+6.2f  a_real=%+6.2f\n",
                   i * kDt, theta * 180.0 / M_PI, s, a_cmd, a_real);
        }
    }

    r.final_angle_deg = std::fabs(theta) * 180.0 / M_PI;
    r.rms_cmd = std::sqrt(sum_cmd2 / kSteps);
    r.passed = (r.max_angle_deg < kMaxAngleDeg) && (r.final_angle_deg < kFinalAngleDeg);
    return r;
}

}  // namespace

int main() {
    printf("=== Arm-balance controller: pure-C++ plant validation ===\n");
    printf("plant: L=%.2f m, g=%.1f, dt=%.4f s (%d Hz), T=%.0f s\n",
           kL, kG, kDt, (int)std::lround(1.0 / kDt), kT);
    printf("actuator ref-model H_ref: tau_cl=%.3f ms (effective delay)\n", kActTauMs);
    printf("PASS: max|theta| < %.0f deg AND |theta(15s)| < %.0f deg\n\n",
           kMaxAngleDeg, kFinalAngleDeg);

    // 1) Ideal baseline: no noise, no latency, small offset + kick.
    {
        NoiseCfg clean; clean.enabled = false; clean.latency_s = 0.0;
        TrialResult r = run_trial(0, clean, 3.0, 0.3, /*verbose=*/true);
        printf("\n[baseline: clean] max|theta|=%.2f deg  final=%.3f deg  "
               "rms_cmd=%.2f  max_cmd=%.2f  sat=%d  => %s\n\n",
               r.max_angle_deg, r.final_angle_deg, r.rms_cmd, r.max_cmd,
               (int)r.saturated, r.passed ? "PASS" : "FAIL");
        if (!r.passed) { printf("BASELINE FAILED -- control law/signs wrong.\n"); return 1; }
    }

    // 2) Monte-Carlo: noise + 7.5 ms latency, 3 deg offset + 0.3 rad/s kick.
    const int kSeeds = 30;
    NoiseCfg nz;  // defaults: noise on, 7.5 ms latency
    int pass = 0;
    double worst_max = 0.0, worst_final = 0.0, sum_rms = 0.0;
    for (int seed = 1; seed <= kSeeds; ++seed) {
        TrialResult r = run_trial((unsigned)seed, nz, 3.0, 0.3);
        pass += r.passed ? 1 : 0;
        worst_max   = std::max(worst_max, r.max_angle_deg);
        worst_final = std::max(worst_final, r.final_angle_deg);
        sum_rms += r.rms_cmd;
        if (!r.passed)
            printf("  seed %2d FAIL: max|theta|=%.2f final=%.2f rms_cmd=%.2f sat=%d\n",
                   seed, r.max_angle_deg, r.final_angle_deg, r.rms_cmd, (int)r.saturated);
    }
    printf("[monte-carlo: noise + %.1f ms latency] %d/%d passed\n",
           nz.latency_s * 1000.0, pass, kSeeds);
    printf("  worst max|theta|=%.2f deg   worst final=%.2f deg   mean rms_cmd=%.2f m/s^2\n\n",
           worst_max, worst_final, sum_rms / kSeeds);

    // 3) Latency margin sweep (find where it breaks).
    printf("[latency sweep] noise on, 20 seeds each:\n");
    for (double lat_ms : {0.0, 5.0, 7.5, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0}) {
        NoiseCfg s = nz; s.latency_s = lat_ms / 1000.0;
        int p = 0; double wmax = 0.0;
        for (int seed = 1; seed <= 20; ++seed) {
            TrialResult r = run_trial((unsigned)(seed * 100 + (int)lat_ms), s, 3.0, 0.3);
            p += r.passed ? 1 : 0;
            wmax = std::max(wmax, r.max_angle_deg);
        }
        printf("  latency %5.1f ms : %2d/20 pass   worst max|theta|=%.2f deg\n", lat_ms, p, wmax);
    }

    printf("\n=== overall: %s ===\n", (pass == kSeeds) ? "PASS" : "CHECK ABOVE");
    return (pass == kSeeds) ? 0 : 2;
}
