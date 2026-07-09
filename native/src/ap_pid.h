// AC_PID-style rate PID for the attitude inner loop (portable, no Godot).
//
// Mirrors ArduPilot's AC_PID::update_all closely enough to reproduce the real
// closed-loop *feel* (and hence the attitude variance the user cares about):
//   * target passed through a low-pass (FLTT),
//   * error optionally low-passed (FLTE),
//   * derivative taken on the (filtered) error and low-passed (FLTD),
//   * integrator with symmetric clamp (IMAX) and anti-windup when the actuator rails.
// Gains/filters are the vehicle's ATC_RAT_* params. D-on-error (not measurement) is a
// faithful-enough simplification: ATC_RAT_*_D here is 0.001 (roll/pitch) or 0 (yaw), so
// the derivative term is negligible and the P+I dominate the dynamics.
#ifndef AP_PID_H
#define AP_PID_H

#include "ap_types.h"

namespace apstack {

class RatePID {
public:
    float kp = 0.0f, ki = 0.0f, kd = 0.0f, kimax = 0.0f;
    float fltt = 0.0f, flte = 0.0f, fltd = 0.0f;   // Hz, 0 = disabled

    void reset() {
        integ_ = 0.0f;
        tgt_ = 0.0f;
        err_ = 0.0f;
        deriv_ = 0.0f;
        seeded_ = false;
    }

    // target, measurement in rad/s; returns the normalised control demand.
    // saturated: true if the mixer railed this axis last tick -> freeze the integrator.
    float update(float target, float measurement, float dt, bool saturated) {
        if (dt <= 0.0f) return kp * (target - measurement) + integ_;
        if (!seeded_) {
            tgt_ = target;
            err_ = target - measurement;
            deriv_ = 0.0f;
            seeded_ = true;
        }
        tgt_ = lpf1(tgt_, target, fltt, dt);
        const float error_raw = tgt_ - measurement;
        const float error_f = lpf1(err_, error_raw, flte, dt);   // FLTE (disabled -> raw)
        const float d_raw = (error_f - err_) / dt;
        err_ = error_f;
        deriv_ = lpf1(deriv_, d_raw, fltd, dt);

        const float P = kp * error_f;
        const float D = kd * deriv_;
        if (!saturated) {
            integ_ = constrain_float(integ_ + ki * error_f * dt, -kimax, kimax);
        }
        return P + integ_ + D;
    }

private:
    float integ_ = 0.0f, tgt_ = 0.0f, err_ = 0.0f, deriv_ = 0.0f;
    bool seeded_ = false;
};

}  // namespace apstack

#endif  // AP_PID_H
