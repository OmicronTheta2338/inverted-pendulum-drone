// Minimal ArduPilot-flavoured math for the native flight stack (portable, no Godot).
//
// This is deliberately small: just the scalar/vector helpers the ported ModeAccel,
// attitude controller and mixer need, named/behaving like their ArduPilot cousins
// (constrain_float, sqrt_controller, ...) so the control code reads like the firmware.
#ifndef AP_TYPES_H
#define AP_TYPES_H

#include <cmath>

namespace apstack {

// ArduPilot's GRAVITY_MSS. The mode-30 vertical map decodes a_up in [-g,+g] with THIS g,
// and ACCEL_HOVER_THR / the vertical slope were calibrated against it -- keep it exact.
static constexpr float GRAVITY_MSS = 9.80665f;

struct Vec3f {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    Vec3f() {}
    Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

inline float constrain_float(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
inline int constrain_int(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
inline float sq(float x) { return x * x; }
inline float safe_sqrt(float v) { return v <= 0.0f ? 0.0f : sqrtf(v); }
inline float wrap_PI(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

// First-order low-pass. fc<=0 or dt<=0 -> pass-through (no filtering). Matches the
// discrete filter AC_PID / the INS gyro filter use (single-pole, cutoff in Hz).
inline float lpf1(float prev, float x, float fc, float dt) {
    if (fc <= 0.0f || dt <= 0.0f) return x;
    const float rc = 1.0f / (2.0f * (float)M_PI * fc);
    const float a = dt / (dt + rc);
    return prev + a * (x - prev);
}

// AP_Math sqrt_controller: proportional for small error, then a constant-deceleration
// (sqrt) region so a large angle error doesn't demand an impossible rate. second_ord_lim
// is the max second-order response (rad/s^2 for the angle loop); <=0 -> pure linear P.
inline float sqrt_controller(float error, float p, float second_ord_lim, float dt) {
    float correction_rate;
    if (second_ord_lim <= 0.0f || p == 0.0f) {
        correction_rate = error * p;
    } else {
        const float linear_dist = second_ord_lim / (p * p);
        if (error > linear_dist) {
            correction_rate = safe_sqrt(2.0f * second_ord_lim * (error - 0.5f * linear_dist));
        } else if (error < -linear_dist) {
            correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error - 0.5f * linear_dist));
        } else {
            correction_rate = error * p;
        }
    }
    if (dt > 0.0f) {  // never command past the target in one tick
        return constrain_float(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);
    }
    return correction_rate;
}

}  // namespace apstack

#endif  // AP_TYPES_H
