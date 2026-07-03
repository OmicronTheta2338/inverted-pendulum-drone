#include "ap_motors6dof.h"

#include <cmath>

namespace apstack {

// Factor matrix from pendulum_6dof.lua (idx: M1..M8). forward factor is 0 for all.
static const float RF[8] = { 1.0f, -1.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f };
static const float PF[8] = { -1.0f, -1.0f, 1.0f, 1.0f, -0.042f, -0.042f, 0.042f, 0.042f };
static const float YF[8] = { -0.076f, 0.076f, -0.076f, 0.076f, 1.0f, -1.0f, 1.0f, -1.0f };
static const float TF[8] = { 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f };
static const float LF[8] = { 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 1.0f, -1.0f };

// Invert the physical thrust curve thrust = (1-e)*x + e*x^2 to recover the throttle
// position x that yields a requested linearised thrust fraction, then scale into the
// [spin_min, spin_max] band and out to PWM. This is exactly the round-trip the SITL
// bridge did (AP linearises via MOT_THST_EXPO; Godot re-applies the same curve).
float Motors6DoF::thrust_to_pwm(float thrust01) const {
    float x = constrain_float(thrust01, 0.0f, 1.0f);
    if (thst_expo > 0.0f) {
        x = ((thst_expo - 1.0f) +
             safe_sqrt(sq(1.0f - thst_expo) + 4.0f * thst_expo * x)) / (2.0f * thst_expo);
    }
    const float scaled = spin_min + x * (spin_max - spin_min);  // throttle position in band
    return (float)pwm_min + scaled * (float)(pwm_max - pwm_min);
}

void Motors6DoF::output(bool armed, int pwm_out[8]) {
    if (!armed) {
        for (int i = 0; i < 8; i++) pwm_out[i] = pwm_min;
        saturated_ = false;
        return;
    }
    saturated_ = false;
    for (int i = 0; i < 8; i++) {
        float raw = throttle_ * TF[i] + roll_ * RF[i] + pitch_ * PF[i] +
                    yaw_ * YF[i] + lateral_ * LF[i];
        if (raw < 0.0f || raw > 1.0f) saturated_ = true;
        raw = constrain_float(raw, 0.0f, 1.0f);
        pwm_out[i] = (int)lroundf(thrust_to_pwm(raw));
    }
}

}  // namespace apstack
