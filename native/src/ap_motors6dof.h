// 6DoF motor mixer for the inverted-pendulum octocopter (portable, no Godot).
//
// Reproduces the ArduPilot AP_MotorsMatrix_6DoF setup configured by
// sitl/scripts/pendulum_6dof.lua: 4 upright motors carry throttle+roll+pitch+yaw,
// 4 canted motors carry lateral(catch)+yaw. It takes the same normalised control
// demands the firmware mixer does (set_roll/pitch/yaw/throttle/lateral) and emits the
// same 8 PWM values the SITL bridge used to send -- so Godot's existing _pwm_to_thrust
// closes the plant IDENTICALLY, and mode-30's calibrated constants stay valid.
//
// Output pipeline per motor (matches AP_MotorsMulticopter):
//   raw = throttle*TF + roll*RF + pitch*PF + yaw*YF + lateral*LF, clamped [0,1]
//   -> inverse of the thrust-expo curve -> throttle position in [0,1]
//   -> scaled into [SPIN_MIN, SPIN_MAX] -> PWM in [PWM_MIN, PWM_MAX].
#ifndef AP_MOTORS6DOF_H
#define AP_MOTORS6DOF_H

#include "ap_types.h"

namespace apstack {

class Motors6DoF {
public:
    // MOT_ params (this vehicle's tune; see sitl/mav.parm).
    float spin_min = 0.05f;    // MOT_SPIN_MIN
    float spin_max = 0.95f;    // MOT_SPIN_MAX
    float thst_expo = 0.35f;   // MOT_THST_EXPO
    int pwm_min = 1000;        // MOT_PWM_MIN
    int pwm_max = 2000;        // MOT_PWM_MAX

    void set_roll(float r) { roll_ = r; }
    void set_pitch(float p) { pitch_ = p; }
    void set_yaw(float y) { yaw_ = y; }
    void set_throttle(float t) { throttle_ = t; }  // 0..1
    void set_lateral(float l) { lateral_ = l; }    // -1..1

    // True if the lift group railed last output() -> the attitude loop freezes its
    // integrators (anti-windup). One flag for roll/pitch/yaw is enough here: the
    // demands are gentle and only saturate together in an extreme transient.
    bool saturated() const { return saturated_; }

    // Fill pwm_out[8]. armed=false -> all PWM_MIN (0 thrust; the ground state).
    void output(bool armed, int pwm_out[8]);

private:
    float thrust_to_pwm(float thrust01) const;

    float roll_ = 0.0f, pitch_ = 0.0f, yaw_ = 0.0f, throttle_ = 0.0f, lateral_ = 0.0f;
    bool saturated_ = false;
};

}  // namespace apstack

#endif  // AP_MOTORS6DOF_H
