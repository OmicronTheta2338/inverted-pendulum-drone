// Native ModeAccel fork for the TRIPLE inverted-pendulum sandbox (mode_accel3).
//
// Identical to mode_accel.{h,cpp} in EVERY respect EXCEPT the pendulum reaction-force
// feed-forward, which is generalised from the 2-link coupled EOM to the 3-link coupled
// EOM (n_arms 0..3). The n<=2 path is a byte-for-byte copy of mode_accel.cpp, so it
// reduces exactly to the validated double/single behaviour.
//
// This is the deliberately-ISOLATED "stretch goal" copy (Native-Flightstack-Plan.md):
// the validated mode_accel.* stays byte-untouched. If the triple survives, the two
// collapse back into one generalised loop (n-arm) as a follow-up.
#ifndef MODE_ACCEL3_H
#define MODE_ACCEL3_H

#include "ap_types.h"
#include "ap_attitude_control.h"
#include "ap_motors6dof.h"
#include "mode_accel.h"   // reuse the topology-independent AhrsState (vehicle state)

namespace apstack {

// Pilot / higher-level command for one tick -- up to THREE series pendulum links.
struct AccelInput3 {
    float lat_stick = 0.0f;   // [-1,1] lateral (catch) command; *ACCEL_LAT_CMD_MSS -> m/s^2
    float up_stick  = 0.0f;   // [-1,1] vertical command; 0 = hover, *GRAVITY -> [-g,+g]
    float yaw_rate  = 0.0f;   // rad/s  (heading-hold when 0)
    bool  park      = false;  // auto-fly to origin & hold
    int   n_arms    = 0;      // attached series pendulum links, 0..3
    bool  hold_level = false; // true -> disable the pitch-based fore/aft hold, command level attitude
    float theta[3]     = {0.0f, 0.0f, 0.0f};  // rad, angle from straight DOWN, + toward +lateral
    float theta_dot[3] = {0.0f, 0.0f, 0.0f};  // rad/s
};

class ModeAccel3 {
public:
    ModeAccel3() { init(); }
    void init();

    // Advance one control tick. Writes the 8 motor PWM values (1000..2000).
    void run(const AccelInput3 &in, const AhrsState &ahrs, bool on_ground, float dt,
             AttitudeControl &att, Motors6DoF &motors, int pwm_out[8]);

private:
    static constexpr int HIST = 16;

    bool  _have_fwd_ref = false;
    float _fwd_ref_n = 0.0f, _fwd_ref_e = 0.0f;
    float _fwd_cy_ref = 1.0f, _fwd_sy_ref = 0.0f;
    float _fwd_I = 0.0f;

    float _lat_I = 0.0f, _up_I = 0.0f;
    float _lat_ref_ff = 0.0f, _up_ref_ff = 0.0f;
    float _lat_ref_hist[HIST] = {0};
    float _up_ref_hist[HIST]  = {0};
    int   _hist_idx = 0;
};

}  // namespace apstack

#endif  // MODE_ACCEL3_H
