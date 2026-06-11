// Native, typed port of ArduPilot ModeAccel (Copter mode 30) for the Godot sim.
//
// This is the direct-acceleration inner loop for the inverted-pendulum octocopter:
// commanded lateral / vertical acceleration -> canted-thrust + direct-throttle, with a
// forward-hold regulator, an inverse-motor-model overdrive FF, and a pendulum
// reaction-force feed-forward. It is a faithful translation of
// ardupilot/ArduCopter/mode_accel.cpp -- same constants, same control law -- but reads
// TYPED command/state structs instead of RC-PWM channels and AHRS singletons, so it can
// run purely in Godot at the physics rate with no SITL bridge (the ~20-40 ms artifact).
//
// Re-porting a change back to ArduPilot is a manual edit (this is the "typed rewrite"
// the design chose): the math/constants/comments mirror the firmware; only the I/O
// binding differs. Keep the two in sync by hand.
#ifndef MODE_ACCEL_H
#define MODE_ACCEL_H

#include "ap_types.h"
#include "ap_attitude_control.h"
#include "ap_motors6dof.h"

namespace apstack {

// Pilot / higher-level command for one tick (replaces the RC channels the firmware read).
struct AccelInput {
    float lat_stick = 0.0f;   // [-1,1] lateral (catch) command; *ACCEL_LAT_CMD_MSS -> m/s^2
    float up_stick  = 0.0f;   // [-1,1] vertical command; 0 = hover, *GRAVITY -> [-g,+g]
    float yaw_rate  = 0.0f;   // rad/s  (heading-hold when 0)
    bool  park      = false;  // auto-fly to origin & hold (one-button reset/takeoff)
    int   n_arms    = 0;      // attached pendulum arms, 0..2
    float theta[2]     = {0.0f, 0.0f};  // rad, arm angle from straight DOWN, + toward +lateral
    float theta_dot[2] = {0.0f, 0.0f};  // rad/s
};

// Vehicle state for one tick (replaces AP::ahrs()). All in ArduPilot NED / body FRD.
struct AhrsState {
    Vec3f accel_ef;               // earth-frame specific force (NED) [m/s^2]
    Vec3f vel_ned;                // velocity NED [m/s]
    Vec3f pos_ned;                // position relative to origin (NED) [m]
    bool  have_vel = false;
    bool  have_pos = false;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;  // AP euler [rad]
    Vec3f gyro;                   // body FRD angular rate [rad/s]
};

class ModeAccel {
public:
    ModeAccel() { init(); }
    void init();

    // Advance one control tick. Writes the 8 motor PWM values (1000..2000) the mixer
    // produced -- a drop-in for the servo packet the SITL bridge used to deliver.
    void run(const AccelInput &in, const AhrsState &ahrs, bool on_ground, float dt,
             AttitudeControl &att, Motors6DoF &motors, int pwm_out[8]);

private:
    static constexpr int HIST = 16;   // feedback-ref ring buffer (>= ACCEL_FB_DELAY/dt)

    // forward-hold plane reference (captured at takeoff)
    bool  _have_fwd_ref = false;
    float _fwd_ref_n = 0.0f, _fwd_ref_e = 0.0f;
    float _fwd_cy_ref = 1.0f, _fwd_sy_ref = 0.0f;
    float _fwd_I = 0.0f;

    // closed-loop accel state (integrators + fast FF reference per axis)
    float _lat_I = 0.0f, _up_I = 0.0f;
    float _lat_ref_ff = 0.0f, _up_ref_ff = 0.0f;
    float _lat_ref_hist[HIST] = {0};
    float _up_ref_hist[HIST]  = {0};
    int   _hist_idx = 0;
};

}  // namespace apstack

#endif  // MODE_ACCEL_H
