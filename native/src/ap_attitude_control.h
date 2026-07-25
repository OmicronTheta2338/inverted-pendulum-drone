// Attitude + rate controller for the native flight stack (portable, no Godot).
//
// A faithful-enough port of ArduCopter's AC_AttitudeControl cascade -- the part that
// matters for this project is that it reproduces the REAL attitude variance the drone
// experiences (finite loop bandwidth + gyro filtering reacting to the pendulum hinge
// reaction, canted-thrust torque and motor lag), because that wobble is a genuine
// disturbance to the level "catch" axis.
//
// Cascade (per ArduCopter):
//   thrust vector (NED) + heading-rate  ->  desired roll/pitch angle + yaw heading-hold
//   angle error --sqrt_controller(ATC_ANG_*_P, ATC_ACCEL_*_MAX)-->  target body rate
//   (target - gyro) --RatePID(ATC_RAT_*)-->  normalised roll/pitch/yaw demand -> mixer
// The gyro is low-passed at INS_GYRO_FILTER (20 Hz) first, like the real INS, so the
// loop carries the same phase lag near crossover.
#ifndef AP_ATTITUDE_CONTROL_H
#define AP_ATTITUDE_CONTROL_H

#include "ap_types.h"
#include "ap_pid.h"
#include "ap_motors6dof.h"

namespace apstack {

class AttitudeControl {
public:
    // ATC_ANG_*_P and the sqrt-controller second-order limits (rad/s^2), from the
    // vehicle params (ATC_ACCEL_*_MAX defaults: 110000/27000 cdeg/s^2).
    float ang_p_rll = 4.5f, ang_p_pit = 4.5f, ang_p_yaw = 4.5f;
    float accel_rll_max = 19.2f, accel_pit_max = 19.2f, accel_yaw_max = 4.71f;
    float gyro_filt_hz = 20.0f;   // INS_GYRO_FILTER

    RatePID rate_rll, rate_pit, rate_yaw;

    AttitudeControl() { load_default_gains(); }

    // Load this vehicle's ATC_RAT_* rate-loop gains/filters.
    void load_default_gains();

    void reset();
    void reset_yaw(float yaw_meas) { yaw_target_ = yaw_meas; }  // capture heading at takeoff

    // One attitude tick. thrust_vec in NED (its direction sets desired roll/pitch),
    // yaw_rate rad/s (heading-hold when 0). Measured roll/pitch/yaw in rad (AP euler),
    // gyro in rad/s body FRD. Writes roll/pitch/yaw demands into `motors`.
    void input_thrust_vector_rate_heading(const Vec3f &thrust_vec, float yaw_rate,
                                          float roll_meas, float pitch_meas, float yaw_meas,
                                          const Vec3f &gyro, float dt, Motors6DoF &motors);

private:
    float yaw_target_ = 0.0f;
    bool yaw_seeded_ = false;
    float gyro_f_x_ = 0.0f, gyro_f_y_ = 0.0f, gyro_f_z_ = 0.0f;
    bool gyro_seeded_ = false;
};

}  // namespace apstack

#endif  // AP_ATTITUDE_CONTROL_H
