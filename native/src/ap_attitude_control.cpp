#include "ap_attitude_control.h"

#include <cmath>

namespace apstack {

void AttitudeControl::load_default_gains() {
    // ATC_RAT_RLL_* / ATC_RAT_PIT_* (identical tune)
    rate_rll.kp = 0.015f; rate_rll.ki = 0.015f; rate_rll.kd = 0.001f; rate_rll.kimax = 0.5f;
    rate_rll.fltt = 20.0f; rate_rll.flte = 0.0f; rate_rll.fltd = 20.0f;
    rate_pit = rate_rll;
    // ATC_RAT_YAW_*
    rate_yaw.kp = 0.020f; rate_yaw.ki = 0.002f; rate_yaw.kd = 0.0f; rate_yaw.kimax = 0.5f;
    rate_yaw.fltt = 20.0f; rate_yaw.flte = 2.5f; rate_yaw.fltd = 20.0f;
}

void AttitudeControl::reset() {
    rate_rll.reset();
    rate_pit.reset();
    rate_yaw.reset();
    yaw_seeded_ = false;
    gyro_seeded_ = false;
    gyro_f_x_ = gyro_f_y_ = gyro_f_z_ = 0.0f;
}

void AttitudeControl::input_thrust_vector_rate_heading(const Vec3f &thrust_vec, float yaw_rate,
                                                       float roll_meas, float pitch_meas, float yaw_meas,
                                                       const Vec3f &gyro, float dt, Motors6DoF &motors) {
    // --- desired roll/pitch from the thrust vector, decomposed in the heading frame ---
    // Rotate the horizontal thrust into the body heading: fwd along nose, lat to the right.
    const float cy = cosf(yaw_meas), sy = sinf(yaw_meas);
    const float fwd =  thrust_vec.x * cy + thrust_vec.y * sy;
    const float lat = -thrust_vec.x * sy + thrust_vec.y * cy;
    const float up  = -thrust_vec.z;   // NED z is down; thrust points up -> up>0
    const float up_safe = (up < 0.1f) ? 0.1f : up;
    // Nose-DOWN (negative pitch, AP convention nose-up = +) accelerates forward, so a
    // +forward thrust component maps to a negative pitch target. Roll stays ~0 (the
    // catch axis is held level; lat is 0 for mode-30's thrust vector).
    const float pitch_des = -atan2f(fwd, up_safe);
    const float roll_des  =  atan2f(lat, up_safe);

    // --- yaw heading-hold: integrate the commanded rate into a target, drive by angle P ---
    if (!yaw_seeded_) { yaw_target_ = yaw_meas; yaw_seeded_ = true; }
    yaw_target_ = wrap_PI(yaw_target_ + yaw_rate * dt);

    // --- angle error -> target body rate (sqrt controller) ---
    const float roll_err  = wrap_PI(roll_des  - roll_meas);
    const float pitch_err = wrap_PI(pitch_des - pitch_meas);
    const float yaw_err   = wrap_PI(yaw_target_ - yaw_meas);
    const float tgt_roll_rate  = sqrt_controller(roll_err,  ang_p_rll, accel_rll_max, dt);
    const float tgt_pitch_rate = sqrt_controller(pitch_err, ang_p_pit, accel_pit_max, dt);
    const float tgt_yaw_rate   = sqrt_controller(yaw_err,   ang_p_yaw, accel_yaw_max, dt) + yaw_rate;

    // --- INS gyro low-pass (20 Hz), like the real sensor path ---
    if (!gyro_seeded_) { gyro_f_x_ = gyro.x; gyro_f_y_ = gyro.y; gyro_f_z_ = gyro.z; gyro_seeded_ = true; }
    gyro_f_x_ = lpf1(gyro_f_x_, gyro.x, gyro_filt_hz, dt);
    gyro_f_y_ = lpf1(gyro_f_y_, gyro.y, gyro_filt_hz, dt);
    gyro_f_z_ = lpf1(gyro_f_z_, gyro.z, gyro_filt_hz, dt);

    // --- rate PID -> normalised demands -> mixer ---
    const bool sat = motors.saturated();
    motors.set_roll (rate_rll.update(tgt_roll_rate,  gyro_f_x_, dt, sat));
    motors.set_pitch(rate_pit.update(tgt_pitch_rate, gyro_f_y_, dt, sat));
    motors.set_yaw  (rate_yaw.update(tgt_yaw_rate,   gyro_f_z_, dt, sat));
}

}  // namespace apstack
