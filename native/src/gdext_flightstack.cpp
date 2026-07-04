#include "gdext_flightstack.h"

#include <godot_cpp/core/class_db.hpp>

#include <cmath>

using namespace godot;
using apstack::Vec3f;

ApFlightStack::ApFlightStack() {}
ApFlightStack::~ApFlightStack() {}

void ApFlightStack::_bind_methods() {
    ClassDB::bind_method(D_METHOD("reset"), &ApFlightStack::reset);
    ClassDB::bind_method(D_METHOD("set_state", "accel_ef", "vel_ned", "pos_ned",
                                  "have_pos", "have_vel", "quat", "gyro"),
                         &ApFlightStack::set_state);
    ClassDB::bind_method(D_METHOD("set_command", "lat_stick", "up_stick", "yaw_rate",
                                  "park", "n_arms", "theta", "theta_dot"),
                         &ApFlightStack::set_command);
    ClassDB::bind_method(D_METHOD("update", "on_ground", "dt"), &ApFlightStack::update);
}

void ApFlightStack::reset() {
    mode_.init();
    att_.reset();
}

static inline Vec3f v3(const Vector3 &v) {
    return Vec3f((float)v.x, (float)v.y, (float)v.z);
}

void ApFlightStack::set_state(Vector3 accel_ef, Vector3 vel_ned, Vector3 pos_ned,
                              bool have_pos, bool have_vel, Quaternion quat, Vector3 gyro) {
    ahrs_.accel_ef = v3(accel_ef);
    ahrs_.vel_ned = v3(vel_ned);
    ahrs_.pos_ned = v3(pos_ned);
    ahrs_.have_pos = have_pos;
    ahrs_.have_vel = have_vel;
    ahrs_.gyro = v3(gyro);

    // NED attitude quaternion (w,x,y,z) -> AP euler (aerospace ZYX). Matches AP_AHRS
    // get_roll/pitch/yaw so the ported controller sees identical angles.
    const float w = (float)quat.w, x = (float)quat.x, y = (float)quat.y, z = (float)quat.z;
    ahrs_.roll  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    const float sp = apstack::constrain_float(2.0f * (w * y - z * x), -1.0f, 1.0f);
    ahrs_.pitch = asinf(sp);
    ahrs_.yaw   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

void ApFlightStack::set_command(double lat_stick, double up_stick, double yaw_rate, bool park,
                                int n_arms, Vector3 theta, Vector3 theta_dot) {
    cmd_.lat_stick = (float)lat_stick;
    cmd_.up_stick = (float)up_stick;
    cmd_.yaw_rate = (float)yaw_rate;
    cmd_.park = park;
    cmd_.n_arms = apstack::constrain_int(n_arms, 0, 2);
    cmd_.theta[0] = (float)theta.x;
    cmd_.theta[1] = (float)theta.y;
    cmd_.theta_dot[0] = (float)theta_dot.x;
    cmd_.theta_dot[1] = (float)theta_dot.y;
}

PackedInt32Array ApFlightStack::update(bool on_ground, double dt) {
    int pwm[8];
    mode_.run(cmd_, ahrs_, on_ground, (float)dt, att_, motors_, pwm);
    PackedInt32Array out;
    out.resize(8);
    for (int i = 0; i < 8; i++) out[i] = pwm[i];
    return out;
}
