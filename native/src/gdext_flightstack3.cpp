#include "gdext_flightstack3.h"

#include <godot_cpp/core/class_db.hpp>

#include <cmath>

using namespace godot;
using apstack::Vec3f;

ApFlightStack3::ApFlightStack3() {}
ApFlightStack3::~ApFlightStack3() {}

void ApFlightStack3::_bind_methods() {
    ClassDB::bind_method(D_METHOD("reset"), &ApFlightStack3::reset);
    ClassDB::bind_method(D_METHOD("set_state", "accel_ef", "vel_ned", "pos_ned",
                                  "have_pos", "have_vel", "quat", "gyro"),
                         &ApFlightStack3::set_state);
    ClassDB::bind_method(D_METHOD("set_command", "lat_stick", "up_stick", "yaw_rate",
                                  "park", "n_arms", "theta", "theta_dot"),
                         &ApFlightStack3::set_command);
    ClassDB::bind_method(D_METHOD("set_hold_level", "level"), &ApFlightStack3::set_hold_level);
    ClassDB::bind_method(D_METHOD("update", "on_ground", "dt"), &ApFlightStack3::update);
}

void ApFlightStack3::reset() {
    mode_.init();
    att_.reset();
}

static inline Vec3f v3(const Vector3 &v) {
    return Vec3f((float)v.x, (float)v.y, (float)v.z);
}

void ApFlightStack3::set_state(Vector3 accel_ef, Vector3 vel_ned, Vector3 pos_ned,
                               bool have_pos, bool have_vel, Quaternion quat, Vector3 gyro) {
    ahrs_.accel_ef = v3(accel_ef);
    ahrs_.vel_ned = v3(vel_ned);
    ahrs_.pos_ned = v3(pos_ned);
    ahrs_.have_pos = have_pos;
    ahrs_.have_vel = have_vel;
    ahrs_.gyro = v3(gyro);

    const float w = (float)quat.w, x = (float)quat.x, y = (float)quat.y, z = (float)quat.z;
    ahrs_.roll  = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    const float sp = apstack::constrain_float(2.0f * (w * y - z * x), -1.0f, 1.0f);
    ahrs_.pitch = asinf(sp);
    ahrs_.yaw   = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

void ApFlightStack3::set_command(double lat_stick, double up_stick, double yaw_rate, bool park,
                                 int n_arms, Vector3 theta, Vector3 theta_dot) {
    cmd_.lat_stick = (float)lat_stick;
    cmd_.up_stick = (float)up_stick;
    cmd_.yaw_rate = (float)yaw_rate;
    cmd_.park = park;
    cmd_.n_arms = apstack::constrain_int(n_arms, 0, 3);
    cmd_.theta[0] = (float)theta.x;
    cmd_.theta[1] = (float)theta.y;
    cmd_.theta[2] = (float)theta.z;
    cmd_.theta_dot[0] = (float)theta_dot.x;
    cmd_.theta_dot[1] = (float)theta_dot.y;
    cmd_.theta_dot[2] = (float)theta_dot.z;
}

void ApFlightStack3::set_hold_level(bool level) {
    cmd_.hold_level = level;
}

PackedInt32Array ApFlightStack3::update(bool on_ground, double dt) {
    int pwm[8];
    mode_.run(cmd_, ahrs_, on_ground, (float)dt, att_, motors_, pwm);
    PackedInt32Array out;
    out.resize(8);
    for (int i = 0; i < 8; i++) out[i] = pwm[i];
    return out;
}
