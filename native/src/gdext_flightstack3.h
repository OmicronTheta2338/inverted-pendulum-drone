// GDExtension wrapper exposing the TRIPLE-pendulum flight stack (ModeAccel3) to Godot.
// Registered as the Godot class "ApFlightStack3". Identical to ApFlightStack except it
// owns ModeAccel3 (0..3-link reaction FF) and packs up to three arm angles in x,y,z.
#ifndef GDEXT_FLIGHTSTACK3_H
#define GDEXT_FLIGHTSTACK3_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "mode_accel3.h"

namespace godot {

class ApFlightStack3 : public RefCounted {
    GDCLASS(ApFlightStack3, RefCounted)

    apstack::ModeAccel3 mode_;
    apstack::AttitudeControl att_;
    apstack::Motors6DoF motors_;
    apstack::AhrsState ahrs_;
    apstack::AccelInput3 cmd_;

protected:
    static void _bind_methods();

public:
    ApFlightStack3();
    ~ApFlightStack3();

    void reset();

    void set_state(Vector3 accel_ef, Vector3 vel_ned, Vector3 pos_ned,
                   bool have_pos, bool have_vel, Quaternion quat, Vector3 gyro);

    // Feed the command. theta/theta_dot pack up to 3 arms in x,y,z.
    void set_command(double lat_stick, double up_stick, double yaw_rate, bool park,
                     int n_arms, Vector3 theta, Vector3 theta_dot);
    void set_hold_level(bool level);

    PackedInt32Array update(bool on_ground, double dt);
};

}  // namespace godot

#endif  // GDEXT_FLIGHTSTACK3_H
