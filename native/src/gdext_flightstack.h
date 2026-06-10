// GDExtension wrapper exposing the native ArduPilot-lite flight stack to Godot.
//
// Registered as the Godot class "ApFlightStack". It owns the ported ModeAccel (mode 30)
// + attitude/rate cascade + 6DoF mixer, and each physics tick turns the drone's state +
// pilot command into the 8 motor PWM values Godot then feeds through its existing thrust
// curve / motor lag / apply_force plant -- a drop-in for the SITL servo packet, with no
// UDP bridge latency. All control math lives in the portable core (mode_accel.*,
// ap_*.*); this file only marshals Godot types <-> the plain-C++ structs.
#ifndef GDEXT_FLIGHTSTACK_H
#define GDEXT_FLIGHTSTACK_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "mode_accel.h"

namespace godot {

class ApFlightStack : public RefCounted {
    GDCLASS(ApFlightStack, RefCounted)

    apstack::ModeAccel mode_;
    apstack::AttitudeControl att_;
    apstack::Motors6DoF motors_;
    apstack::AhrsState ahrs_;
    apstack::AccelInput cmd_;

protected:
    static void _bind_methods();

public:
    ApFlightStack();
    ~ApFlightStack();

    void reset();

    // Feed the drone state, in ArduPilot NED / body FRD (Godot converts with its P basis).
    //   accel_ef : earth-frame specific force (NED)   vel_ned/pos_ned : NED
    //   quat     : NED attitude (w,x,y,z)             gyro : body FRD rates [rad/s]
    void set_state(Vector3 accel_ef, Vector3 vel_ned, Vector3 pos_ned,
                   bool have_pos, bool have_vel, Quaternion quat, Vector3 gyro);

    // Feed the pilot / higher-level command. theta/theta_dot pack up to 2 arms in x,y.
    void set_command(double lat_stick, double up_stick, double yaw_rate, bool park,
                     int n_arms, Vector3 theta, Vector3 theta_dot);

    // Run one control tick; returns the 8 motor PWM values (1000..2000).
    PackedInt32Array update(bool on_ground, double dt);
};

}  // namespace godot

#endif  // GDEXT_FLIGHTSTACK_H
