// Thin GDExtension wrapper exposing the portable ArmBalanceController to Godot.
// The control math lives entirely in arm_balance_controller.{h,cpp}; this file
// only marshals Godot types <-> the plain-C++ core. Registered as the Godot
// class "ArmBalance" (use `ArmBalance.new()` from GDScript).
#ifndef GDEXT_ARM_BALANCE_H
#define GDEXT_ARM_BALANCE_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "arm_balance_controller.h"

namespace godot {

class ArmBalance : public RefCounted {
    GDCLASS(ArmBalance, RefCounted)

    armbalance::ArmBalanceController ctrl_;

protected:
    static void _bind_methods();

public:
    ArmBalance();
    ~ArmBalance();

    // Apply a config Dictionary (any missing key keeps the shipped default).
    void configure(const Dictionary &cfg);
    void reset();

    // One control tick: measured (noisy, delayed) state + dt -> commanded
    // cart lateral acceleration [m/s^2].
    double update(double theta, double theta_dot, double s, double v, double dt);

    bool last_saturated() const;
    Dictionary get_estimate() const;   // filtered state, for logging
};

}  // namespace godot

#endif  // GDEXT_ARM_BALANCE_H
