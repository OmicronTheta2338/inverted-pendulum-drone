// Thin GDExtension wrapper exposing the portable ThreeArmBalanceController to Godot.
// The control math lives in three_arm_balance_controller.{h,cpp}; this only marshals
// Godot types <-> the plain-C++ core. Registered as the Godot class "ArmBalance3".
#ifndef GDEXT_THREE_ARM_BALANCE_H
#define GDEXT_THREE_ARM_BALANCE_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "three_arm_balance_controller.h"

namespace godot {

class ArmBalance3 : public RefCounted {
    GDCLASS(ArmBalance3, RefCounted)

    armbalance::ThreeArmBalanceController ctrl_;

protected:
    static void _bind_methods();

public:
    ArmBalance3();
    ~ArmBalance3();

    void configure(const Dictionary &cfg);
    void reset();

    // One control tick: measured (noisy, delayed) 8-state -> single commanded
    // cart lateral acceleration [m/s^2].
    double update(double theta1, double theta1_dot, double theta2, double theta2_dot,
                  double theta3, double theta3_dot, double s, double v, double dt);

    bool last_saturated() const;
    Dictionary get_estimate() const;
};

}  // namespace godot

#endif  // GDEXT_THREE_ARM_BALANCE_H
