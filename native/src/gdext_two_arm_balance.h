// Thin GDExtension wrapper exposing the portable TwoArmBalanceController to Godot.
// The control math lives in two_arm_balance_controller.{h,cpp}; this only marshals
// Godot types <-> the plain-C++ core. Registered as the Godot class "ArmBalance2".
#ifndef GDEXT_TWO_ARM_BALANCE_H
#define GDEXT_TWO_ARM_BALANCE_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "two_arm_balance_controller.h"

namespace godot {

class ArmBalance2 : public RefCounted {
    GDCLASS(ArmBalance2, RefCounted)

    armbalance::TwoArmBalanceController ctrl_;

protected:
    static void _bind_methods();

public:
    ArmBalance2();
    ~ArmBalance2();

    void configure(const Dictionary &cfg);
    void reset();

    // One control tick: measured (noisy, delayed) 6-state -> single commanded
    // cart lateral acceleration [m/s^2].
    double update(double theta1, double theta1_dot, double theta2, double theta2_dot,
                  double s, double v, double dt);

    bool last_saturated() const;
    Dictionary get_estimate() const;
};

}  // namespace godot

#endif  // GDEXT_TWO_ARM_BALANCE_H
