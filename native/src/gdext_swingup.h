// Thin GDExtension wrapper exposing the portable SwingUpController to Godot.
// Control math lives in swingup_controller.{h,cpp}; this only marshals types.
// Registered as the Godot class "SwingUp" (use `SwingUp.new()` from GDScript).
#ifndef GDEXT_SWINGUP_H
#define GDEXT_SWINGUP_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "swingup_controller.h"

namespace godot {

class SwingUp : public RefCounted {
    GDCLASS(SwingUp, RefCounted)

    armbalance::SwingUpController ctrl_;

protected:
    static void _bind_methods();

public:
    SwingUp();
    ~SwingUp();

    void configure(const Dictionary &cfg);
    void reset();

    // theta from DOWN, theta_dot; lateral s,v; altitude deviation h_dev, vertical vel vy; dt.
    // Returns {"a_lat","a_up","balancing"}.
    Dictionary update(double theta, double theta_dot, double s, double v,
                      double h_dev, double vy, double dt);

    bool balancing() const;
};

}  // namespace godot

#endif  // GDEXT_SWINGUP_H
