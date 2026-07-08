// Thin GDExtension wrapper exposing the portable SwingUp2Controller (double pendulum) to Godot.
// Registered as the Godot class "SwingUp2" (use `SwingUp2.new()` from GDScript).
#ifndef GDEXT_SWINGUP2_H
#define GDEXT_SWINGUP2_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "swingup2_controller.h"

namespace godot {

class SwingUp2 : public RefCounted {
    GDCLASS(SwingUp2, RefCounted)

    armbalance::SwingUp2Controller ctrl_;

protected:
    static void _bind_methods();

public:
    SwingUp2();
    ~SwingUp2();

    void configure(const Dictionary &cfg);
    void reset();

    // theta1, theta2 from DOWN + rates; cart s,v; altitude dev h_dev, vertical vel vy; dt.
    // Returns {"a_lat","a_up","balancing"}.
    Dictionary update(double th1, double th1d, double th2, double th2d,
                      double s, double v, double h_dev, double vy, double dt);

    bool balancing() const;
};

}  // namespace godot

#endif  // GDEXT_SWINGUP2_H
