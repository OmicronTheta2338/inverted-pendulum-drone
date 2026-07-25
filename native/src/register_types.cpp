#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "gdext_arm_balance.h"
#include "gdext_two_arm_balance.h"
#include "gdext_three_arm_balance.h"
#include "gdext_flightstack.h"
#include "gdext_flightstack3.h"
#include "gdext_swingup.h"
#include "gdext_swingup2.h"

using namespace godot;

void initialize_armbalance_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    GDREGISTER_CLASS(ArmBalance);
    GDREGISTER_CLASS(ArmBalance2);
    GDREGISTER_CLASS(ArmBalance3);
    GDREGISTER_CLASS(ApFlightStack);
    GDREGISTER_CLASS(ApFlightStack3);
    GDREGISTER_CLASS(SwingUp);
    GDREGISTER_CLASS(SwingUp2);
}

void uninitialize_armbalance_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}

extern "C" {
// Entry point named in sim-v-1/armbalance.gdextension.
GDExtensionBool GDE_EXPORT armbalance_library_init(
        GDExtensionInterfaceGetProcAddress p_get_proc_address,
        const GDExtensionClassLibraryPtr p_library,
        GDExtensionInitialization *r_initialization) {
    godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
    init_obj.register_initializer(initialize_armbalance_module);
    init_obj.register_terminator(uninitialize_armbalance_module);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_obj.init();
}
}
