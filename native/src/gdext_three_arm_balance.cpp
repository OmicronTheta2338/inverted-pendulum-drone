#include "gdext_three_arm_balance.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;
using armbalance::Config3;
using armbalance::Measurement3;

namespace {
double dget(const Dictionary &d, const char *k, double def) {
    return d.has(k) ? (double)d[k] : def;
}
}  // namespace

ArmBalance3::ArmBalance3() {}
ArmBalance3::~ArmBalance3() {}

void ArmBalance3::_bind_methods() {
    ClassDB::bind_method(D_METHOD("configure", "cfg"), &ArmBalance3::configure);
    ClassDB::bind_method(D_METHOD("reset"), &ArmBalance3::reset);
    ClassDB::bind_method(
        D_METHOD("update", "theta1", "theta1_dot", "theta2", "theta2_dot",
                 "theta3", "theta3_dot", "s", "v", "dt"),
        &ArmBalance3::update);
    ClassDB::bind_method(D_METHOD("last_saturated"), &ArmBalance3::last_saturated);
    ClassDB::bind_method(D_METHOD("get_estimate"), &ArmBalance3::get_estimate);
}

void ArmBalance3::configure(const Dictionary &cfg) {
    Config3 c;
    c.L1  = dget(cfg, "L1", c.L1);
    c.L2  = dget(cfg, "L2", c.L2);
    c.L3  = dget(cfg, "L3", c.L3);
    c.g   = dget(cfg, "g", c.g);
    c.k1  = dget(cfg, "k1", c.k1);
    c.k1d = dget(cfg, "k1d", c.k1d);
    c.k2  = dget(cfg, "k2", c.k2);
    c.k2d = dget(cfg, "k2d", c.k2d);
    c.k3  = dget(cfg, "k3", c.k3);
    c.k3d = dget(cfg, "k3d", c.k3d);
    c.ks  = dget(cfg, "ks", c.ks);
    c.kv  = dget(cfg, "kv", c.kv);
    c.s_ref = dget(cfg, "s_ref", c.s_ref);
    c.a_max = dget(cfg, "a_max", c.a_max);
    c.fc_theta     = dget(cfg, "fc_theta", c.fc_theta);
    c.fc_theta_dot = dget(cfg, "fc_theta_dot", c.fc_theta_dot);
    c.fc_s         = dget(cfg, "fc_s", c.fc_s);
    c.fc_v         = dget(cfg, "fc_v", c.fc_v);
    ctrl_.configure(c);
}

void ArmBalance3::reset() { ctrl_.reset(); }

double ArmBalance3::update(double theta1, double theta1_dot, double theta2, double theta2_dot,
                           double theta3, double theta3_dot, double s, double v, double dt) {
    Measurement3 m;
    m.theta1 = theta1;
    m.theta1_dot = theta1_dot;
    m.theta2 = theta2;
    m.theta2_dot = theta2_dot;
    m.theta3 = theta3;
    m.theta3_dot = theta3_dot;
    m.s = s;
    m.v = v;
    return ctrl_.update(m, dt);
}

bool ArmBalance3::last_saturated() const { return ctrl_.last_saturated(); }

Dictionary ArmBalance3::get_estimate() const {
    const Measurement3 e = ctrl_.estimate();
    Dictionary d;
    d["theta1"] = e.theta1;
    d["theta1_dot"] = e.theta1_dot;
    d["theta2"] = e.theta2;
    d["theta2_dot"] = e.theta2_dot;
    d["theta3"] = e.theta3;
    d["theta3_dot"] = e.theta3_dot;
    d["s"] = e.s;
    d["v"] = e.v;
    return d;
}
