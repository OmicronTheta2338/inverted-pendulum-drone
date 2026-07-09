#include "gdext_two_arm_balance.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;
using armbalance::Config2;
using armbalance::Measurement2;

namespace {
double dget(const Dictionary &d, const char *k, double def) {
    return d.has(k) ? (double)d[k] : def;
}
}  // namespace

ArmBalance2::ArmBalance2() {}
ArmBalance2::~ArmBalance2() {}

void ArmBalance2::_bind_methods() {
    ClassDB::bind_method(D_METHOD("configure", "cfg"), &ArmBalance2::configure);
    ClassDB::bind_method(D_METHOD("reset"), &ArmBalance2::reset);
    ClassDB::bind_method(
        D_METHOD("update", "theta1", "theta1_dot", "theta2", "theta2_dot", "s", "v", "dt"),
        &ArmBalance2::update);
    ClassDB::bind_method(D_METHOD("last_saturated"), &ArmBalance2::last_saturated);
    ClassDB::bind_method(D_METHOD("get_estimate"), &ArmBalance2::get_estimate);
}

void ArmBalance2::configure(const Dictionary &cfg) {
    Config2 c;
    c.L1  = dget(cfg, "L1", c.L1);
    c.L2  = dget(cfg, "L2", c.L2);
    c.g   = dget(cfg, "g", c.g);
    c.k1  = dget(cfg, "k1", c.k1);
    c.k1d = dget(cfg, "k1d", c.k1d);
    c.k2  = dget(cfg, "k2", c.k2);
    c.k2d = dget(cfg, "k2d", c.k2d);
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

void ArmBalance2::reset() { ctrl_.reset(); }

double ArmBalance2::update(double theta1, double theta1_dot, double theta2,
                           double theta2_dot, double s, double v, double dt) {
    Measurement2 m;
    m.theta1 = theta1;
    m.theta1_dot = theta1_dot;
    m.theta2 = theta2;
    m.theta2_dot = theta2_dot;
    m.s = s;
    m.v = v;
    return ctrl_.update(m, dt);
}

bool ArmBalance2::last_saturated() const { return ctrl_.last_saturated(); }

Dictionary ArmBalance2::get_estimate() const {
    const Measurement2 e = ctrl_.estimate();
    Dictionary d;
    d["theta1"] = e.theta1;
    d["theta1_dot"] = e.theta1_dot;
    d["theta2"] = e.theta2;
    d["theta2_dot"] = e.theta2_dot;
    d["s"] = e.s;
    d["v"] = e.v;
    return d;
}
