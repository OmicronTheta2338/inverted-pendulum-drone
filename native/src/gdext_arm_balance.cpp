#include "gdext_arm_balance.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/string.hpp>

using namespace godot;
using armbalance::Config;
using armbalance::Law;
using armbalance::Measurement;

namespace {
double dget(const Dictionary &d, const char *k, double def) {
    return d.has(k) ? (double)d[k] : def;
}
}  // namespace

ArmBalance::ArmBalance() {}
ArmBalance::~ArmBalance() {}

void ArmBalance::_bind_methods() {
    ClassDB::bind_method(D_METHOD("configure", "cfg"), &ArmBalance::configure);
    ClassDB::bind_method(D_METHOD("reset"), &ArmBalance::reset);
    ClassDB::bind_method(D_METHOD("update", "theta", "theta_dot", "s", "v", "dt"),
                         &ArmBalance::update);
    ClassDB::bind_method(D_METHOD("last_saturated"), &ArmBalance::last_saturated);
    ClassDB::bind_method(D_METHOD("get_estimate"), &ArmBalance::get_estimate);
}

void ArmBalance::configure(const Dictionary &cfg) {
    Config c;  // start from shipped defaults, override any provided key
    c.L          = dget(cfg, "L", c.L);
    c.g          = dget(cfg, "g", c.g);
    c.k_theta    = dget(cfg, "k_theta", c.k_theta);
    c.k_thetadot = dget(cfg, "k_thetadot", c.k_thetadot);
    c.k_s        = dget(cfg, "k_s", c.k_s);
    c.k_v        = dget(cfg, "k_v", c.k_v);
    c.s_ref      = dget(cfg, "s_ref", c.s_ref);
    c.a_max      = dget(cfg, "a_max", c.a_max);
    c.fc_theta     = dget(cfg, "fc_theta", c.fc_theta);
    c.fc_theta_dot = dget(cfg, "fc_theta_dot", c.fc_theta_dot);
    c.fc_s         = dget(cfg, "fc_s", c.fc_s);
    c.fc_v         = dget(cfg, "fc_v", c.fc_v);
    c.pid_kp_th  = dget(cfg, "pid_kp_th", c.pid_kp_th);
    c.pid_kd_th  = dget(cfg, "pid_kd_th", c.pid_kd_th);
    c.pid_kp_s   = dget(cfg, "pid_kp_s", c.pid_kp_s);
    c.pid_kd_s   = dget(cfg, "pid_kd_s", c.pid_kd_s);
    c.pid_ki_s   = dget(cfg, "pid_ki_s", c.pid_ki_s);
    c.pid_i_max  = dget(cfg, "pid_i_max", c.pid_i_max);
    if (cfg.has("law")) {
        const String l = cfg["law"];
        c.law = (l == String("PID")) ? Law::PID : Law::LQR;
    }
    ctrl_.configure(c);
}

void ArmBalance::reset() { ctrl_.reset(); }

double ArmBalance::update(double theta, double theta_dot, double s, double v, double dt) {
    Measurement m;
    m.theta = theta;
    m.theta_dot = theta_dot;
    m.s = s;
    m.v = v;
    return ctrl_.update(m, dt);
}

bool ArmBalance::last_saturated() const { return ctrl_.last_saturated(); }

Dictionary ArmBalance::get_estimate() const {
    const Measurement e = ctrl_.estimate();
    Dictionary d;
    d["theta"] = e.theta;
    d["theta_dot"] = e.theta_dot;
    d["s"] = e.s;
    d["v"] = e.v;
    return d;
}
