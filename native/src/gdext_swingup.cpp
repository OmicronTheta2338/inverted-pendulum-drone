#include "gdext_swingup.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;
using armbalance::SwingUpConfig;
using armbalance::SwingUpOutput;

namespace {
double dget(const Dictionary &d, const char *k, double def) {
    return d.has(k) ? (double)d[k] : def;
}
}  // namespace

SwingUp::SwingUp() {}
SwingUp::~SwingUp() {}

void SwingUp::_bind_methods() {
    ClassDB::bind_method(D_METHOD("configure", "cfg"), &SwingUp::configure);
    ClassDB::bind_method(D_METHOD("reset"), &SwingUp::reset);
    ClassDB::bind_method(
        D_METHOD("update", "theta", "theta_dot", "s", "v", "h_dev", "vy", "dt"),
        &SwingUp::update);
    ClassDB::bind_method(D_METHOD("balancing"), &SwingUp::balancing);
}

void SwingUp::configure(const Dictionary &cfg) {
    SwingUpConfig c;  // shipped defaults; override any provided key
    c.L         = dget(cfg, "L", c.L);
    c.g         = dget(cfg, "g", c.g);
    c.pump_gain = dget(cfg, "pump_gain", c.pump_gain);
    c.rate_eps  = dget(cfg, "rate_eps", c.rate_eps);
    c.a_max_lat = dget(cfg, "a_max_lat", c.a_max_lat);
    c.a_max_up  = dget(cfg, "a_max_up", c.a_max_up);
    c.k_s  = dget(cfg, "k_s", c.k_s);
    c.k_v  = dget(cfg, "k_v", c.k_v);
    c.k_h  = dget(cfg, "k_h", c.k_h);
    c.k_hv = dget(cfg, "k_hv", c.k_hv);
    c.catch_angle = dget(cfg, "catch_angle", c.catch_angle);
    c.catch_rate  = dget(cfg, "catch_rate", c.catch_rate);
    // LQR sub-config used after the catch: single-arm defaults, L/g follow the swingup, gains overridable
    c.balance.L = c.L;
    c.balance.g = c.g;
    c.balance.a_max      = c.a_max_lat;
    c.balance.k_theta    = dget(cfg, "bal_k_theta", c.balance.k_theta);
    c.balance.k_thetadot = dget(cfg, "bal_k_thetadot", c.balance.k_thetadot);
    c.balance.k_s        = dget(cfg, "bal_k_s", c.balance.k_s);
    c.balance.k_v        = dget(cfg, "bal_k_v", c.balance.k_v);
    ctrl_.configure(c);
}

void SwingUp::reset() { ctrl_.reset(); }

Dictionary SwingUp::update(double theta, double theta_dot, double s, double v,
        double h_dev, double vy, double dt) {
    const SwingUpOutput o = ctrl_.update(theta, theta_dot, s, v, h_dev, vy, dt);
    Dictionary d;
    d["a_lat"] = o.a_lat;
    d["a_up"] = o.a_up;
    d["balancing"] = o.balancing;
    return d;
}

bool SwingUp::balancing() const { return ctrl_.balancing(); }
