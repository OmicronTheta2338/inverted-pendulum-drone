#include "gdext_swingup2.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;
using armbalance::SwingUp2Config;
using armbalance::SwingUp2Output;

namespace {
double dget(const Dictionary &d, const char *k, double def) {
    return d.has(k) ? (double)d[k] : def;
}
}  // namespace

SwingUp2::SwingUp2() {}
SwingUp2::~SwingUp2() {}

void SwingUp2::_bind_methods() {
    ClassDB::bind_method(D_METHOD("configure", "cfg"), &SwingUp2::configure);
    ClassDB::bind_method(D_METHOD("reset"), &SwingUp2::reset);
    ClassDB::bind_method(
        D_METHOD("update", "th1", "th1d", "th2", "th2d", "s", "v", "h_dev", "vy", "dt"),
        &SwingUp2::update);
    ClassDB::bind_method(D_METHOD("balancing"), &SwingUp2::balancing);
}

void SwingUp2::configure(const Dictionary &cfg) {
    SwingUp2Config c;
    c.L1 = dget(cfg, "L1", c.L1);
    c.L2 = dget(cfg, "L2", c.L2);
    c.g  = dget(cfg, "g", c.g);
    c.m1 = dget(cfg, "m1", c.m1);
    c.m2 = dget(cfg, "m2", c.m2);
    c.pump_gain = dget(cfg, "pump_gain", c.pump_gain);
    c.a_max_lat = dget(cfg, "a_max_lat", c.a_max_lat);
    c.a_max_up  = dget(cfg, "a_max_up", c.a_max_up);
    c.k_s = dget(cfg, "k_s", c.k_s);
    c.k_v = dget(cfg, "k_v", c.k_v);
    c.k_h = dget(cfg, "k_h", c.k_h);
    c.k_hv = dget(cfg, "k_hv", c.k_hv);
    c.catch_angle = dget(cfg, "catch_angle", c.catch_angle);
    c.catch_rate  = dget(cfg, "catch_rate", c.catch_rate);
    // 6-state LQR sub-config: default gains (compute_lqr2.py) unless overridden; L/g/a_max follow swingup
    c.balance.L1 = c.L1;
    c.balance.L2 = c.L2;
    c.balance.g = c.g;
    c.balance.a_max = c.a_max_lat;
    c.balance.k1  = dget(cfg, "k1", c.balance.k1);
    c.balance.k1d = dget(cfg, "k1d", c.balance.k1d);
    c.balance.k2  = dget(cfg, "k2", c.balance.k2);
    c.balance.k2d = dget(cfg, "k2d", c.balance.k2d);
    c.balance.ks  = dget(cfg, "ks", c.balance.ks);
    c.balance.kv  = dget(cfg, "kv", c.balance.kv);
    ctrl_.configure(c);
}

void SwingUp2::reset() { ctrl_.reset(); }

Dictionary SwingUp2::update(double th1, double th1d, double th2, double th2d,
        double s, double v, double h_dev, double vy, double dt) {
    const SwingUp2Output o = ctrl_.update(th1, th1d, th2, th2d, s, v, h_dev, vy, dt);
    Dictionary d;
    d["a_lat"] = o.a_lat;
    d["a_up"] = o.a_up;
    d["balancing"] = o.balancing;
    return d;
}

bool SwingUp2::balancing() const { return ctrl_.balancing(); }
