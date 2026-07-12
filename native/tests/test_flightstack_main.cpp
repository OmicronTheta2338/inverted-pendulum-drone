// Smoke test for the portable native flight stack (no Godot, no ArduPilot).
//
// Proves the core compiles standalone and produces sane motor PWMs for the two
// canonical conditions:
//   1. level hover, at rest  -> all 4 upright motors near the hover PWM, canted idle
//   2. level, +lateral step  -> +X canted motors (M6,M7) spin up, -X (M5,M8) stay low
//
// This is a UNIT sanity check of the mixer + controller wiring, NOT a closed-loop
// flight test -- that happens against the real Jolt plant in Godot (sim-v-1).
//
//   g++ -O2 -std=c++17 -I../src test_flightstack_main.cpp \
//       ../src/ap_motors6dof.cpp ../src/ap_attitude_control.cpp ../src/mode_accel.cpp \
//       -o test_flightstack && ./test_flightstack
#include "mode_accel.h"

#include <cstdio>

using namespace apstack;

static const char *MNAME[8] = { "M1u", "M2u", "M3u", "M4u", "M5c", "M6c", "M7c", "M8c" };

static void print_pwm(const char *tag, const int pwm[8]) {
    printf("%-18s ", tag);
    for (int i = 0; i < 8; i++) printf("%s=%d ", MNAME[i], pwm[i]);
    printf("\n");
}

int main() {
    ModeAccel mode;
    AttitudeControl att;
    Motors6DoF motors;
    int pwm[8];
    const float dt = 1.0f / 400.0f;

    // Level attitude, at the origin, at rest. accel_ef at hover = (0,0,-g) (specific force up).
    AhrsState ahrs;
    ahrs.accel_ef = Vec3f(0, 0, -GRAVITY_MSS);
    ahrs.have_vel = true;
    ahrs.have_pos = true;
    ahrs.roll = ahrs.pitch = ahrs.yaw = 0.0f;

    // --- 1. hover: up_stick 0 (hover), airborne (on_ground=false) ---
    AccelInput in;
    in.up_stick = 0.0f;
    // settle a few ticks so the FF reference and integrators reach steady hover
    for (int k = 0; k < 40; k++) mode.run(in, ahrs, /*on_ground*/ false, dt, att, motors, pwm);
    print_pwm("hover", pwm);
    bool ok_hover = true;
    for (int i = 0; i < 4; i++) if (pwm[i] < 1120 || pwm[i] > 1300) ok_hover = false;   // ~1174 expected
    printf("  upright motors in hover band [1120,1300]: %s\n", ok_hover ? "PASS" : "FAIL");

    // --- 2. +lateral step (level 2 -> +6 m/s^2): +X canted (M6,M7) should exceed -X (M5,M8) ---
    ModeAccel mode2;
    AttitudeControl att2;
    Motors6DoF motors2;
    AccelInput in2;
    in2.lat_stick = 0.5f;   // 0.5 * 12 = +6 m/s^2
    for (int k = 0; k < 20; k++) mode2.run(in2, ahrs, false, dt, att2, motors2, pwm);
    print_pwm("+lateral step", pwm);
    const bool ok_lat = (pwm[5] > pwm[4] + 20) && (pwm[6] > pwm[7] + 20);   // M6>M5, M7>M8
    printf("  +X canted (M6,M7) > -X canted (M5,M8): %s\n", ok_lat ? "PASS" : "FAIL");

    // --- 3. on the ground, no lift commanded -> all motors idle at PWM_MIN ---
    ModeAccel mode3;
    AttitudeControl att3;
    Motors6DoF motors3;
    AccelInput in3;
    mode3.run(in3, ahrs, /*on_ground*/ true, dt, att3, motors3, pwm);
    print_pwm("grounded idle", pwm);
    bool ok_ground = true;
    for (int i = 0; i < 8; i++) if (pwm[i] != 1000) ok_ground = false;
    printf("  all motors at PWM_MIN on ground: %s\n", ok_ground ? "PASS" : "FAIL");

    const bool all = ok_hover && ok_lat && ok_ground;
    printf("\n%s\n", all ? "ALL SMOKE CHECKS PASS" : "SMOKE CHECKS FAILED");
    return all ? 0 : 1;
}
