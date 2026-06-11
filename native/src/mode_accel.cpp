#include "mode_accel.h"

#include <cmath>

namespace apstack {

// ---- constants (see ardupilot/ArduCopter/mode_accel.cpp for the full rationale) ----
// Lateral FF gain: realised m/s^2 at full set_lateral(1.0). Calibrated for body mass 1.0 kg.
static constexpr float ACCEL_LAT_MAX_MSS       = 38.0f;
static constexpr float ACCEL_LAT_MAX_MSS_NEG_A = 40.6f;   // magnitude-scheduled negative rail
static constexpr float ACCEL_LAT_MAX_MSS_NEG_B = 0.25f;
static constexpr float ACCEL_LAT_CMD_MSS       = 12.0f;   // accel at full lateral stick

static constexpr float ACCEL_FWD_KP  = 1.5f;    // forward-hold: -KP*x -KD*v -KI*int
static constexpr float ACCEL_FWD_KD  = 2.5f;
static constexpr float ACCEL_FWD_KI  = 0.5f;
static constexpr float ACCEL_FWD_IMAX    = 1.0f;
static constexpr float ACCEL_FWD_MAX_MSS = 2.5f;

static constexpr float ACCEL_KP_LAT = 0.0f;     // P disabled (delayed-loop; FF only)
static constexpr float ACCEL_KP_UP  = 0.0f;
static constexpr float ACCEL_P_GATE = 0.6f;
static constexpr float ACCEL_KI_LAT = 2.0f;     // gentle I trims steady residual
static constexpr float ACCEL_KI_UP  = 2.0f;
static constexpr float ACCEL_ACC_IMAX = 10.0f;
static constexpr float ACCEL_VERT_THR_PER_MSS      = 0.0132f;  // throttle per m/s^2 above hover
static constexpr float ACCEL_HOVER_THR             = 0.096f;   // true hover throttle (0 net accel)
static constexpr float ACCEL_VERT_THR_PER_MSS_DOWN = 0.0124f;  // shallower below hover (concave curve)

static constexpr float ACCEL_MOTOR_TAU_LAT   = 0.028f;  // inverse-model overdrive: lateral eff. lag
static constexpr float ACCEL_MOTOR_TAU_UP    = 0.037f;  // vertical eff. lag
static constexpr float ACCEL_MOTOR_TAU_LAT_K = 0.0015f; // lat tau grows with |cmd|
static constexpr float ACCEL_FF_TAU  = 0.008f;  // fast FF reference low-pass
static constexpr float ACCEL_FB_DELAY = 0.015f; // feedback-ref delay (~a_meas lag)
static constexpr float ACCEL_DRAG_K_LAT = 0.05f;// lateral drag-cancel FF (<= true drag coeff)

static constexpr float PEND_MASS_RATIO = 0.10f; // bob / drone-body mass
static constexpr float PEND_L          = 0.60f; // pivot->bob length [m]
static constexpr float PEND_PREDICT_S  = 0.042f;// predict arm state fwd by ~loop delay

static constexpr float PARK_ALT_M   = 10.0f;
static constexpr float PARK_KP_POS  = 1.0f;
static constexpr float PARK_VXY_MAX = 3.0f;
static constexpr float PARK_VZ_MAX  = 2.5f;
static constexpr float PARK_KV      = 1.5f;
static constexpr float PARK_ACC_MAX = 2.5f;
static constexpr float PARK_AZ_MAX  = 6.0f;

void ModeAccel::init() {
    _have_fwd_ref = false;
    _fwd_I = 0.0f;
    _fwd_cy_ref = 1.0f;
    _fwd_sy_ref = 0.0f;
    _lat_I = 0.0f;
    _up_I = 0.0f;
    _lat_ref_ff = 0.0f;
    _up_ref_ff = 0.0f;
    for (int i = 0; i < HIST; i++) { _lat_ref_hist[i] = 0.0f; _up_ref_hist[i] = 0.0f; }
    _hist_idx = 0;
}

void ModeAccel::run(const AccelInput &in, const AhrsState &ahrs, bool on_ground, float dt,
                    AttitudeControl &att, Motors6DoF &motors, int pwm_out[8]) {
    const float G_Dt = dt;

    // ---- decode commands (typed; the firmware read these from RC channels) ----
    const float a_lat_cmd = constrain_float(in.lat_stick, -1.0f, 1.0f) * ACCEL_LAT_CMD_MSS;
    const float a_up_cmd  = constrain_float(in.up_stick,  -1.0f, 1.0f) * GRAVITY_MSS;   // [-g,+g]
    const float target_yaw_rate_rads = in.yaw_rate;
    const bool park = in.park;

    // ---- body-frame yaw basis, forward velocity & displacement ----
    const float yaw = ahrs.yaw;
    const float cy = cosf(yaw);
    const float sy = sinf(yaw);

    Vec3f vel_ned = ahrs.vel_ned;
    const bool have_vel = ahrs.have_vel;
    float v_up = have_vel ? -vel_ned.z : 0.0f;   // up (NED z is down)
    float v_fwd = 0.0f;

    // ---- closed-loop accel control (FF + P + bounded I) ----
    const Vec3f accel_ef = ahrs.accel_ef;
    const float a_lat_meas = -accel_ef.x * sy + accel_ef.y * cy;
    const float a_up_meas  = -accel_ef.z - GRAVITY_MSS;

    // FAST feed-forward reference + inverse-motor-model overdrive, saturation-aware.
    const float tau_lat = constrain_float(ACCEL_MOTOR_TAU_LAT + ACCEL_MOTOR_TAU_LAT_K * fabsf(a_lat_cmd),
                                          ACCEL_MOTOR_TAU_LAT, 0.040f);
    float lat_ff = _lat_ref_ff + tau_lat * (a_lat_cmd - _lat_ref_ff) / ACCEL_FF_TAU;
    float lat_ref_new;
    if (lat_ff > ACCEL_LAT_MAX_MSS || lat_ff < -ACCEL_LAT_MAX_MSS) {
        const float rail = (lat_ff > 0.0f) ? ACCEL_LAT_MAX_MSS : -ACCEL_LAT_MAX_MSS;
        lat_ff = rail;
        lat_ref_new = _lat_ref_ff + ((rail - _lat_ref_ff) / tau_lat) * G_Dt;   // ramp toward rail
    } else {
        lat_ref_new = _lat_ref_ff + ((a_lat_cmd - _lat_ref_ff) / ACCEL_FF_TAU) * G_Dt;
    }
    if ((a_lat_cmd - _lat_ref_ff) * (a_lat_cmd - lat_ref_new) < 0.0f) { lat_ref_new = a_lat_cmd; }
    _lat_ref_ff = lat_ref_new;

    const float up_ff_max = (1.0f - ACCEL_HOVER_THR) / ACCEL_VERT_THR_PER_MSS;
    const float up_ff_min = -ACCEL_HOVER_THR / ACCEL_VERT_THR_PER_MSS;
    float up_ff = _up_ref_ff + ACCEL_MOTOR_TAU_UP * (a_up_cmd - _up_ref_ff) / ACCEL_FF_TAU;
    float up_ref_new;
    if (up_ff > up_ff_max || up_ff < up_ff_min) {
        const float rail = (up_ff > up_ff_max) ? up_ff_max : up_ff_min;
        up_ff = rail;
        up_ref_new = _up_ref_ff + ((rail - _up_ref_ff) / ACCEL_MOTOR_TAU_UP) * G_Dt;
    } else {
        up_ref_new = _up_ref_ff + ((a_up_cmd - _up_ref_ff) / ACCEL_FF_TAU) * G_Dt;
    }
    if ((a_up_cmd - _up_ref_ff) * (a_up_cmd - up_ref_new) < 0.0f) { up_ref_new = a_up_cmd; }
    _up_ref_ff = up_ref_new;

    // DELAYED feedback reference (ring buffer)
    const int dn = constrain_int((int)lroundf(ACCEL_FB_DELAY / G_Dt), 0, HIST - 1);
    _lat_ref_hist[_hist_idx] = _lat_ref_ff;
    _up_ref_hist[_hist_idx]  = _up_ref_ff;
    const int rd = (_hist_idx + HIST - dn) % HIST;
    const float lat_ref_fb = _lat_ref_hist[rd];
    const float up_ref_fb  = _up_ref_hist[rd];
    _hist_idx = (_hist_idx + 1) % HIST;
    const float lat_err = lat_ref_fb - a_lat_meas;
    const float up_err  = up_ref_fb  - a_up_meas;

    // GATED P (disabled: KP=0) + bounded I with saturation anti-windup
    const bool lat_steady = fabsf(a_lat_cmd - _lat_ref_ff) < ACCEL_P_GATE;
    const bool up_steady  = fabsf(a_up_cmd  - _up_ref_ff ) < ACCEL_P_GATE;
    const float kp_lat = lat_steady ? ACCEL_KP_LAT : 0.0f;
    const float kp_up  = up_steady  ? ACCEL_KP_UP  : 0.0f;
    if (!park) {
        const float lat_out_pre = lat_ff + kp_lat * lat_err + _lat_I;
        const float up_out_pre  = up_ff  + kp_up  * up_err  + _up_I;
        const bool lat_railed = (lat_out_pre >=  ACCEL_LAT_MAX_MSS && lat_err > 0.0f) ||
                                (lat_out_pre <= -ACCEL_LAT_MAX_MSS && lat_err < 0.0f);
        const bool up_railed  = (up_out_pre  <= -GRAVITY_MSS && up_err < 0.0f);
        if (!lat_railed) {
            _lat_I = constrain_float(_lat_I + lat_err * ACCEL_KI_LAT * G_Dt, -ACCEL_ACC_IMAX, ACCEL_ACC_IMAX);
        }
        if (!up_railed) {
            _up_I = constrain_float(_up_I + up_err * ACCEL_KI_UP * G_Dt, -ACCEL_ACC_IMAX, ACCEL_ACC_IMAX);
        }
    }
    float a_lat_out = lat_ff + kp_lat * lat_err + _lat_I;
    float a_up_out  = up_ff  + kp_up  * up_err  + _up_I;

    // ---- lateral drag feed-forward: cancel velocity-dependent aero drag ----
    const float v_lat = -vel_ned.x * sy + vel_ned.y * cy;
    a_lat_out += ACCEL_DRAG_K_LAT * v_lat;

    // ---- forward-hold: pin displacement/velocity to the takeoff plane ----
    Vec3f pos_ned = ahrs.pos_ned;
    float x_fwd = 0.0f;
    bool have_pos = ahrs.have_pos;
    if (have_pos) {
        if (!_have_fwd_ref) {
            _fwd_ref_n = pos_ned.x;
            _fwd_ref_e = pos_ned.y;
            _fwd_cy_ref = cy;
            _fwd_sy_ref = sy;
            _have_fwd_ref = true;
        }
        x_fwd = (pos_ned.x - _fwd_ref_n) * _fwd_cy_ref + (pos_ned.y - _fwd_ref_e) * _fwd_sy_ref;
        if (have_vel) {
            v_fwd = vel_ned.x * _fwd_cy_ref + vel_ned.y * _fwd_sy_ref;
        }
    }
    float a_fwd = -ACCEL_FWD_KP * x_fwd - ACCEL_FWD_KD * v_fwd - _fwd_I;
    if (fabsf(a_fwd) < ACCEL_FWD_MAX_MSS) {
        _fwd_I = constrain_float(_fwd_I + ACCEL_FWD_KI * x_fwd * G_Dt, -ACCEL_FWD_IMAX, ACCEL_FWD_IMAX);
    }
    a_fwd = constrain_float(a_fwd, -ACCEL_FWD_MAX_MSS, ACCEL_FWD_MAX_MSS);

    // ---- PARK override: auto-fly to origin NE @ PARK_ALT and hold ----
    if (park && have_pos) {
        const float vN_des = constrain_float(-PARK_KP_POS * pos_ned.x, -PARK_VXY_MAX, PARK_VXY_MAX);
        const float vE_des = constrain_float(-PARK_KP_POS * pos_ned.y, -PARK_VXY_MAX, PARK_VXY_MAX);
        float a_N = (vN_des - vel_ned.x) * PARK_KV;
        float a_E = (vE_des - vel_ned.y) * PARK_KV;
        const float amag = sqrtf(a_N * a_N + a_E * a_E);
        if (amag > PARK_ACC_MAX) {
            const float s = PARK_ACC_MAX / amag;
            a_N *= s; a_E *= s;
        }
        a_fwd     =  a_N * cy + a_E * sy;
        a_lat_out = -a_N * sy + a_E * cy;
        const float alt = -pos_ned.z;
        const float vup_des = constrain_float(PARK_KP_POS * (PARK_ALT_M - alt), -PARK_VZ_MAX, PARK_VZ_MAX);
        a_up_out = constrain_float((vup_des - v_up) * PARK_KV, -PARK_AZ_MAX, PARK_AZ_MAX);
        _have_fwd_ref = false;
        _fwd_I = 0.0f;
        _lat_I = 0.0f;
        _up_I = 0.0f;
        _lat_ref_ff = 0.0f;
        _up_ref_ff = 0.0f;
    }

    // ---- pendulum reaction-force feed-forward (0/1/2 hanging arms) ----
    {
        const int n_arms = constrain_int(in.n_arms, 0, 2);
        const float w_lat = lat_ref_fb;                 // (a_d - g_vec)_lateral
        const float w_up  = up_ref_fb + GRAVITY_MSS;    // (a_d - g_vec)_up
        float a_pend_lat = 0.0f, a_pend_up = 0.0f;
        for (int i = 0; i < n_arms; i++) {
            const float theta0 = in.theta[i];
            const float rate0  = in.theta_dot[i];
            const float th_ddot = (-w_lat * cosf(theta0) - w_up * sinf(theta0)) / PEND_L;
            const float theta = theta0 + rate0 * PEND_PREDICT_S + 0.5f * th_ddot * PEND_PREDICT_S * PEND_PREDICT_S;
            const float rate  = rate0 + th_ddot * PEND_PREDICT_S;
            const float su = sinf(theta), cu = -cosf(theta);   // u = (sinθ, -cosθ)
            const float mag = PEND_MASS_RATIO * ((w_lat * su + w_up * cu) - PEND_L * rate * rate);
            a_pend_lat += mag * su;
            a_pend_up  += mag * cu;
        }
        a_lat_out += a_pend_lat;
        a_up_out  += a_pend_up;
    }

    // ---- spool: idle on the ground until lift is commanded (or PARK wants takeoff) ----
    const bool want_lift = (a_up_cmd > 0.0f) || park;
    const bool airborne = want_lift || !on_ground;

    // vertical accel -> DIRECT throttle (asymmetric slope; /cos(pitch) for world-vertical thrust)
    const float cp = cosf(constrain_float(ahrs.pitch, -1.04f, 1.04f));
    const float vert_slope = (a_up_out >= 0.0f) ? ACCEL_VERT_THR_PER_MSS : ACCEL_VERT_THR_PER_MSS_DOWN;
    float throttle_out = (ACCEL_HOVER_THR + vert_slope * a_up_out) / cp;

    if (!airborne) {
        // ground: hold level, re-capture the hold plane on the next takeoff, clean state
        a_fwd = 0.0f;
        _have_fwd_ref = false;
        _fwd_I = 0.0f;
        _lat_I = 0.0f;
        _up_I = 0.0f;
        _lat_ref_ff = 0.0f;
        _up_ref_ff = 0.0f;
        att.reset();
        att.reset_yaw(yaw);
        motors.set_throttle(0.0f);
        motors.set_roll(0.0f);
        motors.set_pitch(0.0f);
        motors.set_yaw(0.0f);
        motors.set_lateral(0.0f);
        motors.output(false, pwm_out);
        return;
    }

    // ---- attitude: roll=0, pitch from forward-hold, yaw = pilot rate (heading-hold) ----
    const Vec3f thrust_vec(a_fwd * cy, a_fwd * sy, -GRAVITY_MSS);
    att.input_thrust_vector_rate_heading(thrust_vec, target_yaw_rate_rads,
                                         ahrs.roll, ahrs.pitch, ahrs.yaw, ahrs.gyro, dt, motors);

    // vertical throttle (direct)
    motors.set_throttle(constrain_float(throttle_out, 0.0f, 1.0f));

    // lateral (catch) axis: DIRECT canted thrust, throttle-independent. Magnitude-scheduled
    // negative rail (empirical LUT) because the canted mixer's negative gain is nonlinear.
    float lat_rail_out = ACCEL_LAT_MAX_MSS;
    if (a_lat_out < 0.0f) {
        lat_rail_out = constrain_float(ACCEL_LAT_MAX_MSS_NEG_A - ACCEL_LAT_MAX_MSS_NEG_B * fabsf(a_lat_out),
                                       36.5f, 40.5f);
    }
    motors.set_lateral(constrain_float(a_lat_out / lat_rail_out, -1.0f, 1.0f));

    motors.output(true, pwm_out);
}

}  // namespace apstack
