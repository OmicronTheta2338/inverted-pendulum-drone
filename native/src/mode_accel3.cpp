#include "mode_accel3.h"

#include <cmath>

namespace apstack {

// ---- constants: byte-identical to mode_accel.cpp, plus link-3 mass/length ----
static constexpr float ACCEL_LAT_MAX_MSS       = 38.0f;
static constexpr float ACCEL_LAT_MAX_MSS_NEG_A = 40.6f;
static constexpr float ACCEL_LAT_MAX_MSS_NEG_B = 0.25f;
static constexpr float ACCEL_LAT_CMD_MSS       = 12.0f;

static constexpr float ACCEL_FWD_KP  = 0.5f;
static constexpr float ACCEL_FWD_KD  = 1.3f;
static constexpr float ACCEL_FWD_KI  = 0.2f;
static constexpr float ACCEL_FWD_IMAX    = 1.0f;
static constexpr float ACCEL_FWD_MAX_MSS = 2.5f;
static constexpr float ACCEL_FWD_HOLD_KP  = 0.15f; // hold_level: gentle fore/aft position recenter gain
static constexpr float ACCEL_FWD_HOLD_KV  = 0.5f;  // hold_level: gentle fore/aft velocity damping gain
static constexpr float ACCEL_FWD_HOLD_MAX = 0.8f;  // hold_level: fore/aft accel cap [m/s^2] (pitch <= ~4.7 deg)

static constexpr float ACCEL_KP_LAT = 0.0f;
static constexpr float ACCEL_KP_UP  = 0.0f;
static constexpr float ACCEL_P_GATE = 0.6f;
static constexpr float ACCEL_KI_LAT = 2.0f;
static constexpr float ACCEL_KI_UP  = 2.0f;
static constexpr float ACCEL_ACC_IMAX = 10.0f;
static constexpr float ACCEL_VERT_THR_PER_MSS      = 0.0132f;
static constexpr float ACCEL_HOVER_THR             = 0.096f;
static constexpr float ACCEL_VERT_THR_PER_MSS_DOWN = 0.0124f;

static constexpr float ACCEL_MOTOR_TAU_LAT   = 0.028f;
static constexpr float ACCEL_MOTOR_TAU_UP    = 0.037f;
static constexpr float ACCEL_MOTOR_TAU_LAT_K = 0.0015f;
static constexpr float ACCEL_FF_TAU  = 0.008f;
static constexpr float ACCEL_FB_DELAY = 0.015f;
static constexpr float ACCEL_DRAG_K_LAT = 0.05f;

static constexpr float PEND_MASS_RATIO = 0.10f; // bob1 / drone-body mass (series link 1)
static constexpr float PEND_M2_R       = 0.10f; // bob2 / drone-body mass (series link 2)
static constexpr float PEND_M3_R       = 0.10f; // bob3 / drone-body mass (series link 3)
static constexpr float PEND_L1         = 0.70f; // link1: pivot -> bob1 length [m]
static constexpr float PEND_L2         = 0.60f; // link2: bob1  -> bob2 length [m]
static constexpr float PEND_L3         = 0.60f; // link3: bob2  -> bob3 length [m]
static constexpr float PEND_PREDICT_S  = 0.0f;  // Smith horizon: 0 = off (tune in the sandbox).

static constexpr float PARK_ALT_M   = 10.0f;
static constexpr float PARK_KP_POS  = 1.0f;
static constexpr float PARK_VXY_MAX = 3.0f;
static constexpr float PARK_VZ_MAX  = 2.5f;
static constexpr float PARK_KV      = 1.5f;
static constexpr float PARK_ACC_MAX = 2.5f;
static constexpr float PARK_AZ_MAX  = 6.0f;

void ModeAccel3::init() {
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

void ModeAccel3::run(const AccelInput3 &in, const AhrsState &ahrs, bool on_ground, float dt,
                     AttitudeControl &att, Motors6DoF &motors, int pwm_out[8]) {
    const float G_Dt = dt;

    // ---- decode commands ----
    const float a_lat_cmd = constrain_float(in.lat_stick, -1.0f, 1.0f) * ACCEL_LAT_CMD_MSS;
    const float a_up_cmd  = constrain_float(in.up_stick,  -1.0f, 1.0f) * GRAVITY_MSS;
    const float target_yaw_rate_rads = in.yaw_rate;
    const bool park = in.park;

    // ---- body-frame yaw basis, forward velocity & displacement ----
    const float yaw = ahrs.yaw;
    const float cy = cosf(yaw);
    const float sy = sinf(yaw);

    Vec3f vel_ned = ahrs.vel_ned;
    const bool have_vel = ahrs.have_vel;
    float v_up = have_vel ? -vel_ned.z : 0.0f;
    float v_fwd = 0.0f;

    // ---- closed-loop accel control (FF + P + bounded I) ----
    const Vec3f accel_ef = ahrs.accel_ef;
    const float a_lat_meas = -accel_ef.x * sy + accel_ef.y * cy;
    const float a_up_meas  = -accel_ef.z - GRAVITY_MSS;

    const float tau_lat = constrain_float(ACCEL_MOTOR_TAU_LAT + ACCEL_MOTOR_TAU_LAT_K * fabsf(a_lat_cmd),
                                          ACCEL_MOTOR_TAU_LAT, 0.040f);
    float lat_ff = _lat_ref_ff + tau_lat * (a_lat_cmd - _lat_ref_ff) / ACCEL_FF_TAU;
    float lat_ref_new;
    if (lat_ff > ACCEL_LAT_MAX_MSS || lat_ff < -ACCEL_LAT_MAX_MSS) {
        const float rail = (lat_ff > 0.0f) ? ACCEL_LAT_MAX_MSS : -ACCEL_LAT_MAX_MSS;
        lat_ff = rail;
        lat_ref_new = _lat_ref_ff + ((rail - _lat_ref_ff) / tau_lat) * G_Dt;
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

    const int dn = constrain_int((int)lroundf(ACCEL_FB_DELAY / G_Dt), 0, HIST - 1);
    _lat_ref_hist[_hist_idx] = _lat_ref_ff;
    _up_ref_hist[_hist_idx]  = _up_ref_ff;
    const int rd = (_hist_idx + HIST - dn) % HIST;
    const float lat_ref_fb = _lat_ref_hist[rd];
    const float up_ref_fb  = _up_ref_hist[rd];
    _hist_idx = (_hist_idx + 1) % HIST;
    const float lat_err = lat_ref_fb - a_lat_meas;
    const float up_err  = up_ref_fb  - a_up_meas;

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

    // ---- lateral drag feed-forward ----
    const float v_lat = -vel_ned.x * sy + vel_ned.y * cy;
    a_lat_out += ACCEL_DRAG_K_LAT * v_lat;

    // ---- forward-hold ----
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

    // ---- PARK override ----
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

    // ---- pendulum reaction-force feed-forward: SERIES chain (0/1/2/3 links) ----
    // The chain loads the drone through link-1's root hinge ONLY. Reaction on the cart:
    //     R = sum_i m_i (g_vec - a_bob_i)        (isolate the chain; Newton's 3rd law)
    // Angular accelerations come from the coupled n-link EOM under effective gravity
    // g_eff = g_vec - a_d. The FF adds -R/m_d to cancel it. Masses are ratios (m_i/m_d),
    // so R is already an acceleration.
    //   n<=2  : byte-identical to mode_accel.cpp (2x2 hardcoded solve).
    //   n==3  : symmetric 3x3 coupled mass matrix, general form (see plan appendix).
    {
        const int n = constrain_int(in.n_arms, 0, 3);
        float a_pend_lat = 0.0f, a_pend_up = 0.0f;
        const float gx = -(lat_ref_fb);                 // g_eff lateral = -(a_d)_lat
        const float gy = -(up_ref_fb + GRAVITY_MSS);    // g_eff up      = -((a_d)_up + g)

        if (n == 1 || n == 2) {
            const float m1 = PEND_MASS_RATIO;
            const float m2 = (n >= 2) ? PEND_M2_R : 0.0f;
            const float M  = m1 + m2;
            const float L1 = PEND_L1, L2 = PEND_L2;
            auto solve = [&](float t1, float t2, float o1, float o2, float &d1, float &d2) {
                const float c1 = cosf(t1), s1 = sinf(t1), c2 = cosf(t2), s2 = sinf(t2);
                const float dD = t1 - t2, cD = cosf(dD), sD = sinf(dD);
                const float b1 = M * (gx * c1 + gy * s1) - m2 * L2 * sD * o2 * o2;
                const float b2 = (gx * c2 + gy * s2) + L1 * sD * o1 * o1;
                const float det = L1 * L2 * (M - m2 * cD * cD);
                d1 = (L2 * b1 - m2 * L2 * cD * b2) / det;
                d2 = (n >= 2) ? (-L1 * cD * b1 + M * L1 * b2) / det : 0.0f;
            };
            float th1 = in.theta[0], w1 = in.theta_dot[0];
            float th2 = (n >= 2) ? in.theta[1] : 0.0f;
            float w2  = (n >= 2) ? in.theta_dot[1] : 0.0f;
            const float T = PEND_PREDICT_S;
            float dd1, dd2;
            solve(th1, th2, w1, w2, dd1, dd2);
            th1 += w1 * T + 0.5f * dd1 * T * T;  w1 += dd1 * T;
            th2 += w2 * T + 0.5f * dd2 * T * T;  w2 += dd2 * T;
            solve(th1, th2, w1, w2, dd1, dd2);
            const float c1 = cosf(th1), s1 = sinf(th1);
            const float c2 = cosf(th2), s2 = sinf(th2);
            float R_lat = M * gx - M * L1 * (dd1 * c1 - w1 * w1 * s1);
            float R_up  = M * gy - M * L1 * (dd1 * s1 + w1 * w1 * c1);
            if (n >= 2) {
                R_lat -= m2 * L2 * (dd2 * c2 - w2 * w2 * s2);
                R_up  -= m2 * L2 * (dd2 * s2 + w2 * w2 * c2);
            }
            a_pend_lat = -R_lat;
            a_pend_up  = -R_up;
        } else if (n == 3) {
            // series triple: mu_j = sum_{k>=j} m_k (mass at/below joint j).
            const float Ln[3] = { PEND_L1, PEND_L2, PEND_L3 };
            const float mu[3] = { PEND_MASS_RATIO + PEND_M2_R + PEND_M3_R,
                                  PEND_M2_R + PEND_M3_R, PEND_M3_R };
            float th[3] = { in.theta[0], in.theta[1], in.theta[2] };
            float w[3]  = { in.theta_dot[0], in.theta_dot[1], in.theta_dot[2] };

            // Coupled EOM solver: symmetric A dd = b under effective gravity g_eff=(gx,gy).
            //   A_ij = L_i L_j mu_max(i,j) cos(th_i-th_j)
            //   b_i  = L_i mu_i (g_eff . u_perp_i) - sum_{j!=i} L_i L_j mu_max(i,j) sin(th_i-th_j) w_j^2
            // Solved with an explicit 3x3 cofactor inverse (well-conditioned near upright).
            auto solve3 = [&](const float t[3], const float o[3], float dd[3]) {
                float A[3][3], b[3];
                for (int i = 0; i < 3; i++) {
                    const float ci = cosf(t[i]), si = sinf(t[i]);
                    b[i] = Ln[i] * mu[i] * (gx * ci + gy * si);
                    for (int j = 0; j < 3; j++) {
                        const int mx = (i > j) ? i : j;
                        A[i][j] = Ln[i] * Ln[j] * mu[mx] * cosf(t[i] - t[j]);
                        if (j != i) {
                            b[i] -= Ln[i] * Ln[j] * mu[mx] * sinf(t[i] - t[j]) * o[j] * o[j];
                        }
                    }
                }
                const float c00 = A[1][1]*A[2][2] - A[1][2]*A[2][1];
                const float c01 = A[1][2]*A[2][0] - A[1][0]*A[2][2];
                const float c02 = A[1][0]*A[2][1] - A[1][1]*A[2][0];
                const float det = A[0][0]*c00 + A[0][1]*c01 + A[0][2]*c02;
                if (fabsf(det) < 1e-12f) { dd[0] = dd[1] = dd[2] = 0.0f; return; }
                const float inv = 1.0f / det;
                const float c10 = A[0][2]*A[2][1] - A[0][1]*A[2][2];
                const float c11 = A[0][0]*A[2][2] - A[0][2]*A[2][0];
                const float c12 = A[0][1]*A[2][0] - A[0][0]*A[2][1];
                const float c20 = A[0][1]*A[1][2] - A[0][2]*A[1][1];
                const float c21 = A[0][2]*A[1][0] - A[0][0]*A[1][2];
                const float c22 = A[0][0]*A[1][1] - A[0][1]*A[1][0];
                // dd = A^-1 b, with A^-1 = adjugate/det = cofactor^T / det
                dd[0] = inv * (c00*b[0] + c10*b[1] + c20*b[2]);
                dd[1] = inv * (c01*b[0] + c11*b[1] + c21*b[2]);
                dd[2] = inv * (c02*b[0] + c12*b[1] + c22*b[2]);
            };

            const float T = PEND_PREDICT_S;
            float dd[3];
            solve3(th, w, dd);
            for (int i = 0; i < 3; i++) { th[i] += w[i]*T + 0.5f*dd[i]*T*T; w[i] += dd[i]*T; }
            solve3(th, w, dd);
            // reaction: R = mu1 g_eff - sum_i mu_i L_i (tip accel of link i)
            float R_lat = mu[0] * gx, R_up = mu[0] * gy;
            for (int i = 0; i < 3; i++) {
                const float ci = cosf(th[i]), si = sinf(th[i]);
                R_lat -= mu[i] * Ln[i] * (dd[i]*ci - w[i]*w[i]*si);
                R_up  -= mu[i] * Ln[i] * (dd[i]*si + w[i]*w[i]*ci);
            }
            a_pend_lat = -R_lat;
            a_pend_up  = -R_up;
        }
        a_lat_out += a_pend_lat;
        a_up_out  += a_pend_up;
    }

    // ---- spool ----
    const bool want_lift = (a_up_cmd > 0.0f) || park;
    const bool airborne = want_lift || !on_ground;

    const float cp = cosf(constrain_float(ahrs.pitch, -1.04f, 1.04f));
    const float vert_slope = (a_up_out >= 0.0f) ? ACCEL_VERT_THR_PER_MSS : ACCEL_VERT_THR_PER_MSS_DOWN;
    float throttle_out = (ACCEL_HOVER_THR + vert_slope * a_up_out) / cp;

    if (!airborne) {
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

    // hold_level: the aggressive fore/aft POSITION hold pitches hard to chase position, which tilts a
    // swinging arm's reaction into the fore/aft axis -> a large swing modulates that coupling into a
    // parametric pump (divergence). Replace it with gentle fore/aft VELOCITY damping only: enough to
    // arrest the drift a purely-level drone would coast into, but with pitch small enough (prop. to the
    // small residual velocity) to stay well below the arm-coupling pump threshold. No position term ->
    // no long recentering, but the drone stays put instead of drifting. Balance keeps hold_level off.
    if (in.hold_level) {
        a_fwd = constrain_float(-ACCEL_FWD_HOLD_KP * x_fwd - ACCEL_FWD_HOLD_KV * v_fwd,
                                -ACCEL_FWD_HOLD_MAX, ACCEL_FWD_HOLD_MAX);
        _fwd_I = 0.0f;
    }

    // ---- attitude: roll=0, pitch from forward-hold, yaw = pilot rate ----
    const Vec3f thrust_vec(a_fwd * cy, a_fwd * sy, -GRAVITY_MSS);
    att.input_thrust_vector_rate_heading(thrust_vec, target_yaw_rate_rads,
                                         ahrs.roll, ahrs.pitch, ahrs.yaw, ahrs.gyro, dt, motors);

    motors.set_throttle(constrain_float(throttle_out, 0.0f, 1.0f));

    float lat_rail_out = ACCEL_LAT_MAX_MSS;
    if (a_lat_out < 0.0f) {
        lat_rail_out = constrain_float(ACCEL_LAT_MAX_MSS_NEG_A - ACCEL_LAT_MAX_MSS_NEG_B * fabsf(a_lat_out),
                                       36.5f, 40.5f);
    }
    motors.set_lateral(constrain_float(a_lat_out / lat_rail_out, -1.0f, 1.0f));

    motors.output(true, pwm_out);
}

}  // namespace apstack
