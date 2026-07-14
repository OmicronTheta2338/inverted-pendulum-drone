#!/usr/bin/env python3
"""Double-pendulum SWING-UP via trajectory optimization (iLQR) -- numpy only.

SERIES double pendulum on a pivot that accelerates with (ax, ay). The KEY SIMPLIFICATION
(cf. Swingup-Double-TrajOpt.md "The model") is that such a pivot is EXACTLY a fixed-pivot
double pendulum in a uniform effective-gravity field

    g_eff = ( -ax , -(g + ay) )

because every bob feels a uniform pseudo-acceleration -(ax, ay) on top of real (0, -g).
So the plant is "textbook double pendulum + swap the gravity vector for g_eff(u)".

Angle convention (matches the sim `_update_pendulum` and `_react_validate` in main.gd):
    theta from straight DOWN, ABSOLUTE for both links. 0 = hanging, pi = upright.
    bob1 = L1(sin th1, -cos th1);  bob2 = bob1 + L2(sin th2, -cos th2).
State x = [th1, w1, th2, w2].  Input u = [ax, ay].

The closed-form angular accelerations below are byte-for-byte the same expressions already
in main.gd `_react_validate`, which the project validated against Jolt truth at 3-5%
(memory: packetC-reaction-ff-validated). So the instantaneous model is pre-trusted; the
self-test here (energy conservation + linearization match) catches any transcription error.

Usage:
    python3 swingup2_traj.py selftest      # model correctness checks (no Godot needed)
"""
import sys
import numpy as np

# ---- plant parameters (match main.tscn series double pendulum) ----
G = 9.8
L1, L2 = 0.7, 0.6
M1, M2 = 0.1, 0.1
A_LAT_MAX = 12.0        # |ax| bound  [m/s^2]  (full lat_stick)
A_UP_MAX = G            # |ay| bound  [m/s^2]  (full up_stick -> a_y in [-g, +g])

NX = 4                  # [th1, w1, th2, w2]
NU = 2                  # [ax, ay]


def f_cont(x, u):
    """Continuous dynamics xdot = f(x,u). Effective-gravity double pendulum.

    Mirrors main.gd `_react_validate` exactly:
        gx=-ax, gy=-(g+ay)
        b1 = M*(gx c1 + gy s1) - m2 L2 sinD w2^2
        b2 = (gx c2 + gy s2)   + L1 sinD w1^2
        det= L1 L2 (M - m2 cosD^2)
        dd1= (L2 b1 - m2 L2 cosD b2)/det
        dd2= (-L1 cosD b1 + M L1 b2)/det
    """
    th1, w1, th2, w2 = x
    ax, ay = u
    gx = -ax
    gy = -(G + ay)
    c1, s1 = np.cos(th1), np.sin(th1)
    c2, s2 = np.cos(th2), np.sin(th2)
    dD = th1 - th2
    cD, sD = np.cos(dD), np.sin(dD)
    M = M1 + M2
    b1 = M * (gx * c1 + gy * s1) - M2 * L2 * sD * w2 * w2
    b2 = (gx * c2 + gy * s2) + L1 * sD * w1 * w1
    det = L1 * L2 * (M - M2 * cD * cD)
    dd1 = (L2 * b1 - M2 * L2 * cD * b2) / det
    dd2 = (-L1 * cD * b1 + M * L1 * b2) / det
    return np.array([w1, dd1, w2, dd2])


def f_aug(x, u):
    """8-state augmented dynamics: pendulum + cart double-integrator.
    x = [th1, w1, th2, w2, xc, vx, yc, vy];  cart accel IS the input u=(ax,ay).
    The pendulum depends on the cart only through its ACCELERATION (=u), never its
    position/velocity, so the cart rows are a trivial double-integrator -- but costing
    xc/yc/vx/vy keeps the swing-up from flying the drone away or arriving in a dive."""
    pend = f_cont(x[:4], u)
    ax, ay = u
    return np.array([pend[0], pend[1], pend[2], pend[3], x[5], ax, x[7], ay])


def rk4_step(x, u, dt, f=f_cont):
    """One classical RK4 step (u held constant across the step -- zero-order hold)."""
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def energy(x):
    """Total mechanical energy in the (uncontrolled) pivot frame, bob velocities relative to pivot.
    Conserved when u=0 (pivot inertial). Used only as a self-test."""
    th1, w1, th2, w2 = x
    c1, s1 = np.cos(th1), np.sin(th1)
    c2, s2 = np.cos(th2), np.sin(th2)
    v1x, v1y = L1 * c1 * w1, L1 * s1 * w1
    v2x, v2y = v1x + L2 * c2 * w2, v1y + L2 * s2 * w2
    ke = 0.5 * M1 * (v1x * v1x + v1y * v1y) + 0.5 * M2 * (v2x * v2x + v2y * v2y)
    y1 = -L1 * c1
    y2 = -L1 * c1 - L2 * c2
    pe = M1 * G * y1 + M2 * G * y2
    return ke + pe


def fd_jac_cont(x, u, eps=1e-6):
    """Finite-difference continuous Jacobians A=df/dx (4x4), B=df/du (4x2)."""
    A = np.zeros((NX, NX))
    B = np.zeros((NX, NU))
    for i in range(NX):
        dx = np.zeros(NX); dx[i] = eps
        A[:, i] = (f_cont(x + dx, u) - f_cont(x - dx, u)) / (2 * eps)
    for j in range(NU):
        du = np.zeros(NU); du[j] = eps
        B[:, j] = (f_cont(x, u + du) - f_cont(x, u - du)) / (2 * eps)
    return A, B


def analytic_A_up():
    """Analytic continuous A about UPRIGHT (u=0), 4-state pendulum block, from compute_lqr2.py's
    linearization. Angle-from-up there vs angle-from-down here differ by a sign that cancels in the
    2nd-order homogeneous block, so EIGENVALUES match either convention -- that's what we compare."""
    Mm = np.array([[(M1 + M2) * L1, M2 * L2], [L1, L2]])
    Gm = np.array([[(M1 + M2) * G, 0.0], [0.0, G]])
    MG = np.linalg.inv(Mm) @ Gm
    A = np.zeros((4, 4))
    A[0, 1] = 1.0
    A[1, 0], A[1, 2] = MG[0, 0], MG[0, 1]
    A[2, 3] = 1.0
    A[3, 0], A[3, 2] = MG[1, 0], MG[1, 1]
    return A


def selftest():
    print(f"params: L1={L1} L2={L2} m1={M1} m2={M2} g={G}")
    ok = True

    # --- Check 1: energy conservation with u=0 (catches any EOM coefficient/sign typo) ---
    dt = 1e-3
    x = np.array([2.0, 0.5, 1.0, -0.7])   # arbitrary chaotic-ish IC
    e0 = energy(x)
    emax = 0.0
    for _ in range(20000):                # 20 s
        x = rk4_step(x, np.zeros(2), dt)
        emax = max(emax, abs(energy(x) - e0))
    rel = emax / max(abs(e0), 1e-9)
    print(f"[1] energy drift over 20s, u=0:  abs={emax:.3e}  rel={rel:.3e}  "
          f"({'PASS' if rel < 1e-4 else 'FAIL'})")
    ok = ok and rel < 1e-4

    # --- Check 2: finite-diff linearization about upright matches analytic (eigenvalues) ---
    x_up = np.array([np.pi, 0.0, np.pi, 0.0])
    A_fd, _ = fd_jac_cont(x_up, np.zeros(2))
    ev_fd = np.sort(np.linalg.eigvals(A_fd).real)
    ev_an = np.sort(np.linalg.eigvals(analytic_A_up()).real)
    err = np.max(np.abs(ev_fd - ev_an))
    print(f"[2] upright eigenvalues  fd ={np.array2string(ev_fd, precision=4)}")
    print(f"                         anly={np.array2string(ev_an, precision=4)}  "
          f"max|d|={err:.2e} ({'PASS' if err < 1e-4 else 'FAIL'})")
    ok = ok and err < 1e-4

    # --- Check 3: hanging is a stable oscillator (all eigenvalues imaginary) ---
    A_dn, _ = fd_jac_cont(np.array([0.0, 0.0, 0.0, 0.0]), np.zeros(2))
    ev = np.linalg.eigvals(A_dn)
    max_re = np.max(np.abs(ev.real))
    freqs = np.sort(np.abs(ev.imag))
    print(f"[3] hanging eigen-freqs (rad/s) = {np.array2string(freqs, precision=4)}  "
          f"max|Re|={max_re:.2e} ({'PASS' if max_re < 1e-6 else 'FAIL'})")
    ok = ok and max_re < 1e-6

    print("SELFTEST:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# =====================================================================================
# iLQR  (step B): find a swing-up trajectory.  numpy-only.
# =====================================================================================
# iLQR = iterative LQR. Repeat: (1) roll the current controls forward to get a nominal
# trajectory; (2) LINEARIZE the dynamics and QUADRATIZE the cost about that nominal;
# (3) solve the resulting time-varying LQR by a backward Riccati pass -> a local
# feedforward correction k(t) and feedback gain K(t); (4) line-search forward applying
# u += alpha*k + K*dx. The backward pass is the SAME Riccati machinery we'll reuse for
# TVLQR tracking (step D) -- that reuse is why iLQR beats direct collocation here.

PI = np.pi
XF = np.array([PI, 0.0, PI, 0.0])   # goal: both links UP, at rest


def wrap(a):
    return (a + PI) % (2 * PI) - PI


def state_err(x, xf=XF):
    """x - xf with the two ANGLE rows wrapped to (-pi, pi]. d(err)/dx = I a.e., so the
    cost gradient/Hessian below use Q directly (the wrap only affects the residual value)."""
    e = x - xf
    e[0] = wrap(e[0])
    e[2] = wrap(e[2])
    return e


def discrete_jac(x, u, dt, f, nx, nu, eps=1e-6):
    """Finite-difference Jacobians of the RK4 step:  A=d x_{k+1}/d x_k, B=.../d u."""
    A = np.zeros((nx, nx))
    B = np.zeros((nx, nu))
    for i in range(nx):
        dx = np.zeros(nx); dx[i] = eps
        A[:, i] = (rk4_step(x + dx, u, dt, f) - rk4_step(x - dx, u, dt, f)) / (2 * eps)
    for j in range(nu):
        du = np.zeros(nu); du[j] = eps
        B[:, j] = (rk4_step(x, u + du, dt, f) - rk4_step(x, u - du, dt, f)) / (2 * eps)
    return A, B


def rollout(x0, U, dt, f):
    N = U.shape[0]
    X = np.zeros((N + 1, x0.shape[0]))
    X[0] = x0
    for k in range(N):
        X[k + 1] = rk4_step(X[k], U[k], dt, f)
    return X


def traj_cost(X, U, Q, R, Qf, xf):
    N = U.shape[0]
    J = 0.0
    for k in range(N):
        e = state_err(X[k], xf)
        J += 0.5 * e @ Q @ e + 0.5 * U[k] @ R @ U[k]
    eN = state_err(X[N], xf)
    J += 0.5 * eN @ Qf @ eN
    return J


def ilqr(x0, U0, dt, Q, R, Qf, xf, f, iters=300, tol=1e-5, verbose=True):
    """Returns X*, U*, K*(feedback gains, N x nu x nx), final cost.  f = continuous dynamics."""
    N = U0.shape[0]
    nx, nu = x0.shape[0], U0.shape[1]
    U = U0.copy()
    X = rollout(x0, U, dt, f)
    J = traj_cost(X, U, Q, R, Qf, xf)

    mu, mu_min, dmu, dmu0 = 1.0, 1e-6, 2.0, 2.0   # LM-style regularization on Quu
    alphas = 1.1 ** (-np.arange(10) ** 2)          # 1, .9, .66, .41, ... line-search steps
    Kfb = np.zeros((N, nu, nx))

    for it in range(iters):
        # ---- backward pass ----
        bp_ok = False
        while not bp_ok:
            Vx = Qf @ state_err(X[N], xf)
            Vxx = Qf.copy()
            kff = np.zeros((N, nu))
            Kfb = np.zeros((N, nu, nx))
            dV1 = dV2 = 0.0
            bp_ok = True
            for k in range(N - 1, -1, -1):
                A, B = discrete_jac(X[k], U[k], dt, f, nx, nu)
                lx = Q @ state_err(X[k], xf)
                lu = R @ U[k]
                Qx = lx + A.T @ Vx
                Qu = lu + B.T @ Vx
                Qxx = Q + A.T @ Vxx @ A
                Quu = R + B.T @ Vxx @ B
                Qux = B.T @ Vxx @ A
                Quu_reg = Quu + mu * np.eye(nu)
                # require Quu_reg PD (else bump regularization and restart backward pass)
                if np.any(np.linalg.eigvalsh(Quu_reg) <= 0):
                    dmu = max(dmu0, dmu * dmu0); mu = max(mu_min, mu * dmu)
                    bp_ok = False
                    break
                Quu_inv = np.linalg.inv(Quu_reg)
                k_k = -Quu_inv @ Qu
                K_k = -Quu_inv @ Qux
                kff[k] = k_k
                Kfb[k] = K_k
                dV1 += k_k @ Qu
                dV2 += 0.5 * k_k @ Quu @ k_k
                Vx = Qx + K_k.T @ Quu @ k_k + K_k.T @ Qu + Qux.T @ k_k
                Vxx = Qxx + K_k.T @ Quu @ K_k + K_k.T @ Qux + Qux.T @ K_k
                Vxx = 0.5 * (Vxx + Vxx.T)

        # ---- forward line search ----
        accepted = False
        for alpha in alphas:
            Xn = np.zeros_like(X); Xn[0] = x0
            Un = np.zeros_like(U)
            for k in range(N):
                Un[k] = U[k] + alpha * kff[k] + Kfb[k] @ (Xn[k] - X[k])
                Xn[k + 1] = rk4_step(Xn[k], Un[k], dt, f)
            Jn = traj_cost(Xn, Un, Q, R, Qf, xf)
            exp_red = alpha * dV1 + alpha * alpha * dV2   # predicted (negative) cost change
            if exp_red < 0 and (Jn - J) / exp_red > 1e-4:
                accepted = True
                break
        if accepted:
            dJ = J - Jn
            X, U, J = Xn, Un, Jn
            dmu = min(1 / dmu0, dmu / dmu0); mu = mu * dmu if mu * dmu > mu_min else 0.0
            if verbose and (it % 20 == 0 or dJ < tol):
                a1 = abs(wrap(X[N][0] - PI)) * 180 / PI
                a2 = abs(wrap(X[N][2] - PI)) * 180 / PI
                dr = np.hypot(X[N][4], X[N][6]) if nx >= 8 else 0.0
                print(f"  it{it:3d}  J={J:10.3f}  dJ={dJ:.2e}  a=({a1:5.1f},{a2:5.1f})deg drift={dr:4.2f}m "
                      f"|u|max=({np.max(np.abs(U[:,0])):5.2f},{np.max(np.abs(U[:,1])):5.2f}) mu={mu:.1e}")
            if dJ < tol:
                print(f"  converged at it{it} (dJ<{tol})")
                break
        else:
            dmu = max(dmu0, dmu * dmu0); mu = max(mu_min, mu * dmu)
            if verbose:
                print(f"  it{it:3d}  no step accepted; mu -> {mu:.1e}")
            if mu > 1e10:
                print("  regularization blew up; stopping")
                break

    return X, U, Kfb, J


def export_csv(path, X, U, K, dt):
    """Write the trajectory for the Godot harness. One row per knot k=0..N.
    cols: t, th1,w1,th2,w2,xc,vx,yc,vy, ax,ay, K(16 = 2x8 row-major).
    Terminal row (k=N) carries u=0 and repeats the last K (never actually applied)."""
    N = U.shape[0]
    with open(path, "w") as fh:
        kcols = ",".join(f"K{r}{c}" for r in range(2) for c in range(8))
        fh.write("t,th1,w1,th2,w2,xc,vx,yc,vy,ax,ay," + kcols + "\n")
        for k in range(N + 1):
            u = U[k] if k < N else np.zeros(NU)
            Kk = K[k] if k < N else K[N - 1]
            row = [k * dt, *X[k], u[0], u[1], *Kk.flatten()]
            fh.write(",".join(f"{v:.6f}" for v in row) + "\n")
    print(f"exported CSV -> {path} ({N + 1} rows)")


def run_ilqr(dt=0.02, T=3.0, save=None, seed_lean_deg=5.0,
             wf_ang=200.0, wf_rate=20.0, wr=0.02, wq_ang=0.0, wq_rate=0.0,
             wf_pos=20.0, wf_vel=40.0, wq_pos=2.0, wq_vel=0.3):
    """8-state iLQR: [th1,w1,th2,w2, xc,vx,yc,vy].  Cart costs bound drift / arrive at rest."""
    N = int(round(T / dt))
    lean = np.deg2rad(seed_lean_deg)
    x0 = np.array([lean, 0.0, lean, 0.0, 0.0, 0.0, 0.0, 0.0])
    xf = np.array([PI, 0.0, PI, 0.0, 0.0, 0.0, 0.0, 0.0])   # both up, cart home & at rest
    U0 = np.zeros((N, NU))
    Q = np.diag([wq_ang, wq_rate, wq_ang, wq_rate, wq_pos, wq_vel, wq_pos, wq_vel])
    R = np.diag([wr, wr])
    Qf = np.diag([wf_ang, wf_rate, wf_ang, wf_rate, wf_pos, wf_vel, wf_pos, wf_vel])
    print(f"iLQR(8): T={T}s dt={dt} N={N}  Qf(ang,rate)=({wf_ang},{wf_rate}) "
          f"Qf(pos,vel)=({wf_pos},{wf_vel}) R={wr}  Qrun(pos,vel)=({wq_pos},{wq_vel})")
    X, U, K, J = ilqr(x0, U0, dt, Q, R, Qf, xf, f_aug)
    a1 = abs(wrap(X[-1][0] - PI)) * 180 / PI
    a2 = abs(wrap(X[-1][2] - PI)) * 180 / PI
    w1 = abs(X[-1][1]) * 180 / PI
    w2 = abs(X[-1][3]) * 180 / PI
    drift = np.hypot(X[-1][4], X[-1][6])
    cart_v = np.hypot(X[-1][5], X[-1][7])
    max_drift = np.max(np.hypot(X[:, 4], X[:, 6]))
    umax = (np.max(np.abs(U[:, 0])), np.max(np.abs(U[:, 1])))
    feasible = umax[0] <= A_LAT_MAX + 1e-6 and umax[1] <= A_UP_MAX + 1e-6
    up = a1 < 10 and a2 < 10 and w1 < 60 and w2 < 60
    print(f"\nFINAL: link angles from up=({a1:.1f},{a2:.1f})deg  rates=({w1:.1f},{w2:.1f})deg/s")
    print(f"       cart: final drift={drift:.2f}m  max drift={max_drift:.2f}m  final vel={cart_v:.2f}m/s")
    print(f"       |u|max=({umax[0]:.2f},{umax[1]:.2f})  bounds=({A_LAT_MAX},{A_UP_MAX:.1f})  "
          f"feasible={feasible}  BOTH-UP={up}")
    print(f"RESULT: {'PASS (swing-up found)' if (up and feasible) else 'FAIL'}")
    if save:
        np.savez(save, X=X, U=U, K=K, dt=dt, T=T, xf=xf)
        print(f"saved trajectory -> {save}")
        export_csv(save.replace(".npz", ".csv"), X, U, K, dt)
    return X, U, K


def export_header(npz_path, out_path):
    """Bake the nominal (u*, x*) + TVLQR gains K(t) into a C++ header for SwingUp2 (step E)."""
    d = np.load(npz_path)
    X, U, K, dt = d["X"], d["U"], d["K"], float(d["dt"])
    N = U.shape[0]
    def row(vals):
        return "{" + ",".join(f"{v:.9e}" for v in vals) + "}"
    L = ["// AUTO-GENERATED by native/tools/swingup2_traj.py `header` -- do not edit by hand.",
         "// iLQR double-pendulum swing-up: nominal control U*, state X* (=[th1,w1,th2,w2,xc,vx,yc,vy],",
         "// angles from DOWN), and time-varying feedback K (2x8 row-major) for u=U*+K*(x-X*).",
         "#ifndef SWINGUP2_TRAJ_DATA_H", "#define SWINGUP2_TRAJ_DATA_H",
         "namespace armbalance { namespace swingup2_traj {",
         f"constexpr int N = {N};", f"constexpr double DT = {dt:.9e};",
         f"static const double U[{N}][2] = {{"]
    L += ["  " + row(U[k]) + "," for k in range(N)]
    L += ["};", f"static const double X[{N + 1}][8] = {{"]
    L += ["  " + row(X[k]) + "," for k in range(N + 1)]
    L += ["};", f"static const double K[{N}][16] = {{"]
    L += ["  " + row(K[k].flatten()) + "," for k in range(N)]
    L += ["};", "}}  // namespace armbalance::swingup2_traj", "#endif"]
    with open(out_path, "w") as fh:
        fh.write("\n".join(L) + "\n")
    print(f"wrote header {out_path}  (N={N}, dt={dt})")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "selftest"
    if cmd == "selftest":
        sys.exit(selftest())
    elif cmd == "header":
        args = {a.split("=")[0]: a.split("=")[1] for a in sys.argv[2:] if "=" in a}
        export_header(args["npz"], args["out"])
        sys.exit(0)
    elif cmd == "ilqr":
        kw = {}
        for a in sys.argv[2:]:
            key, _, val = a.partition("=")
            kw[key] = val if key == "save" else float(val)
        run_ilqr(**kw)
        sys.exit(0)
    print(f"unknown command: {cmd}")
    sys.exit(2)
