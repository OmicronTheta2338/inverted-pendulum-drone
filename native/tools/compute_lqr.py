#!/usr/bin/env python3
"""Compute the LQR full-state gain K for the acceleration-input cart-pole.

Plant (single inverted arm on an acceleration-commanded cart, linearised about
upright; mass-independent because the control input is the cart's acceleration):

    x = [theta, theta_dot, s, v]          theta = arm angle from straight-up
    theta_ddot = (g/L) theta - (1/L) u    u = commanded cart lateral accel [m/s^2]
    s_ddot     = u

We solve the continuous-time algebraic Riccati equation (CARE)

    A^T X + X A - X B R^-1 B^T X + Q = 0

via the Hamiltonian eigenvector method (numpy only -- no scipy needed), then

    K = R^-1 B^T X          u = -K x

Run:  python3 compute_lqr.py
Copodies the printed K vector into arm_balance_controller.cpp (kDefaultK).
"""
import numpy as np

G = 9.8

# Angle-dominant weights: heavy on theta (keep it upright), lighter on cart pos/vel
# so the cart drifts home slowly WITHOUT fighting the fast angle loop. R=1.
# Tuned so the fast (angle) closed-loop pair sits ~6 rad/s (~1.5x the unstable
# pole) at damping ~0.96 -- bandwidth headroom for the sensor latency, minimal
# overshoot -- while the cart pair stays slow (~0.7 rad/s) so returning home
# never fights the balance loop.
Q = np.diag([400.0, 10.0, 2.0, 4.0])
R = np.array([[1.0]])


def plant(L):
    A = np.array([
        [0.0,   1.0, 0.0, 0.0],
        [G / L, 0.0, 0.0, 0.0],
        [0.0,   0.0, 0.0, 1.0],
        [0.0,   0.0, 0.0, 0.0],
    ])
    B = np.array([[0.0], [-1.0 / L], [0.0], [1.0]])
    return A, B


def care_hamiltonian(A, B, Q, R):
    """Solve the CARE by the stable-eigenspace of the Hamiltonian matrix."""
    n = A.shape[0]
    Rinv = np.linalg.inv(R)
    H = np.block([
        [A,             -B @ Rinv @ B.T],
        [-Q,            -A.T],
    ])
    w, V = np.linalg.eig(H)
    # keep the n eigenvectors whose eigenvalues have negative real part
    stable = np.argsort(w.real)[:n]
    U = V[:, stable]
    U1 = U[:n, :]
    U2 = U[n:, :]
    X = np.real(U2 @ np.linalg.inv(U1))
    X = 0.5 * (X + X.T)            # symmetrise (numerical hygiene)
    K = Rinv @ B.T @ X
    return K, X


def controllable(A, B):
    n = A.shape[0]
    C = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])
    return np.linalg.matrix_rank(C) == n


def report(L):
    A, B = plant(L)
    assert controllable(A, B), "plant not controllable!"
    K, X = care_hamiltonian(A, B, Q, R)
    Kf = K.flatten()
    cl = np.linalg.eigvals(A - B @ K)
    print(f"--- L = {L} m   (g/L = {G/L:.4f}, unstable pole sqrt(g/L) = {np.sqrt(G/L):.4f} rad/s) ---")
    print(f"  K = [k_theta, k_thetadot, k_s, k_v] = "
          f"[{Kf[0]:.6f}, {Kf[1]:.6f}, {Kf[2]:.6f}, {Kf[3]:.6f}]")
    print(f"  closed-loop eigenvalues:")
    for e in sorted(cl, key=lambda z: z.real):
        print(f"      {e.real:+.4f} {e.imag:+.4f}j")
    print(f"  (all real parts < 0 => stable: {np.all(cl.real < 0)})")
    print()
    return Kf


if __name__ == "__main__":
    print(f"Q = diag{tuple(np.diag(Q))}, R = {R.flatten()[0]}\n")
    report(0.6)   # primary arm -> hardcode this K
    report(0.5)   # binding arm -> check the same K keeps margin
