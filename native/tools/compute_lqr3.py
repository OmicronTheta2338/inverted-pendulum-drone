#!/usr/bin/env python3
"""Compute the MIMO-LQR gain K for the TRIPLE INVERTED PENDULUM on an accel cart.

link1 (L1) hinged to the cart; link2 (L2) to link1's tip; link3 (L3) to link2's tip.
Point masses m1,m2,m3 at the link ends. Absolute angles th1,th2,th3 from the upward
vertical; single input u = cart lateral acceleration. Balance all three links upright.

This is the direct generalisation of compute_lqr2.py to n=3. The linearised chain-on-cart
plant is derived from the Lagrangian (verified to reduce to the single link th'' = (g/L)th
- (1/L)u and, via the n=2 self-check below, to the validated double's 7.2 rad/s pole):

    let mu_j = sum_{k>=j} m_k             (total mass at/below joint j)
    symmetric mass matrix  Mt_jk = L_j L_k mu_max(j,k)
    gravity (destabilising) Gt = diag( g L_j mu_j )
    input coupling          Ht_j = -L_j mu_j
    =>  Mt th'' = Gt th + Ht u   =>   th'' = Mt^-1 Gt th + Mt^-1 Ht u

State x = [th1,w1, th2,w2, th3,w3, s, v]. Solve the CARE (Hamiltonian method, numpy only,
same as compute_lqr2) and print K for hardcoding in three_arm_balance_controller.h.

Run:  python3 compute_lqr3.py
"""
import numpy as np

G = 9.8


def plant(L, m):
    """Linearised n-link chain on an acceleration cart. Returns (A, B).
    State = [th1,w1, ..., thn,wn, s, v]; input u = cart accel (v' = u)."""
    n = len(L)
    mu = np.array([m[i:].sum() for i in range(n)])          # mu_j = sum_{k>=j} m_k
    Mt = np.array([[L[j] * L[k] * mu[max(j, k)] for k in range(n)] for j in range(n)])
    Gt = np.diag([G * L[j] * mu[j] for j in range(n)])
    Ht = np.array([-L[j] * mu[j] for j in range(n)])
    Mi = np.linalg.inv(Mt)
    MiG, MiH = Mi @ Gt, Mi @ Ht
    ns = 2 * n + 2
    A = np.zeros((ns, ns))
    B = np.zeros((ns, 1))
    for j in range(n):
        A[2 * j, 2 * j + 1] = 1.0                           # th_j' = w_j
        for k in range(n):
            A[2 * j + 1, 2 * k] = MiG[j, k]                 # w_j' = sum_k MiG_jk th_k
        B[2 * j + 1, 0] = MiH[j]
    A[2 * n, 2 * n + 1] = 1.0                               # s' = v
    B[2 * n + 1, 0] = 1.0                                   # v' = u
    return A, B


def care_hamiltonian(A, B, Q, R):
    n = A.shape[0]
    Rinv = np.linalg.inv(R)
    H = np.block([[A, -B @ Rinv @ B.T], [-Q, -A.T]])
    w, V = np.linalg.eig(H)
    U = V[:, np.argsort(w.real)[:n]]                        # stable-subspace eigenvectors
    X = np.real(U[n:, :] @ np.linalg.inv(U[:n, :]))
    return (Rinv @ B.T @ (0.5 * (X + X.T))).flatten()


def ctrl_rank(A, B):
    C = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(A.shape[0])])
    return np.linalg.matrix_rank(C)


def fastest_unstable(A):
    ev = np.linalg.eigvals(A)
    unst = ev[ev.real > 1e-6]
    return float(np.max(unst.real)) if unst.size else float("nan")


if __name__ == "__main__":
    # ---- self-check: n=2 reduction must reproduce the validated double (compute_lqr2) ----
    A2, B2 = plant(np.array([0.7, 0.6]), np.array([0.1, 0.1]))
    p2 = fastest_unstable(A2)
    print("[self-check n=2] fastest unstable pole = %.4f rad/s  (compute_lqr2 = 7.2)  1/p = %.1f ms"
          % (p2, 1000.0 / p2))
    K2 = care_hamiltonian(A2, B2, np.diag([400., 10., 400., 10., 2., 4.]), np.array([[1.0]]))
    print("[self-check n=2] K = " + np.array2string(K2, precision=3)
          + "   (compute_lqr2: -213.5 -14.5 260.8 45.1 1.41 4.53)")
    print()

    # ---- cross-check the USER's fixed-base 3x3 A matrix (theta'' = A theta, no cart) ----
    # Different derivation (base fixed, eigenvalues of A = omega^2); must give the same
    # unstable poles as the cart plant above, since open-loop the cart does not feed back.
    def user_A(L, m):
        m1, m2, m3 = m
        l1, l2, l3 = L
        Mtot, M23 = m1 + m2 + m3, m2 + m3
        return np.array([
            [Mtot * G / (m1 * l1),       -M23 * G / (m1 * l1),                 0.0],
            [-Mtot * G / (m1 * l2), (m1 + m2) * M23 * G / (m1 * m2 * l2), -m3 * G / (m2 * l2)],
            [0.0,                        -M23 * G / (m2 * l3),        M23 * G / (m2 * l3)]])
    Au = user_A(np.array([0.7, 0.6, 0.6]), np.array([0.1, 0.1, 0.1]))
    w2 = np.sort(np.linalg.eigvals(Au).real)[::-1]           # = omega^2 (descending)
    lam = np.sqrt(w2[0])
    print("[user-A check] trace=%.1f det=%.0f  omega^2=%s" % (np.trace(Au), np.linalg.det(Au),
          np.array2string(w2, precision=2)))
    print("[user-A check] lambda_max=%.4f rad/s  tau=1/lambda=%.1f ms  tau/3=%.1f ms\n"
          % (lam, 1000.0 / lam, 1000.0 / (3.0 * lam)))

    # ---- the triple: L1=0.7, L2=L3=0.6, all bobs 0.1 kg ----
    L = np.array([0.7, 0.6, 0.6])
    m = np.array([0.1, 0.1, 0.1])
    Q = np.diag([400., 10., 400., 10., 400., 10., 2., 4.])
    R = np.array([[1.0]])
    A, B = plant(L, m)

    print("L=%s  m=%s  Q=diag%s  R=%g" % (L, m, tuple(np.diag(Q)), R.flatten()[0]))
    print("controllability rank = %d of 8" % ctrl_rank(A, B))
    poles = np.sort_complex(np.linalg.eigvals(A))
    print("open-loop poles = " + np.array2string(poles, precision=3))
    p = fastest_unstable(A)
    print(">>> fastest unstable pole p = %.4f rad/s   1/p = %.1f ms   (double: 7.2 / 139 ms)" % (p, 1000.0 / p))
    print(">>> p*tau break points:  hard wall (p*tau=1) at tau = %.1f ms;  practical (p*tau=0.5) at tau = %.1f ms"
          % (1000.0 / p, 500.0 / p))

    K = care_hamiltonian(A, B, Q, R)
    print("\nLQR gain K:")
    for nm, k in zip(["k1", "k1d", "k2", "k2d", "k3", "k3d", "ks", "kv"], K):
        print("    %-4s= %13.6f;" % (nm, k))
    cl = np.linalg.eigvals(A - B @ K.reshape(1, -1))
    print("closed-loop poles:")
    for e in sorted(cl, key=lambda z: z.real):
        print("    %+.4f %+.4fj" % (e.real, e.imag))
    print("  (stable: %s)" % np.all(cl.real < 0))
