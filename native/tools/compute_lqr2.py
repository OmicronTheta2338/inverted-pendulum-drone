#!/usr/bin/env python3
"""Compute the MIMO-LQR gain K for the DOUBLE INVERTED PENDULUM on an accel cart.

link1 (L1) hinged to the cart; link2 (L2) hinged to link1's TIP; point masses m1, m2
at the link ends. Absolute angles th1, th2 from the upward vertical; single input
u = cart lateral acceleration. Balance both links upright.

Linearised about upright (M th'' = G th + H u):
    M = [[(m1+m2)L1, m2 L2], [L1, L2]]
    G = diag((m1+m2) g, g)
    H = [-(m1+m2), -1]
=>  [th1'',th2''] = M^-1 G [th1,th2] + M^-1 H u
State x = [th1, w1, th2, w2, s, v]. We solve the CARE (Hamiltonian method, numpy only)
and print K for hardcoding in two_arm_balance_controller.h.

Run:  python3 compute_lqr2.py
"""
import numpy as np

G = 9.8
L1, L2 = 0.7, 0.6
m1, m2 = 0.1, 0.1
Q = np.diag([400.0, 10.0, 400.0, 10.0, 2.0, 4.0])
R = np.array([[1.0]])


def plant():
    M = np.array([[(m1 + m2) * L1, m2 * L2], [L1, L2]])
    Gm = np.array([[(m1 + m2) * G, 0.0], [0.0, G]])
    H = np.array([-(m1 + m2), -1.0])
    Mi = np.linalg.inv(M)
    MG, MH = Mi @ Gm, Mi @ H
    A = np.zeros((6, 6))
    A[0, 1] = 1.0
    A[1, 0], A[1, 2] = MG[0, 0], MG[0, 1]
    A[2, 3] = 1.0
    A[3, 0], A[3, 2] = MG[1, 0], MG[1, 1]
    A[4, 5] = 1.0
    B = np.array([[0.0], [MH[0]], [0.0], [MH[1]], [0.0], [1.0]])
    return A, B


def care_hamiltonian(A, B, Q, R):
    n = A.shape[0]
    Rinv = np.linalg.inv(R)
    H = np.block([[A, -B @ Rinv @ B.T], [-Q, -A.T]])
    w, V = np.linalg.eig(H)
    U = V[:, np.argsort(w.real)[:n]]
    X = np.real(U[n:, :] @ np.linalg.inv(U[:n, :]))
    return (Rinv @ B.T @ (0.5 * (X + X.T))).flatten()


def ctrl_rank(A, B):
    C = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(A.shape[0])])
    return np.linalg.matrix_rank(C)


if __name__ == "__main__":
    A, B = plant()
    print(f"L1={L1} L2={L2} m1={m1} m2={m2}  Q=diag{tuple(np.diag(Q))} R={R.flatten()[0]}")
    print(f"controllability rank = {ctrl_rank(A, B)} of 6")
    print(f"open-loop poles = {np.array2string(np.sort_complex(np.linalg.eigvals(A)), precision=3)}")
    K = care_hamiltonian(A, B, Q, R)
    for n, k in zip(["k1", "k1d", "k2", "k2d", "ks", "kv"], K):
        print(f"    {n:4s}= {k:12.6f};")
    cl = np.linalg.eigvals(A - B @ K.reshape(1, 6))
    print("closed-loop poles:")
    for e in sorted(cl, key=lambda z: z.real):
        print(f"    {e.real:+.4f} {e.imag:+.4f}j")
    print(f"  (stable: {np.all(cl.real < 0)})")
