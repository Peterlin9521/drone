"""Follow-me MPC (translation only), ENU world frame.
Drone: velocity command -> first-order lag -> velocity -> position
Target: constant velocity over the horizon
Goal: keep distance d along reference direction, never closer than d_min (soft).
"""
import numpy as np
import scipy.sparse as sp
import osqp


class FollowMPC:
    def __init__(self, N=20, dt=0.1, tau=0.35, vmax=2.0, q_pos=4.0, q_vel=0.5,
                 r_du=2.0, w_slack=500.0):
        self.N, self.dt, self.vmax = N, dt, vmax
        a = dt / tau
        self.A = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1 - a, 0], [0, 0, 0, 1 - a]])
        self.B = np.array([[0, 0], [0, 0], [a, 0], [0, a]])
        self.q_pos, self.q_vel, self.r_du, self.w_slack = q_pos, q_vel, r_du, w_slack
        self.nX, self.nU = (N + 1) * 4, N * 2
        self.nz = self.nX + self.nU + N

    def _ix(self, k): return 4 * k
    def _iu(self, k): return self.nX + 2 * k
    def _is(self, k): return self.nX + self.nU + k

    def solve(self, p0, v0, u_prev, t0, vt, d, d_min, n_ref=None):
        N, dt = self.N, self.dt
        p0, v0, u_prev, t0, vt = map(lambda a: np.asarray(a, float), (p0, v0, u_prev, t0, vt))
        rel = p0 - t0
        n0 = rel / max(np.linalg.norm(rel), 1e-6)
        T = np.array([t0 + vt * dt * k for k in range(N + 1)])
        nr = n0 if n_ref is None else np.asarray(n_ref, float) / max(np.linalg.norm(n_ref), 1e-6)
        R = T + d * nr

        Pd = np.zeros(self.nz); q = np.zeros(self.nz); Poff = {}
        for k in range(1, N + 1):
            i = self._ix(k)
            Pd[i:i + 2] += 2 * self.q_pos; q[i:i + 2] += -2 * self.q_pos * R[k]
            Pd[i + 2:i + 4] += 2 * self.q_vel; q[i + 2:i + 4] += -2 * self.q_vel * vt
        for k in range(N):
            i = self._iu(k)
            Pd[i:i + 2] += 2 * self.r_du
            if k == 0:
                q[i:i + 2] += -2 * self.r_du * u_prev
            else:
                j = self._iu(k - 1)
                Pd[j:j + 2] += 2 * self.r_du
                for c in range(2):
                    Poff[(j + c, i + c)] = Poff.get((j + c, i + c), 0) - 2 * self.r_du
        for k in range(N):
            q[self._is(k)] = self.w_slack
        rows = list(range(self.nz)) + [a for a, b in Poff] + [b for a, b in Poff]
        cols = list(range(self.nz)) + [b for a, b in Poff] + [a for a, b in Poff]
        vals = list(Pd) + list(Poff.values()) + list(Poff.values())
        P = sp.triu(sp.csc_matrix((vals, (rows, cols)), shape=(self.nz, self.nz)), format='csc')

        Ar, Ac, Av, lo, hi = [], [], [], [], []
        r = 0
        def add(entries, l, u):
            nonlocal r
            for c, v in entries:
                Ar.append(r); Ac.append(c); Av.append(v)
            lo.append(l); hi.append(u); r += 1
        x0 = np.r_[p0, v0]
        for c in range(4):
            add([(self._ix(0) + c, 1.0)], x0[c], x0[c])
        for k in range(N):
            for c in range(4):
                e = [(self._ix(k + 1) + c, -1.0)]
                e += [(self._ix(k) + j, self.A[c, j]) for j in range(4) if self.A[c, j] != 0]
                e += [(self._iu(k) + j, self.B[c, j]) for j in range(2) if self.B[c, j] != 0]
                add(e, 0.0, 0.0)
        for k in range(N):
            for c in range(2):
                add([(self._iu(k) + c, 1.0)], -self.vmax, self.vmax)
        for k in range(N):
            i = self._ix(k + 1)
            add([(i, n0[0]), (i + 1, n0[1]), (self._is(k), 1.0)], d_min + n0 @ T[k + 1], np.inf)
            add([(self._is(k), 1.0)], 0.0, np.inf)
        A = sp.csc_matrix((Av, (Ar, Ac)), shape=(r, self.nz))

        m = osqp.OSQP()
        try:
            m.setup(P, q, A, np.array(lo), np.array(hi), verbose=False, polishing=True,
                    eps_abs=1e-4, eps_rel=1e-4, max_iter=4000)
        except TypeError:
            m.setup(P, q, A, np.array(lo), np.array(hi), verbose=False, polish=True,
                    eps_abs=1e-4, eps_rel=1e-4, max_iter=4000)
        res = m.solve()
        z = res.x
        if z is None or res.info.status_val not in (1, 2):
            return np.clip(u_prev, -self.vmax, self.vmax), None, T
        u0 = z[self._iu(0):self._iu(0) + 2]
        Xp = np.array([z[self._ix(k):self._ix(k) + 2] for k in range(N + 1)])
        return u0, Xp, T
