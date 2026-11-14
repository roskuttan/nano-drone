"""2-D finite-difference electrostatic solver for multi-conductor PCB cross-sections.

Graded rectilinear mesh (fine at every material/conductor edge), node potentials, permittivity per cell, the outer box
held at 0 V (it acts as a distant ground). Returns the Maxwell capacitance matrix per unit length. The inductance
matrix comes from the air-filled solution, L = mu0*eps0*inv(C0), which is the usual quasi-TEM result.
"""
import math

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

EPS0 = 8.8541878128e-12
MU0 = 4e-7 * math.pi
C_LIGHT = 299792458.0


def graded(a, b, dmin, dmax, r=1.3):
    L = b - a
    if L <= 0:
        return np.array([a])
    steps, s, tot = [], dmin, 0.0
    while tot + s < L / 2:
        steps.append(s)
        tot += s
        s = min(s * r, dmax)
    half = np.array(steps) if steps else np.array([L / 2])
    rem = L - 2 * half.sum()
    seq = np.concatenate([half, [rem] if rem > 1e-15 else [], half[::-1]])
    seq = seq * (L / seq.sum())
    return a + np.concatenate([[0.0], np.cumsum(seq)])


def axis(keys, dmin, dmax):
    keys = sorted(set(round(k, 12) for k in keys))
    pts = [np.array([keys[0]])]
    for a, b in zip(keys[:-1], keys[1:]):
        pts.append(graded(a, b, dmin, dmax)[1:])
    return np.concatenate(pts)


class Section:
    """conductors: list of lists of rectangles (x0, x1, y0, y1) in metres; index = conductor number.
    grounds: rectangles held at 0 V (planes, pours). slabs: [(y0, y1, er)] dielectric layers, plus optional
    'extra' [(x0, x1, y0, y1, er)] rectangles painted last (solder mask pieces)."""

    def __init__(self, conductors, grounds, slabs, extra=(), box=(-2e-3, 2e-3, -1e-3, 2e-3), dmin=2e-6, dmax=40e-6):
        self.conductors, self.grounds, self.slabs, self.extra = conductors, grounds, slabs, list(extra)
        xs = [box[0], box[1]]
        ys = [box[2], box[3]]
        for rects in conductors + [grounds]:
            for (x0, x1, y0, y1) in rects:
                xs += [x0, x1]
                ys += [y0, y1]
        for (y0, y1, _er) in slabs:
            ys += [y0, y1]
        for (x0, x1, y0, y1, _er) in self.extra:
            xs += [x0, x1]
            ys += [y0, y1]
        xs = [min(max(v, box[0]), box[1]) for v in xs]
        ys = [min(max(v, box[2]), box[3]) for v in ys]
        self.x = axis(xs, dmin, dmax)
        self.y = axis(ys, dmin, dmax)
        self.box = box

    def _eps(self, air=False):
        x, y = self.x, self.y
        xc, yc = 0.5 * (x[:-1] + x[1:]), 0.5 * (y[:-1] + y[1:])
        XC, YC = np.meshgrid(xc, yc, indexing="ij")
        eps = np.ones(XC.shape)
        if air:
            return eps
        for (y0, y1, er) in self.slabs:
            eps[(YC > y0) & (YC < y1)] = er
        for (x0, x1, y0, y1, er) in self.extra:
            eps[(XC > x0) & (XC < x1) & (YC > y0) & (YC < y1)] = er
        return eps

    def _masks(self):
        X, Y = np.meshgrid(self.x, self.y, indexing="ij")
        tol = 1e-12
        cond = []
        for rects in self.conductors:
            m = np.zeros(X.shape, dtype=bool)
            for (x0, x1, y0, y1) in rects:
                m |= (X >= x0 - tol) & (X <= x1 + tol) & (Y >= y0 - tol) & (Y <= y1 + tol)
            cond.append(m)
        g = np.zeros(X.shape, dtype=bool)
        for (x0, x1, y0, y1) in self.grounds:
            g |= (X >= x0 - tol) & (X <= x1 + tol) & (Y >= y0 - tol) & (Y <= y1 + tol)
        g[0, :] = g[-1, :] = True
        g[:, 0] = g[:, -1] = True
        return cond, g

    def cmatrix(self, air=False):
        x, y = self.x, self.y
        nx, ny = x.size, y.size
        eps = self._eps(air)
        dx, dy = np.diff(x), np.diff(y)
        ah = np.zeros((nx - 1, ny))
        ah[:, 1:] += eps * dy[None, :] / 2
        ah[:, :-1] += eps * dy[None, :] / 2
        ah /= dx[:, None]
        av = np.zeros((nx, ny - 1))
        av[1:, :] += eps * dx[:, None] / 2
        av[:-1, :] += eps * dx[:, None] / 2
        av /= dy[None, :]
        cond, gnd = self._masks()
        fixed = gnd.copy()
        for m in cond:
            fixed |= m
        free = ~fixed
        nid = -np.ones((nx, ny), dtype=np.int64)
        nf = int(free.sum())
        nid[free] = np.arange(nf)
        # matrix (free-free) and coupling to fixed nodes
        rows, cols, vals = [], [], []
        frows, fnodes, fvals = [], [], []
        flat = np.arange(nx * ny).reshape(nx, ny)

        def edges(i1, j1, i2, j2, a):
            for (ia, ja, ib, jb) in ((i1, j1, i2, j2), (i2, j2, i1, j1)):
                fa = free[ia, ja]
                rows.append(nid[ia, ja][fa])
                cols.append(nid[ia, ja][fa])
                vals.append(a[fa])
                ff = fa & free[ib, jb]
                rows.append(nid[ia, ja][ff])
                cols.append(nid[ib, jb][ff])
                vals.append(-a[ff])
                fx = fa & ~free[ib, jb]
                frows.append(nid[ia, ja][fx])
                fnodes.append(flat[ib, jb][fx])
                fvals.append(a[fx])
        I, J = np.meshgrid(np.arange(nx - 1), np.arange(ny), indexing="ij")
        edges(I, J, I + 1, J, ah)
        I, J = np.meshgrid(np.arange(nx), np.arange(ny - 1), indexing="ij")
        edges(I, J, I, J + 1, av)
        A = sp.csc_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(nf, nf))
        frows, fnodes, fvals = np.concatenate(frows), np.concatenate(fnodes), np.concatenate(fvals)
        lu = spla.splu(A)
        nc = len(cond)
        Cm = np.zeros((nc, nc))
        for k in range(nc):
            phi = np.zeros(nx * ny)
            phi[cond[k].ravel()] = 1.0
            b = np.zeros(nf)
            np.add.at(b, frows, fvals * phi[fnodes])
            phi_free = lu.solve(b)
            full = phi.copy()
            full[flat[free]] = phi_free
            P = full.reshape(nx, ny)
            # charge on conductor i: sum of flux leaving through edges with one end inside it
            for i in range(nc):
                m = cond[i]
                q = 0.0
                q += (ah * (P[:-1, :] - P[1:, :]))[m[:-1, :] & ~m[1:, :]].sum()
                q += (ah * (P[1:, :] - P[:-1, :]))[m[1:, :] & ~m[:-1, :]].sum()
                q += (av * (P[:, :-1] - P[:, 1:]))[m[:, :-1] & ~m[:, 1:]].sum()
                q += (av * (P[:, 1:] - P[:, :-1]))[m[:, 1:] & ~m[:, :-1]].sum()
                Cm[i, k] = EPS0 * q
        return Cm

    def lc(self):
        C = self.cmatrix()
        C0 = self.cmatrix(air=True)
        L = MU0 * EPS0 * np.linalg.inv(C0)
        return L, C, C0


def z0_single(sec):
    L, C, C0 = sec.lc()
    z = math.sqrt(L[0, 0] / C[0, 0])
    eeff = C[0, 0] / C0[0, 0]
    return z, eeff
