"""
Study 7 - RF: 50-ohm grounded coplanar waveguide (CPWG) on L6 over L5 GND and the SX1280
reference matching / harmonic filter.

(a) 2-D quasi-static finite-volume Laplace solver (numpy/scipy sparse): half cross-section with
    symmetry wall, graded non-uniform grid refined at metal edges, finite metal thickness, optional
    solder mask.  C' from the discrete field energy; Z0 = 1/(c*sqrt(C'*C'_air)), eps_eff = C'/C'_air.
    Cross-checks: GCPW conformal-mapping closed form (t = 0) and Hammerstad-Jensen microstrip.
(b) Lumped matching network (Semtech SX1280 DS rev 3.2 Fig. 15-1 / Table 15-1) with 0402 parasitics:
    S21/S11 between 50-ohm ports 2.0-8 GHz, insertion loss in-band and harmonic attenuation.

Run:  python sim_07_rf.py
"""
import math
import simlib as S
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy.special import ellipk
from simlib import plt, PAL, CRIT, INK2

EPS0 = 8.8541878128e-12
C0 = 299792458.0
H = 0.0994e-3
ER = 4.45
T = 35e-6
S.asm("rf_stack", {"h_mm": 0.0994, "er": ER, "t_um": 35}, "L6-L5 prepreg per JLC stack (user data); er at 1 GHz")
SM = S.asm("solder_mask", {"t_um": 15, "er": 3.8}, "LPI solder mask over trace (sensitivity case)")


# ----------------------------------------------------------------------------------- grid
def graded(a, b, dmin, dmax, r=1.25):
    """points from a to b, fine (dmin) at both ends, growing by r up to dmax."""
    L = b - a
    if L <= 0:
        return np.array([a])
    steps = []
    s = dmin
    tot = 0.0
    while tot + s < L / 2:
        steps.append(s)
        tot += s
        s = min(s * r, dmax)
    half = np.array(steps)
    rem = L - 2 * half.sum()
    seq = np.concatenate([half, [rem] if rem > 1e-15 else [], half[::-1]])
    seq = seq * (L / seq.sum())
    return a + np.concatenate([[0.0], np.cumsum(seq)])


def build_axis(keys, dmin, dmax, r=1.25):
    keys = sorted(set(keys))
    pts = [np.array([keys[0]])]
    for a, b in zip(keys[:-1], keys[1:]):
        pts.append(graded(a, b, dmin, dmax, r)[1:])
    return np.concatenate(pts)


# ----------------------------------------------------------------------------------- solver
def solve_cpwg(W, g, h=H, er=ER, t=T, mask=None, dmin=1.5e-6, side=2.0e-3, top=2.0e-3, air=False):
    """Return capacitance per unit length (full line) [F/m]."""
    xs = [0.0, W / 2, W / 2 + g, W / 2 + g + side]
    tm = mask["t_um"] * 1e-6 if mask else 0.0
    ys = [0.0, h, h + t, h + t + top] + ([h + t + tm] if mask else [])
    x = build_axis(xs, dmin, 40e-6)
    y = build_axis(ys, dmin, 40e-6)
    nx, ny = x.size, y.size
    xc = 0.5 * (x[:-1] + x[1:])
    yc = 0.5 * (y[:-1] + y[1:])
    XC, YC = np.meshgrid(xc, yc, indexing="ij")
    eps = np.ones((nx - 1, ny - 1))
    if not air:
        eps[YC < h] = er
        if mask:
            eps[(YC > h) & (YC < h + t + tm) & ~((XC < W / 2) & (YC < h + t)) & ~((XC > W / 2 + g) & (YC < h + t))] = mask["er"]
    # Dirichlet nodes
    X, Y = np.meshgrid(x, y, indexing="ij")
    tol = 1e-12
    strip = (X <= W / 2 + tol) & (Y >= h - tol) & (Y <= h + t + tol)
    gnd = ((X >= W / 2 + g - tol) & (Y >= h - tol) & (Y <= h + t + tol)) | (Y <= tol) | (Y >= y[-1] - tol)
    fixed = strip | gnd
    phi_fixed = np.where(strip, 1.0, 0.0)
    dx = np.diff(x)
    dy = np.diff(y)
    # edge coefficients (permittivity-weighted face length / edge length)
    # horizontal edges (i,j)-(i+1,j): faces from cells (i, j-1) and (i, j)
    ah = np.zeros((nx - 1, ny))
    ah[:, 1:] += eps * dy[None, :] / 2
    ah[:, :-1] += eps * dy[None, :] / 2
    ah /= dx[:, None]
    av = np.zeros((nx, ny - 1))
    av[1:, :] += eps * dx[:, None] / 2
    av[:-1, :] += eps * dx[:, None] / 2
    av /= dy[None, :]
    nid = -np.ones((nx, ny), dtype=np.int64)
    free = ~fixed
    nid[free] = np.arange(free.sum())
    nf = int(free.sum())
    rows, cols, vals = [], [], []
    b = np.zeros(nf)

    def couple(i1, j1, i2, j2, a):
        # contributions for an edge with coefficient array a (same shape as index arrays)
        f1 = free[i1, j1]
        f2 = free[i2, j2]
        # node 1 equation
        m = f1
        rows.append(nid[i1, j1][m]); cols.append(nid[i1, j1][m]); vals.append(a[m])
        mm = f1 & f2
        rows.append(nid[i1, j1][mm]); cols.append(nid[i2, j2][mm]); vals.append(-a[mm])
        md = f1 & ~f2
        np.add.at(b, nid[i1, j1][md], a[md] * phi_fixed[i2, j2][md])
        # node 2 equation
        m = f2
        rows.append(nid[i2, j2][m]); cols.append(nid[i2, j2][m]); vals.append(a[m])
        rows.append(nid[i2, j2][mm]); cols.append(nid[i1, j1][mm]); vals.append(-a[mm])
        md = f2 & ~f1
        np.add.at(b, nid[i2, j2][md], a[md] * phi_fixed[i1, j1][md])

    I, J = np.meshgrid(np.arange(nx - 1), np.arange(ny), indexing="ij")
    couple(I, J, I + 1, J, ah)
    I, J = np.meshgrid(np.arange(nx), np.arange(ny - 1), indexing="ij")
    couple(I, J, I, J + 1, av)
    A = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(nf, nf))
    sol = spla.spsolve(A.tocsc(), b)
    phi = phi_fixed.copy()
    phi[free] = sol
    energy = (ah * (phi[1:, :] - phi[:-1, :]) ** 2).sum() + (av * (phi[:, 1:] - phi[:, :-1]) ** 2).sum()
    return 2 * EPS0 * energy, (x, y, phi)


def z0_fd(W, g, **kw):
    c, field = solve_cpwg(W, g, **kw)
    ca, _ = solve_cpwg(W, g, air=True, **{k: v for k, v in kw.items() if k not in ("er", "mask")})
    return 1 / (C0 * math.sqrt(c * ca)), c / ca, field


def kk(k):
    return ellipk(k * k) / ellipk(1 - k * k)


def z0_gcpw_closed(W, g, h=H, er=ER):
    k = W / (W + 2 * g)
    k3 = math.tanh(math.pi * W / (4 * h)) / math.tanh(math.pi * (W + 2 * g) / (4 * h))
    q = kk(k3) / kk(k)
    eeff = (1 + er * q) / (1 + q)
    z = 60 * math.pi / math.sqrt(eeff) / (kk(k) + kk(k3))
    return z, eeff


def z0_msl_hj(W, h=H, er=ER, t=T):
    u = W / h
    eta0 = 376.73
    def zair(u):
        f = 6 + (2 * math.pi - 6) * math.exp(-(30.666 / u) ** 0.7528)
        return eta0 / (2 * math.pi) * math.log(f / u + math.sqrt(1 + 4 / u ** 2))
    def ee(u):
        a = 1 + math.log((u ** 4 + (u / 52) ** 2) / (u ** 4 + 0.432)) / 49 + math.log(1 + (u / 18.1) ** 3) / 18.7
        bb = 0.564 * ((er - 0.9) / (er + 3)) ** 0.053
        return (er + 1) / 2 + (er - 1) / 2 * (1 + 10 / u) ** (-a * bb)
    if t > 0:
        th = t / h
        du1 = th / math.pi * math.log(1 + 4 * math.e / (th / math.tanh(math.sqrt(6.517 * u)) ** 2))
        dur = 0.5 * (1 + 1 / math.cosh(math.sqrt(er - 1))) * du1
    else:
        du1 = dur = 0.0
    u1, ur = u + du1, u + dur
    eeff = ee(ur) * (zair(u1) / zair(ur)) ** 2
    return zair(ur) / math.sqrt(ee(ur)), eeff


def width_for_50(g, **kw):
    lo, hi = 0.06e-3, 0.40e-3
    zlo = z0_fd(lo, g, **kw)[0]
    zhi = z0_fd(hi, g, **kw)[0]
    for _ in range(14):
        # interpolate in log(W)
        f = (math.log(zlo) - math.log(50)) / (math.log(zlo) - math.log(zhi))
        wm = math.exp(math.log(lo) + f * (math.log(hi) - math.log(lo)))
        wm = min(max(wm, lo * 1.02), hi * 0.98)
        zm = z0_fd(wm, g, **kw)[0]
        if abs(zm - 50) < 0.02:
            return wm, zm
        if zm > 50:
            lo, zlo = wm, zm
        else:
            hi, zhi = wm, zm
    return wm, zm


# ----------------------------------------------------------------------------------- network
def parts(q_l=30.0, srf_l=8e9, esr_c=0.2, esl_c=0.35e-9):
    return {"C0": ("Csh", 0.8e-12), "L1": ("Lse", 3.0e-9), "C1": ("Csh", 1.2e-12),
            "L2C4": ("LCpar", 2.5e-9, 0.5e-12), "C2": ("Csh", 1.2e-12), "C3": ("Cse", 100e-12),
            "_par": dict(q_l=q_l, srf_l=srf_l, esr_c=esr_c, esl_c=esl_c)}


def z_ind(L, w, q_l, srf):
    f = w / (2 * math.pi)
    r = 2 * math.pi * 2.45e9 * L / q_l * np.sqrt(f / 2.45e9)
    cp = 1 / ((2 * math.pi * srf) ** 2 * L)
    zl = r + 1j * w * L
    return zl / (1 + 1j * w * cp * zl)


def z_cap(C, w, esr, esl):
    return esr + 1j * w * esl + 1 / (1j * w * C)


def abcd_network(f, p):
    w = 2 * np.pi * f
    pr = p["_par"]
    A = np.ones_like(w, dtype=complex)
    B = np.zeros_like(w, dtype=complex)
    Cc = np.zeros_like(w, dtype=complex)
    D = np.ones_like(w, dtype=complex)

    def cascade(a2, b2, c2, d2):
        nonlocal A, B, Cc, D
        A, B, Cc, D = A * a2 + B * c2, A * b2 + B * d2, Cc * a2 + D * c2, Cc * b2 + D * d2
    one = np.ones_like(w, dtype=complex)
    zero = np.zeros_like(w, dtype=complex)
    for key in ("C0", "L1", "C1", "L2C4", "C2", "C3"):
        el = p[key]
        if el[0] == "Csh":
            y = 1 / z_cap(el[1], w, pr["esr_c"], pr["esl_c"])
            cascade(one, zero, y, one)
        elif el[0] == "Lse":
            cascade(one, z_ind(el[1], w, pr["q_l"], pr["srf_l"]), zero, one)
        elif el[0] == "LCpar":
            zl = z_ind(el[1], w, pr["q_l"], pr["srf_l"])
            zc = z_cap(el[2], w, pr["esr_c"], pr["esl_c"])
            cascade(one, zl * zc / (zl + zc), zero, one)
        elif el[0] == "Cse":
            cascade(one, z_cap(el[1], w, 0.1, 0.4e-9), zero, one)
    z0 = 50.0
    den = A + B / z0 + Cc * z0 + D
    s21 = 2 / den
    s11 = (A + B / z0 - Cc * z0 - D) / den
    return s21, s11


def main():
    print("Study 7: RF")
    res = {}
    # ---------------- solver validation ----------------
    val = {}
    for (W, g) in ((0.15e-3, 0.15e-3), (0.20e-3, 0.25e-3)):
        z_fd0, ee0, _ = z0_fd(W, g, t=0.0)
        z_cf, ee_cf = z0_gcpw_closed(W, g)
        val[f"W{W*1e3:.2f}_g{g*1e3:.2f}_t0"] = {"FD": z_fd0, "closed_form_GCPW": z_cf, "eeff_FD": ee0, "eeff_CF": ee_cf}
        print(f"  validate t=0 W={W*1e3:.2f} g={g*1e3:.2f}: FD {z_fd0:.2f} ohm (eeff {ee0:.3f}) vs GCPW closed form "
              f"{z_cf:.2f} ohm (eeff {ee_cf:.3f})")
    zmsl_fd = z0_fd(0.18e-3, 1.5e-3)[0]
    zmsl_hj = z0_msl_hj(0.18e-3)[0]
    val["microstrip_W0.18_t35_wide_gap"] = {"FD_g1.5mm": zmsl_fd, "Hammerstad_Jensen": zmsl_hj}
    print(f"  validate microstrip W=0.18 mm t=35 um: FD (g=1.5 mm) {zmsl_fd:.2f} ohm vs H-J {zmsl_hj:.2f} ohm")
    # grid convergence on one geometry
    zc1 = z0_fd(0.15e-3, 0.15e-3, dmin=3e-6)[0]
    zc2 = z0_fd(0.15e-3, 0.15e-3, dmin=1.5e-6)[0]
    zc3 = z0_fd(0.15e-3, 0.15e-3, dmin=0.75e-6)[0]
    val["grid_convergence_W0.15_g0.15"] = {"dmin3um": zc1, "dmin1.5um(default)": zc2, "dmin0.75um": zc3}
    print(f"  grid convergence: {zc1:.3f} / {zc2:.3f} / {zc3:.3f} ohm")
    res["validation"] = val
    # ---------------- width table ----------------
    gaps = [0.10e-3, 0.125e-3, 0.15e-3, 0.20e-3, 0.25e-3, 0.30e-3]
    table = []
    for g in gaps:
        W, z = width_for_50(g)
        zm = z0_fd(W, g, mask=SM)[0]
        _, ee, _ = z0_fd(W, g)
        Wm, _ = width_for_50(g, mask=SM)
        zcf, _ = z0_gcpw_closed(W, g)
        table.append({"gap_mm": g * 1e3, "W50_mm_bare": W * 1e3, "Z0_check": z, "eeff": ee,
                      "Z0_with_mask_at_same_W": zm, "W50_mm_with_mask": Wm * 1e3,
                      "closed_form_t0_at_W": zcf})
        print(f"  gap {g*1e3:.3f} mm: W50 = {W*1e3:.4f} mm (bare, eeff {ee:.3f}); with 15 um mask Z0 = {zm:.2f} ohm,"
              f" W50(mask) = {Wm*1e3:.4f} mm")
    res["width_table"] = table
    # sensitivity at the recommended geometry
    rec_g = 0.15e-3
    rec_W = round([r for r in table if abs(r["gap_mm"] - 0.15) < 1e-6][0]["W50_mm_bare"] * 1e3 / 5) * 5 / 1e6
    sens = {"W_mm": rec_W * 1e3, "g_mm": rec_g * 1e3}
    sens["Z0_nominal"] = z0_fd(rec_W, rec_g)[0]
    sens["Z0_mask"] = z0_fd(rec_W, rec_g, mask=SM)[0]
    sens["Z0_h_minus10pct"] = z0_fd(rec_W, rec_g, h=0.9 * H)[0]
    sens["Z0_h_plus10pct"] = z0_fd(rec_W, rec_g, h=1.1 * H)[0]
    sens["Z0_W_minus20um_etch"] = z0_fd(rec_W - 20e-6, rec_g + 20e-6)[0]
    sens["Z0_er_4.2"] = z0_fd(rec_W, rec_g, er=4.2)[0]
    sens["Z0_er_4.6"] = z0_fd(rec_W, rec_g, er=4.6)[0]
    res["recommended_sensitivity"] = sens
    print("  recommended", {k: round(v, 2) for k, v in sens.items()})
    # field plot data
    _, _, field = z0_fd(rec_W, rec_g)

    # ---------------- matching network ----------------
    f = np.linspace(1.5e9, 8.0e9, 3000)
    cases = {"ideal (lossless, no parasitics)": parts(q_l=1e9, srf_l=1e15, esr_c=1e-6, esl_c=1e-15),
             "0402 parasitics, L Q=30, SRF 8 GHz": parts(q_l=30, srf_l=8e9),
             "0402 parasitics, L Q=60, SRF 8 GHz": parts(q_l=60, srf_l=8e9),
             "0402 parasitics, L Q=30, SRF 6 GHz": parts(q_l=30, srf_l=6e9)}
    net = {}
    curves = {}
    for name, p in cases.items():
        s21, s11 = abcd_network(f, p)
        band = (f >= 2.4e9) & (f <= 2.5e9)
        il = -20 * np.log10(np.abs(s21))
        rl = -20 * np.log10(np.abs(s11))
        def at(fx):
            k = np.argmin(np.abs(f - fx))
            return il[k]
        diss = 1 - np.abs(s11) ** 2 - np.abs(s21) ** 2
        net[name] = {"IL_band_max_dB": float(il[band].max()), "IL_band_min_dB": float(il[band].min()),
                     "RL_band_min_dB": float(rl[band].min()), "dissipated_pct_band_max": float(100 * diss[band].max()),
                     "atten_2f0_4.9GHz_dB": float(at(4.9e9)), "atten_3f0_7.35GHz_dB": float(at(7.35e9)),
                     "rel_2f0_dB": float(at(4.9e9) - at(2.45e9)), "rel_3f0_dB": float(at(7.35e9) - at(2.45e9)),
                     "min_atten_4.8-5.0GHz_dB": float(il[(f >= 4.8e9) & (f <= 5.0e9)].min()),
                     "min_atten_7.2-7.5GHz_dB": float(il[(f >= 7.2e9) & (f <= 7.5e9)].min())}
        curves[name] = (il, rl)
        print(f"  {name:38s} IL(2.4-2.5) {il[band].min():.2f}..{il[band].max():.2f} dB, RL >= {rl[band].min():.1f} dB, "
              f"2f0 {net[name]['min_atten_4.8-5.0GHz_dB']:.1f} dB, 3f0 {net[name]['min_atten_7.2-7.5GHz_dB']:.1f} dB")
    res["matching_network"] = net
    res["matching_note"] = ("50-ohm source assumed at RFIO; the SX1280 PA optimum load and LNA input are not 50 ohm, "
                            "so the real in-band match must be tuned on a VNA; the harmonic numbers are filter "
                            "attenuation between 50-ohm terminations")

    # ================= plots =================
    fig, axs = plt.subplots(1, 2, figsize=(11, 4.3))
    ax = axs[0]
    ax.plot([r["gap_mm"] for r in table], [r["W50_mm_bare"] for r in table], marker="o", ms=4, color=PAL[0],
            label="50 ohm width, bare copper (FD)")
    ax.plot([r["gap_mm"] for r in table], [r["W50_mm_with_mask"] for r in table], marker="s", ms=4, color=PAL[1],
            label="50 ohm width, 15 um solder mask (FD)")
    ax.axhline(0.09, color=INK2, ls=":", lw=1)
    ax.text(0.1, 0.092, "0.09 mm min feature", fontsize=7.5, color=INK2)
    ax.set_xlabel("gap to coplanar ground g (mm)")
    ax.set_ylabel("trace width W for 50 ohm (mm)")
    ax.set_title("CPWG on L6 over L5 (h 0.0994 mm, er 4.45, t 35 um)")
    ax.set_ylim(0.05, 0.25)
    ax.legend(loc="lower right")
    ax = axs[1]
    x, y, phi = field
    m = x < 0.6e-3
    mm = y < 0.35e-3
    im = ax.contourf(x[m] * 1e3, y[mm] * 1e3, phi[np.ix_(m, mm)].T, levels=21, cmap="Blues")
    ax.add_patch(plt.Rectangle((0, H * 1e3), rec_W / 2 * 1e3, T * 1e3, color=INK2))
    ax.add_patch(plt.Rectangle(((rec_W / 2 + rec_g) * 1e3, H * 1e3), 0.6, T * 1e3, color=INK2))
    ax.axhline(H * 1e3, color=INK2, lw=0.5, ls=":")
    ax.set_xlabel("x (mm), symmetry plane at 0")
    ax.set_ylabel("y (mm), L5 GND at 0")
    ax.set_title(f"Potential, W = {rec_W*1e3:.3f} mm, g = {rec_g*1e3:.2f} mm")
    ax.grid(False)
    fig.colorbar(im, ax=ax, shrink=0.85)
    fig.tight_layout()
    S.save(fig, "s7_cpwg.png")

    fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
    for (name, (il, rl)), col in zip(curves.items(), PAL):
        axs[0].plot(f / 1e9, -il, color=col, label=name)
        axs[1].plot(f / 1e9, -rl, color=col, label=name)
    for ax in axs:
        for fx in (2.45, 4.9, 7.35):
            ax.axvline(fx, color=INK2, lw=0.7, ls=":")
        ax.set_xlabel("frequency (GHz)")
    axs[0].set_ylabel("S21 (dB)")
    axs[0].set_title("SX1280 ref. match + PI filter, 50-ohm ports: S21")
    axs[0].set_ylim(-60, 2)
    axs[0].legend(loc="lower left", fontsize=7.5)
    axs[1].set_ylabel("S11 (dB)")
    axs[1].set_title("S11 (50-ohm source assumption)")
    axs[1].set_ylim(-35, 1)
    fig.tight_layout()
    S.save(fig, "s7_matching_network.png")

    S.write_results("study7_rf", res)


if __name__ == "__main__":
    main()
