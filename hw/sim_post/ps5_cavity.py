"""Post-layout study P5: VBAT/GND plane-pair cavity modes and PDN impedance up to 6 GHz.

Method
------
* The L4 VBAT plane and the L5 GND plane (0.16 mm core, er 4.6) form a parallel-plate cavity whose shape is the
  real VBAT copper (0.1 mm raster, antipads included). Its open edges are magnetic walls. A 5-point FD Laplacian with
  natural (Neumann) boundaries gives the eigenmodes k_n and phi_n (scipy eigsh, shift-invert).
  f_n = c*k_n/(2*pi*sqrt(er)).
* Port impedances come from the modal expansion
      Z_ij(w) = j*w*mu0*d * sum_n phi_n(i)*phi_n(j) / (k_n^2 - k^2*(1 - j*tan_d)).
  Each VBAT decoupling capacitor is a port at its plane via, terminated with ESR + j*w*ESL + 1/(j*w*C)
  (ESL includes the via, by package [A]). The loaded impedance at the driver VM ports is the Schur complement.
* Same analysis for a candidate change: VBAT plane trimmed to the hub (|x|,|y| <= 10 mm) with L4 GND in the motor
  discs. This shows whether keeping the VBAT cavity small moves its first mode above the 2.4 GHz ISM band.

Outputs: out/p5_cavity_modes.png, out/p5_cavity_z.png, out/p5_cavity.json
"""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import postlib as PL  # noqa: E402
import fd2d  # noqa: E402
import numpy as np  # noqa: E402
import scipy.sparse as sp  # noqa: E402
import scipy.sparse.linalg as spla  # noqa: E402

S = PL.S
plt = PL.plt
RES = 0.1
K = int(round(RES / PL.X.RES))
D = PL.T_DIEL[3]
ER = PL.ER_DIEL[3]
TAN_D = 0.02
NMODES = 150                     # enough modes for the port (via) inductance to converge at 0.1 mm cells
DRIVERS = ["U11", "U12", "U13", "U14"]
ESL = {"0201": 0.30e-9, "0402": 0.40e-9, "0603": 0.55e-9, "0805": 0.65e-9}   # incl. ~0.15 nH via [A]


def cap_value(v):
    v = v.replace("µ", "u").split()[0]
    mul = {"p": 1e-12, "n": 1e-9, "u": 1e-6}
    m = re.match(r"([0-9.]+)\s*([pnu])", v)
    return float(m.group(1)) * mul[m.group(2)] if m else None


def modes(mask):
    n = mask.shape[0]
    lin = -np.ones(mask.shape, dtype=np.int64)
    lin[mask] = np.arange(int(mask.sum()))
    rows, cols = [], []
    for di, dj in ((0, 1), (1, 0)):
        a, c = lin[:n - di, :n - dj], lin[di:, dj:]
        m = (a >= 0) & (c >= 0)
        rows += [a[m], c[m]]
        cols += [c[m], a[m]]
    r, c = np.concatenate(rows), np.concatenate(cols)
    N = int(mask.sum())
    W = sp.coo_matrix((np.ones(r.size), (r, c)), shape=(N, N)).tocsr()
    deg = np.asarray(W.sum(axis=1)).ravel()
    h = RES * 1e-3
    Lap = (sp.diags(deg) - W) / h ** 2
    vals, vecs = spla.eigsh(Lap.tocsc(), k=NMODES, sigma=-1.0, which="LM")
    order = np.argsort(vals)
    vals, vecs = np.clip(vals[order], 0, None), vecs[:, order]
    k = np.sqrt(vals)
    f = fd2d.C_LIGHT * k / (2 * np.pi * math.sqrt(ER))
    return lin, k, f, vecs


def zmatrix(freqs, k, vecs, idx, A_cell):
    """Z (n_f x P x P) at port node indices idx."""
    phi = vecs[idx, :] / math.sqrt(A_cell)          # P x M, normalised so that int phi^2 dA = 1
    out = np.zeros((len(freqs), len(idx), len(idx)), dtype=complex)
    for i, fr in enumerate(freqs):
        w = 2 * np.pi * fr
        kk = (w * math.sqrt(ER) / fd2d.C_LIGHT) ** 2 * (1 - 1j * TAN_D)
        den = k ** 2 - kk
        out[i] = 1j * w * fd2d.MU0 * D * (phi / den[None, :]) @ phi.T
    return out


def analyse(label, mask, b, fp, caps, loads):
    lin, k, f, vecs = modes(mask)
    A_cell = (RES * 1e-3) ** 2
    print("%s: %d cells (%.0f mm2), first modes (GHz): %s" % (label, int(mask.sum()), mask.sum() * RES * RES,
                                                             " ".join("%.2f" % v for v in f[1:9] / 1e9)), flush=True)

    def node(x, y):
        i, j = PL.cell_of(x, y, RES)
        if lin[i, j] >= 0:
            return lin[i, j]
        # nearest plane cell (port via just outside the copper raster)
        ii, jj = np.nonzero(lin >= 0)
        q = np.argmin((ii - i) ** 2 + (jj - j) ** 2)
        return lin[ii[q], jj[q]]
    ports = [node(*xy) for xy in loads.values()]
    cports = [node(c["x"], c["y"]) for c in caps]
    idx = ports + cports
    freqs = np.logspace(7, np.log10(6e9), 500)
    Z = zmatrix(freqs, k, vecs, idx, A_cell)
    P = len(ports)
    zl_bare = np.array([np.diag(Z[i, :P, :P]) for i in range(len(freqs))])
    zc = np.array([[c["esr"] + 1j * 2 * np.pi * fr * c["esl"] + 1 / (1j * 2 * np.pi * fr * c["C"]) for c in caps]
                   for fr in freqs])
    zl = np.zeros((len(freqs), P), dtype=complex)
    for i in range(len(freqs)):
        Zll, Zlc = Z[i, :P, :P], Z[i, :P, P:]
        Zcl, Zcc = Z[i, P:, :P], Z[i, P:, P:]
        Zt = Zll - Zlc @ np.linalg.solve(Zcc + np.diag(zc[i]), Zcl)
        zl[i] = np.diag(Zt)
    band = (freqs > 2.4e9) & (freqs < 2.4835e9)
    near = [float(v) for v in f[1:] if 1.8e9 < v < 3.2e9]
    # effective VM-loop inductance seen by each driver (plane spreading + via + cap ESL), from Im Z at 20-50 MHz
    sel = (freqs > 20e6) & (freqs < 50e6)
    l_eff = [float(np.median(np.imag(zl[sel, kk]) / (2 * np.pi * freqs[sel]))) for kk in range(P)]
    i_step, t_edge = 2.8, 4.0 / 35e6                  # hover phase-current peak, VM/SR edge (study 2, SLEW=AGND)
    bounce = [max(lv, 0.0) * i_step / t_edge for lv in l_eff]
    return dict(label=label, f_modes=f, k=k, vecs=vecs, lin=lin, freqs=freqs, z_bare=zl_bare, z=zl,
                ism_modes=near, z_ism_max=float(np.abs(zl[band]).max()), z_peak=float(np.abs(zl).max()),
                f_peak=float(freqs[np.argmax(np.abs(zl).max(axis=1))]), l_eff=l_eff, bounce=bounce)


def main():
    S.style()
    b = PL.load()
    fp = PL.footprints(b)
    fine = PL.fine_rasters(b)
    vb = PL.block_mean(fine["VBAT", "In3.Cu"].astype(float), K) > 0.5
    # decoupling capacitors between VBAT and GND, port at the VBAT pad's plane via (nearest VBAT via)
    vvias = [(x, y) for (x, y, n, d, p) in PL.power_vias(b) if n == "VBAT"]
    caps = []
    for ref, f in fp.items():
        ps = list(f.Pads())
        if len(ps) != 2 or {p.GetNetname() for p in ps} != {"VBAT", "GND"} or not ref.startswith("C"):
            continue
        C = cap_value(f.GetValue())
        if not C:
            continue
        pk = re.search(r"(0201|0402|0603|0805)", f.GetFPIDAsString())
        pkg = pk.group(1) if pk else "0402"
        vp = [p for p in ps if p.GetNetname() == "VBAT"][0]
        x, y = PL.xy(vp.GetPosition())
        if vvias:
            q = min(vvias, key=lambda v: (v[0] - x) ** 2 + (v[1] - y) ** 2)
            x, y = q
        caps.append(dict(ref=ref, C=C * (0.5 if C > 1e-6 else 0.8), esl=ESL[pkg], esr=0.005 if C > 1e-6 else 0.02,
                         x=x, y=y, pkg=pkg, value=f.GetValue()))
    print("VBAT decoupling: %d caps: %s" % (len(caps), ", ".join("%s %s" % (c["ref"], c["value"]) for c in caps)))
    loads = {}
    for r in DRIVERS:
        vp = [p for p in fp[r].Pads() if p.GetNetname() == "VBAT"][0]
        x, y = PL.xy(vp.GetPosition())
        q = min(vvias, key=lambda v: (v[0] - x) ** 2 + (v[1] - y) ** 2)
        loads[r] = q
    rx, ry = PL.xy(fp["U4"].GetPosition())
    loads["radio"] = (rx, ry)
    res = {}
    runs = [analyse("as routed", vb, b, fp, caps, loads)]
    n = vb.shape[0]
    xs = (np.arange(n) + 0.5) * RES - PL.X.HALF
    XX, YY = np.meshgrid(xs, xs)
    hub = vb & (np.abs(XX) <= 10.0) & (np.abs(YY) <= 10.0)
    runs.append(analyse("VBAT trimmed to hub", hub, b, fp, caps, loads))
    for r in runs:
        res[r["label"]] = dict(modes_GHz=(r["f_modes"][1:16] / 1e9), ism_modes=r["ism_modes"],
                               z_ism_max=r["z_ism_max"], z_peak=r["z_peak"], f_peak=r["f_peak"],
                               l_eff_nH={nm: 1e9 * lv for nm, lv in zip(loads, r["l_eff"])},
                               vm_bounce_mV={nm: 1e3 * bv for nm, bv in zip(loads, r["bounce"])})
        print("  %-20s VM-loop L_eff (nH): %s -> bounce on a 2.8 A / 114 ns edge (mV): %s" % (
            r["label"], " ".join("%s %.2f" % (nm, 1e9 * lv) for nm, lv in zip(loads, r["l_eff"])),
            " ".join("%s %.0f" % (nm, 1e3 * bv) for nm, bv in zip(loads, r["bounce"]))))
        print("  %-20s modes in 1.8-3.2 GHz: %s | max |Z| at VM ports in ISM band %.2f ohm, peak %.2f ohm @ %.2f GHz"
              % (r["label"], ["%.2f" % (v / 1e9) for v in r["ism_modes"]], r["z_ism_max"], r["z_peak"],
                 r["f_peak"] / 1e9))
    res["caps"] = [{k: v for k, v in c.items()} for c in caps]
    # ---- figures -------------------------------------------------------------------------------------------------
    fig, ax = plt.subplots(2, 4, figsize=(16, 8))
    for row, r in enumerate(runs):
        for kk in range(4):
            a = ax[row, kk]
            img = np.full(r["lin"].shape, np.nan)
            m = r["lin"] >= 0
            img[m] = r["vecs"][r["lin"][m], kk + 1]
            lim = np.nanmax(np.abs(img))
            a.imshow(img, extent=(-PL.X.HALF, PL.X.HALF, PL.X.HALF, -PL.X.HALF), cmap="RdBu_r", vmin=-lim, vmax=lim)
            PL.board_extent_axes(a)
            a.set_title("%s: mode %d, %.2f GHz" % (r["label"], kk + 1, r["f_modes"][kk + 1] / 1e9), fontsize=8.5)
            a.grid(False)
    PL.save(fig, "p5_cavity_modes.png")
    fig, ax = plt.subplots(1, 2, figsize=(15, 4.8), sharey=True)
    for a, r in zip(ax, runs):
        for kk, name in enumerate(list(loads)):
            a.loglog(r["freqs"], np.abs(r["z"][:, kk]), color=S.PAL[kk], label=name)
        a.loglog(r["freqs"], np.abs(r["z_bare"][:, 0]), color=S.INK2, ls=":", label="U11, plane only")
        a.axvspan(2.4e9, 2.4835e9, color=S.PAL[2], alpha=0.15)
        a.set(xlabel="Hz", title="VBAT-GND |Z| at the ports: %s" % r["label"])
        a.legend(fontsize=7)
    ax[0].set_ylabel("ohm")
    PL.save(fig, "p5_cavity_z.png")
    PL.write_results("p5_cavity", res)


if __name__ == "__main__":
    main()
