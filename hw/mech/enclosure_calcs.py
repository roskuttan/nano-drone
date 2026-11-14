"""enclosure_calcs.py - hand-calculation checks for the 3D-printed enclosure (no Fusion needed).

Inputs mirror the constants in mech/fusion/nanodrone_enclosure/nanodrone_enclosure.py (keep them in sync) plus the
shell volumes that script reports. Prints a markdown block for docs/ENCLOSURE.md:
  1. duct cross-section properties (area, in-plane / out-of-plane second moments)
  2. first in-plane ring mode of a duct (free-ring estimate, the struts only stiffen it)
  3. fin strut: section, length, Euler buckling, compressive yield; landing load the bottom ducts carry
  4. mass, all-up weight and thrust-to-weight for the Pro and Lite tops
These are estimates for sizing and sanity, not a substitute for a tap test or a blackbox spectrum.
"""
import math

# ---- geometry (mm) from nanodrone_enclosure.py
R_IN, WALL, LIP_R, BEAD_R = 16.5, 0.7, 0.55, 0.55
TOP_DUCT_Z, BOT_DUCT_Z = (10.6, 15.5), (-14.8, -9.4)
STRUT_W, STRUT_T = 0.9, 1.5
STRUTS = {"top": (3.8, 16.85, 0.737), "bottom": (5.9, 16.85, 0.705)}   # root r, ring r, slope (from the build)
# ---- shell volumes (mm3) reported by the Fusion build
V_TOP, V_TOP_LITE, V_BOT = 1645.0, 733.0, 1696.0
# ---- material: HP MJF PA12 (datasheet ballpark)
E_PA12, RHO_PA12, SY_PA12 = 1800.0, 1.01e-3, 48.0      # MPa, g/mm3, MPa (tensile)
DENS = (("PA12 MJF", 1.01), ("SLA resin", 1.15), ("PETG FDM", 1.27))
# ---- flight budget (sim/SIMULATION_REPORT.md study 4, 300 mAh pack): AUW 16.0 g without the enclosure
# (dry 7.7 g + pack 8.3 g); max vehicle thrust (4 motors x 0.9 stacked-prop factor) at 4.30 / 3.45 / 3.30 V OCV
AUW0 = 16.0
THRUST = ((4.30, 56.3), (3.45, 41.1), (3.30, 38.6))
PLATE_AREA = 470.0      # mm2 per body plate (23.7 mm rounded square less vents and windows), for the variant estimate


def arc(c, r, a0, a1, n=24):
    return [(c[0] + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             c[1] + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def duct_polygon(zb, zt, lip_r, bead_r, wall=WALL):
    """Same outline as duct_section() in the generator, arcs discretised: (r, z) points, counter-clockwise."""
    ri, ro = R_IN, R_IN + wall
    lc, bc = zt - lip_r, zb + bead_r
    pts = [(ri, bc), (ri, lc)]
    pts += arc((ri + lip_r, lc), lip_r, 180, 0)[1:]                 # rolled lip over the top
    pts.append((ro, lc - (2 * lip_r - wall) - 0.6))
    if 2 * bead_r > wall + 1e-6:
        pts.append((ro, bc + (2 * bead_r - wall) + 0.6))
    pts.append((ri + 2 * bead_r, bc))
    pts += arc((ri + bead_r, bc), bead_r, 0, -180)[1:-1]            # bead / rounded bottom edge
    return pts


def section(pts):
    """Area, centroid and second moments of a polygon in the (r, z) plane."""
    A = cr = cz = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        k = x0 * y1 - x1 * y0
        A += k
        cr += (x0 + x1) * k
        cz += (y0 + y1) * k
    A /= 2.0
    cr /= 6.0 * A
    cz /= 6.0 * A
    Irr = Izz = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        x0, x1, y0, y1 = x0 - cr, x1 - cr, y0 - cz, y1 - cz
        k = x0 * y1 - x1 * y0
        Irr += (x0 * x0 + x0 * x1 + x1 * x1) * k      # integral of (r - r_c)^2: in-plane bending
        Izz += (y0 * y0 + y0 * y1 + y1 * y1) * k      # integral of (z - z_c)^2: out-of-plane bending
    return abs(A), cr, cz, abs(Irr) / 12.0, abs(Izz) / 12.0


def ring_mode(A, I, R, n=2):
    """Free circular ring, in-plane flexural mode n (Hz). E in MPa = N/mm2, rho in g/mm3."""
    EI = E_PA12 * 1e6 * I * 1e-12            # N m2
    m = RHO_PA12 * A                         # kg/m  (g/mm3 * mm2 = g/mm = kg/m)
    Rm = R * 1e-3
    return n * (n * n - 1) / math.sqrt(n * n + 1) * math.sqrt(EI / (m * Rm ** 4)) / (2 * math.pi)


def stadium_props(w, t, n=400):
    """Area and second moments of a stadium (width w, height t, round ends of radius w/2 top and bottom)."""
    A = Iw = It = 0.0
    dz = t / n
    for i in range(n):
        z = -t / 2 + (i + 0.5) * dz
        h = (t - w) / 2
        if abs(z) <= h:
            b = w
        else:
            dzc = abs(z) - h
            b = 2 * math.sqrt(max((w / 2) ** 2 - dzc ** 2, 0.0))
        A += b * dz
        It += b * dz * z * z                 # bending in the plane of the height (stiff)
        Iw += b ** 3 / 12.0 * dz             # bending sideways (weak)
    return A, Iw, It


def main():
    out = ["| Check | Top duct | Bottom duct |", "|---|---|---|"]
    props = {}
    for key, (zr, bead) in (("top", (TOP_DUCT_Z, WALL / 2)), ("bottom", (BOT_DUCT_Z, BEAD_R))):
        A, cr, cz, Irr, Izz = section(duct_polygon(zr[0], zr[1], LIP_R, bead))
        props[key] = (A, cr, Irr, Izz, ring_mode(A, Irr, cr))
    t, b = props["top"], props["bottom"]
    out.append("| Section area | %.2f mm2 | %.2f mm2 |" % (t[0], b[0]))
    out.append("| I in-plane (ovalising) | %.3f mm4 | %.3f mm4 |" % (t[2], b[2]))
    out.append("| I out-of-plane (vertical) | %.1f mm4 | %.1f mm4 |" % (t[3], b[3]))
    out.append("| Free-ring in-plane mode n=2 | %.0f Hz | %.0f Hz |" % (t[4], b[4]))
    A, Iw, It = stadium_props(STRUT_W, STRUT_T)
    rows = []
    for key in ("top", "bottom"):
        r0, r1, s = STRUTS[key]
        L = (r1 - r0) * math.sqrt(1 + s * s)
        ang = math.atan(s)
        pcr = math.pi ** 2 * E_PA12 * Iw / (0.7 * L) ** 2           # fixed-ish ends, weak axis
        py = SY_PA12 * A
        vert = 3 * min(pcr, py) * math.sin(ang)                      # one duct, three struts, vertical load
        rows.append((key, L, math.degrees(ang), pcr, py, vert))
    out.append("| Strut length / angle | %.1f mm / %.0f deg | %.1f mm / %.0f deg |" % (rows[0][1], rows[0][2], rows[1][1], rows[1][2]))
    out.append("| Strut Euler buckling (K 0.7, weak axis) | %.1f N | %.1f N |" % (rows[0][3], rows[1][3]))
    out.append("| Strut compressive yield | %.0f N | %.0f N |" % (rows[0][4], rows[1][4]))
    out.append("| Vertical load per duct before struts bow | %.0f N | %.0f N |" % (rows[0][5], rows[1][5]))
    out.append("")
    out.append("Strut section %.1f x %.1f mm stadium: A %.2f mm2, I weak %.3f mm4, I strong %.3f mm4." % (
        STRUT_W, STRUT_T, A, Iw, It))
    auw_pro = AUW0 + (V_TOP + V_BOT) * DENS[0][1] / 1000
    auw_lite = AUW0 + (V_TOP_LITE + V_BOT) * DENS[0][1] / 1000
    land = 2 * rows[1][5] / (auw_pro * 9.81e-3)
    out.append("Landing flat on both bottom ducts, the struts stay straight up to about %.0f g of deceleration "
               "(%.0f N at %.1f g AUW); PA12 struts bow elastically beyond that." % (land, 2 * rows[1][5], auw_pro))
    out.append("")
    out.append("| Variant | Top | Bottom | Enclosure | AUW (300 mAh) | " +
               " | ".join("TWR %.2f V" % v for v, _ in THRUST) + " |")
    out.append("|---|---|---|---|---|" + "---|" * len(THRUST))
    for name, vt in (("Pro (4 ducts)", V_TOP), ("Lite (bottom ducts only)", V_TOP_LITE)):
        for mat, d in DENS:
            m_t, m_b = vt * d / 1000, V_BOT * d / 1000
            auw = AUW0 + m_t + m_b
            out.append("| %s, %s | %.2f g | %.2f g | %.2f g | %.1f g | " % (name, mat, m_t, m_b, m_t + m_b, auw) +
                       " | ".join("%.2f" % (t / auw) for _, t in THRUST) + " |")
    out.append("| No enclosure | - | - | - | %.1f g | " % AUW0 + " | ".join("%.2f" % (t / AUW0) for _, t in THRUST) + " |")
    # sturdier build at JLC's MJF guideline: duct walls 1.0 mm, plates 0.8 mm (Pappus volumes of the duct sections)

    def ring_volume(zr, bead, wall):
        A, cr = section(duct_polygon(zr[0], zr[1], LIP_R, bead, wall))[:2]
        return A * 2 * math.pi * cr

    d_top = 2 * (ring_volume(TOP_DUCT_Z, 0.5, 1.0) - ring_volume(TOP_DUCT_Z, WALL / 2, WALL))
    d_bot = 2 * (ring_volume(BOT_DUCT_Z, BEAD_R, 1.0) - ring_volume(BOT_DUCT_Z, BEAD_R, WALL))
    d_plates = 2 * PLATE_AREA * (0.8 - 0.6)
    out.append("")
    out.append("Sturdier variant (WALL 1.0, SKIN 0.8): +%.0f mm3 ducts, +%.0f mm3 plates, so +%.2f g in PA12 "
               "(Pro %.2f g, Lite %.2f g)." % (
                   d_top + d_bot, d_plates, (d_top + d_bot + d_plates) * 1.01e-3,
                   (V_TOP + V_BOT + d_top + d_bot + d_plates) * 1.01e-3,
                   (V_TOP_LITE + V_BOT + d_bot + d_plates) * 1.01e-3))
    print("\n".join(out))
    return auw_pro, auw_lite


if __name__ == "__main__":
    main()
