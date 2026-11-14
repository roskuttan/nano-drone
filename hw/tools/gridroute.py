"""gridroute - a small multi-layer grid maze router for dense KiCad boards (pcbnew + numpy).

Model
  * routing layers: a list of copper layers (e.g. F.Cu, In2.Cu, B.Cu); through vias join them
  * per layer an int32 "owner" raster (net code per 0.05 mm cell, 0 = free, -1 = always blocked:
    NPTH holes, board-edge band)
  * routing net N with half-width hw and clearance c: a cell is blocked if any copper of another net lies
    within hw + c (shifted-OR disc dilation of the "other copper" mask inside the search window)
  * vias: blocked unless free of other copper within r_via + c on every routing layer
  * A* (octile, 8-neighbour, 45 deg moves), via cost VIA_COST cells
  * after each routed connection the new copper is stamped into the owner rasters
Connections: per net, clusters of already-connected copper (pads, tracks, vias) are joined along the
shortest pad pair (MST order); shortest connections are routed first.
"""
import heapq
import math
import os
import sys

import numpy as np
import pcbnew

_PYLIBS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sim", ".pylibs")
if os.path.isdir(_PYLIBS) and _PYLIBS not in sys.path:
    sys.path.insert(0, _PYLIBS)
try:
    from scipy import ndimage as _ndi        # connected-component labels: instant "no path" answers
except ImportError:                          # pragma: no cover - plain A* still works, only slower to fail
    _ndi = None

OX, OY = 100.0, 100.0
HOLE_WEB = 0.20          # min drill-to-drill web (mm); new vias are 0.15 mm drill
# weighted A* (heuristic x ASTAR_W > 1) trades a little path quality for far fewer expansions on fine grids;
# ASTAR_LIMIT caps the expansions of one search
ASTAR_W = float(os.environ.get("NANODRONE_ASTAR_W", "1.0"))
ASTAR_LIMIT = int(float(os.environ.get("NANODRONE_ASTAR_LIMIT", "4e7")))
NONET_CLEAR = 0.15       # copper/hole clearance to no-net pads (fiducials, NPTH): JLC hole-to-copper 0.15 mm


def mm(v):
    return pcbnew.FromMM(float(v))


class Grid:
    def __init__(self, board, layers, res=0.05, half=18.5, edge_clear=0.25):
        self.b = board
        self.layers = list(layers)
        self.res = res
        self.half = half
        self.n = int(round(2 * half / res))
        self.owner = {l: np.zeros((self.n, self.n), dtype=np.int32) for l in self.layers}
        self.trk = {l: np.zeros((self.n, self.n), dtype=np.int32) for l in self.layers}   # tracks only
        self.edge_clear = edge_clear
        self._build()

    # ---- coordinates -------------------------------------------------------------------------
    def to_cell(self, x, y):
        return int((y + self.half) / self.res), int((x + self.half) / self.res)

    def to_xy(self, r, c):
        return (c + 0.5) * self.res - self.half, (r + 0.5) * self.res - self.half

    # ---- rasterisation -----------------------------------------------------------------------
    def _fill_poly(self, img, pts, val):
        pts = np.asarray(pts, float)
        x0, y0 = pts[:, 0], pts[:, 1]
        x1, y1 = np.roll(x0, -1), np.roll(y0, -1)
        rmin = max(0, int((min(y0.min(), y1.min()) + self.half) / self.res) - 1)
        rmax = min(self.n - 1, int((max(y0.max(), y1.max()) + self.half) / self.res) + 1)
        for r in range(rmin, rmax + 1):
            y = (r + 0.5) * self.res - self.half
            m = (y0 <= y) != (y1 <= y)
            if not m.any():
                continue
            xs = np.sort(x0[m] + (y - y0[m]) * (x1[m] - x0[m]) / (y1[m] - y0[m]))
            for a, b in zip(xs[0::2], xs[1::2]):
                c0 = max(0, int(math.ceil((a + self.half) / self.res - 0.5)))
                c1 = min(self.n - 1, int(math.floor((b + self.half) / self.res - 0.5)))
                if c1 >= c0:
                    img[r, c0:c1 + 1] = val

    def _stamp_shape(self, item, layer, val, img, grow=0.0):
        sps = pcbnew.SHAPE_POLY_SET()
        item.TransformShapeToPolygon(sps, layer, mm(grow), mm(0.005), pcbnew.ERROR_OUTSIDE)
        sps.Simplify()
        for i in range(sps.OutlineCount()):
            o = sps.Outline(i)
            pts = [(o.CPoint(k).x / 1e6 - OX, o.CPoint(k).y / 1e6 - OY) for k in range(o.PointCount())]
            if len(pts) >= 3:
                self._fill_poly(img, pts, val)

    def stamp_segment(self, layer, x0, y0, x1, y1, hw, val, track=False):
        if track:
            self._stamp_seg_into(self.trk[layer], x0, y0, x1, y1, hw, val)
        self._stamp_seg_into(self.owner[layer], x0, y0, x1, y1, hw, val)

    def _stamp_seg_into(self, img, x0, y0, x1, y1, hw, val):
        xa, xb = min(x0, x1) - hw, max(x0, x1) + hw
        ya, yb = min(y0, y1) - hw, max(y0, y1) + hw
        r0, c0 = self.to_cell(xa, ya)
        r1, c1 = self.to_cell(xb, yb)
        r0, c0 = max(r0, 0), max(c0, 0)
        r1, c1 = min(r1 + 1, self.n - 1), min(c1 + 1, self.n - 1)
        rr, cc = np.mgrid[r0:r1 + 1, c0:c1 + 1]
        X = (cc + 0.5) * self.res - self.half
        Y = (rr + 0.5) * self.res - self.half
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = np.zeros_like(X) if L2 == 0 else np.clip(((X - x0) * dx + (Y - y0) * dy) / L2, 0, 1)
        d = np.hypot(X - (x0 + t * dx), Y - (y0 + t * dy))
        sub = img[r0:r1 + 1, c0:c1 + 1]
        sub[(d <= hw) & (sub == 0)] = val

    def stamp_disc(self, x, y, r, val, layers=None):
        for l in (layers or self.layers):
            self.stamp_segment(l, x, y, x, y, r, val)

    def _build(self):
        b = self.b
        # board-edge band
        edge = np.ones((self.n, self.n), dtype=np.int32) * -1
        for d in b.GetDrawings():
            if d.GetLayer() == pcbnew.Edge_Cuts and d.GetShape() == pcbnew.SHAPE_T_POLY:
                sps = pcbnew.SHAPE_POLY_SET(d.GetPolyShape())
                sps.Deflate(mm(self.edge_clear), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, mm(0.01))
                o = sps.Outline(0)
                pts = [(o.CPoint(k).x / 1e6 - OX, o.CPoint(k).y / 1e6 - OY) for k in range(o.PointCount())]
                self._fill_poly(edge, pts, 0)
        for l in self.layers:
            self.owner[l][:] = edge
        for f in b.GetFootprints():
            for p in f.Pads():
                code = p.GetNetCode() if p.GetNetCode() > 0 else -1
                # no-net copper (fiducials, NPTH holes) becomes a hard cell, which A* only dilates by the track
                # half-width: grow it here by the copper/hole clearance and by its solder-mask opening, so no track
                # runs exposed inside a fiducial's mask aperture
                grow = 0.0
                if code == -1:
                    try:
                        mex = max(p.GetSolderMaskExpansion(pcbnew.F_Mask) / 1e6, 0.0)
                    except Exception:
                        mex = 0.3 if "Fiducial" in str(f.GetFPID().GetLibItemName()) else 0.0
                    grow = NONET_CLEAR + (mex if p.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH else 0.0)
                for l in self.layers:
                    if p.IsOnLayer(l) or p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                        lay = l if p.IsOnLayer(l) else pcbnew.F_Cu
                        self._stamp_shape(p, lay, code, self.owner[l], grow)
        # rule areas that forbid tracks on a routing layer are hard obstacles there
        for z in b.Zones():
            if not (z.GetIsRuleArea() and z.GetDoNotAllowTracks()):
                continue
            o = z.Outline().Outline(0)
            pts = [(o.CPoint(k).x / 1e6 - OX, o.CPoint(k).y / 1e6 - OY) for k in range(o.PointCount())]
            for l in self.layers:
                if z.IsOnLayer(l):
                    self._fill_poly(self.owner[l], pts, -1)
        # drilled holes (any net): a new 0.15 mm via must keep HOLE_WEB edge to edge -> keep-out disc per hole
        self.hole = np.zeros((self.n, self.n), dtype=np.int32)
        for f in b.GetFootprints():
            for p in f.Pads():
                if p.GetDrillSize().x > 0:
                    x, y = p.GetPosition().x / 1e6 - OX, p.GetPosition().y / 1e6 - OY
                    self._stamp_seg_into(self.hole, x, y, x, y,
                                         max(p.GetDrillSize().x, p.GetDrillSize().y) / 2e6 + 0.075 + HOLE_WEB, 1)
        for t in b.GetTracks():
            code = t.GetNetCode() if t.GetNetCode() > 0 else -1
            if isinstance(t, pcbnew.PCB_VIA):
                x, y = t.GetPosition().x / 1e6 - OX, t.GetPosition().y / 1e6 - OY
                self._stamp_seg_into(self.hole, x, y, x, y, t.GetDrillValue() / 2e6 + 0.075 + HOLE_WEB, 1)
                self.stamp_disc(x, y, t.GetWidth(pcbnew.F_Cu) / 2e6, code)
            elif t.GetLayer() in self.layers:
                self.stamp_segment(t.GetLayer(), t.GetStart().x / 1e6 - OX, t.GetStart().y / 1e6 - OY,
                                   t.GetEnd().x / 1e6 - OX, t.GetEnd().y / 1e6 - OY, t.GetWidth() / 2e6, code,
                                   track=True)

    # ---- routing -----------------------------------------------------------------------------
    @staticmethod
    def dilate(mask, rad):
        if rad <= 0:
            return mask.copy()
        out = mask.copy()
        H, W = mask.shape
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                if dr * dr + dc * dc > rad * rad + rad:
                    continue
                if dr == 0 and dc == 0:
                    continue
                src = mask[max(0, -dr):H - max(0, dr), max(0, -dc):W - max(0, dc)]
                out[max(0, dr):H - max(0, -dr), max(0, dc):W - max(0, -dc)] |= src
        return out

    special = []      # [(set_of_netcodes, clearance)] extra clearance against specific nets (set per call)
    layer_cost = {}   # {layer: step-cost multiplier >= 1}: a layer used only where the others are full

    def keepout(self, layer, x0, y0, x1, y1, r, vias=True):
        """Keep-out on one routing layer: a capsule of radius r around a segment. vias=True: hard (no tracks, no
        vias). vias=False: tracks only - vias may still pass through the layer there."""
        if vias:
            self._stamp_seg_into(self.owner[layer], x0, y0, x1, y1, r, -1)
        else:
            if not hasattr(self, "tko"):
                self.tko = {}
            m = self.tko.setdefault(layer, np.zeros((self.n, self.n), dtype=np.int32))
            self._stamp_seg_into(m, x0, y0, x1, y1, r, 1)

    # ---- rip-up and reroute support ---------------------------------------------------------------
    def init_rrr(self):
        """Freeze the current rasters as the fixed base; later routes are tracked per route id so they can be
        ripped up. hist is a per-cell congestion history added to the A* step cost."""
        self.base_owner = {l: self.owner[l].copy() for l in self.layers}
        self.base_trk = {l: self.trk[l].copy() for l in self.layers}
        self.base_hole = self.hole.copy()
        self.rid = {l: np.zeros((self.n, self.n), dtype=np.int32) for l in self.layers}
        self.hist = {l: np.zeros((self.n, self.n), dtype=np.float32) for l in self.layers}
        self.routes = {}
        self._next_rid = 1

    def _stamp_route(self, rid, R):
        for (l, x0, y0, x1, y1, hw) in R["segs"]:
            self._stamp_seg_into(self.owner[l], x0, y0, x1, y1, hw, R["net"])
            self._stamp_seg_into(self.trk[l], x0, y0, x1, y1, hw, R["net"])
            self._stamp_seg_into(self.rid[l], x0, y0, x1, y1, hw, rid)
        for (x, y, r) in R["vias"]:
            for l in self.layers:
                self._stamp_seg_into(self.owner[l], x, y, x, y, r, R["net"])
                self._stamp_seg_into(self.rid[l], x, y, x, y, r, rid)
            self._stamp_seg_into(self.hole, x, y, x, y, 0.075 + 0.075 + HOLE_WEB, 1)

    def add_route(self, net, segs, vias, **meta):
        segs = [s for s in segs if (s[1], s[2]) != (s[3], s[4])]      # zero-length bridges carry nothing
        if not segs and not vias:
            return None
        xs = [v for s in segs for v in (s[1], s[3])] + [v[0] for v in vias]
        ys = [v for s in segs for v in (s[2], s[4])] + [v[1] for v in vias]
        R = dict(net=net, segs=segs, vias=vias, bbox=(min(xs) - 0.3, min(ys) - 0.3, max(xs) + 0.3, max(ys) + 0.3))
        R.update(meta)
        rid = self._next_rid
        self._next_rid += 1
        self.routes[rid] = R
        self._stamp_route(rid, R)
        return rid

    def rip(self, rid):
        R = self.routes.pop(rid)
        x0, y0, x1, y1 = R["bbox"]
        r0, c0 = self.to_cell(x0, y0)
        r1, c1 = self.to_cell(x1, y1)
        r0, c0, r1, c1 = max(r0, 0), max(c0, 0), min(r1, self.n - 1), min(c1, self.n - 1)
        for l in self.layers:
            self.owner[l][r0:r1 + 1, c0:c1 + 1] = self.base_owner[l][r0:r1 + 1, c0:c1 + 1]
            self.trk[l][r0:r1 + 1, c0:c1 + 1] = self.base_trk[l][r0:r1 + 1, c0:c1 + 1]
            self.rid[l][r0:r1 + 1, c0:c1 + 1] = 0
        self.hole[r0:r1 + 1, c0:c1 + 1] = self.base_hole[r0:r1 + 1, c0:c1 + 1]
        for k, R2 in self.routes.items():
            a0, b0, a1, b1 = R2["bbox"]
            if a0 <= x1 and a1 >= x0 and b0 <= y1 and b1 >= y0:
                self._stamp_route(k, R2)
        return R

    def rip_net(self, net):
        return [self.rip(k) for k in [k for k, R in self.routes.items() if R["net"] == net]]

    _discs = {}

    def _disc(self, rad):
        """Boolean disc of the same shape dilate() uses (dr^2 + dc^2 <= rad^2 + rad)."""
        m = self._discs.get(rad)
        if m is None:
            dr, dc = np.mgrid[-rad:rad + 1, -rad:rad + 1]
            m = self._discs[rad] = (dr * dr + dc * dc) <= rad * rad + rad
        return m

    def blockers(self, path, net, hw, clear, via_r, special=(), self_rid=None, tol=1):
        """Route ids whose copper lies inside the clearance envelope of a path, with exactly the A* criterion:
        the same disc as dilate(), special rules applied to tracks only where the rule says so, and path cells on
        copper the net already owned (its pads, its earlier routes - A* treats them as free) not checked.
        special: (codes, clearance[, tracks_only]); self_rid: the route id of this path once it is stamped."""
        q = 0.72 * self.res
        found = set()
        hot = []
        prev = None
        rules = [(None, clear, False)] + [(s[0], s[1], s[2] if len(s) > 2 else False) for s in special]
        for (l, x, y) in path:
            is_via = prev is not None and prev[0] != l
            prev = (l, x, y)
            r, c = self.to_cell(x, y)
            if not (0 <= r < self.n and 0 <= c < self.n):
                continue
            if not is_via and self.owner[l][r, c] == net and (self_rid is None or self.rid[l][r, c] != self_rid):
                continue                      # inside the net's own earlier copper: no new copper here
            checks = [(self.layers, via_r)] if is_via else [([l], hw)]
            for lays, h in checks:
                for ll in lays:
                    for codes, sc, tracks_only in rules:
                        # one cell less than A*'s radius: A* tested the new path against the old copper, this tests
                        # the old path against the new copper, and the raster rounding of the two differs by up to a
                        # cell at the boundary (inside q, the A* safety margin, so no real violation is missed)
                        rad = max(int(math.ceil((h + sc + q) / self.res)) - tol, 0)
                        r0, r1 = max(r - rad, 0), min(r + rad + 1, self.n)
                        c0, c1 = max(c - rad, 0), min(c + rad + 1, self.n)
                        m = self._disc(rad)[r0 - (r - rad):r1 - (r - rad), c0 - (c - rad):c1 - (c - rad)]
                        win = self.rid[ll][r0:r1, c0:c1]
                        src = (self.trk if tracks_only else self.owner)[ll][r0:r1, c0:c1]   # as in _astar
                        sel = m & (win != 0) & (src != net) & (src != 0)
                        if codes is not None:
                            sel &= np.isin(src, list(codes))
                        if not sel.any():
                            continue
                        for k in np.unique(win[sel]):
                            R = self.routes.get(int(k))
                            if R is None or R["net"] == net:
                                continue
                            found.add(int(k))
                            hot.append((ll, r, c))
        return found, hot

    def route(self, net, starts, goals, hw, clear, via_r=0.15, via_cost=24, window=2.5, max_window=40.0,
              allowed_layers=None, soft=False, soft_pen=25.0):
        """starts/goals: list of (layer, x, y, own_pad_area_cells(optional)). Returns path [(layer, x, y)...]
        or None. soft=True lets the path cross ripable (routed) copper of other nets at soft_pen per cell."""
        lays = [l for l in self.layers if allowed_layers is None or l in allowed_layers]
        pts = [(x, y) for _, x, y in starts + goals]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        w = window
        while w <= max_window:
            x0, x1 = min(xs) - w, max(xs) + w
            y0, y1 = min(ys) - w, max(ys) + w
            r0, c0 = self.to_cell(x0, y0)
            r1, c1 = self.to_cell(x1, y1)
            r0, c0 = max(r0, 0), max(c0, 0)
            r1, c1 = min(r1, self.n - 1), min(c1, self.n - 1)
            path = self._astar(net, lays, starts, goals, hw, clear, via_r, via_cost, r0, c0, r1, c1, soft, soft_pen)
            if path:
                return path
            if r0 == 0 and c0 == 0 and r1 == self.n - 1 and c1 == self.n - 1:
                return None
            w *= 2.2
        return None

    def _may_connect(self, lays, free, vfree, starts, goal_set, r0, c0, H, W):
        """Necessary condition for an A* path: some start and some goal lie in one connected component of the
        free space (8-connected per layer - a superset of the A* moves - joined across layers where a via may
        stand). False means no path exists in this window, so the search is skipped."""
        st8 = np.ones((3, 3), dtype=bool)
        labs, off = [], [0]
        for li in range(len(lays)):
            lab, n = _ndi.label(free[li], structure=st8)
            labs.append(lab)
            off.append(off[-1] + n)
        parent = list(range(off[-1] + 1))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a
        for li in range(len(lays)):
            for lj in range(li + 1, len(lays)):
                m = vfree & free[li] & free[lj]
                if not m.any():
                    continue
                pairs = np.unique(np.stack([labs[li][m] + off[li], labs[lj][m] + off[lj]], axis=1), axis=0)
                for a, b in pairs:
                    ra, rb = find(int(a)), find(int(b))
                    if ra != rb:
                        parent[ra] = rb
        comp_s = set()
        for sl, sx, sy in starts:
            if sl not in lays:
                continue
            li = lays.index(sl)
            sr, sc = self.to_cell(sx, sy)
            sr, sc = sr - r0, sc - c0
            if not (0 <= sr < H and 0 <= sc < W):
                continue
            if (li, sr, sc) in goal_set:
                return True
            # the start cell itself need not be free: A* steps from it to any free neighbour or drops a via
            for lj in range(len(lays)):
                if lj != li and not vfree[sr, sc]:
                    continue
                win = labs[lj][max(sr - 1, 0):sr + 2, max(sc - 1, 0):sc + 2]
                comp_s.update(find(int(v) + off[lj]) for v in np.unique(win[win > 0]))
        for gi, gr, gc in goal_set:
            if 0 <= gr < H and 0 <= gc < W and labs[gi][gr, gc] > 0:
                if find(int(labs[gi][gr, gc]) + off[gi]) in comp_s:
                    return True
        return False

    def _astar(self, net, lays, starts, goals, hw, clear, via_r, via_cost, r0, c0, r1, c1, soft=False,
               soft_pen=25.0):
        H, W = r1 - r0 + 1, c1 - c0 + 1
        q = 0.72 * self.res                         # grid quantisation margin (res / sqrt 2)
        rad = int(math.ceil((hw + clear + q) / self.res))
        vrad = int(math.ceil((via_r + clear + q) / self.res))
        hrq = int(math.ceil((hw + q) / self.res))   # half-width + quantisation, in cells
        free, vfree = [], np.ones((H, W), dtype=bool)
        softc, vsoft = [], np.zeros((H, W), dtype=bool)
        hist = getattr(self, "hist", None)
        hists = []
        # every grid layer constrains vias (they are through-holes); only the allowed layers carry tracks
        for l in self.layers:
            routable = l in lays
            o = self.owner[l][r0:r1 + 1, c0:c1 + 1]
            other = (o != 0) & (o != net)
            hard = o == -1
            own = o == net
            if soft and hasattr(self, "rid"):
                rip = (self.rid[l][r0:r1 + 1, c0:c1 + 1] != 0) & other
                fixed = other & ~rip
            else:
                rip = np.zeros_like(other)
                fixed = other
            vblocked = self.dilate(fixed | hard, vrad)
            svblk = self.dilate(rip, vrad) if rip.any() else np.zeros_like(other)
            if routable:
                # hard cells (board-edge band, grown no-net copper) already include their clearance: the track
                # centre still has to stay its half-width away from them
                hard_t = hard
                if getattr(self, "tko", None) and l in self.tko:     # track-only keep-outs (vias may pass)
                    hard_t = hard | (self.tko[l][r0:r1 + 1, c0:c1 + 1] != 0)
                blocked = self.dilate(fixed, rad) | self.dilate(hard_t, hrq)
                sblk = self.dilate(rip, rad) if rip.any() else np.zeros_like(other)
            for codes, sc, tracks_only in self.special:
                src = self.trk[l][r0:r1 + 1, c0:c1 + 1] if tracks_only else o
                sm = np.isin(src, list(codes)) & (src != net)
                if sm.any():
                    smr = sm & rip if soft else np.zeros_like(sm)
                    smf = sm & ~smr
                    vblocked |= self.dilate(smf, int(math.ceil((via_r + sc + q) / self.res)))
                    if routable:
                        blocked |= self.dilate(smf, int(math.ceil((hw + sc + q) / self.res)))
                    if smr.any():
                        svblk |= self.dilate(smr, int(math.ceil((via_r + sc + q) / self.res)))
                        if routable:
                            sblk |= self.dilate(smr, int(math.ceil((hw + sc + q) / self.res)))
            vfree &= ~vblocked
            vsoft |= svblk
            if routable:
                # own copper is free only where a track centred there stays inside it (eroded by hw + q). With the
                # bare own mask, each new connection could ride on the edge of the last one and creep half a
                # track width per hop into a neighbour's clearance zone (U24 pad 27 short, 2026-11-09)
                own_core = ~self.dilate(~own, hrq) & own
                free.append((~blocked) | own_core)
                softc.append(sblk & ~own)
                hists.append(hist[l][r0:r1 + 1, c0:c1 + 1] if hist is not None else None)
        lcost = [self.layer_cost.get(l, 1.0) for l in lays]
        vfree &= self.hole[r0:r1 + 1, c0:c1 + 1] == 0
        # vias must also clear the plane layers' foreign copper (only PTH pads there): approximated by the
        # routing layers' masks, which contain every PTH pad.
        goal_set = set()
        for gl, gx, gy in goals:
            if gl not in lays:
                continue
            gr, gc = self.to_cell(gx, gy)
            goal_set.add((lays.index(gl), gr - r0, gc - c0))
        if not goal_set:
            return None
        if _ndi is not None and not self._may_connect(lays, free, vfree, starts, goal_set, r0, c0, H, W):
            return None
        gpts = [(g[1], g[2]) for g in goal_set]

        def h(r, c):
            return ASTAR_W * min(max(abs(r - gr), abs(c - gc)) + 0.4142 * min(abs(r - gr), abs(c - gc))
                                 for gr, gc in gpts)

        openq, came, g = [], {}, {}
        for sl, sx, sy in starts:
            if sl not in lays:
                continue
            sr, sc = self.to_cell(sx, sy)
            key = (lays.index(sl), sr - r0, sc - c0)
            if not (0 <= key[1] < H and 0 <= key[2] < W):
                continue
            g[key] = 0.0
            heapq.heappush(openq, (h(key[1], key[2]), 0.0, key))
            came[key] = None
        moves = [(0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
                 (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        found = None
        limit = min(int(2_500_000 * (0.05 / self.res) ** 2), ASTAR_LIMIT)
        pops = 0
        while openq:
            f, gc_, key = heapq.heappop(openq)
            if gc_ > g.get(key, 1e18):
                continue
            pops += 1
            if pops > limit:
                break
            if key in goal_set:
                found = key
                break
            li, r, c = key
            f_l, s_l, h_l, lc = free[li], softc[li], hists[li], lcost[li]
            for dr, dc, cost in moves:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < H and 0 <= nc < W):
                    continue
                if not f_l[nr, nc]:
                    continue
                if dr and dc and not (f_l[r, nc] or f_l[nr, c]):
                    continue
                nk = (li, nr, nc)
                ng = gc_ + cost * lc
                if s_l[nr, nc]:
                    ng += soft_pen * cost
                if h_l is not None:
                    ng += float(h_l[nr, nc]) * cost
                if ng < g.get(nk, 1e18):
                    g[nk] = ng
                    came[nk] = key
                    heapq.heappush(openq, (ng + h(nr, nc), ng, nk))
            if vfree[r, c]:
                for lj in range(len(lays)):
                    if lj == li or not free[lj][r, c]:
                        continue
                    nk = (lj, r, c)
                    ng = gc_ + via_cost + (soft_pen * 4 if vsoft[r, c] else 0.0)
                    if ng < g.get(nk, 1e18):
                        g[nk] = ng
                        came[nk] = key
                        heapq.heappush(openq, (ng + h(r, c), ng, nk))
        if found is None:
            return None
        path = []
        k = found
        while k is not None:
            li, r, c = k
            x, y = self.to_xy(r + r0, c + c0)
            path.append((lays[li], x, y))
            k = came[k]
        return path[::-1]


def simplify(path):
    """Merge collinear grid steps; returns list of (layer, [(x,y)...]) polylines and via points."""
    segs, vias = [], []
    cur_l, pts = path[0][0], [(path[0][1], path[0][2])]
    for l, x, y in path[1:]:
        if l != cur_l:
            segs.append((cur_l, pts))
            vias.append((x, y))
            cur_l, pts = l, [(x, y)]
            continue
        pts.append((x, y))
    segs.append((cur_l, pts))
    out = []
    for l, p in segs:
        if len(p) < 2:
            continue
        keep = [p[0]]
        for i in range(1, len(p) - 1):
            ax, ay = keep[-1]
            bx, by = p[i]
            cx, cy = p[i + 1]
            d1 = (round((bx - ax) / 1e-3), round((by - ay) / 1e-3))
            d2 = (round((cx - bx) / 1e-3), round((cy - by) / 1e-3))
            n1 = math.hypot(*d1) or 1
            n2 = math.hypot(*d2) or 1
            if abs(d1[0] / n1 - d2[0] / n2) > 1e-6 or abs(d1[1] / n1 - d2[1] / n2) > 1e-6:
                keep.append(p[i])
        keep.append(p[-1])
        out.append((l, keep))
    return out, vias
