"""kipcb - build a KiCad 10 board from a kicad-cli netlist with pcbnew (layout-as-code).

Features: footprint instantiation with schematic links (parity-clean), nets, flip to
bottom, polygon outline from boolean shapes, zones, passive auto-placement next to an
anchor pad with courtyard collision avoidance and keep-outs, via-in-pad fan-out.
"""
import math
import os
import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kisch import parse, find, findall  # noqa: E402

FPROOT = r"C:\Users\angel\AppData\Local\Programs\KiCad\10.0\share\kicad\footprints"


OX, OY = 100.0, 100.0          # design coordinates are board-centre relative; page origin offset


def mm(v):
    return pcbnew.FromMM(float(v))


def V(x, y):
    """Design (centre-relative) mm -> board internal units."""
    return pcbnew.VECTOR2I(mm(x + OX), mm(y + OY))


def tomm(v):
    return pcbnew.ToMM(v)


def XY(vec):
    """Board internal position -> design (centre-relative) mm."""
    return tomm(vec.x) - OX, tomm(vec.y) - OY


# ----------------------------------------------------------------------------
class Netlist:
    def __init__(self, path):
        tree = parse(open(path, encoding="utf-8").read())
        self.comps = []
        for c in findall(find(tree, "components"), "comp"):
            fields = {}
            fl = find(c, "fields")
            if fl:
                for f in findall(fl, "field"):
                    name = str(find(f, "name")[1])
                    val = str(f[2]) if len(f) > 2 and not isinstance(f[2], list) else ""
                    fields[name] = val
            sp = find(c, "sheetpath")
            self.comps.append(dict(
                ref=str(find(c, "ref")[1]), value=str(find(c, "value")[1]),
                footprint=str(find(c, "footprint")[1]) if find(c, "footprint") else "",
                fields=fields, sheet=str(find(sp, "names")[1]), sheet_ts=str(find(sp, "tstamps")[1]),
                ts=str(find(c, "tstamps")[1])))
        self.nets = {}
        for n in findall(find(tree, "nets"), "net"):
            name = str(find(n, "name")[1])
            self.nets[name] = [(str(find(x, "ref")[1]), str(find(x, "pin")[1])) for x in findall(n, "node")]
        self.pin_net = {}
        for name, nodes in self.nets.items():
            for r, p in nodes:
                self.pin_net[(r, p)] = name


# ----------------------------------------------------------------------------
class Box:
    __slots__ = ("x0", "y0", "x1", "y1")

    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0, self.x1, self.y1 = min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)

    def hit(self, o, gap=0.0):
        return not (self.x1 + gap <= o.x0 or o.x1 + gap <= self.x0 or self.y1 + gap <= o.y0 or o.y1 + gap <= self.y0)

    def grow(self, g):
        return Box(self.x0 - g, self.y0 - g, self.x1 + g, self.y1 + g)


class PCB:
    def __init__(self, path, netlist, fplibs, copper_layers=6):
        self.path = path
        self.nl = netlist
        self.fplibs = fplibs
        self.board = pcbnew.NewBoard(path)
        self.board.SetCopperLayerCount(copper_layers)
        ds = self.board.GetDesignSettings()
        ds.SetAuxOrigin(V(0, 0))
        ds.SetGridOrigin(V(0, 0))
        self.fps = {}
        self.netinfo = {}
        self.side = {}
        self.keepouts = {"F": [], "B": []}
        self.outline_poly = None
        for name in self.nl.nets:
            ni = pcbnew.NETINFO_ITEM(self.board, name)
            self.board.Add(ni)
            self.netinfo[name] = ni

    # -- footprints ---------------------------------------------------------
    def _libpath(self, nick):
        return self.fplibs.get(nick) or os.path.join(FPROOT, nick + ".pretty")

    def load_all(self):
        for c in self.nl.comps:
            nick, name = c["footprint"].split(":", 1)
            fp = pcbnew.FootprintLoad(self._libpath(nick), name)
            if fp is None:
                raise RuntimeError("footprint %s not found" % c["footprint"])
            fp.SetReference(c["ref"])
            fp.SetValue(c["value"])
            fp.SetFPIDAsString(c["footprint"]) if hasattr(fp, "SetFPIDAsString") else None
            for k, v in c["fields"].items():
                if k in ("MPN", "LCSC") and v:
                    fp.SetField(k, v)
            fp.SetPath(pcbnew.KIID_PATH(c["sheet_ts"] + c["ts"]))
            for pad in fp.Pads():
                net = self.nl.pin_net.get((c["ref"], pad.GetNumber()))
                if net:
                    pad.SetNet(self.netinfo[net])
            # review-friendly: tiny reference on the Fab layer (never fabricated), value hidden
            ref_t = fp.Reference()
            ref_t.SetLayer(pcbnew.F_Fab)
            ref_t.SetTextSize(pcbnew.VECTOR2I(mm(0.3), mm(0.3)))
            ref_t.SetTextThickness(mm(0.05))
            ref_t.SetPosition(fp.GetPosition())
            ref_t.SetVisible(True)
            fp.Value().SetVisible(False)
            for item in fp.GraphicalItems():
                if isinstance(item, pcbnew.PCB_TEXT) and item.GetText() in ("${REFERENCE}", "${VALUE}"):
                    item.SetVisible(False) if hasattr(item, "SetVisible") else None
            fp.SetPosition(V(0, 0))
            self.board.Add(fp)
            self.fps[c["ref"]] = fp
            self.side[c["ref"]] = "F"

    def place(self, ref, x, y, rot=0, side="F"):
        fp = self.fps[ref]
        if self.side[ref] != side:
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
            self.side[ref] = side
        fp.SetPosition(V(x, y))
        fp.SetOrientationDegrees(rot)
        return fp

    def is_tht(self, ref):
        """True for real through-hole parts (connectors, mounts). SMD parts whose footprint
        carries plated thermal vias are NOT through-hole; their via fields are keep-outs."""
        fp = self.fps[ref]
        if fp.GetAttributes() & pcbnew.FP_SMD:
            return False
        return any(p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH) for p in fp.Pads())

    CIRCLE_FPS = ("Motor_0702_Mount",)

    def courtyard(self, ref, margin=0.1):
        """Tight keep-out box in mm: pads (copper/mask) + body outline on the Fab layer, + margin.
        (KiCad's IPC 'nominal' courtyards are ~3x too large for a board this dense.)"""
        fp = self.fps[ref]
        xs, ys = [], []
        for p in fp.Pads():
            if not p.GetNumber() and p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD and not p.IsOnCopperLayer():
                continue                                  # paste-aperture pads
            bb = p.GetBoundingBox()
            xs += [tomm(bb.GetLeft()) - OX, tomm(bb.GetRight()) - OX]
            ys += [tomm(bb.GetTop()) - OY, tomm(bb.GetBottom()) - OY]
        fab = pcbnew.F_Fab if self.side[ref] == "F" else pcbnew.B_Fab
        for item in fp.GraphicalItems():
            if item.GetLayer() == fab and not isinstance(item, pcbnew.PCB_TEXT):
                bb = item.GetBoundingBox()
                xs += [tomm(bb.GetLeft()) - OX, tomm(bb.GetRight()) - OX]
                ys += [tomm(bb.GetTop()) - OY, tomm(bb.GetBottom()) - OY]
        if not xs:
            x, y = XY(fp.GetPosition())
            return Box(x - 0.5, y - 0.5, x + 0.5, y + 0.5)
        return Box(min(xs), min(ys), max(xs), max(ys)).grow(margin)

    def circle_of(self, ref):
        fp = self.fps[ref]
        if any(k in fp.GetFPIDAsString() for k in self.CIRCLE_FPS):
            x, y = XY(fp.GetPosition())
            return (x, y, 4.75)
        return None

    def oriented_box(self, ref, margin):
        """Courtyard corners for a 2-pad part at an arbitrary angle: rectangle aligned with the pad axis that holds
        every pad polygon and Fab outline point, grown by margin. Returns 4 (x, y) design-mm points or None."""
        fp = self.fps[ref]
        pads = [p for p in fp.Pads() if p.IsOnCopperLayer()]
        if len(pads) != 2:
            return None
        (ax, ay), (bx, by) = XY(pads[0].GetPosition()), XY(pads[1].GetPosition())
        L = math.hypot(bx - ax, by - ay)
        if L < 1e-6:
            return None
        ex, ey = (bx - ax) / L, (by - ay) / L
        nx, ny = -ey, ex
        pts = []
        for p in pads:
            sps = pcbnew.SHAPE_POLY_SET()
            lay = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
            p.TransformShapeToPolygon(sps, lay, 0, mm(0.005), pcbnew.ERROR_OUTSIDE)
            for i in range(sps.OutlineCount()):
                o = sps.Outline(i)
                pts += [XY(o.CPoint(k)) for k in range(o.PointCount())]
        fab = pcbnew.F_Fab if self.side[ref] == "F" else pcbnew.B_Fab
        for item in fp.GraphicalItems():
            if item.GetLayer() == fab and not isinstance(item, pcbnew.PCB_TEXT):
                sps = pcbnew.SHAPE_POLY_SET()
                item.TransformShapeToPolygon(sps, fab, 0, mm(0.005), pcbnew.ERROR_OUTSIDE)
                for i in range(sps.OutlineCount()):
                    o = sps.Outline(i)
                    pts += [XY(o.CPoint(k)) for k in range(o.PointCount())]
        us = [(x - ax) * ex + (y - ay) * ey for x, y in pts]
        vs = [(x - ax) * nx + (y - ay) * ny for x, y in pts]
        u0, u1, v0, v1 = min(us) - margin, max(us) + margin, min(vs) - margin, max(vs) + margin
        return [(ax + ex * u + nx * v, ay + ey * u + ny * v) for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]

    def rewrite_courtyards(self, tight=(), tight_margin=0.02):
        """Replace every footprint courtyard with its tight box (rectangle), circle, or - for 2-pad parts placed at
        an angle that is not a multiple of 90 deg - a rotated rectangle. Parts in `tight` get tight_margin."""
        for ref, fp in self.fps.items():
            for item in list(fp.GraphicalItems()):
                if item.GetLayer() in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
                    fp.Remove(item)
            layer = pcbnew.F_CrtYd if self.side[ref] == "F" else pcbnew.B_CrtYd
            c = self.circle_of(ref)
            layers = (pcbnew.F_CrtYd, pcbnew.B_CrtYd) if self.is_tht(ref) else (layer,)
            margin = tight_margin if ref in tight else 0.1
            ang = fp.GetOrientationDegrees()
            odd = abs(((ang + 45.0) % 90.0) - 45.0) > 0.5
            for lay in layers:
                s = pcbnew.PCB_SHAPE(fp)
                corners = self.oriented_box(ref, margin) if (odd and not c) else None
                if c:
                    s.SetShape(pcbnew.SHAPE_T_CIRCLE)
                    s.SetCenter(V(c[0], c[1]))
                    s.SetEnd(V(c[0] + c[2], c[1]))
                elif corners:
                    s.SetShape(pcbnew.SHAPE_T_POLY)
                    s.SetPolyPoints([V(x, y) for x, y in corners])
                else:
                    b = self.courtyard(ref, margin=margin)
                    s.SetShape(pcbnew.SHAPE_T_RECT)
                    s.SetStart(V(b.x0, b.y0))
                    s.SetEnd(V(b.x1, b.y1))
                s.SetLayer(lay)
                s.SetWidth(mm(0.02))
                fp.Add(s)

    def pad_xy(self, ref, num):
        for p in self.fps[ref].Pads():
            if p.GetNumber() == num:
                return XY(p.GetPosition())
        raise KeyError("%s pad %s" % (ref, num))

    def pads_of_net(self, ref, net):
        return [p for p in self.fps[ref].Pads() if p.GetNetname() == net]

    # -- outline --------------------------------------------------------------
    @staticmethod
    def poly_from_pts(pts):
        sp = pcbnew.SHAPE_POLY_SET()
        sp.NewOutline()
        for x, y in pts:
            sp.Append(mm(x + OX), mm(y + OY))
        return sp

    @staticmethod
    def circle_pts(cx, cy, r, n=72):
        return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]

    @staticmethod
    def rect_pts(x0, y0, x1, y1):
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    @staticmethod
    def bar_pts(xa, ya, xb, yb, w):
        dx, dy = xb - xa, yb - ya
        n = math.hypot(dx, dy)
        ox, oy = -dy / n * w / 2, dx / n * w / 2
        return [(xa + ox, ya + oy), (xb + ox, yb + oy), (xb - ox, yb - oy), (xa - ox, ya - oy)]

    def set_outline(self, shapes, round_r=0.0):
        acc = None
        for pts in shapes:
            sp = self.poly_from_pts(pts)
            if acc is None:
                acc = sp
            else:
                acc.BooleanAdd(sp)
        acc.Simplify()
        if round_r:
            # close then open: rounds concave corners (fillets) and convex corners
            acc.Inflate(mm(round_r), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, mm(0.005))
            acc.Deflate(mm(2 * round_r), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, mm(0.005))
            acc.Inflate(mm(round_r), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, mm(0.005))
        shape = pcbnew.PCB_SHAPE(self.board)
        shape.SetShape(pcbnew.SHAPE_T_POLY)
        shape.SetPolyShape(acc)
        shape.SetLayer(pcbnew.Edge_Cuts)
        shape.SetWidth(mm(0.05))
        shape.SetFilled(False)
        self.board.Add(shape)
        self.outline_poly = acc
        return acc

    def inside_outline(self, box, clearance=0.3):
        if self.outline_poly is None:
            return True
        for x, y in ((box.x0, box.y0), (box.x1, box.y0), (box.x1, box.y1), (box.x0, box.y1),
                     ((box.x0 + box.x1) / 2, box.y0), ((box.x0 + box.x1) / 2, box.y1),
                     (box.x0, (box.y0 + box.y1) / 2), (box.x1, (box.y0 + box.y1) / 2)):
            if not self.outline_poly.Contains(V(x, y), -1, mm(0)):
                return False
            # distance to edge
            d = self.outline_poly.SquaredDistance(V(x, y))
            if d >= 0 and math.sqrt(max(d, 0)) / 1e6 < clearance and \
                    not self.outline_poly.Contains(V(x, y), -1, mm(0)):
                return False
        # shrink test for edge clearance
        sh = pcbnew.SHAPE_POLY_SET(self.outline_poly)
        sh.Deflate(mm(clearance), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, mm(0.01))
        for x, y in ((box.x0, box.y0), (box.x1, box.y0), (box.x1, box.y1), (box.x0, box.y1)):
            if not sh.Contains(V(x, y)):
                return False
        return True

    # -- auto placement of small parts ----------------------------------------
    def occupied(self, side, exclude=()):
        """List of obstacles on a side: Box objects or (cx, cy, r) circles."""
        obs = []
        for r, fp in self.fps.items():
            if r in exclude or getattr(self, "_unplaced", set()) and r in self._unplaced:
                continue
            if self.side[r] == side or self.is_tht(r):
                c = self.circle_of(r)
                obs.append(c if c else self.courtyard(r))
                if c:   # holes/pads of the mount still need their own boxes on both sides
                    pass
        return obs + self.keepouts[side]

    @staticmethod
    def collide(box, ob, gap):
        if isinstance(ob, Box):
            return box.hit(ob, gap)
        cx, cy, r = ob
        nx = min(max(cx, box.x0), box.x1)
        ny = min(max(cy, box.y0), box.y1)
        return math.hypot(cx - nx, cy - ny) < r + gap

    def autoplace(self, ref, anchor_xy, center_xy, side, gap=0.06, max_r=6.0, prefer_rot=None):
        """Put a small part as close as possible to anchor_xy, preferring the direction
        away from center_xy, without overlapping anything on that side."""
        ax, ay = anchor_xy
        cx, cy = center_xy
        base = math.atan2(ay - cy, ax - cx)
        occ = self.occupied(side, exclude=(ref,))
        best = None
        rots = [prefer_rot] if prefer_rot is not None else [0, 90]
        r = 0.35
        while r <= max_r and best is None:
            n = max(8, int(2 * math.pi * r / 0.15))
            cands = []
            for k in range(n):
                a = base + 2 * math.pi * k / n
                da = abs((a - base + math.pi) % (2 * math.pi) - math.pi)
                cands.append((da, ax + r * math.cos(a), ay + r * math.sin(a)))
            cands.sort()
            for da, x, y in cands:
                for rot in rots:
                    self.place(ref, x, y, rot, side)
                    bx = self.courtyard(ref)
                    if any(self.collide(bx, o, gap) for o in occ):
                        continue
                    if not self.inside_outline(bx, 0.3):
                        continue
                    best = (x, y, rot)
                    break
                if best:
                    break
            r += 0.1
        if best is None:
            self.place(ref, 40.0, 40.0 + 2.0 * len(getattr(self, "_unplaced", ())), 0, side)  # park off-board
            raise RuntimeError("no room for %s near (%.2f, %.2f)" % (ref, ax, ay))
        self.place(ref, best[0], best[1], best[2], side)
        if hasattr(self, "_unplaced"):
            self._unplaced.discard(ref)
        return best

    # -- copper ---------------------------------------------------------------
    def via(self, x, y, net, drill=0.15, size=0.25, filled=False):
        v = pcbnew.PCB_VIA(self.board)
        v.SetPosition(V(x, y))
        v.SetDrill(mm(drill))
        v.SetWidth(mm(size))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetNet(self.netinfo[net])
        if filled:
            for meth in ("SetFillMode", "SetIsFilled"):
                pass
        self.board.Add(v)
        return v

    def track(self, pts, width, layer, net):
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            t = pcbnew.PCB_TRACK(self.board)
            t.SetStart(V(x0, y0))
            t.SetEnd(V(x1, y1))
            t.SetWidth(mm(width))
            t.SetLayer(layer)
            t.SetNet(self.netinfo[net])
            self.board.Add(t)

    def zone(self, net, layers, pts_or_poly, priority=0, clearance=0.15, min_w=0.12, solid_pads=False,
             name=None):
        z = pcbnew.ZONE(self.board)
        ls = pcbnew.LSET()
        for l in layers:
            ls.AddLayer(l)
        z.SetLayerSet(ls)
        if net:
            z.SetNet(self.netinfo[net])
        if isinstance(pts_or_poly, pcbnew.SHAPE_POLY_SET):
            z.SetOutline(pcbnew.SHAPE_POLY_SET(pts_or_poly))
        else:
            z.SetOutline(self.poly_from_pts(pts_or_poly))
        z.SetAssignedPriority(priority)
        z.SetLocalClearance(mm(clearance))
        z.SetMinThickness(mm(min_w))
        z.SetThermalReliefGap(mm(0.15))
        z.SetThermalReliefSpokeWidth(mm(0.2))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if solid_pads else pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        if name:
            z.SetZoneName(name)
        self.board.Add(z)
        return z

    def fill_zones(self):
        f = pcbnew.ZONE_FILLER(self.board)
        f.Fill(self.board.Zones())

    def save(self):
        pcbnew.SaveBoard(self.path, self.board)
