"""phase_check.py <board> - check the motor-current path of every phase net.

For each DRV8311 OUT pin -> motor pad, it follows the copper of that net from the OUT pad to the motor PTH pad. The
path is a breadth-first search over the net's tracks and vias whose ends touch. The script reports the layers the
current path uses and its narrowest track. Phase current is up to about 4 A peak and 1.5 A RMS at hover. Inner layers
are 15 um copper, so the power leg must stay on the outer 35 um layers. The finish stage widens it toward 0.5 mm.
"""
import collections
import sys

import pcbnew

b = pcbnew.LoadBoard(sys.argv[1])
OUTER = {pcbnew.F_Cu, pcbnew.B_Cu}
bad = 0
for m in (1, 2, 3, 4):
    drv = b.FindFootprintByReference("U%d" % (10 + m))
    mot = b.FindFootprintByReference("J%d" % (10 + m))
    for pin, pad in (("10", "1"), ("11", "2"), ("12", "3")):
        sp = [p for p in drv.Pads() if p.GetNumber() == pin][0]
        tp = [p for p in mot.Pads() if p.GetNumber() == pad][0]
        net = sp.GetNetname()
        items = [t for t in b.GetTracks() if t.GetNetname() == net]
        # graph: items touching each other (shape collision on a shared layer), plus the two pads
        nodes = [sp, tp] + items

        def touch(a, c):
            for lay in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.In3_Cu, pcbnew.In4_Cu, pcbnew.B_Cu):
                if a.IsOnLayer(lay) and c.IsOnLayer(lay) and \
                        a.GetEffectiveShape(lay).Collide(c.GetEffectiveShape(lay), 0):
                    return True
            return False
        adj = collections.defaultdict(list)
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                if touch(nodes[i], nodes[j]):
                    adj[i].append(j)
                    adj[j].append(i)
        prev, q = {0: None}, collections.deque([0])
        while q:
            i = q.popleft()
            for j in adj[i]:
                if j not in prev:
                    prev[j] = i
                    q.append(j)
        if 1 not in prev:
            print("%-8s U%d.%s -> J%d.%s: NOT CONNECTED" % (net, 10 + m, pin, 10 + m, pad))
            bad += 1
            continue
        path, k = [], 1
        while k is not None:
            path.append(nodes[k])
            k = prev[k]
        trk = [t for t in path if isinstance(t, pcbnew.PCB_TRACK) and not isinstance(t, pcbnew.PCB_VIA)]
        lays = sorted({b.GetLayerName(t.GetLayer()) for t in trk})
        inner = [t for t in trk if t.GetLayer() not in OUTER]
        wmin = min((t.GetWidth() / 1e6 for t in trk), default=0)
        nvia = sum(1 for t in path if isinstance(t, pcbnew.PCB_VIA))
        flag = "  <-- INNER LAYER" if inner else ""
        bad += bool(inner)
        print("%-8s U%d.%s -> J%d.%s: layers %s, min width %.2f mm, %d vias%s" % (
            net, 10 + m, pin, 10 + m, pad, lays, wmin, nvia, flag))
print("phase power paths on inner layers / unconnected: %d" % bad)
