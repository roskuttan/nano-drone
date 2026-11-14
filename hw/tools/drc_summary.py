"""drc_summary.py <drc.json> [max_lines] - counts per (severity, type), unconnected items grouped by net, and the
first violations of each type with positions in design coordinates (board centre = 0,0)."""
import collections
import json
import re
import sys

r = json.load(open(sys.argv[1], encoding="utf-8"))
mx = int(sys.argv[2]) if len(sys.argv) > 2 else 6
c = collections.Counter((v["severity"], v["type"]) for v in r.get("violations", []))
for k, n in sorted(c.items(), key=lambda kv: -kv[1]):
    print("%4d %s" % (n, k))
unc = r.get("unconnected_items", [])
nets = collections.Counter()
for v in unc:
    for it in v["items"][:1]:
        m = re.search(r"\[([^\]]*)\]", it["description"])
        nets[m.group(1) if m else "?"] += 1
print("unconnected: %d %s" % (len(unc), dict(nets.most_common(40))))
print("parity:", len(r.get("schematic_parity", [])), [p["description"][:80] for p in r.get("schematic_parity", [])[:5]])
seen = collections.Counter()
for v in r.get("violations", []):
    if v["type"] in ("track_dangling", "nonmirrored_text_on_back_layer"):
        continue
    seen[v["type"]] += 1
    if seen[v["type"]] > mx:
        continue
    print("  %s: %s | %s" % (v["type"], v["description"][-48:], " / ".join(
        "%s @(%.2f,%.2f)" % (i["description"][:48], i["pos"]["x"] - 100, i["pos"]["y"] - 100) for i in v["items"])))
