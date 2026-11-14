"""kisch - schematic-as-code generator for KiCad 10.

Describe parts and nets in Python; kisch emits a hierarchical KiCad schematic
(root sheet + one child sheet per block). Every connected pin gets a short wire
stub and a global label carrying its net name, unconnected pins get explicit
no-connect flags, so the result is ERC-clean, reviewable and fully editable in
the KiCad GUI.
"""
import copy
import datetime
import math
import os
import re
import uuid as _uuid

KICAD = r"C:\Users\angel\AppData\Local\Programs\KiCad\10.0"
SYMDIR = os.path.join(KICAD, "share", "kicad", "symbols")
SCH_VERSION = "20250610"
GRID = 2.54


# ----------------------------------------------------------------------------
# S-expressions
# ----------------------------------------------------------------------------
class Q(str):
    """A quoted string atom. Holds the *escaped* text exactly as in the file."""


def q(text):
    """Make a quoted atom from a plain Python string."""
    return Q(str(text).replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n"))


_TOK = re.compile(r'\s*(\(|\)|"(?:\\.|[^"\\])*"|[^\s()"]+)')


def parse(text):
    stack = [[]]
    for m in _TOK.finditer(text):
        t = m.group(1)
        if t == "(":
            stack.append([])
        elif t == ")":
            node = stack.pop()
            stack[-1].append(node)
        elif t.startswith('"'):
            stack[-1].append(Q(t[1:-1]))
        else:
            stack[-1].append(t)
    return stack[0][0]


def _atom(x):
    if isinstance(x, Q):
        return '"' + x + '"'
    if isinstance(x, float):
        return fmt(x)
    return str(x)


def dump(node, ind=0):
    if not isinstance(node, list):
        return _atom(node)
    if all(not isinstance(x, list) for x in node):
        return "(" + " ".join(_atom(x) for x in node) + ")"
    tabs = "\t" * (ind + 1)
    head = []
    i = 0
    while i < len(node) and not isinstance(node[i], list):
        head.append(_atom(node[i]))
        i += 1
    out = "(" + " ".join(head)
    for x in node[i:]:
        out += "\n" + tabs + dump(x, ind + 1)
    return out + "\n" + "\t" * ind + ")"


def fmt(v):
    s = ("%.4f" % float(v)).rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def find(node, key):
    for ch in node:
        if isinstance(ch, list) and ch and ch[0] == key:
            return ch
    return None


def findall(node, key):
    return [ch for ch in node if isinstance(ch, list) and ch and ch[0] == key]


def uid():
    return Q(str(_uuid.uuid4()))


def font(size=1.27, justify=None, hide=False):
    eff = ["effects", ["font", ["size", fmt(size), fmt(size)]]]
    if justify:
        eff.append(["justify"] + justify.split())
    if hide:
        eff.append(["hide", "yes"])
    return eff


# ----------------------------------------------------------------------------
# Symbol libraries
# ----------------------------------------------------------------------------
CUSTOM_LIBS = {}  # lib nickname -> .kicad_sym path
_lib_cache = {}


def _load_lib_file(path):
    if path not in _lib_cache:
        with open(path, encoding="utf-8") as f:
            _lib_cache[path] = parse(f.read())
    return _lib_cache[path]


def _raw_symbol(lib, name):
    if lib in CUSTOM_LIBS:
        paths = [CUSTOM_LIBS[lib]]
    else:
        paths = [os.path.join(SYMDIR, lib + ".kicad_symdir", name + ".kicad_sym"),
                 os.path.join(SYMDIR, lib + ".kicad_sym")]
    for p in paths:
        if os.path.exists(p):
            tree = _load_lib_file(p)
            for s in findall(tree, "symbol"):
                if str(s[1]) == name:
                    return s
    raise KeyError("symbol %s:%s not found" % (lib, name))


def load_symbol(lib, name):
    """Return a flattened (no 'extends') deep copy of a library symbol."""
    sym = copy.deepcopy(_raw_symbol(lib, name))
    ext = find(sym, "extends")
    if not ext:
        return sym
    parent_name = str(ext[1])
    flat = load_symbol(lib, parent_name)
    flat[1] = Q(name)
    child_props = {str(p[1]): p for p in findall(sym, "property")}
    for i, ch in enumerate(flat):
        if isinstance(ch, list) and ch and ch[0] == "property" and str(ch[1]) in child_props:
            flat[i] = child_props.pop(str(ch[1]))
    last_prop = max(i for i, ch in enumerate(flat) if isinstance(ch, list) and ch and ch[0] == "property")
    for p in child_props.values():
        last_prop += 1
        flat.insert(last_prop, p)
    for ch in flat:
        if isinstance(ch, list) and ch and ch[0] == "symbol" and str(ch[1]).startswith(parent_name + "_"):
            ch[1] = Q(name + str(ch[1])[len(parent_name):])
    return flat


def symbol_pins(sym):
    pins = []
    name = str(sym[1])
    for sub in findall(sym, "symbol"):
        m = re.match(re.escape(name) + r"_(\d+)_(\d+)$", str(sub[1]))
        unit = int(m.group(1)) if m else 0
        for p in findall(sub, "pin"):
            at = find(p, "at")
            pins.append(dict(
                unit=unit, type=p[1], x=float(at[1]), y=float(at[2]),
                a=int(float(at[3])) if len(at) > 3 else 0,
                name=str(find(p, "name")[1]), num=str(find(p, "number")[1]),
                hidden=bool(find(p, "hide")),
            ))
    return pins


def symbol_prop(sym, key):
    p = find_prop(sym, key)
    return str(p[2]) if p else ""


def find_prop(sym, key):
    for p in findall(sym, "property"):
        if str(p[1]) == key:
            return p
    return None


# ----------------------------------------------------------------------------
# Custom symbol builder
# ----------------------------------------------------------------------------
def make_symbol(name, pins, ref="U", footprint="", datasheet="", description="", min_width=15.24):
    """pins: list of (number, name, electrical_type, side) with side in L/R/T/B.

    Pins are listed top-to-bottom (L/R) or left-to-right (T/B) in the given order.
    A name of "~" leaves the slot empty (spacer) when number is None.
    """
    sides = {s: [p for p in pins if p[3] == s] for s in "LRTB"}

    def textw(lst):
        return max([len(p[1]) for p in lst if p[1] != "~"] + [0]) * 1.27

    rows = max(len(sides["L"]), len(sides["R"]), 1)
    cols = max(len(sides["T"]), len(sides["B"]), 0)
    w = max(min_width, textw(sides["L"]) + textw(sides["R"]) + 5.08, (cols + 1) * GRID)
    w = math.ceil(w / (2 * GRID)) * 2 * GRID  # half-width on grid
    hw = w / 2
    y0 = (rows // 2) * GRID
    top = y0 + GRID
    bot = y0 - (rows - 1) * GRID - GRID
    if cols:
        top = max(top, GRID)
    out_pins = []

    def pin(num, pname, etype, x, y, a):
        if num is None:
            return
        out_pins.append(["pin", etype, "line", ["at", fmt(x), fmt(y), str(a)], ["length", "2.54"],
                         ["name", q(pname), font()], ["number", q(num), font()]])

    for i, p in enumerate(sides["L"]):
        pin(p[0], p[1], p[2], -hw - GRID, y0 - i * GRID, 0)
    for i, p in enumerate(sides["R"]):
        pin(p[0], p[1], p[2], hw + GRID, y0 - i * GRID, 180)
    x0 = -((len(sides["T"]) - 1) // 2) * GRID
    for i, p in enumerate(sides["T"]):
        pin(p[0], p[1], p[2], x0 + i * GRID, top + GRID, 270)
    x0 = -((len(sides["B"]) - 1) // 2) * GRID
    for i, p in enumerate(sides["B"]):
        pin(p[0], p[1], p[2], x0 + i * GRID, bot - GRID, 90)

    def prop(k, v, x, y, hide=False, justify=None):
        return ["property", q(k), q(v), ["at", fmt(x), fmt(y), "0"], font(justify=justify, hide=hide)]

    return ["symbol", q(name),
            ["pin_names", ["offset", "1.016"]],
            ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"],
            prop("Reference", ref, -hw, top + 1.27, justify="left"),
            prop("Value", name, -hw, bot - 1.27, justify="left"),
            prop("Footprint", footprint, 0, 0, hide=True),
            prop("Datasheet", datasheet, 0, 0, hide=True),
            prop("Description", description, 0, 0, hide=True),
            ["symbol", q(name + "_0_1"),
             ["rectangle", ["start", fmt(-hw), fmt(top)], ["end", fmt(hw), fmt(bot)],
              ["stroke", ["width", "0.254"], ["type", "default"]], ["fill", ["type", "background"]]]],
            ["symbol", q(name + "_1_1")] + out_pins,
            ["embedded_fonts", "no"]]


def write_symbol_lib(path, symbols):
    lib = ["kicad_symbol_lib", ["version", "20251024"], ["generator", q("kisch")],
           ["generator_version", q("10.0")]] + symbols
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(dump(lib) + "\n")
    _lib_cache.pop(path, None)


# ----------------------------------------------------------------------------
# Design model
# ----------------------------------------------------------------------------
OUTWARD = {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}   # schematic coords (y down)
LABEL_ANGLE = {(-1, 0): 180, (1, 0): 0, (0, 1): 270, (0, -1): 90}


class Part:
    def __init__(self, ref, lib_id, nets, value=None, footprint=None, props=None, nc_ok=False, dnp=False):
        """nets: {pin number or pin name: net name}. A pin name maps every pin with that name.
        Unmapped pins get a no-connect flag (power inputs must be mapped unless nc_ok)."""
        self.ref, self.lib_id = ref, lib_id
        lib, name = lib_id.split(":", 1)
        self.sym = load_symbol(lib, name)
        self.pins = [p for p in symbol_pins(self.sym)]
        self.value = value or symbol_prop(self.sym, "Value")
        self.footprint = footprint if footprint is not None else symbol_prop(self.sym, "Footprint")
        self.props = props or {}
        self.dnp = dnp
        self.pin_net = {}
        by_num = {p["num"]: p for p in self.pins}
        for key, net in nets.items():
            key = str(key)
            if key in by_num:
                self.pin_net[key] = net
            else:
                hits = [p for p in self.pins if p["name"] == key]
                if not hits:
                    raise KeyError("%s (%s): no pin '%s'" % (ref, lib_id, key))
                for p in hits:
                    self.pin_net[p["num"]] = net
        for p in self.pins:
            if p["num"] not in self.pin_net and p["type"] == "power_in" and not nc_ok:
                raise ValueError("%s: power pin %s (%s) left unconnected" % (ref, p["num"], p["name"]))
        units = sorted({p["unit"] for p in self.pins if p["unit"] > 0}) or [1]
        self.units = units


class Sheet:
    def __init__(self, name, title=None, notes=None):
        self.name, self.title = name, title or name
        self.parts = []
        self.notes = notes or []
        self.flags = []   # net names that get a PWR_FLAG on this sheet

    def add(self, part):
        self.parts.append(part)
        return part


class Design:
    def __init__(self, project, title, rev="0.1", company=""):
        self.project, self.title, self.rev, self.company = project, title, rev, company
        self.sheets = []

    def sheet(self, name, title=None, notes=None):
        s = Sheet(name, title, notes)
        self.sheets.append(s)
        return s

    # -- checks --------------------------------------------------------------
    def nets(self):
        nets = {}
        for s in self.sheets:
            for p in s.parts:
                for num, net in p.pin_net.items():
                    nets.setdefault(net, []).append("%s.%s" % (p.ref, num))
            for n in s.flags:
                nets.setdefault(n, [])
        return nets

    def check(self):
        warns = []
        refs = {}
        for s in self.sheets:
            for p in s.parts:
                if p.ref in refs:
                    warns.append("duplicate ref %s" % p.ref)
                refs[p.ref] = 1
        for net, pins in self.nets().items():
            if len(pins) == 1:
                warns.append("net %s has a single pin (%s)" % (net, pins[0]))
        return warns

    # -- output --------------------------------------------------------------
    def write(self, outdir):
        os.makedirs(outdir, exist_ok=True)
        root_uuid = uid()
        today = datetime.date.today().isoformat()
        sheet_nodes = []
        flag_n = [0]
        for idx, sh in enumerate(self.sheets):
            sheet_uuid = uid()
            path = "/%s/%s" % (root_uuid, sheet_uuid)
            fname = "%s.kicad_sch" % sh.name
            body, paper = self._render_sheet(sh, path, flag_n)
            doc = ["kicad_sch", ["version", SCH_VERSION], ["generator", q("eeschema")],
                   ["generator_version", q("10.0")], ["uuid", uid()], ["paper", q(paper)],
                   self._title_block(sh.title, today, idx + 2)] + body + [["embedded_fonts", "no"]]
            with open(os.path.join(outdir, fname), "w", encoding="utf-8", newline="\n") as f:
                f.write(dump(doc) + "\n")
            col, row = idx % 4, idx // 4
            x, y = 25.4 + col * 63.5, 38.1 + row * 30.48
            sheet_nodes.append(["sheet", ["at", fmt(x), fmt(y)], ["size", "50.8", "15.24"],
                                ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"], ["dnp", "no"],
                                ["fields_autoplaced", "yes"],
                                ["stroke", ["width", "0.1524"], ["type", "solid"]],
                                ["fill", ["color", "0", "0", "0", "0.0000"]],
                                ["uuid", sheet_uuid],
                                ["property", q("Sheetname"), q(sh.title), ["at", fmt(x), fmt(y - 0.7), "0"],
                                 font(justify="left bottom")],
                                ["property", q("Sheetfile"), q(fname), ["at", fmt(x), fmt(y + 15.9), "0"],
                                 font(justify="left top", hide=True)],
                                ["instances", ["project", q(self.project),
                                               ["path", q("/" + root_uuid), ["page", q(str(idx + 2))]]]]])
        root = ["kicad_sch", ["version", SCH_VERSION], ["generator", q("eeschema")],
                ["generator_version", q("10.0")], ["uuid", root_uuid], ["paper", q("A4")],
                self._title_block(self.title, today, 1), ["lib_symbols"]] + sheet_nodes + [
            ["sheet_instances", ["path", q("/"), ["page", q("1")]]], ["embedded_fonts", "no"]]
        with open(os.path.join(outdir, self.project + ".kicad_sch"), "w", encoding="utf-8", newline="\n") as f:
            f.write(dump(root) + "\n")

    def _title_block(self, title, date, page):
        return ["title_block", ["title", q(title)], ["date", q(date)], ["rev", q(self.rev)],
                ["company", q(self.company)]]

    # -- sheet rendering -----------------------------------------------------
    def _render_sheet(self, sh, inst_path, flag_n):
        items, lib_syms = [], {}
        blocks = []  # (part, unit, bbox) for layout
        for part in sh.parts:
            lib_syms[part.lib_id] = part.sym
            for unit in part.units:
                pins = [p for p in part.pins if p["unit"] in (0, unit) and not p["hidden"]]
                blocks.append((part, unit, pins, self._bbox(part, pins)))
        flag_sym = None
        if sh.flags:
            flag_sym = load_symbol("power", "PWR_FLAG")
            lib_syms["power:PWR_FLAG"] = flag_sym

        # shelf packing
        page_w = 400.0
        widths = sum((b[3][2] - b[3][0]) + 12.7 for b in blocks)
        if widths > 1100:
            page_w, paper = 820.0, "A0"
        elif widths > 700:
            page_w, paper = 570.0, "A1"
        elif widths > 380:
            page_w, paper = 400.0, "A2"
        else:
            page_w, paper = 400.0, "A3"
        x, y, shelf_h = 20.32, 30.48 + 7.62 * len(sh.notes), 0.0
        for part, unit, pins, (x0, y0, x1, y1) in blocks:
            bw, bh = x1 - x0, y1 - y0
            if x + bw > page_w and x > 20.32:
                x, y, shelf_h = 20.32, y + shelf_h + 12.7, 0.0
            ox = snap(x - x0)
            oy = snap(y - y0)
            items += self._place(part, unit, pins, ox, oy, oy + y0, oy + y1, inst_path)
            x += bw + 12.7
            shelf_h = max(shelf_h, bh)
        y += shelf_h + 15.24
        for i, net in enumerate(sh.flags):
            fx, fy = snap(20.32 + i * 30.48), snap(y)
            flag_n[0] += 1
            items += self._place_flag(flag_sym, net, fx, fy, "#FLG%02d" % flag_n[0], inst_path)
        for i, note in enumerate(sh.notes):
            items.append(["text", q(note), ["exclude_from_sim", "no"], ["at", "20.32", fmt(25.4 + i * 7.62), "0"],
                          font(1.524, justify="left bottom"), ["uuid", uid()]])

        libs = ["lib_symbols"]
        for lib_id, sym in lib_syms.items():
            s = copy.deepcopy(sym)
            s[1] = q(lib_id)
            libs.append(s)
        return [libs] + items, paper

    @staticmethod
    def _bbox(part, pins):
        xs, ys = [-5.08, 5.08], [-5.08, 5.08]
        for p in pins:
            px, py = p["x"], -p["y"]
            dx, dy = OUTWARD.get(p["a"], (-1, 0))
            net = part.pin_net.get(p["num"])
            ln = GRID + (len(net) * 1.1 + 4 if net else 1)
            xs += [px, px + dx * ln]
            ys += [py, py + dy * ln]
        return min(xs), min(ys) - 3.81, max(xs), max(ys) + 3.81

    def _place(self, part, unit, pins, ox, oy, top, bot, inst_path):
        out = []
        sym = part.sym
        in_bom = find(sym, "in_bom")[1] if find(sym, "in_bom") else "yes"
        on_board = find(sym, "on_board")[1] if find(sym, "on_board") else "yes"
        props = [
            ["property", q("Reference"), q(part.ref), ["at", fmt(ox), fmt(top + 1.27), "0"], font()],
            ["property", q("Value"), q(part.value), ["at", fmt(ox), fmt(bot - 1.27), "0"], font()],
            ["property", q("Footprint"), q(part.footprint), ["at", fmt(ox), fmt(oy), "0"], font(hide=True)],
            ["property", q("Datasheet"), q(symbol_prop(sym, "Datasheet")), ["at", fmt(ox), fmt(oy), "0"], font(hide=True)],
            ["property", q("Description"), q(symbol_prop(sym, "Description")), ["at", fmt(ox), fmt(oy), "0"], font(hide=True)],
        ]
        for k, v in part.props.items():
            props.append(["property", q(k), q(v), ["at", fmt(ox), fmt(oy), "0"], font(hide=True)])
        node = ["symbol", ["lib_id", q(part.lib_id)], ["at", fmt(ox), fmt(oy), "0"], ["unit", str(unit)],
                ["exclude_from_sim", "no"], ["in_bom", in_bom], ["on_board", on_board],
                ["dnp", "yes" if part.dnp else "no"], ["uuid", uid()]] + props
        for p in pins:
            node.append(["pin", q(p["num"]), ["uuid", uid()]])
        node.append(["instances", ["project", q(self.project),
                                   ["path", q(inst_path), ["reference", q(part.ref)], ["unit", str(unit)]]]])
        out.append(node)
        seen = set()
        for p in pins:
            px, py = ox + p["x"], oy - p["y"]
            key = (round(px, 3), round(py, 3))
            if key in seen:      # stacked pins share one connection point
                continue
            seen.add(key)
            net = part.pin_net.get(p["num"])
            if net is None:
                if p["type"] != "no_connect":
                    out.append(["no_connect", ["at", fmt(px), fmt(py)], ["uuid", uid()]])
                continue
            dx, dy = OUTWARD.get(p["a"], (-1, 0))
            ex, ey = px + dx * GRID, py + dy * GRID
            out.append(_wire(px, py, ex, ey))
            out.append(_glabel(net, ex, ey, LABEL_ANGLE[(dx, dy)]))
        return out

    def _place_flag(self, sym, net, x, y, ref, inst_path):
        node = ["symbol", ["lib_id", q("power:PWR_FLAG")], ["at", fmt(x), fmt(y), "0"], ["unit", "1"],
                ["exclude_from_sim", "no"], ["in_bom", "no"], ["on_board", "no"], ["dnp", "no"], ["uuid", uid()],
                ["property", q("Reference"), q(ref), ["at", fmt(x), fmt(y - 1.9), "0"], font(hide=True)],
                ["property", q("Value"), q("PWR_FLAG"), ["at", fmt(x), fmt(y - 3.81), "0"], font()],
                ["property", q("Footprint"), q(""), ["at", fmt(x), fmt(y), "0"], font(hide=True)],
                ["property", q("Datasheet"), q(""), ["at", fmt(x), fmt(y), "0"], font(hide=True)],
                ["property", q("Description"), q(""), ["at", fmt(x), fmt(y), "0"], font(hide=True)],
                ["pin", q("1"), ["uuid", uid()]],
                ["instances", ["project", q(self.project),
                               ["path", q(inst_path), ["reference", q(ref)], ["unit", "1"]]]]]
        return [node, _wire(x, y, x, y + GRID), _glabel(net, x, y + GRID, 270)]


def snap(v, g=GRID):
    return round(v / g) * g


def _wire(x0, y0, x1, y1):
    return ["wire", ["pts", ["xy", fmt(x0), fmt(y0)], ["xy", fmt(x1), fmt(y1)]],
            ["stroke", ["width", "0"], ["type", "default"]], ["uuid", uid()]]


def _glabel(net, x, y, angle):
    just = "left" if angle in (0, 90) else "right"
    return ["global_label", q(net), ["shape", "passive"], ["at", fmt(x), fmt(y), str(angle)],
            ["fields_autoplaced", "yes"], font(justify=just), ["uuid", uid()],
            ["property", q("Intersheetrefs"), q("${INTERSHEET_REFS}"), ["at", fmt(x), fmt(y), "0"],
             font(justify=just, hide=True)]]


def write_project_files(outdir, project, sym_libs=None, fp_libs=None):
    """Minimal .kicad_pro plus project lib tables. sym_libs/fp_libs: {nickname: path rel. to project}."""
    pro = os.path.join(outdir, project + ".kicad_pro")
    if not os.path.exists(pro):
        with open(pro, "w", encoding="utf-8", newline="\n") as f:
            f.write('{\n  "meta": {\n    "filename": "%s.kicad_pro",\n    "version": 3\n  }\n}\n' % project)
    if sym_libs:
        rows = "".join('\t(lib (name "%s") (type "KiCad") (uri "${KIPRJMOD}/%s") (options "") (descr ""))\n'
                       % (n, p.replace("\\", "/")) for n, p in sym_libs.items())
        with open(os.path.join(outdir, "sym-lib-table"), "w", encoding="utf-8", newline="\n") as f:
            f.write("(sym_lib_table\n\t(version 7)\n" + rows + ")\n")
    if fp_libs:
        rows = "".join('\t(lib (name "%s") (type "KiCad") (uri "${KIPRJMOD}/%s") (options "") (descr ""))\n'
                       % (n, p.replace("\\", "/")) for n, p in fp_libs.items())
        with open(os.path.join(outdir, "fp-lib-table"), "w", encoding="utf-8", newline="\n") as f:
            f.write("(fp_lib_table\n\t(version 7)\n" + rows + ")\n")
