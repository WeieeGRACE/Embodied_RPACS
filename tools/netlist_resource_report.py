"""Per-module primitive census of the Gowin synthesized netlist.

Parses the hierarchical .vg netlist produced by GowinSynthesis, counts the
direct primitive instances (LUT4/ALU/DFF*/MUX2_LUT/INV) of every module and
rolls them up through the instantiation tree so hotspots can be ranked both
standalone and with children.
"""
import re
import sys
from collections import Counter, defaultdict

PRIM = re.compile(r"^\s{2}(LUT[1-8]|ALU|DFFCE|DFFRE|DFFPE|DFFC|DFFR|DFFP|DFF|MUXF|MUX2_LUT\d*|INV|IBUF|OBUF|IOBUF|GCLK|SDPB|ROM16)\b")
MOD = re.compile(r"^module\s+([A-Za-z_][A-Za-z0-9_]*)")
INST = re.compile(r"^\s{2}([A-Za-z_][A-Za-z0-9_]*)\s*(?:#\(.*\)\s*)?\\?\S*")

def main(path):
    modules = {}          # name -> Counter of direct primitives
    children = defaultdict(Counter)  # name -> Counter of child module names
    order = []
    current = None
    names = set()
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = MOD.match(line)
            if m:
                current = m.group(1)
                if current not in modules:
                    modules[current] = Counter()
                    order.append(current)
                names.add(current)
                continue
            if current is None:
                continue
            if line.startswith("endmodule"):
                current = None
                continue
            p = PRIM.match(line)
            if p:
                modules[current][p.group(1).rstrip("0123456789") if p.group(1).startswith("DFF") else p.group(1)] += 1
                continue
            i = INST.match(line)
            if i:
                kind = i.group(1)
                if kind not in ("input", "output", "inout", "wire", "reg", "supply", "assign"):
                    children[current][kind] += 1

    # Some instantiations reference parameterized copies (module_0 suffix); map
    # them onto their base module for rollup while keeping the copy's own census.
    def base(n):
        return n[:-2] if n not in names and n.endswith("_0") and n[:-2] in names else n

    direct_total = Counter()
    for name, c in modules.items():
        for k, v in c.items():
            direct_total[k] += v

    def rollup(name, seen=None):
        if seen is None:
            seen = set()
        if name in seen:
            return Counter()
        seen.add(name)
        total = Counter(modules.get(name, {}))
        for ch, cnt in children.get(name, {}).items():
            sub = rollup(base(ch), set(seen))
            for k, v in sub.items():
                total[k] += v * cnt
        return total

    logic = lambda c: c.get("LUT4", 0) + c.get("ALU", 0) + c.get("MUX2_LUT", 0) + c.get("INV", 0)
    ff = lambda c: c.get("DFF", 0)
    rows = []
    for name in order:
        d = modules[name]
        r = rollup(name)
        if logic(d) + logic(r) == 0:
            continue
        rows.append((logic(r), name, logic(d), ff(d), logic(r), ff(r)))

    print(f"{'module':<44}{'direct-L':>9}{'direct-FF':>10}{'total-L':>9}{'total-FF':>9}")
    for tot, name, dl, df, tl, tf in sorted(rows, reverse=True):
        print(f"{name:<44}{dl:>9}{df:>10}{tl:>9}{tf:>9}")
    print("\nDirect primitive totals:", dict(direct_total))

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "impl/gwsynthesis/Embodied_RPACS.vg")
