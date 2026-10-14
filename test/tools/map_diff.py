#!/usr/bin/env python3
"""Attribute the size difference between two GNU ld map files to input files.

Usage:
    python3 -I test/tools/map_diff.py <base.map> <new.map> [--top N]

Sums the .text/.rodata/.data/.bss input sections placed at a non-zero
address per input file (object or archive member) in each map, prints the
per-file differences, then totals per group: ADCS objects, libm, libgcc,
libc, other. Typical use: default build vs ./build.sh --adcs-bench, to see
what the ADCS models pull in, including library code.
"""

import re
import sys
from collections import defaultdict

FULL_RE = re.compile(r"^ (\.(?:text|rodata|data|bss)\S*)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S+)")
NAME_ONLY_RE = re.compile(r"^ (\.(?:text|rodata|data|bss)\S*)\s*$")
CONT_RE = re.compile(r"^\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S+)")


def load(path):
    sizes = defaultdict(int)
    text = open(path, encoding="utf-8", errors="replace").read()
    lines = text[text.find("Linker script and memory map"):].splitlines()
    i = 0
    while i < len(lines):
        m = FULL_RE.match(lines[i])
        if m:
            sec, addr, size, src = m.groups()
        else:
            m = NAME_ONLY_RE.match(lines[i])
            c = CONT_RE.match(lines[i + 1]) if m and i + 1 < len(lines) else None
            if not c:
                i += 1
                continue
            sec = m.group(1)
            addr, size, src = c.groups()
            i += 1
        if int(addr, 16) != 0:
            sizes[(sec.split(".")[1], src)] += int(size, 16)
        i += 1
    return sizes


def short(src):
    src = re.sub(r".*/(lib[a-z_]+\.a)\(", r"\1(", src)
    return re.sub(r".*/subsystems/adcs/", "adcs/", src)


def group(src):
    if src.startswith("adcs/"):
        return "ADCS objects"
    for lib in ("libm", "libgcc", "libc"):
        if src.startswith(lib + ".a") or src.startswith(lib + "_"):
            return lib
    return "other"


def main():
    args = sys.argv[1:]
    top = 40
    if "--top" in args:
        k = args.index("--top")
        top = int(args[k + 1])
        del args[k:k + 2]
    if len(args) != 2:
        sys.exit(__doc__)
    base, new = load(args[0]), load(args[1])

    rows = []
    totals = defaultdict(int)
    for key in set(base) | set(new):
        d = new.get(key, 0) - base.get(key, 0)
        if d:
            sec, src = key[0], short(key[1])
            rows.append((d, sec, src))
            region = "RAM" if sec == "bss" else "FLASH"
            totals[(group(src), region)] += d
            if sec == "data":
                totals[(group(src), "RAM")] += d

    print("%8s  %-7s %s" % ("bytes", "section", "input"))
    for d, sec, src in sorted(rows, reverse=True)[:top]:
        print("%8d  %-7s %s" % (d, sec, src))
    print("\nTotals (FLASH = text + rodata + data, RAM = data + bss):")
    for (grp, region), d in sorted(totals.items()):
        print("  %-13s %-5s %+8d" % (grp, region, d))


if __name__ == "__main__":
    main()
