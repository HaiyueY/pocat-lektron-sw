#!/usr/bin/env python3
"""Worst-case static stack depth of a call tree, from an ARM ELF disassembly.

Usage:
    python3 -I test/tools/stack_depth.py <firmware.elf> <root_function> [...]

For every function the frame is the sum of all stack-pointer decrements in
its body (push / stmdb sp! / vpush / sub sp), which is conservative when a
function pushes on more than one path. Functions that exist in several
copies under one name (static inline helpers) take the largest copy. Edges are direct calls (bl/blx) and
tail branches (b/b.w to another function's entry, counted like calls, also
conservative). Indirect calls are listed so the result can be checked; the
depth below them is not included.

Covers library code (libm, libgcc) that has no -fstack-usage data. Needs
arm-none-eabi-objdump on PATH.
"""

import re
import subprocess
import sys
from collections import defaultdict

FUNC_RE = re.compile(r"^([0-9a-f]+) <([^>]+)>:$")
INSN_RE = re.compile(r"^\s*([0-9a-f]+):\s+(\S+)\s*(.*)$")
CALL_RE = re.compile(r"^(bl|blx)\s+[0-9a-f]+ <([^>+]+)>")
TAIL_RE = re.compile(r"^b(?:\.w|\.n)?\s+[0-9a-f]+ <([^>+]+)>")


def reg_count(reglist):
    n = 0
    for part in reglist.split(","):
        part = part.strip()
        m = re.match(r"([rds])(\d+)-[rds](\d+)$", part)
        if m:
            n += int(m.group(3)) - int(m.group(2)) + 1
        elif part:
            n += 1
    return n


def disassemble(elf):
    out = subprocess.run(["arm-none-eabi-objdump", "-d", "--no-show-raw-insn", elf],
                         check=True, capture_output=True, text=True).stdout
    frames = defaultdict(int)
    calls = defaultdict(set)
    indirect = defaultdict(int)
    func = None
    acc = 0

    def close():
        if func is not None:
            frames[func] = max(frames[func], acc)

    for line in out.splitlines():
        m = FUNC_RE.match(line)
        if m:
            close()
            func = m.group(2)
            acc = 0
            continue
        if func is None:
            continue
        m = INSN_RE.match(line)
        if not m:
            continue
        op, args = m.group(2), m.group(3)
        insn = (op + " " + args).strip()

        if op in ("push", "push.w"):
            acc += 4 * reg_count(args.strip("{} "))
        elif op in ("stmdb", "stmdb.w", "stmfd") and args.startswith("sp!"):
            acc += 4 * reg_count(args.split("{", 1)[1].rstrip("} "))
        elif op == "vpush":
            regs = args.strip("{} ")
            size = 8 if regs.startswith("d") else 4
            acc += size * reg_count(regs)
        elif op in ("sub", "sub.w", "subw") and re.match(r"sp,\s*(sp,\s*)?#", args):
            acc += int(args.split("#")[1].split()[0], 0)

        c = CALL_RE.match(insn)
        if c:
            calls[func].add(c.group(2))
            continue
        if re.match(r"^blx\s+r\d+", insn):
            indirect[func] += 1
            continue
        t = TAIL_RE.match(insn)
        if t and t.group(1) != func:
            calls[func].add(t.group(1))
    close()
    return frames, calls, indirect


def worst(fn, frames, calls, memo, stack):
    if fn in memo:
        return memo[fn]
    if fn in stack:
        raise ValueError("recursion through " + fn)
    stack.add(fn)
    best, path = 0, []
    for callee in calls.get(fn, ()):
        d, p = worst(callee, frames, calls, memo, stack)
        if d > best:
            best, path = d, p
    stack.discard(fn)
    memo[fn] = (frames.get(fn, 0) + best, [fn] + path)
    return memo[fn]


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    frames, calls, indirect = disassemble(sys.argv[1])
    memo = {}
    for root in sys.argv[2:]:
        if root not in frames:
            print("%s: not found in the ELF (garbage-collected?)" % root)
            continue
        depth, path = worst(root, frames, calls, memo, set())
        print("%s: %d bytes" % (root, depth))
        print("  " + " -> ".join("%s(%d)" % (f, frames[f]) for f in path))
        reach, todo = set(), [root]
        while todo:
            f = todo.pop()
            if f not in reach:
                reach.add(f)
                todo.extend(calls.get(f, ()))
        ind = sorted(f for f in reach if indirect.get(f))
        if ind:
            print("  indirect calls (not followed): " + ", ".join(ind))


if __name__ == "__main__":
    main()
