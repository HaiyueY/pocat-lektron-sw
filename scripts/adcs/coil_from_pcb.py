#!/usr/bin/env python3
"""Resistance and area-turns of a PCB-etched magnetorquer coil from KiCad.

Reads the track segments and arcs of one net in a .kicad_pcb file and
reports, per copper layer and in total:

  R    = rho * sum(l / (w * t))        DC resistance at 20 degC
  N*A  = |(1/2) * closed-path integral of (x dy - y dx)|   area-turns

The copper thickness t of each layer comes from the board stackup. For
N*A, the tracks are chained end to end on each layer, and the vias of the
net join the layers, giving one open path from pad to pad. The path is
projected onto the board plane and closed with a straight line between its
two ends, which adds a negligible area because both ends sit at the
connector.
Every turn on every layer is wound in the same sense (the coil would cancel
itself otherwise), so the signed area of the chained path is N*A directly.

Usage (from the repository root):
    python3 scripts/adcs/coil_from_pcb.py \
        ../pocat-hw/pq_latboard/PQ_Lat_Board/PQ_Lat_Board.kicad_pcb /VCC_MAG

Results (pocat-hw, 2026-10-21; coil net /VCC_MAG on every board):
    PQ_Lat_Board  (X, Z faces)  R = 21.66 ohm  N*A = 0.052935 m^2
    PQ_BotBoard   (-Y face)     R = 21.81 ohm  N*A = 0.052255 m^2
    PQ_MagYBoard  (+Y face)     R = 12.49 ohm  N*A = 0.028066 m^2
"""

import math
import re
import sys
from collections import defaultdict

RHO_CU_20C = 1.72e-8      # copper resistivity at 20 degC [ohm m]
ALPHA_CU = 3.93e-3        # copper temperature coefficient [1/K]
ARC_STEPS = 16            # polyline segments per arc for the area integral
NODE_TOL_MM = 0.2         # track ends closer than this are one node [mm]
                          # (< half the 0.44 mm turn pitch; the layout has
                          # overlapping ends up to ~0.15 mm apart)


def parse_stackup(src):
    """Return {copper layer name: thickness [m]} from the board stackup."""
    thickness = {}
    for m in re.finditer(r'\(layer "([^"]+\.Cu)"\s*\(type "copper"\)\s*'
                         r'\(thickness ([\d.]+)\)', src):
        thickness[m.group(1)] = float(m.group(2)) * 1e-3
    return thickness


def point(body, key):
    m = re.search(r'\(%s ([-\d.]+) ([-\d.]+)\)' % key, body)
    return (float(m.group(1)), float(m.group(2))) if m else None


def arc_polyline(s, mid, e):
    """Polyline through an arc given by start, mid and end points."""
    ax, ay = s
    bx, by = mid
    cx, cy = e
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return [s, e]
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay)
          + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx)
          + (cx * cx + cy * cy) * (bx - ax)) / d
    r = math.hypot(ax - ux, ay - uy)
    a0 = math.atan2(ay - uy, ax - ux)
    am = math.atan2(by - uy, bx - ux)
    a1 = math.atan2(cy - uy, cx - ux)
    # Sweep from a0 to a1 in the direction that passes through am.
    sweep = (a1 - a0) % (2 * math.pi)
    if (am - a0) % (2 * math.pi) > sweep:
        sweep -= 2 * math.pi
    return [(ux + r * math.cos(a0 + sweep * k / ARC_STEPS),
             uy + r * math.sin(a0 + sweep * k / ARC_STEPS))
            for k in range(ARC_STEPS + 1)]


def parse_tracks(src, net):
    """Return a list of (layer, width [mm], polyline [mm]) for one net."""
    tracks = []
    for m in re.finditer(r'\((segment|arc)\s*\n(.*?)\n\t\)', src, re.S):
        body = m.group(2)
        if '(net "%s")' % net not in body:
            continue
        s, e = point(body, 'start'), point(body, 'end')
        width = float(re.search(r'\(width ([\d.]+)\)', body).group(1))
        layer = re.search(r'\(layer "([^"]+)"\)', body).group(1)
        poly = arc_polyline(s, point(body, 'mid'), e) if m.group(1) == 'arc' else [s, e]
        tracks.append((layer, width, poly))
    return tracks


def parse_vias(src, net):
    """Return (xy, pad radius) [mm] of every via on one net."""
    vias = []
    for m in re.finditer(r'\(via\s*\n(.*?)\n\t\)', src, re.S):
        body = m.group(1)
        if '(net "%s")' % net in body:
            size = float(re.search(r'\(size ([\d.]+)\)', body).group(1))
            vias.append((point(body, 'at'), size / 2.0))
    return vias


def polyline_length(poly):
    return sum(math.dist(poly[i], poly[i + 1]) for i in range(len(poly) - 1))


def chain(tracks, vias):
    """Chain the polylines into one path; return (path, unchained length [mm]).

    A node is a cluster of track ends on one layer closer than NODE_TOL_MM
    (the layout has small gaps, overlapping ends and duplicated stubs); a
    track end inside a via pad becomes that via's node, which joins the
    layers. The graph is then walked as an Euler path (Hierholzer) from one
    of its two odd nodes, the pads.
    """
    nodes = []                      # (layer or None for a via, x, y)
    for v in vias:
        nodes.append((None,) + tuple(v[0]))

    def node_of(layer, p):
        for k, v in enumerate(vias):
            if math.dist(p, v[0]) <= v[1]:
                return k
        for k, (lay, x, y) in enumerate(nodes):
            if lay == layer and math.dist(p, (x, y)) <= NODE_TOL_MM:
                return k
        nodes.append((layer, p[0], p[1]))
        return len(nodes) - 1

    edges = []                      # (node a, node b, polyline a -> b)
    adj = defaultdict(list)
    for layer, _, poly in tracks:
        a, b = node_of(layer, poly[0]), node_of(layer, poly[-1])
        if a == b and polyline_length(poly) < NODE_TOL_MM:
            continue                # a stub shorter than the clustering
        adj[a].append(len(edges))
        adj[b].append(len(edges))
        edges.append((a, b, poly))

    odd = [n for n in adj if len(adj[n]) % 2]
    start = odd[0] if odd else edges[0][0]
    used = [False] * len(edges)
    stack, trail = [(start, None)], []
    while stack:
        node, via_edge = stack[-1]
        while adj[node] and used[adj[node][-1]]:
            adj[node].pop()
        if adj[node]:
            e = adj[node].pop()
            used[e] = True
            a, b, _ = edges[e]
            stack.append((b if a == node else a, e))
        else:
            stack.pop()
            if via_edge is not None:
                trail.append((node, via_edge))

    path = []
    for node, e in reversed(trail):
        a, b, poly = edges[e]
        poly = poly if b == node else poly[::-1]
        path.extend(poly if not path else poly[1:])
    unchained = sum(polyline_length(edges[e][2]) for e in range(len(edges)) if not used[e])
    return path, unchained


def signed_area(path):
    """Signed area of the path closed by a chord [mm^2]."""
    closed = path + [path[0]]
    return 0.5 * sum(closed[i][0] * closed[i + 1][1] - closed[i + 1][0] * closed[i][1]
                     for i in range(len(closed) - 1))


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src = open(sys.argv[1]).read()
    net = sys.argv[2]
    thickness = parse_stackup(src)
    tracks = parse_tracks(src, net)
    if not tracks:
        sys.exit('no tracks on net %s' % net)

    per_layer = defaultdict(lambda: [0, 0.0, 0.0, set()])
    for layer, width, poly in tracks:
        length = polyline_length(poly)
        t = thickness.get(layer, 35e-6)
        rec = per_layer[layer]
        rec[0] += 1
        rec[1] += length
        rec[2] += RHO_CU_20C * (length * 1e-3) / (width * 1e-3 * t)
        rec[3].add(width)

    print('%s  net %s' % (sys.argv[1], net))
    for layer in sorted(per_layer):
        n, length, r, widths = per_layer[layer]
        print('  %-7s %4d tracks  L = %6.3f m  w = %s mm  t = %.0f um  R = %6.2f ohm'
              % (layer, n, length / 1e3, sorted(widths),
                 thickness.get(layer, 35e-6) * 1e6, r))
    r_total = sum(v[2] for v in per_layer.values())
    path, unchained = chain(tracks, parse_vias(src, net))
    na = abs(signed_area(path)) * 1e-6
    print('  R(20 degC) = %.2f ohm   (%.3f ohm/K)' % (r_total, r_total * ALPHA_CU))
    print('  N*A        = %.6f m^2   (path closure %.2f mm, %.1f mm of track not chained)'
          % (na, math.dist(path[0], path[-1]), unchained))


if __name__ == '__main__':
    main()
