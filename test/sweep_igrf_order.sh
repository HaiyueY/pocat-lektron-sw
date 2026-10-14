#!/bin/bash
# sweep_igrf_order.sh — IGRF truncation-degree trade study (S3)
#
# 1. Open loop:   test_igrf_order → direction/magnitude error vs degree 13
# 2. Closed loop: test_nadir --ephem --igrf-order N for every degree in
#                 DEGREES, paired over RUNS cases (noise seed k, start
#                 k·START_STEP_H hours after the TLE epoch → different
#                 ground tracks). Truth is always degree 13.
#
# Pairing matters: case k sees the same noise and geometry for every N, so
# the per-case difference to N = 13 isolates the effect of the degree. The
# spread of that difference at N = 10–12 is the closed-loop noise floor.
#
# Usage: ./sweep_igrf_order.sh [build_dir]
# Output: results/igrf_order_open_loop.csv, results/igrf_order_closed_loop.csv
#         and a console summary

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${1:-$SCRIPT_DIR/build}"
NADIR_BIN="$BUILD_DIR/test_nadir"
ORDER_BIN="$BUILD_DIR/test_igrf_order"
OUT_DIR="$BUILD_DIR/results"
CL_CSV="$OUT_DIR/igrf_order_closed_loop.csv"

DEGREES=(1 2 3 4 5 6 7 8 10 12 13)
RUNS=10
START_STEP_H=14.4

for bin in "$NADIR_BIN" "$ORDER_BIN"; do
    if [ ! -x "$bin" ]; then
        echo "ERROR: $bin not found. Build first: cmake -S test -B test/build && cmake --build test/build"
        exit 1
    fi
done
mkdir -p "$OUT_DIR"
cd "$BUILD_DIR"

echo "=== 1. Open loop ==="
"$ORDER_BIN" --csv "$OUT_DIR/igrf_order_open_loop.csv"
echo ""

echo "=== 2. Closed loop: ${#DEGREES[@]} degrees x $RUNS cases ==="
echo "n_max,case,seed,start_h,pass,ape,mean_last_orbit,min,peak" > "$CL_CSV"
for n in "${DEGREES[@]}"; do
    for k in $(seq 1 "$RUNS"); do
        start_h=$(awk -v k="$k" -v s="$START_STEP_H" 'BEGIN { printf "%.1f", (k - 1) * s }')
        line=$(ADCS_TEST_NO_PLOT=1 "$NADIR_BIN" --ephem --igrf-order "$n" \
                   --seed "$k" --start-hours "$start_h" | grep '^SUMMARY' || true)
        if [ -z "$line" ]; then
            echo "ERROR: no SUMMARY from n=$n case=$k" >&2
            exit 1
        fi
        # SUMMARY pass=1 ape=0.9910 mean_last_orbit=3.330 min=0.110 peak=21.000
        vals=$(echo "$line" | sed -E 's/SUMMARY pass=([^ ]+) ape=([^ ]+) mean_last_orbit=([^ ]+) min=([^ ]+) peak=([^ ]+)/\1,\2,\3,\4,\5/')
        echo "$n,$k,$k,$start_h,$vals" >> "$CL_CSV"
    done
    echo "  n=$n done"
done
echo ""

python3 -I - "$CL_CSV" <<'EOF'
import csv
import math
import sys
from collections import defaultdict

rows = list(csv.DictReader(open(sys.argv[1])))
by = defaultdict(dict)
for r in rows:
    by[int(r["n_max"])][int(r["case"])] = r

ref = by[13]
cases = sorted(ref)


def stats(values):
    m = sum(values) / len(values)
    s = math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1)) if len(values) > 1 else 0.0
    return m, s


print("Closed loop, %d paired cases per degree (truth: degree 13)" % len(cases))
print(" N  pass   APE mean  APE worst  last-orbit mean  peak mean   dAPE vs N13 (pp)   dLast vs N13 (deg)")
print("--  -----  --------  ---------  ---------------  ---------  -----------------  ------------------")
for n in sorted(by):
    rs = [by[n][c] for c in cases]
    ape = [float(r["ape"]) * 100.0 for r in rs]
    last = [float(r["mean_last_orbit"]) for r in rs]
    peak = [float(r["peak"]) for r in rs]
    npass = sum(int(r["pass"]) for r in rs)
    d_ape = [float(by[n][c]["ape"]) * 100.0 - float(ref[c]["ape"]) * 100.0 for c in cases]
    d_last = [float(by[n][c]["mean_last_orbit"]) - float(ref[c]["mean_last_orbit"]) for c in cases]
    ma, sa = stats(d_ape)
    ml, sl = stats(d_last)
    print("%2d  %2d/%-2d  %7.2f%%  %8.2f%%  %13.2f°  %8.2f°   %+6.2f ± %5.2f     %+6.2f ± %5.2f"
          % (n, npass, len(rs), sum(ape) / len(ape), min(ape), sum(last) / len(last),
             sum(peak) / len(peak), ma, sa / math.sqrt(len(cases)), ml, sl / math.sqrt(len(cases))))
print("\n(± is the standard error of the paired mean difference)")
EOF
echo ""
echo "CSV: $OUT_DIR/igrf_order_open_loop.csv"
echo "CSV: $CL_CSV"
