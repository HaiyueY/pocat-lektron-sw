#!/bin/bash
# sweep_freq_omega.sh — Find max acceptable initial ω for each control frequency
#
# For each DT_FLOOR (1Hz to 10Hz), binary-searches the maximum initial
# angular velocity (per axis) that converges within 12 hours.
#
# Usage: ./sweep_freq_omega.sh [build_dir]
# Output: results/freq_sweep.csv  and console table

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${1:-$SCRIPT_DIR/build}"
SWEEP_BIN="$BUILD_DIR/test_freq_sweep"
OUT_DIR="$BUILD_DIR/results"
CSV_FILE="$OUT_DIR/freq_sweep.csv"

mkdir -p "$OUT_DIR"

# Check binary exists
if [ ! -x "$SWEEP_BIN" ]; then
    echo "ERROR: $SWEEP_BIN not found. Build first: cd build && cmake .. && make test_freq_sweep"
    exit 1
fi

# --- Configuration ---
OMEGA_MIN=1       # °/s per axis (lower bound)
OMEGA_MAX=180     # °/s per axis (upper bound)
PRECISION=1       # °/s precision for binary search

# Frequencies to test (Hz) → DT_FLOOR values (s)
FREQS=(1 2 3 4 5 6 7 8 9 10)

echo "======================================================================"
echo "  Control Frequency vs Max Acceptable Initial Angular Velocity"
echo "  Convergence limit: 12 hours (43200s)"
echo "  ω range: ${OMEGA_MIN}–${OMEGA_MAX} °/s per axis, precision: ${PRECISION}°/s"
echo "======================================================================"
echo ""

# CSV header
echo "freq_hz,dt_floor_s,max_omega_deg,conv_time_s" > "$CSV_FILE"

# --- Special test: 1Hz @ 90°/s (with CSV + plots) ---
echo "--- Special test: 1Hz @ 90°/s (with full trajectory logging) ---"
SPECIAL_CSV="$OUT_DIR/detumble_1hz_90deg.csv"
RESULT=$("$SWEEP_BIN" 1.0 90 --csv "$SPECIAL_CSV" 2>&1 || true)
STATUS=$(echo "$RESULT" | awk '{print $1}')
VALUE=$(echo "$RESULT" | awk '{print $2}')
if [ "$STATUS" = "CONVERGED" ]; then
    echo "  1Hz @ 90°/s: CONVERGED in ${VALUE}s"
else
    echo "  1Hz @ 90°/s: DIVERGED (final |ω| = ${VALUE}°/s)"
fi
echo "  CSV saved: $SPECIAL_CSV"

# Generate plots for the special case
if [ -f "$SPECIAL_CSV" ]; then
    SPECIAL_PLOT_DIR="$OUT_DIR/1hz_90deg"
    echo "  Generating plots..."
    python3 "$SCRIPT_DIR/plot_results.py" detumble "$SPECIAL_CSV" "$SPECIAL_PLOT_DIR" 2>&1 || echo "  WARNING: Plot generation failed"
    echo "  Plots saved to: ${SPECIAL_PLOT_DIR}/"
fi
echo ""

# --- Main sweep ---
printf "%-8s  %-12s  %-16s  %-12s\n" "Freq" "DT_FLOOR" "Max ω (°/s)" "Conv time"
printf "%-8s  %-12s  %-16s  %-12s\n" "------" "----------" "--------------" "----------"

for FREQ in "${FREQS[@]}"; do
    DT_FLOOR=$(python3 -c "print(f'{1.0/$FREQ:.4f}')")

    # Binary search for max convergent ω
    lo=$OMEGA_MIN
    hi=$OMEGA_MAX
    best_omega=0
    best_time="N/A"

    # First check if even the minimum ω converges
    RESULT=$("$SWEEP_BIN" "$DT_FLOOR" "$OMEGA_MIN" 2>&1 || true)
    STATUS=$(echo "$RESULT" | awk '{print $1}')
    if [ "$STATUS" != "CONVERGED" ]; then
        printf "%-8s  %-12s  %-16s  %-12s\n" "${FREQ}Hz" "${DT_FLOOR}s" "<${OMEGA_MIN}" "N/A"
        echo "${FREQ},${DT_FLOOR},0,0" >> "$CSV_FILE"
        continue
    fi

    # Check if max ω converges (skip binary search if so)
    RESULT=$("$SWEEP_BIN" "$DT_FLOOR" "$OMEGA_MAX" 2>&1 || true)
    STATUS=$(echo "$RESULT" | awk '{print $1}')
    if [ "$STATUS" = "CONVERGED" ]; then
        VALUE=$(echo "$RESULT" | awk '{print $2}')
        printf "%-8s  %-12s  %-16s  %-12s\n" "${FREQ}Hz" "${DT_FLOOR}s" ">=${OMEGA_MAX}" "${VALUE}s"
        echo "${FREQ},${DT_FLOOR},${OMEGA_MAX},${VALUE}" >> "$CSV_FILE"
        continue
    fi

    # Binary search
    while [ $((hi - lo)) -gt $PRECISION ]; do
        mid=$(( (lo + hi) / 2 ))
        RESULT=$("$SWEEP_BIN" "$DT_FLOOR" "$mid" 2>&1 || true)
        STATUS=$(echo "$RESULT" | awk '{print $1}')
        VALUE=$(echo "$RESULT" | awk '{print $2}')

        if [ "$STATUS" = "CONVERGED" ]; then
            lo=$mid
            best_omega=$mid
            best_time="${VALUE}s"
        else
            hi=$mid
        fi
    done

    # Get convergence time at best_omega
    if [ "$best_omega" -gt 0 ]; then
        RESULT=$("$SWEEP_BIN" "$DT_FLOOR" "$best_omega" 2>&1 || true)
        best_time=$(echo "$RESULT" | awk '{print $2}')"s"
    fi

    printf "%-8s  %-12s  %-16s  %-12s\n" "${FREQ}Hz" "${DT_FLOOR}s" "${best_omega}" "${best_time}"
    CONV_S=$(echo "$best_time" | sed 's/s$//')
    echo "${FREQ},${DT_FLOOR},${best_omega},${CONV_S}" >> "$CSV_FILE"
done

echo ""
echo "Results saved to: $CSV_FILE"
echo "Run plot_freq_sweep.py to generate the figure."
