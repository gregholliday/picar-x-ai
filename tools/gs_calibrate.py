#!/usr/bin/env python3
"""
gs_calibrate.py — PiCar-X Grayscale Sensor Calibration Tool (v2)

Run this directly on the Pi via SSH:
    python3 gs_calibrate.py

No FastAPI agent needed. Just the PiCar-X library.

What to do:
  1. Hold the car still over the surface you want to measure
  2. Wait for "RECORDING STARTED" to appear (warmup complete)
  3. Hold still for a few more seconds
  4. Press Ctrl+C to see the summary

Changes from v1:
  - Warmup period discards first 20 readings to eliminate init spikes
  - Min/max tracking only starts after warmup is complete
  - Live output clearly shows when recording has started
"""

import time
import sys

try:
    from picarx import Picarx
except ImportError:
    print("ERROR: picarx library not found. Run this on the Pi.")
    sys.exit(1)

# ── Configuration ──────────────────────────────────────────────────────────────
SAMPLE_RATE_HZ = 10     # readings per second
WARMUP_SAMPLES = 20     # discard first N readings (2 seconds at 10Hz)

# ── Init ───────────────────────────────────────────────────────────────────────
print("Initializing PiCar-X...")
px = Picarx()
time.sleep(1.0)         # extra settle time before first read
print("Grayscale sensor ready.\n")
print(f"Warming up — discarding first {WARMUP_SAMPLES} readings...")
print("Hold the car still over your target surface.\n")

# ── Tracking ───────────────────────────────────────────────────────────────────
mins         = [9999, 9999, 9999]
maxs         = [0,    0,    0   ]
sample_count = 0
recording    = False

try:
    while True:
        gs = px.grayscale.read()

        if not gs or len(gs) < 3:
            print("  -- sensor read failed --")
            time.sleep(1 / SAMPLE_RATE_HZ)
            continue

        left, center, right = gs[0], gs[1], gs[2]
        sample_count += 1

        # ── Warmup phase — show live readings but don't record ─────────────────
        if sample_count <= WARMUP_SAMPLES:
            remaining = WARMUP_SAMPLES - sample_count
            print(
                f"  WARMUP [{remaining:2d} left]  "
                f"L={left:4d}  C={center:4d}  R={right:4d}",
                end="\r"
            )
            time.sleep(1 / SAMPLE_RATE_HZ)
            continue

        # ── First sample after warmup ──────────────────────────────────────────
        if not recording:
            recording = True
            print("\n\n*** RECORDING STARTED — hold still, press Ctrl+C when done ***\n")
            print("=" * 55)
            print("  LEFT    CENTER    RIGHT")
            print("=" * 55)

        # ── Track min/max ──────────────────────────────────────────────────────
        for i, v in enumerate([left, center, right]):
            mins[i] = min(mins[i], v)
            maxs[i] = max(maxs[i], v)

        print(f"  {left:4d}    {center:4d}      {right:4d}", end="\r")

        time.sleep(1 / SAMPLE_RATE_HZ)

except KeyboardInterrupt:
    print("\n")

    if not recording:
        print("Stopped during warmup — no data recorded.")
        print("Run again and wait for RECORDING STARTED message.")
        sys.exit(0)

    recorded = sample_count - WARMUP_SAMPLES
    print("=" * 55)
    print("  CALIBRATION SUMMARY")
    print("=" * 55)
    print(f"  Samples recorded: {recorded}")
    print(f"  {'Sensor':<10} {'Min':>8} {'Max':>8} {'Midpoint':>10}")
    print(f"  {'-'*38}")
    for name, mn, mx in zip(["LEFT", "CENTER", "RIGHT"], mins, maxs):
        mid = (mn + mx) // 2
        print(f"  {name:<10} {mn:>8} {mx:>8} {mid:>10}")
    print("=" * 55)
    print("\nMidpoint = suggested starting threshold for this surface.")
    print("Done.")