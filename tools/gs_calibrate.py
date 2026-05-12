#!/usr/bin/env python3
"""
gs_calibrate.py — PiCar-X Grayscale Sensor Calibration Tool

Run this directly on the Pi via SSH:
    python3 gs_calibrate.py

No FastAPI agent needed. Just the PiCar-X library.

What to do:
  1. Place car on bare floor — note baseline values (should be HIGH)
  2. Slide left sensor over blue tape — note the drop
  3. Slide center sensor over blue tape — note the drop
  4. Slide right sensor over blue tape — note the drop
  5. Press Ctrl+C when done

Record your findings — you'll need them to set thresholds in the navigator.
"""

import time
import sys

try:
    from picarx import Picarx
except ImportError:
    print("ERROR: picarx library not found. Run this on the Pi.")
    sys.exit(1)

# ── Configuration ──────────────────────────────────────────────────────────────
SAMPLE_RATE_HZ  = 10       # readings per second
DARK_THRESHOLD  = 500      # values BELOW this are considered "dark" (tape)
                           # adjust after seeing your baseline readings

# ── Init ───────────────────────────────────────────────────────────────────────
print("Initializing PiCar-X...")
px = Picarx()
time.sleep(0.5)
print("Grayscale sensor ready.\n")

print("=" * 55)
print("  LEFT    CENTER    RIGHT    |  INDICATORS")
print("=" * 55)

# ── Track min/max seen per sensor for summary ──────────────────────────────────
mins = [9999, 9999, 9999]
maxs = [0, 0, 0]

def indicator(val):
    """Simple visual indicator — X means dark (tape detected)."""
    return " X " if val < DARK_THRESHOLD else " . "

try:
    while True:
        gs = px.grayscale.read()

        if not gs or len(gs) < 3:
            print("  -- sensor read failed --")
            time.sleep(1 / SAMPLE_RATE_HZ)
            continue

        left, center, right = gs[0], gs[1], gs[2]

        # Track range
        for i, v in enumerate([left, center, right]):
            mins[i] = min(mins[i], v)
            maxs[i] = max(maxs[i], v)

        # Indicators
        ind_l = indicator(left)
        ind_c = indicator(center)
        ind_r = indicator(right)

        print(
            f"  {left:4d}    {center:4d}      {right:4d}    |"
            f"  L={ind_l} C={ind_c} R={ind_r}",
            end="\r"
        )

        time.sleep(1 / SAMPLE_RATE_HZ)

except KeyboardInterrupt:
    print("\n\n" + "=" * 55)
    print("  CALIBRATION SUMMARY")
    print("=" * 55)
    print(f"  {'Sensor':<10} {'Min seen':>10} {'Max seen':>10}")
    print(f"  {'-'*30}")
    for name, mn, mx in zip(["LEFT", "CENTER", "RIGHT"], mins, maxs):
        print(f"  {name:<10} {mn:>10} {mx:>10}")
    print("=" * 55)
    print("\nSuggested DARK_THRESHOLD: midpoint between your tape")
    print("min and floor min values.")
    for name, mn, mx in zip(["LEFT", "CENTER", "RIGHT"], mins, maxs):
        midpoint = (mn + mx) // 2
        print(f"  {name}: {midpoint}")
    print("\nRecord these values before starting the navigator build.")
    print("Done.")