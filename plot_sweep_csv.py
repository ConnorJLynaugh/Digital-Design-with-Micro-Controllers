#!/usr/bin/env python3
"""
Offline plotter for 360° sweep CSV dumped over serial.

Usage:
    python plot_sweep_csv.py sweep.csv

The script tolerates extra log lines; it only consumes rows that look like
"angle_deg,distance_cm,temp_c". Distance == 0 rows are dropped by default.
"""

import argparse
import csv
import math
from pathlib import Path
from typing import List, Tuple

import matplotlib.cm as cm
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize


def parse_rows(csv_path: Path, keep_zero: bool) -> Tuple[List[float], List[float], List[float]]:
    angles: List[float] = []
    distances: List[float] = []
    temps: List[float] = []

    with csv_path.open("r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue

            # Skip comments / headers
            if row[0].startswith("#"):
                continue
            if row[0].lower().strip() == "angle_deg":
                continue

            if len(row) < 3:
                continue

            try:
                ang = float(row[0])
                dist = float(row[1])
                temp = float(row[2])
            except ValueError:
                continue

            if dist == 0.0 and not keep_zero:
                continue

            angles.append(ang)
            distances.append(dist)
            temps.append(temp)

    return angles, distances, temps


def plot_sweep(angles: List[float], distances: List[float], temps: List[float]) -> None:
    theta = [math.radians(a) for a in angles]
    x = [d * math.sin(t) for d, t in zip(distances, theta)]
    y = [d * math.cos(t) for d, t in zip(distances, theta)]

    temps_f = [(t * 9.0 / 5.0) + 32.0 for t in temps]

    vmin = min(temps_f)
    vmax = max(temps_f)
    norm = Normalize(vmin=vmin, vmax=vmax)
    cmap = cm.plasma

    fig = plt.figure(figsize=(12, 6))
    gs = fig.add_gridspec(1, 3, width_ratios=[1, 0.12, 1], wspace=0.5, left=0.06, right=0.94)
    ax_polar = fig.add_subplot(gs[0, 0], projection="polar")
    ax_cart = fig.add_subplot(gs[0, 2])
    cax = fig.add_subplot(gs[0, 1])

    sc1 = ax_polar.scatter(theta, distances, c=temps_f, cmap=cmap, norm=norm, s=36, linewidth=0.4, edgecolors="black")
    ax_polar.set_theta_zero_location("N")  # Heading 0° at the top
    ax_polar.set_theta_direction(1)        # Counterclockwise rotation (flipped to match real model)
    ax_polar.set_title("Polar sweep (angle vs distance)\ncolor = temp °F")
    ax_polar.grid(True, alpha=0.3)

    sc2 = ax_cart.scatter(x, y, c=temps_f, cmap=cmap, norm=norm, s=40, linewidth=0.4, edgecolors="black")
    ax_cart.axhline(0, color="#444", linewidth=1)
    ax_cart.axvline(0, color="#444", linewidth=1)
    ax_cart.set_aspect("equal", "box")
    ax_cart.set_xlabel("X (cm)")
    ax_cart.set_ylabel("Y (cm)")
    ax_cart.set_title("Top-down map (robot at 0,0)")
    ax_cart.grid(True, alpha=0.3, linestyle="--")

    cbar = fig.colorbar(sc1, cax=cax)
    cbar.set_label("Temp (°F)")

    plt.tight_layout(rect=[0.03, 0.03, 0.97, 0.97])
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot a 360° sweep CSV dumped from the robot.")
    parser.add_argument("csv_path", nargs="?", default="sweep.csv", help="Path to the CSV captured from serial (default: sweep.csv)")
    parser.add_argument("--keep-zero", action="store_true", help="Keep samples with distance == 0 (ignored by default)")
    args = parser.parse_args()

    csv_file = Path(args.csv_path)
    if not csv_file.exists():
        raise SystemExit(f"CSV file not found: {csv_file}")

    angles, distances, temps = parse_rows(csv_file, keep_zero=args.keep_zero)
    if not angles:
        raise SystemExit("No sweep samples found in the CSV. Check the file or run with --keep-zero.")

    print(f"Loaded {len(angles)} samples from {csv_file}")
    plot_sweep(angles, distances, temps)


if __name__ == "__main__":
    main()
