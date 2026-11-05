#!/usr/bin/env python3
"""Plot the latest RoboMaster mission CSV."""

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main():
    files = sorted(
        (Path.home() / "robomaster_ws" / "logs").glob("mission_*.csv")
    )
    if not files:
        raise SystemExit("No mission CSV files found.")

    path = Path(sys.argv[1]) if len(sys.argv) > 1 else files[-1]

    with path.open() as f:
        rows = list(csv.DictReader(f))

    rows = [r for r in rows if r["state"] in ("CHASING", "RETURNING")]
    if not rows:
        raise SystemExit("No trajectory samples in CSV.")

    def col(name):
        return np.array([float(r[name]) for r in rows])

    t = col("time")
    t -= t[0]
    xr, yr = col("x_ref"), col("y_ref")
    x, y = col("x_actual"), col("y_actual")
    v, w = col("vx_cmd"), col("wz_cmd")

    fig, ax = plt.subplots(2, 2, figsize=(11, 7))

    ax[0, 0].plot(t, xr, label="Reference X")
    ax[0, 0].plot(t, x, label="Actual X")
    ax[0, 0].set_ylabel("Position (m)")
    ax[0, 0].legend()

    ax[0, 1].plot(xr, yr, label="Reference")
    ax[0, 1].plot(x, y, label="Actual")
    ax[0, 1].set_xlabel("Local X (m)")
    ax[0, 1].set_ylabel("Local Y (m)")
    ax[0, 1].axis("equal")
    ax[0, 1].legend()

    ax[1, 0].plot(t, v, label="Commanded Vx")
    ax[1, 0].plot(t, w, label="Commanded Wz")
    ax[1, 0].set_xlabel("Time (s)")
    ax[1, 0].legend()

    ax[1, 1].plot(t, xr-x, label="X error")
    ax[1, 1].plot(t, yr-y, label="Y error")
    ax[1, 1].set_xlabel("Time (s)")
    ax[1, 1].set_ylabel("Error (m)")
    ax[1, 1].legend()

    for a in ax.flat:
        a.grid(True)

    fig.tight_layout()
    output = path.with_suffix(".png")
    fig.savefig(output, dpi=160)
    print(f"Saved: {output}")
    plt.show()


if __name__ == "__main__":
    main()
