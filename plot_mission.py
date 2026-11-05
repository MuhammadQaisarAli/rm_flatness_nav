#!/usr/bin/env python3
"""Plot the most recent mission CSV (CHASING only)."""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

logs = Path.home() / "robomaster_ws" / "logs"
files = sorted(logs.glob("mission_*.csv"))
if len(sys.argv) > 1:
    path = Path(sys.argv[1])
elif files:
    path = files[-1]
else:
    raise SystemExit("No mission CSV found; pass the CSV path as an argument")
with path.open(newline="") as f:
    rows = [r for r in csv.DictReader(f) if r["state"] == "CHASING"]
if len(rows) < 2:
    raise SystemExit("No CHASING samples found")

def col(key):
    return np.array([float(r[key]) if r[key] else np.nan for r in rows])

t = col("time")
t -= t[0]
xr, yr, x, y = (col(k) for k in ("x_ref", "y_ref", "x_actual", "y_actual"))
v, w, bearing = (col(k) for k in ("vx_cmd", "wz_cmd", "bearing_deg"))
# Cubic reference speed: derivative of the reference position profile.
T = float(sys.argv[2]) if len(sys.argv) > 2 else 13.0
D = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
vref = 6 * D * t * (T - t) / T**3
out = path.parent / "plots"
out.mkdir(exist_ok=True)
fig, axs = plt.subplots(3, 2, figsize=(12, 12))
a = axs.flat
for arr, label in ((xr,"Reference"),(x,"Actual")): a[0].plot(t,arr,label=label)
a[0].set(title="X position",xlabel="Time (s)",ylabel="m")
a[1].plot(xr,yr,label="Reference"); a[1].plot(x,y,label="Actual")
a[1].set(title="XY path",xlabel="X (m)",ylabel="Y (m)")
a[1].axis("equal")
a[2].plot(t,vref,label="Cubic reference"); a[2].plot(t,v,label="Commanded")
a[2].set(title="Forward velocity",xlabel="Time (s)",ylabel="m/s")
a[3].plot(t,xr-x,label="X error"); a[3].plot(t,yr-y,label="Y error")
a[3].set(title="Tracking error",xlabel="Time (s)",ylabel="m")
a[4].plot(t,bearing,label="Bearing")
a[4].set(title="AprilTag bearing",xlabel="Time (s)",ylabel="degrees")
a[5].plot(t,w,label="Commanded angular velocity")
a[5].set(title="Angular velocity",xlabel="Time (s)",ylabel="rad/s")
for ax in a:
    ax.grid(True,alpha=0.3)
    ax.legend()
fig.tight_layout()
fig.savefig(out / f"{path.stem}_summary.png",dpi=160)
fig.savefig(out / f"{path.stem}_summary.pdf")
print("Saved plots in",out)
plt.show()
