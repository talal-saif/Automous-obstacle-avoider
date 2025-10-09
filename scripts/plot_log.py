#!/usr/bin/env python3
import csv, sys
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "logs/aoa_log.csv"

t, dC, dL, dR = [], [], [], []
with open(path, newline="") as f:
    r = csv.DictReader(f)
    for row in r:
        t.append(float(row["sim_time"]))
        dC.append(float(row["dC_cm"]))
        dL.append(float(row["dL_cm"]))
        dR.append(float(row["dR_cm"]))

plt.figure()
plt.plot(t, dC, label="Center (cm)")
plt.plot(t, dL, label="Left (cm)")
plt.plot(t, dR, label="Right (cm)")
plt.xlabel("Sim Time (s)")
plt.ylabel("Distance (cm)")
plt.title("AOA Ray Distances")
plt.legend()
plt.grid(True)
plt.show()
