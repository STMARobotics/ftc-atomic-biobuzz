"""Can a goBILDA speed servo (25-3) swing the intake arm under load? Simple torque-speed model, gravity only."""
import math
g = 9.81
m = 1.6 * 0.45359                    # kg
R_FAR = 0.15527509                   # m, furthest point from shaft
for frac in (0.5, 0.55, 0.6, 0.65):
    r = frac * R_FAR                 # centre of mass
    tau_g = m * g * r                # N*m, arm horizontal (worst case)
    print(f"CoM at {frac*100:.0f}% = {r*1000:.0f} mm: holding torque {tau_g:.3f} N*m = {tau_g/9.80665*100:.2f} kg*cm")
STALL = 9.3 * 9.80665 / 100          # N*m at 6 V
W_FREE = math.radians(60) / 0.09     # rad/s no-load at 6 V
I = 1.3 * m * (0.575 * R_FAR) ** 2   # a little extra for the arm shape
for volts in (6.0, 5.0):
    ts = STALL * volts / 6.0; wf = W_FREE * volts / 6.0
    for a0, a1 in ((0, 90), (-90, 90)):
        th, w, t, dt = math.radians(a0), 0.0, 0.0, 0.0005
        ok = True
        while th < math.radians(a1) - 1e-3 and t < 5:
            tau_load = m * g * (0.575 * R_FAR) * math.cos(th)
            tau_m = max(0, ts * (1 - w / wf))
            w += (tau_m - tau_load) / I * dt
            if w < 0: w = 0
            th += w * dt; t += dt
            if tau_m < tau_load and w == 0: ok = False; break
        print(f"{volts:.0f} V, {a0:+d} to {a1:+d} deg: " + (f"stalls, can't lift it" if not ok else f"takes {t:.2f} s"))
