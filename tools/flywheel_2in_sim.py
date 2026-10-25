"""2 in vs 3 in flywheel on two 6000 rpm goBILDA motors: can it reach and hold the shot speeds?"""
import math
IN = 0.0254
KS = 0.45                         # ball speed = KS * rim speed
STALL = 2 * 1.47 * 9.80665 / 100  # N*m, two motors
EFF = 0.9
M_BALL = 0.025
WHEEL_KG = 0.45359                # 1 lb flywheel assembly

def run(dia_in, gear, volts, label):
    r = dia_in / 2 * IN
    I = 0.5 * WHEEL_KG * r * r
    wfree_m = 6000 * 2 * math.pi / 60 * volts / 12      # motor rad/s at this voltage
    k = KS * r                                           # ball m/s per wheel rad/s
    def alpha(ww):                                       # wheel accel
        wm = ww / gear
        t = max(0.0, STALL * (1 - wm / wfree_m) * EFF)
        return t / gear / I                              # geared UP: wheel torque = motor torque / gear
    def time(w0, w1, dt=0.0005):
        w, t = w0, 0.0
        while w < w1 and t < 10:
            a = alpha(w)
            if a <= 1e-9: return None
            w += a * dt; t += dt
        return t if t < 10 else None
    print(f"\n{label}: {dia_in} in wheel, {gear}:1 up, {volts:.1f} V  (wheel max {wfree_m*gear*60/2/math.pi:.0f} rpm)")
    print("  ball m/s  wheel rpm  motor %free  spin-up from 0   recover after shot   +500rpm(3in-eq) step")
    for v in (5.55, 6.03, 6.70, 8.62):
        w = v / k
        wm = w / gear
        if wm >= wfree_m * 0.995:
            print(f"  {v:5.2f}     {w*60/2/math.pi:6.0f}     can't reach"); continue
        t0 = time(0, w)
        # dip: ball takes ~2x its kinetic energy out of the wheel
        e = 0.5 * I * w * w - 2 * 0.5 * M_BALL * v * v
        w1 = math.sqrt(2 * e / I); tr = time(w1, w)
        wup = (v + 500 * KS * math.pi * 3 * IN / 60) / k      # +500 rpm of a 3in wheel
        tup = time(w, wup) if wup / gear < wfree_m * 0.995 else None
        f = lambda x: "never" if x is None else f"{x:5.2f} s"
        print(f"  {v:5.2f}     {w*60/2/math.pi:6.0f}     {wm/wfree_m*100:5.0f}%     {f(t0):>9}        {f(tr):>9}           {f(tup):>9}")
for volts in (12.0, 11.0):
    run(3, 1.0, volts, "A")
    run(2, 1.0, volts, "B direct")
    run(2, 1.5, volts, "C gear-up")
