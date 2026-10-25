"""
Shoot-on-the-move (SOTM) Monte Carlo simulator.

Answers: if the robot drives at speed V past the hive and fires, what fraction of balls go into the cell, and
which part of the system (localization, latency, flywheel, turret, shooter spread) is holding the number back?

Each trial:
  1. The robot has been driving in a straight line at constant velocity. A controller (the same algorithm we
     plan to write: predict the pose forward, iterate on flight time, aim at the shifted target) runs every
     LOOP_DT using delayed, noisy pose and velocity.
  2. The flywheel, turret and hood chase the commands with realistic response limits.
  3. At t = FIRE_TIME the robot fires. The ball leaves RELEASE_DELAY_TRUE seconds later (with jitter), using
     whatever the flywheel, turret and hood are actually doing at that instant, plus shooter spread.
  4. The ball flies in 3D (gravity + air drag, with the robot's own velocity added) and either goes through the
     tilted cell opening or misses.

Run:  python tools/sotm_sim.py              (hit rate vs driving speed, then what limits it at 24 in/s)
      python tools/sotm_sim.py --speed=30   (change the speed used for the "what limits it" table)
      python tools/sotm_sim.py --n=1000     (more trials per row, slower and smoother)

The shot physics (cell geometry, drag, wheel speed) comes from tools/hood_sim.py. Every number in PARAMS is
either from CAD / the manual / datasheets or is marked as a guess. Pure Python, no packages needed.
"""
import importlib.util
import math
import os
import random
import sys
from bisect import bisect_left

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("hood_sim", os.path.join(HERE, "hood_sim.py"))
hs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(hs)

IN = 0.0254
G = 9.81
K = hs.SLIP_FACTOR * math.pi * hs.WHEEL_DIAMETER_IN * IN / 60.0     # ball speed (m/s) per wheel rpm

# ----------------------------------------------------------------------------- PARAMS
PARAMS = {
    # --- the drive ---
    "robot_speed_in_s": 24.0,        # 2 ft/s
    # --- measurement and control timing ---
    "loop_dt": 0.02,                 # controller period. 50 Hz is a realistic FTC loop
    "sensor_latency": 0.02,          # age of the pose/velocity we read (OTOS read + loop)  GUESS
    "command_latency": 0.02,         # delay before a command reaches the motors/servos     GUESS
    "release_delay_true": 0.15,      # fire decision -> ball actually leaves the wheel      GUESS
    "release_jitter": 0.015,         # +/- seconds of variation in that delay (1 sigma)     GUESS
    # How far ahead of the latency-corrected pose the controller aims. The turret keeps tracking continuously, so
    # this is only the command-to-motion delay (about command_latency plus a little actuator lag). It is NOT the
    # feed delay: a shot that leaves later is aimed by the turret's NEXT command, which is already tracking.
    "aim_lookahead": 0.04,
    # --- measurement quality ---
    "pos_noise_in": 0.2,             # white noise on each pose reading
    "pos_bias_in": 1.0,              # constant localization error for a shot (OTOS drift) GUESS
    "vel_noise_in_s": 1.5,           # noise on each velocity reading                       GUESS
    "vel_filter_tau": 0.1,           # low-pass time constant on the velocity reading, seconds
    # --- shooter spread ---
    "ball_speed_noise": 0.015,       # 1 sigma, fraction of ball speed                      GUESS
    "exit_dir_noise_deg": 0.5,       # 1 sigma, elevation and yaw                           GUESS
    # --- flywheel (2x goBILDA 6000 rpm, 6000->4000 gearing, 1 lb) ---
    "fly_alpha_stall": 11288.0,      # rpm/s of acceleration at zero speed. Rim-heavy wheel is about half
    "fly_alpha_down": 400.0,         # rpm/s of slowing down when coasting                  GUESS
    "fly_kp": 20.0,                  # how hard the flywheel controller chases its target (1/s)
    "fly_free_rpm": 4000.0,
    # --- turret and hood ---
    "turret_kp": 15.0,               # 1/s
    "turret_max_dps": 360.0,         # degrees per second                                   GUESS
    "turret_ff": True,               # feed-forward from the commanded yaw rate
    "hood_kp": 20.0,
    "hood_max_dps": 300.0,           # Axon servo                                           GUESS
    "fly_instant": False, "turret_instant": False, "hood_instant": False,   # idealized: follow commands exactly
    # --- the trial itself ---
    "fire_time": 1.0,                # seconds of driving/tracking before the shot
    "gate_rpm": 100.0, "gate_deg": 1.0,   # "ready to feed" tolerances, only used for the ready rate
}
IDEAL = dict(fly_instant=True, turret_instant=True, hood_instant=True)
# -----------------------------------------------------------------------------

TILT = math.radians(hs.OPENING_TILT_DEG)
TARGET = (72 - 12.75, 72 + 16.0)              # red hive, up cell at the south end: opening centre (x, z) in inches
TX, TZ = TARGET[0] * IN, TARGET[1] * IN
E = 1.0                                       # opening faces south (+z)
CY = hs.OPENING_CENTER_IN * IN
N_IN = (0.0, -math.sin(TILT), -E * math.cos(TILT))          # into the cell
T_VERT = (0.0, math.cos(TILT), -E * math.sin(TILT))         # along the opening's tilted height
TURRET_OFFSET = (0.0, 62.1 / 1000)                          # turret centre is 62.1 mm behind the robot centre (robot faces north)
FWD = hs.TURRET_TO_EXIT_FORWARD_IN * IN
EXIT_H = hs.EXIT_HEIGHT_IN * IN


# ----------------------------------------------------------------------------- shot table (what a real robot would measure)
def _solve_fast(d_in, speed):
    cx = hs.exit_distance(d_in)
    prev = None
    a = 5.0
    while a <= 85.0:
        hit = hs.cross_opening(a, speed, cx, dt=0.002)
        if hit is not None and prev is not None:
            pa, ph = prev
            if (ph["offset"] > 0) != (hit["offset"] > 0) and hit["path"] <= hs.ENTRY_ANGLE_MAX_DEG:
                f = ph["offset"] / (ph["offset"] - hit["offset"])
                ang = pa + f * (a - pa)
                h2 = hs.cross_opening(ang, speed, cx, dt=0.002)
                return ang, (h2["time"] if h2 else hit["time"])
        prev = (a, hit) if hit is not None else None
        a += 0.5
    return None


def speed_schedule(d_in):
    """Ball speed chosen for each distance: the middle of the feasible speeds found with hood_sim.py."""
    return min(9.0, max(5.0, 5.4 + 0.035 * (d_in - 40.0)))


class Table:
    def __init__(self):
        self.d, self.hood, self.tof, self.v = [], [], [], []
        d = 24.0
        while d <= 138.0:
            v = speed_schedule(d)
            r = _solve_fast(d, v)
            if r is not None:
                self.d.append(d); self.hood.append(r[0]); self.tof.append(r[1]); self.v.append(v)
            d += 2.0

    def lookup(self, d):
        d = min(max(d, self.d[0]), self.d[-1])
        i = max(1, bisect_left(self.d, d))
        d0, d1 = self.d[i - 1], self.d[i]
        f = (d - d0) / (d1 - d0)
        mix = lambda arr: arr[i - 1] + f * (arr[i] - arr[i - 1])
        return mix(self.hood), mix(self.tof), mix(self.v)


# ----------------------------------------------------------------------------- the controller
def controller(table, turret_pos_est, vel_est, delay):
    """Predict forward, then iterate on flight time. Returns (yaw rad, wheel rpm, hood deg)."""
    px = turret_pos_est[0] + vel_est[0] * delay
    pz = turret_pos_est[1] + vel_est[1] * delay
    tof = 0.35
    yaw = 0.0
    hood = v = 0.0
    for _ in range(3):
        vx, vz = TX - vel_est[0] * tof, TZ - vel_est[1] * tof
        dx, dz = vx - px, vz - pz
        hood, tof, v = table.lookup(math.hypot(dx, dz) / IN)
        yaw = math.atan2(dx, dz)
    return yaw, v / K, hood


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ----------------------------------------------------------------------------- one trial
def trial(table, P, rng, sampler=None):
    # all random numbers are drawn in a fixed order so runs can be compared fairly
    while True:                          # a random firing spot inside the field, in the cell's firing cone
        if sampler is not None:
            fx, fz = sampler(rng)
            break
        ang = math.radians(rng.uniform(-45, 45))
        dist = rng.uniform(45, 95)
        fx = TARGET[0] + dist * math.sin(ang)
        fz = TARGET[1] + dist * math.cos(ang)
        if 9 <= fx <= 135 and fz <= 135:
            break
    ok_pos = True
    heading = rng.uniform(0, 2 * math.pi)
    g = [rng.gauss(0, 1) for _ in range(64)]
    gi = iter(g)
    vr = P["robot_speed_in_s"] * IN
    vel = (vr * math.cos(heading), vr * math.sin(heading))
    fire_pos = (fx * IN, fz * IN)
    bias = (next(gi) * P["pos_bias_in"] * IN, next(gi) * P["pos_bias_in"] * IN)
    t_fire = P["fire_time"]
    t_rel = t_fire + max(0.03, P["release_delay_true"] + next(gi) * P["release_jitter"])

    def true_pos(t):
        return (fire_pos[0] + vel[0] * (t - t_fire), fire_pos[1] + vel[1] * (t - t_fire))

    def turret_at(t):
        p = true_pos(t)
        return (p[0] + TURRET_OFFSET[0], p[1] + TURRET_OFFSET[1])

    def command(t):
        lag_pos = true_pos(t - P["sensor_latency"])
        est = (lag_pos[0] + bias[0] + next(gi_noise) * P["pos_noise_in"] * IN + TURRET_OFFSET[0],
               lag_pos[1] + bias[1] + next(gi_noise) * P["pos_noise_in"] * IN + TURRET_OFFSET[1])
        raw = (vel[0] + next(gi_noise) * P["vel_noise_in_s"] * IN, vel[1] + next(gi_noise) * P["vel_noise_in_s"] * IN)
        if vfilt[0] is None:
            vfilt[0], vfilt[1] = raw
        else:
            k = P["loop_dt"] / (P["vel_filter_tau"] + P["loop_dt"])
            vfilt[0] += (raw[0] - vfilt[0]) * k
            vfilt[1] += (raw[1] - vfilt[1]) * k
        delay = P["sensor_latency"] + P["aim_lookahead"]
        return controller(table, est, (vfilt[0], vfilt[1]), delay)

    vfilt = [None, None]

    noise_rng = random.Random(rng.random())
    gi_noise = iter(lambda: noise_rng.gauss(0, 1), None)

    h = 0.005
    t = 0.0
    yaw_c, rpm_c, hood_c = command(0.0)
    yaw, rpm, hood = yaw_c, rpm_c, hood_c                    # start already tracking
    queue = [(0.0, yaw_c, rpm_c, hood_c)]
    prev_yaw_c, yaw_rate_ff = yaw_c, 0.0
    next_tick = P["loop_dt"]
    ready = False
    while t < t_rel:
        if t >= next_tick - 1e-9:
            c = command(t)
            yaw_rate_ff = wrap(c[0] - prev_yaw_c) / P["loop_dt"] if P["turret_ff"] else 0.0
            prev_yaw_c = c[0]
            queue.append((t + P["command_latency"], *c))
            next_tick += P["loop_dt"]
        while len(queue) > 1 and queue[1][0] <= t:
            queue.pop(0)
        _, yaw_c, rpm_c, hood_c = queue[0]
        # turret
        if P["turret_instant"]:
            yaw = yaw_c
        else:
            rate = P["turret_kp"] * wrap(yaw_c - yaw) + yaw_rate_ff
            lim = math.radians(P["turret_max_dps"])
            yaw += max(-lim, min(lim, rate)) * h
        # flywheel
        if P["fly_instant"]:
            rpm = rpm_c
        else:
            up = P["fly_alpha_stall"] * max(0.0, 1 - rpm / P["fly_free_rpm"])
            acc = max(-P["fly_alpha_down"], min(up, P["fly_kp"] * (rpm_c - rpm)))
            rpm += acc * h
        # hood
        if P["hood_instant"]:
            hood = hood_c
        else:
            hr = P["hood_kp"] * (hood_c - hood)
            hl = P["hood_max_dps"]
            hood += max(-hl, min(hl, hr)) * h
        t += h
        if abs(t - t_fire) < h / 2:
            ready = (abs(rpm - rpm_c) <= P["gate_rpm"] and abs(math.degrees(wrap(yaw_c - yaw))) <= P["gate_deg"]
                     and abs(hood - hood_c) <= P["gate_deg"])
    # ---- release and fly
    yaw_r = yaw + math.radians(next(gi_noise) * P["exit_dir_noise_deg"])
    hood_r = math.radians(hood + next(gi_noise) * P["exit_dir_noise_deg"])
    vb = K * rpm * (1 + next(gi_noise) * P["ball_speed_noise"])
    tc = turret_at(t_rel)
    px, pz = tc[0] + math.sin(yaw_r) * FWD, tc[1] + math.cos(yaw_r) * FWD
    py = EXIT_H
    vx = vel[0] + vb * math.cos(hood_r) * math.sin(yaw_r)
    vz = vel[1] + vb * math.cos(hood_r) * math.cos(yaw_r)
    vy = vb * math.sin(hood_r)
    kd = hs.DRAG_PER_METRE
    dt = 0.002
    s_prev = (px - TX) * N_IN[0] + (py - CY) * N_IN[1] + (pz - TZ) * N_IN[2]
    for _ in range(1500):
        v = math.sqrt(vx * vx + vy * vy + vz * vz)
        vx -= kd * v * vx * dt
        vy += (-G - kd * v * vy) * dt
        vz -= kd * v * vz * dt
        px += vx * dt; py += vy * dt; pz += vz * dt
        if py < 0:
            return {"hit": False, "ready": ready, "a": None, "b": None, "valid": ok_pos, "dir": heading, "fx": fx, "fz": fz}
        s = (px - TX) * N_IN[0] + (py - CY) * N_IN[1] + (pz - TZ) * N_IN[2]
        if s_prev < 0 <= s:
            rx, ry, rz = px - TX, py - CY, pz - TZ
            a = (ry * T_VERT[1] + rz * T_VERT[2]) / IN
            b = rx / IN
            cos_t = max((vx * N_IN[0] + vy * N_IN[1] + vz * N_IN[2]) / max(v, 1e-9), 0.2)
            hit = abs(a) <= hs.OPENING_LENGTH_IN / 2 - hs.BALL_RADIUS_IN / cos_t and abs(b) <= hs.OPENING_WIDTH_IN / 2 - hs.BALL_RADIUS_IN
            return {"hit": hit, "ready": ready, "a": a, "b": b, "valid": ok_pos, "dir": heading, "fx": fx, "fz": fz}
        s_prev = s
    return {"hit": False, "ready": ready, "a": None, "b": None, "valid": ok_pos, "dir": heading, "fx": fx, "fz": fz}


def run(table, params, n, seed=1, sampler=None):
    results = []
    for i in range(n):
        results.append(trial(table, params, random.Random(seed * 100003 + i), sampler))
    results = [r for r in results if r["valid"]]
    hits = sum(r["hit"] for r in results)
    ready = sum(r["ready"] for r in results)
    a = [r["a"] for r in results if r["a"] is not None]
    b = [r["b"] for r in results if r["b"] is not None]
    sd = lambda xs: math.sqrt(sum((x - sum(xs) / len(xs)) ** 2 for x in xs) / len(xs)) if xs else float("nan")
    return {"n": len(results), "hit": hits / len(results), "ready": ready / len(results),
            "a_sd": sd(a), "b_sd": sd(b), "results": results}


def main():
    n = 300
    speed = 24.0
    for arg in sys.argv[1:]:
        if arg.startswith("--n="): n = int(arg.split("=")[1])
        if arg.startswith("--speed="): speed = float(arg.split("=")[1])
    print("building the shot table (about 10 seconds)...")
    table = Table()
    print(f"  {len(table.d)} distances, {table.d[0]:.0f} to {table.d[-1]:.0f} in; "
          f"hood {min(table.hood):.0f} to {max(table.hood):.0f} deg; "
          f"wheel {min(table.v)/K:.0f} to {max(table.v)/K:.0f} rpm\n")

    print(f"=== hit rate vs driving speed ({n} shots each, firing from random spots 45 to 95 in from the cell) ===")
    print(f"{'speed':>14} {'hit rate':>9} {'ready at fire':>14} {'miss spread (in)':>18}")
    for s in (0, 12, 18, 24, 30, 36, 48):
        p = dict(PARAMS, robot_speed_in_s=float(s))
        r = run(table, p, n)
        print(f"{s:5d} in/s ({s/12:3.1f} ft/s) {r['hit']*100:6.0f}% {r['ready']*100:11.0f}% "
              f"{r['a_sd']:9.1f} up/down {r['b_sd']:5.1f} side".replace("up/down", "v"))
    print()

    base = dict(PARAMS, robot_speed_in_s=speed)
    ref = run(table, base, n)
    print(f"=== what is limiting the hit rate at {speed:.0f} in/s ({speed/12:.1f} ft/s)? ===")
    print(f"{'baseline':<52} {ref['hit']*100:5.0f}%")
    tests = [
        ("perfect localization (no pose/velocity error)", dict(pos_noise_in=0, pos_bias_in=0, vel_noise_in_s=0)),
        ("zero latency and exact release delay", dict(sensor_latency=0, command_latency=0, release_jitter=0)),
        ("instant flywheel", dict(fly_instant=True)),
        ("instant turret and hood", dict(turret_instant=True, hood_instant=True)),
        ("perfect shooter (no ball speed or direction spread)", dict(ball_speed_noise=0, exit_dir_noise_deg=0)),
        ("everything perfect", dict(pos_noise_in=0, pos_bias_in=0, vel_noise_in_s=0, sensor_latency=0, command_latency=0,
                                    release_jitter=0, ball_speed_noise=0, exit_dir_noise_deg=0, **IDEAL)),
    ]
    for name, change in tests:
        r = run(table, dict(base, **change), n)
        print(f"fix only: {name:<42} {r['hit']*100:5.0f}%   ({(r['hit']-ref['hit'])*100:+.0f})")
    print()
    print("how far ahead should the controller aim? (the feed delay is about 0.15 s, and it does NOT belong here)")
    for la in (0.0, 0.02, 0.04, 0.08, 0.15):
        r = run(table, dict(base, aim_lookahead=la), n)
        print(f"  lookahead {la:.2f} s{'  <- baseline' if la == PARAMS['aim_lookahead'] else ''}: {r['hit']*100:5.0f}%")
    print()
    print("what if the turret does not use the feed-forward?")
    r = run(table, dict(base, turret_ff=False), n)
    print(f"  no turret feed-forward: {r['hit']*100:5.0f}%")
    print("what if the wheel is rim-heavy (half the acceleration) and coasts down slowly?")
    r = run(table, dict(base, fly_alpha_stall=5644.0, fly_alpha_down=250.0), n)
    print(f"  slower flywheel: {r['hit']*100:5.0f}%")


if __name__ == "__main__":
    main()
