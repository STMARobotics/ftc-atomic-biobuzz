"""
Finds the flywheel speed(s) that let the HOOD do the adjusting during shoot-and-move.

Question: at one fixed flywheel speed, with only the hood changing to cover different distances, what fraction
of shoot-and-move shots go in when the robot fires from anywhere in the cell's firing area? And if no single
speed gets 90%, what is the smallest set of speeds (zone presets) that does?

"Hood angle" everywhere here is the launch angle above horizontal: 30 means 30 degrees up from horizontal.

Run:  python tools/rpm_optimizer.py             (24 in/s = 2 ft/s)
      python tools/rpm_optimizer.py --speed=18  (a different driving speed)
      python tools/rpm_optimizer.py --n=600     (more shots, smoother numbers, slower)

Uses the shot physics from hood_sim.py and the controller/actuator model from sotm_sim.py, so every guess in
those files (shooter spread, latency, flywheel response) carries over.
"""
import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("sotm_sim", os.path.join(HERE, "sotm_sim.py"))
ss = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ss)
hs = ss.hs
K, IN = ss.K, ss.IN

HOOD_MAX_DEG = 70.0          # a hood steeper than this is not practical to build or aim
HOOD_MIN_DEG = 20.0
BANDS = [(36, 50), (50, 65), (65, 80), (80, 100)]       # distance from the opening (in), turret centre to centre


class FixedSpeedTable(ss.Table):
    """Same lookup interface as the real controller's table, but the ball speed never changes."""

    def __init__(self, speed):
        self.d, self.hood, self.tof, self.v = [], [], [], []
        d = 24.0
        while d <= 138.0:
            r = ss._solve_fast(d, speed)
            if r is not None and HOOD_MIN_DEG <= r[0] <= HOOD_MAX_DEG:
                self.d.append(d); self.hood.append(r[0]); self.tof.append(r[1]); self.v.append(speed)
            d += 2.0

    def lookup(self, d):
        if not self.d:
            return 45.0, 0.4, self.v[0] if self.v else 6.0
        return super().lookup(d)        # outside the solvable range this clamps, so the shot simply misses


def area_sampler(rng):
    """Uniform over the field, kept only if it is in the cell's 60 degree firing cone and at least 36 in away."""
    while True:
        x, z = rng.uniform(9, 135), rng.uniform(9, 135)
        dx, dz = x - ss.TARGET[0], z - ss.TARGET[1]
        d = math.hypot(dx, dz)
        if dz > 0 and d >= 36 and math.acos(dz / d) <= math.radians(60):
            return x, z


def band_of(r):
    d = math.hypot(r["fx"] - ss.TARGET[0], r["fz"] - ss.TARGET[1])
    for i, (lo, hi) in enumerate(BANDS):
        if lo <= d < hi:
            return i
    return None


def evaluate(speed_mps, params, n):
    table = FixedSpeedTable(speed_mps)
    res = ss.run(table, params, n, seed=1, sampler=area_sampler)["results"]
    out = []
    for i in range(len(BANDS)):
        rs = [r for r in res if band_of(r) == i]
        out.append((sum(r["hit"] for r in rs) / len(rs), len(rs)) if rs else (float("nan"), 0))
    overall = sum(r["hit"] for r in res) / len(res)
    return table, overall, out, res


def band_sampler(lo, hi):
    """Random firing spots whose distance from the opening is in [lo, hi] inches, inside the field and the cone."""
    def sample(rng):
        while True:
            d = rng.uniform(lo, hi)
            a = math.radians(rng.uniform(-60, 60))
            x, z = ss.TARGET[0] + d * math.sin(a), ss.TARGET[1] + d * math.cos(a)
            if 9 <= x <= 135 and z <= 135:
                return x, z
    return sample


def band_analysis(params, n, width=10, speeds=None):
    """For each narrow distance band, which fixed flywheel speed lets the hood alone handle the driving?"""
    if speeds is None:
        speeds = [5.0 + 0.125 * i for i in range(0, 29)]
    tables = {v: FixedSpeedTable(v) for v in speeds}
    edges = list(range(36, 100, width)) + [102]
    print(f"{'distance':>10} {'best rpm':>9} {'ball m/s':>9} {'hit rate':>9} {'rpm that reach 90%':>22} {'hood used (deg above horizontal)':>34}")
    plan = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        scores = []
        for v in speeds:
            r = ss.run(tables[v], params, n, seed=1, sampler=band_sampler(lo, hi))["results"]
            scores.append((sum(x["hit"] for x in r) / len(r), v))
        best_hit, best_v = max(scores)
        good = [v for h, v in scores if h >= 0.90]
        swing = params["robot_speed_in_s"] * 0.4          # how far the aim distance moves during a shot
        used = [h for d, h in zip(tables[best_v].d, tables[best_v].hood) if lo - swing <= d <= hi + swing]
        hood = f"{min(used):.0f} to {max(used):.0f}" if used else "none"
        rng90 = f"{min(good)/K:.0f} to {max(good)/K:.0f}" if good else "none reaches 90%"
        print(f"{lo:4d}-{hi:<5d} {best_v/K:9.0f} {best_v:9.2f} {best_hit*100:8.0f}% {rng90:>22} {hood:>34}")
        plan.append((lo, hi, best_v, best_hit, hood))
    lo_rpm = min(p[2] for p in plan) / K
    hi_rpm = max(p[2] for p in plan) / K
    print(f"\nwheel rpm across the field: {lo_rpm:.0f} (closest) to {hi_rpm:.0f} (farthest), a spread of {hi_rpm - lo_rpm:.0f} rpm")
    return plan


def main():
    n, speed_in = 300, 24.0
    mode_all = "--all" in sys.argv
    for arg in sys.argv[1:]:
        if arg.startswith("--n="): n = int(arg.split("=")[1])
        if arg.startswith("--speed="): speed_in = float(arg.split("=")[1])
    params = dict(ss.PARAMS, robot_speed_in_s=speed_in)
    if not mode_all:
        print(f"driving at {speed_in:.0f} in/s ({speed_in/12:.1f} ft/s). For each 10 in distance band: the fixed wheel "
              f"speed where the hood alone covers the driving ({n} shots per speed)\n")
        band_analysis(params, n)
        return
    print(f"driving at {speed_in:.0f} in/s ({speed_in/12:.1f} ft/s), firing from anywhere in the cell's cone, "
          f"{n} shots per speed; hood limited to {HOOD_MIN_DEG:.0f} to {HOOD_MAX_DEG:.0f} deg above horizontal\n")
    head = "".join(f"{lo:>3}-{hi:<4}" for lo, hi in BANDS)
    print(f"{'ball m/s':>9} {'wheel rpm':>10} {'hood range (deg)':>18} {'overall':>8}   hit rate by distance (in): {head}")
    rows = []
    v = 5.0
    while v <= 8.76:
        table, overall, bands, _ = evaluate(v, params, n)
        hr = f"{min(table.hood):.0f} to {max(table.hood):.0f}" if table.hood else "none"
        cells = "".join(f"{('--' if b[1] == 0 else f'{b[0]*100:.0f}%'):>8}" for b in bands)
        print(f"{v:9.2f} {v/K:10.0f} {hr:>18} {overall*100:7.0f}%   {' ' * 26}{cells}")
        rows.append((v, bands))
        v += 0.25

    print("\n--- best single speed ---")
    best = max(rows, key=lambda r: sum(b[0] * b[1] for b in r[1] if b[1]))
    print(f"{best[0]:.2f} m/s = {best[0]/K:.0f} wheel rpm")

    print("\n--- smallest set of speeds (presets) for 90% in every distance band ---")
    goal = 0.90
    usable = {i: [(v, b[i][0]) for v, b in rows if b[i][1] and b[i][0] >= goal] for i in range(len(BANDS))}
    chosen = []
    for i, lo_hi in enumerate(BANDS):
        opts = usable[i]
        if not opts:
            print(f"  {lo_hi[0]}-{lo_hi[1]} in: no single speed reaches 90% here")
            continue
        centre = sorted(v for v, _ in opts)[len(opts) // 2]
        print(f"  {lo_hi[0]}-{lo_hi[1]} in: speeds that reach 90%: {min(v for v, _ in opts):.2f} to {max(v for v, _ in opts):.2f} m/s "
              f"({min(v for v, _ in opts)/K:.0f} to {max(v for v, _ in opts)/K:.0f} rpm)")
        chosen.append((i, centre))
    # greedy cover: fewest speeds such that every band has one of them in its 90% set
    bands_left = {i for i in usable if usable[i]}
    presets = []
    while bands_left:
        cand = sorted({v for i in bands_left for v, _ in usable[i]})
        pick = max(cand, key=lambda v: sum(1 for i in bands_left if any(abs(v - w) < 1e-9 for w, _ in usable[i])))
        covered = {i for i in bands_left if any(abs(pick - w) < 1e-9 for w, _ in usable[i])}
        presets.append((pick, sorted(covered)))
        bands_left -= covered
    print()
    for pick, covered in presets:
        table = FixedSpeedTable(pick)
        names = ", ".join(f"{BANDS[i][0]}-{BANDS[i][1]} in" for i in covered)
        print(f"  preset {pick/K:5.0f} rpm ({pick:.2f} m/s): covers {names}; "
              f"hood {min(table.hood):.0f} to {max(table.hood):.0f} deg above horizontal")


if __name__ == "__main__":
    main()
