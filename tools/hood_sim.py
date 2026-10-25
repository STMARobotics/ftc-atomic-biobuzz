"""
Hood angle simulator for shoot-and-move at a FIXED flywheel speed.

Question it answers: if the flywheel always runs at one speed, what range of hood (launch) angles does the
hood need to cover so the ball still lands in the hive cell at every distance shoot-and-move can ask for?

How it works:
  1. The target is the real hive cell opening from the competition manual: a 20 in. x 14 in. rectangle
     tilted 30 degrees from vertical, facing up and toward the robot, its bottom edge 53.5 in. and its top
     edge 65.6 in. above the tiles.
  2. For a shot from the wheel to the middle of that opening, solve (by scanning) for the launch angle that
     crosses the opening at an allowed entry angle (flattest valid angle wins).
  3. Sweep that over every effective distance the lead math can produce (static distance range, widened by
     robot speed times flight time toward or away from the target).
  4. Report the hood angle range, and check the exit error against the opening.

Run:  python tools/hood_sim.py            (one table, using the numbers below)
      python tools/hood_sim.py --sweep    (hood min/max for a range of fixed ball speeds)
Options: --entry=N  steepest RISING flight angle allowed at the opening, degrees (default 10, 0 = must descend)
         --speed=N  override the wheel-based ball speed estimate with a measured one, in m/s

Every number in the INPUTS block is either from the manual, from CAD, or marked as a guess.
Pure Python, no packages needed. Units: inputs in inches, internals in metres.
"""
import math
import sys

IN = 0.0254
G = 9.81

# ----------------------------------------------------------------------------- INPUTS
# --- Hive cell opening, from the manual (figures 9-9 to 9-11, TU05) ---
OPENING_BOTTOM_IN = 53.5        # bottom edge of the upward-facing cell's opening, above the tiles
OPENING_TOP_IN = 65.6           # top edge
OPENING_LENGTH_IN = 14.0        # the opening is 14 in. tall measured along its tilt...
OPENING_TILT_DEG = 30.0         # ...and tilted 30 deg from vertical (14 * cos 30 = 12.1 = 65.6 - 53.5)
OPENING_WIDTH_IN = 20.0
BALL_RADIUS_IN = 1.8            # nectar is 3.6 in. across, the bigger ball, so the tighter fit
OPENING_CENTER_IN = (OPENING_BOTTOM_IN + OPENING_TOP_IN) / 2   # about 59.6 in.

# Steepest RISING flight angle allowed when the ball reaches the opening. The opening faces up and toward
# the robot, so a flat or descending ball goes straight in, while a rising ball sees a much smaller opening.
ENTRY_ANGLE_MAX_DEG = 10.0

# --- Ball speed from the wheel ---
# One wheel and a hood: the ball is squeezed between them, so it leaves at about HALF the wheel's surface
# speed at best, less with slip. SLIP_FACTOR is the biggest unknown in this whole file: 0.5 is the
# theoretical best, 0.35 to 0.45 is more realistic for a light wiffle ball.
WHEEL_DIAMETER_IN = 3.0         # AndyMark 3 in. stealth wheel
WHEEL_RPM = 3000.0
SLIP_FACTOR = 0.45              # ball speed / wheel surface speed. MEASURE THIS when the robot exists.
WHEEL_SURFACE_SPEED_MPS = math.pi * WHEEL_DIAMETER_IN * IN * WHEEL_RPM / 60.0
BALL_EXIT_SPEED_MPS = WHEEL_SURFACE_SPEED_MPS * SLIP_FACTOR   # about 5.4 m/s with the numbers above

# --- Shooter geometry, from CAD ---
# Centre of the turret bearing is 248.2 mm above the floor in CAD, +/- 5 mm (top and bottom of the bearing).
# The foam tiles squish a little under the robot, which lowers this by a few mm. Too small to matter here.
TURRET_CENTER_HEIGHT_IN = 248.2 / 25.4   # 9.77 in
WHEEL_ABOVE_CENTER_IN = 1.5     # wheel is this far above the centre of rotation
EXIT_HEIGHT_IN = TURRET_CENTER_HEIGHT_IN + WHEEL_ABOVE_CENTER_IN   # where the ball actually leaves
TURRET_TO_EXIT_FORWARD_IN = 6.0 # turret centre of rotation to the wheel contact point, along the aim. FROM CAD.
BARREL_OFFSET_IN = 30 / 25.4    # the nectar and pollen wheels sit +/- 30 mm sideways of the turret centre
EXIT_ERROR_DEG = 1.0            # +/- error in the direction the ball leaves (elevation and yaw). A GUESS.

# --- Where we shoot from ---
# The field is 144 in. square with the hive in the middle. A corner is about 102 in. from the centre, and the
# hive frame (about 39 in. deep) plus the robot keeps you at least ~36 in. away.
DIST_MIN_IN = 36.0              # closest shooting distance, turret centre to the opening centre, horizontal
DIST_MAX_IN = 100.0             # farthest
ROBOT_MAX_SPEED_IN_S = 50.0     # fastest the robot drives while shooting (a fast mecanum is about 50 to 60)

# Air drag: deceleration = k * speed^2, with k = air_density * Cd * area / (2 * mass).
# The manual does not give the ball's mass or say how it's built, so these are estimates. A perforated
# (wiffle) ball has a Cd around 0.4 to 0.6. Mass for a hollow 2.8 in. plastic ball is maybe 20 to 30 g.
AIR_DENSITY = 1.2               # kg/m^3
BALL_DRAG_COEFFICIENT = 0.5     # a guess for a perforated ball
BALL_MASS_KG = 0.025            # a guess
BALL_DIAMETER_IN = 2.8          # pollen. Nectar is 3.6 in. but is heavier and flies similarly.
DRAG_PER_METRE = (AIR_DENSITY * BALL_DRAG_COEFFICIENT * math.pi * (BALL_DIAMETER_IN * IN / 2) ** 2
                  / (2 * BALL_MASS_KG))   # about 0.05 per metre with the numbers above
# -----------------------------------------------------------------------------

TILT = math.radians(OPENING_TILT_DEG)
# The opening is a line in the side view. Its inward normal points down and away from the robot, 30 deg below
# horizontal, and its long direction (the tangent) points up and toward the hive.
NORMAL = (math.cos(TILT), -math.sin(TILT))
TANGENT = (math.sin(TILT), math.cos(TILT))


def cross_opening(angle_deg, speed, center_x, drag=None, dt=0.0005):
    """
    Fire from the wheel and find where the ball crosses the opening.
    center_x: horizontal distance (m) from the wheel to the centre of the opening.
    Returns None if the ball never reaches the opening, else a dict:
      offset  - how far along the opening (m, + is toward the top edge) the ball crosses, from its centre
      path    - flight path angle at the crossing (deg, + is rising)
      time    - flight time (s)
    """
    if drag is None:
        drag = DRAG_PER_METRE
    a = math.radians(angle_deg)
    x, y = 0.0, EXIT_HEIGHT_IN * IN
    vx, vy = speed * math.cos(a), speed * math.sin(a)
    cy = OPENING_CENTER_IN * IN
    t = 0.0

    def side(px, py):
        return (px - center_x) * NORMAL[0] + (py - cy) * NORMAL[1]

    s_prev = side(x, y)
    while t < 5.0:
        v = math.hypot(vx, vy)
        vx += -drag * v * vx * dt
        vy += (-G - drag * v * vy) * dt
        px, py = x, y
        x += vx * dt
        y += vy * dt
        t += dt
        if y < 0:
            return None
        s = side(x, y)
        if s_prev < 0 <= s:
            frac = -s_prev / (s - s_prev)
            cx_, cy_ = px + frac * (x - px), py + frac * (y - py)
            return {
                "offset": (cx_ - center_x) * TANGENT[0] + (cy_ - cy) * TANGENT[1],
                "path": math.degrees(math.atan2(vy, vx)),
                "time": t,
            }
        s_prev = s
    return None


def solve_angle(center_x, speed):
    """Flattest launch angle that crosses the middle of the opening at an allowed entry angle, or None."""
    prev = None
    angle = 5.0
    while angle <= 85.0:
        hit = cross_opening(angle, speed, center_x)
        if hit is not None and prev is not None:
            p_angle, p_hit = prev
            if ((p_hit["offset"] > 0) != (hit["offset"] > 0)
                    and hit["path"] <= ENTRY_ANGLE_MAX_DEG and p_hit["path"] <= ENTRY_ANGLE_MAX_DEG):
                frac = p_hit["offset"] / (p_hit["offset"] - hit["offset"])
                return p_angle + frac * (angle - p_angle)
        prev = (angle, hit) if hit is not None else None
        angle += 0.25
    return None


def exit_distance(turret_distance):
    """Horizontal distance (m) from the wheel (not the turret centre) to the opening centre."""
    forward = turret_distance - TURRET_TO_EXIT_FORWARD_IN
    return math.hypot(forward, BARREL_OFFSET_IN) * IN


def effective_distances(speed):
    """Every effective distance the lead math can ask for: static range, widened by speed * flight time."""
    lo, hi = DIST_MIN_IN, DIST_MAX_IN
    for d in (DIST_MIN_IN, DIST_MAX_IN):
        angle = solve_angle(exit_distance(d), speed)
        hit = cross_opening(angle, speed, exit_distance(d)) if angle else None
        if hit is None:
            continue
        shift = ROBOT_MAX_SPEED_IN_S * hit["time"]
        lo = min(lo, d - shift)   # driving toward the target
        hi = max(hi, d + shift)   # driving away from it
    return max(lo, 10.0), hi


def margin_check(distance, angle, speed):
    """
    If the launch direction is off by +/- EXIT_ERROR_DEG, how far along the opening does the ball miss its
    centre, against how far it can miss and still fit (the ball has a radius, and a slanted entry makes the
    opening look smaller). Also the sideways miss against the 20 in. width.
    Returns (worst miss in, allowed miss in, sideways miss in, ok).
    """
    cx = exit_distance(distance)
    nominal = cross_opening(angle, speed, cx)
    off_axis = math.radians(nominal["path"] + OPENING_TILT_DEG)    # angle between travel and the opening's normal
    allowed = OPENING_LENGTH_IN / 2 - BALL_RADIUS_IN / max(math.cos(off_axis), 0.2)
    worst = 0.0
    for sign in (-1, 1):
        hit = cross_opening(angle + sign * EXIT_ERROR_DEG, speed, cx)
        worst = max(worst, abs(hit["offset"]) / IN if hit else 99.0)
    sideways = cx * math.tan(math.radians(EXIT_ERROR_DEG)) / IN
    ok = worst <= allowed and sideways <= OPENING_WIDTH_IN / 2 - BALL_RADIUS_IN
    return worst, allowed, sideways, ok


def report(speed, verbose=True):
    lo, hi = effective_distances(speed)
    angles = []
    rows = []
    d = lo
    while d <= hi + 1e-9:
        angle = solve_angle(exit_distance(d), speed)
        if angle is not None:
            hit = cross_opening(angle, speed, exit_distance(d))
            worst, allowed, sideways, ok = margin_check(d, angle, speed)
            angles.append(angle)
            rows.append((d, angle, hit["time"], hit["path"], worst, allowed, sideways, ok))
        else:
            rows.append((d, None))
        d += 5.0
    if verbose:
        print(f"Ball exit speed {speed:.1f} m/s ({speed / IN:.0f} in/s), "
              f"effective distance {lo:.0f} to {hi:.0f} in, rising entry up to {ENTRY_ANGLE_MAX_DEG:.0f} deg")
        print(f"{'dist in':>8} {'hood deg':>9} {'flight s':>9} {'entry deg':>10} {'miss in':>8} {'can miss':>9}  margin")
        for row in rows:
            if row[1] is None:
                print(f"{row[0]:8.0f}   cannot reach the opening at this speed and entry angle")
            else:
                d, angle, t, path, worst, allowed, sideways, ok = row
                print(f"{d:8.0f} {angle:9.1f} {t:9.2f} {path:10.1f} {worst:8.1f} {allowed:9.1f}  "
                      f"{'ok' if ok else 'TOO WIDE'}")
    if angles:
        span = max(angles) - min(angles)
        unreachable = any(r[1] is None for r in rows)
        if verbose:
            print(f"\nHood must cover {min(angles):.1f} to {max(angles):.1f} deg: a span of {span:.1f} deg"
                  + ("  (some distances unreachable!)" if unreachable else ""))
        return span, unreachable, min(angles), max(angles)
    return None, True, None, None


HOOD_PRACTICAL_MAX_DEG = 70.0   # a hood much steeper than this is hard to build and the lob is inaccurate


def serviceable_window(speed):
    """
    Distances this fixed ball speed can serve, with a hood angle at most HOOD_PRACTICAL_MAX_DEG and the
    exit error still fitting the opening. Returns (nearest, farthest, hood at nearest, hood at farthest) or None.
    """
    good = []
    d = 15.0
    while d <= 150.0:
        angle = solve_angle(exit_distance(d), speed)
        if angle is not None and angle <= HOOD_PRACTICAL_MAX_DEG and margin_check(d, angle, speed)[3]:
            good.append((d, angle))
        d += 2.0
    if not good:
        return None
    return good[0][0], good[-1][0], good[0][1], good[-1][1]


if __name__ == "__main__":
    for arg in sys.argv:
        if arg.startswith("--entry="):
            ENTRY_ANGLE_MAX_DEG = float(arg.split("=")[1])
        if arg.startswith("--speed="):          # override the wheel-based estimate with a measured speed
            BALL_EXIT_SPEED_MPS = float(arg.split("=")[1])
    if "--window" in sys.argv:
        print(f"Distances each fixed ball speed can serve (hood <= {HOOD_PRACTICAL_MAX_DEG:.0f} deg, margin ok), "
              f"rising entry up to {ENTRY_ANGLE_MAX_DEG:.0f} deg")
        print(f"{'speed m/s':>10} {'nearest in':>11} {'farthest in':>12} {'hood near':>10} {'hood far':>9}")
        speed = 5.0
        while speed <= 11.0:
            w = serviceable_window(speed)
            if w is None:
                print(f"{speed:10.1f} {'none':>11}")
            else:
                print(f"{speed:10.1f} {w[0]:11.0f} {w[1]:12.0f} {w[2]:10.1f} {w[3]:9.1f}")
            speed += 0.5
    elif "--sweep" in sys.argv:
        print(f"rising entry up to {ENTRY_ANGLE_MAX_DEG:.0f} deg (0 = must be descending)")
        print(f"{'speed m/s':>10} {'hood min':>9} {'hood max':>9} {'span':>6}")
        speed = 5.0
        while speed <= 11.0:
            span, unreachable, lo, hi = report(speed, verbose=False)
            note = "  some distances unreachable" if unreachable else ""
            if span is None:
                print(f"{speed:10.1f} {'-':>9} {'-':>9} {'-':>6}{note}")
            else:
                print(f"{speed:10.1f} {lo:9.1f} {hi:9.1f} {span:6.1f}{note}")
            speed += 0.5
    else:
        report(BALL_EXIT_SPEED_MPS)
