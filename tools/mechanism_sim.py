"""
Mechanism simulator for the intake, shooter gate, flywheel and turret.

The logic mirrors the Java in TeamCode (ChunkMode, the two intake default commands, ShootCommand,
AimCommand, TurretSubsystem), so what you see here is what the robot should do.

    pip install pygame
    python tools/mechanism_sim.py                      run it
    python tools/mechanism_sim.py --selftest           check the turret logic, no window
    python tools/mechanism_sim.py --debug-controller   print raw axes and buttons

Controls (Logitech F310, X mode)      Keyboard
  left trigger   intake                I
  B              outtake               O
  right trigger  shoot                 S
  X              battle mode toggle    X
  A              flower mech toggle    F
  dpad up        turret home           H
  dpad down      auto aim toggle       T
  dpad left      turret ccw (hold)     Q
  dpad right     turret cw (hold)      E
  right stick x  turn the robot        left / right arrows
  left stick y   tag distance (sim)    up / down arrows
                                       R to reset, Esc to quit
"""
import math
import sys

# ---- mirrored from RobotConstants.java ----
MIN_ANGLE = 0.0
MAX_ANGLE = 540.0
FORWARD_ANGLE = 270.0
TOLERANCE = 1.5
UNWIND_MIN_TRAVEL = 90.0
SHOOT_RPM = 2910.0
READY_TOLERANCE_RPM = 75.0
ENCODER_CLOCKWISE_POSITIVE = True

# ---- simulation only ----
CAMERA_HZ = 30.0
CAMERA_FOV_HALF = 40.0      # degrees either side of the crosshair
TURRET_MAX_SPEED = 400.0    # deg/s
TURRET_KP = 10.0
FLYWHEEL_RATE = 2.5         # spin up speed
TURN_RATE = 120.0           # deg/s at full stick
# demo shot table: distance in, rpm, pollen hood, nectar hood. The robot's real one is in
# RobotConstants.Shooter.SHOT_TABLE and still has flat placeholders
SHOT_TABLE = [(24.0, 2500.0, 0.15, 0.85), (84.0, 2910.0, 0.50, 0.50), (144.0, 3400.0, 0.85, 0.15)]
DISTANCE_RATE = 40.0        # inches per second of stick
MANUAL_SPEED = 90.0         # deg/s, Turret.MANUAL_SPEED_DEG_PER_SEC
SETTLE = 0.4                # seconds a servo needs to finish a move (SETTLE_MS)
TAG_WORLD_BEARING = 270.0   # clockwise from world up, so the tag starts dead ahead of the turret


def lookup_shot(distance):
    points = SHOT_TABLE
    if distance <= points[0][0]:
        return points[0][1:]
    for low, high in zip(points, points[1:]):
        if distance <= high[0]:
            t = (distance - low[0]) / (high[0] - low[0])
            return tuple(a + t * (b - a) for a, b in zip(low[1:], high[1:]))
    return points[-1][1:]


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def signed(angle):
    angle %= 360.0
    return angle - 360.0 if angle > 180.0 else angle


class Turret:
    """Same algorithm as TurretSubsystem.java, with a speed limited P loop standing in for the PID."""

    def __init__(self, dir_sign=1.0):
        self.dir = dir_sign
        self.angle = FORWARD_ANGLE
        self.target = FORWARD_ANGLE
        self.unwinding = False
        self.unwind_target = 0.0
        self.heading_at_unwind = 0.0
        self.angle_at_unwind = 0.0

    def choose_angle(self, desired):
        best = None
        for k in (-1, 0, 1):
            candidate = desired + 360.0 * k
            if candidate < MIN_ANGLE or candidate > MAX_ANGLE:
                continue
            if best is None or abs(candidate - self.angle) < abs(best - self.angle):
                best = candidate
        return best if best is not None else clip(desired, MIN_ANGLE, MAX_ANGLE)

    def _set_target(self, angle):
        self.target = clip(angle, MIN_ANGLE, MAX_ANGLE)

    def hold_current_angle(self):
        self.unwinding = False
        self._set_target(self.angle)

    def aim_at_tag(self, tx, heading_ccw):
        if self.unwinding:
            if abs(self.angle - self.angle_at_unwind) < UNWIND_MIN_TRAVEL:
                return
            self.unwinding = False

        desired = self.angle + self.dir * tx
        reachable = self.choose_angle(desired)

        if abs(reachable - desired) > 1:
            self.unwinding = True
            self.unwind_target = reachable
            self.heading_at_unwind = heading_ccw
            self.angle_at_unwind = self.angle
        self._set_target(reachable)

    def aim_at_angle(self, angle):
        self.unwinding = False
        self._set_target(angle)

    def nudge_clockwise(self, delta):
        self.unwinding = False
        self._set_target(self.target + self.dir * delta)

    def is_at_target(self):
        return abs(self.target - self.angle) <= TOLERANCE

    def periodic(self, dt, heading_ccw):
        if self.unwinding:
            change = heading_ccw - self.heading_at_unwind
            self._set_target(self.unwind_target + self.dir * change)
            if self.is_at_target():
                self.unwinding = False
        velocity = clip(TURRET_KP * (self.target - self.angle), -TURRET_MAX_SPEED, TURRET_MAX_SPEED)
        self.angle = clip(self.angle + velocity * dt, MIN_ANGLE, MAX_ANGLE)


class Camera:
    """Limelight stand in. Frames arrive at CAMERA_HZ and the tag is seen inside the FOV."""

    def __init__(self):
        self.accumulated = 0.0
        self.new_frame = False
        self.visible = False
        self.tx = 0.0

    def update(self, dt, tag_rel_cw, turret_angle):
        self.accumulated += dt
        self.new_frame = False
        if self.accumulated >= 1.0 / CAMERA_HZ:
            self.accumulated -= 1.0 / CAMERA_HZ
            self.new_frame = True
            error = signed(tag_rel_cw - turret_angle)
            self.visible = abs(error) <= CAMERA_FOV_HALF
            self.tx = error if self.visible else 0.0


class Sim:
    def __init__(self, dir_sign=1.0):
        self.turret = Turret(dir_sign)
        self.camera = Camera()
        self.heading_cw = 0.0
        self.rpm = 0.0
        self.battle = False
        self.intake = False
        self.outtake = False
        self.shoot = False
        self.pivot = "STOWED"
        self.wheels = "IDLE"
        self.gate_open = False
        self.flower_wanted = False
        self.pivot_timer = 1.0
        self.flower_state = "STOWED"
        self.flower_timer = 1.0
        self.auto_aim = True
        self.distance = 84.0
        self.shot_rpm = SHOOT_RPM
        self.pollen, self.nectar = lookup_shot(self.distance)[1:]
        self.manual_dir = 0

    @property
    def heading_ccw(self):
        return -self.heading_cw

    def tag_relative_cw(self):
        return (TAG_WORLD_BEARING - self.heading_cw) % 360.0

    def flywheel_ready(self):
        return self.shoot and abs(self.rpm - self.shot_rpm) <= READY_TOLERANCE_RPM

    def aim_ready(self):
        return self.turret.is_at_target() and not self.turret.unwinding and self.camera.visible

    def shoot_aim_ok(self):
        # right trigger waits for the lock unless auto aim is off, then we're aiming by hand
        return (not self.auto_aim) or self.aim_ready()

    def set_inputs(self, intake, outtake, shoot, battle_pressed, flower_pressed=False,
                   home_pressed=False, auto_toggle_pressed=False, manual_dir=0):
        if intake and not self.intake:
            self.battle = False
        if battle_pressed:
            self.battle = not self.battle
            if self.battle:
                self.flower_wanted = False
        if flower_pressed:
            self.flower_wanted = not self.flower_wanted
            if self.flower_wanted:
                self.battle = False
        if home_pressed:
            self.auto_aim = False
            self.turret.aim_at_angle(FORWARD_ANGLE)
        if auto_toggle_pressed:
            self.auto_aim = not self.auto_aim
            if not self.auto_aim:
                self.turret.hold_current_angle()
        if self.manual_dir != 0 and manual_dir == 0:
            self.turret.hold_current_angle()  # ManualTurretCommand.end
        self.manual_dir = manual_dir
        self.intake, self.outtake, self.shoot = intake, outtake, shoot

    def step(self, dt, turn, distance_input=0.0):
        self.heading_cw += turn * TURN_RATE * dt
        self.distance = clip(self.distance + distance_input * DISTANCE_RATE * dt, 20.0, 160.0)

        self.camera.update(dt, self.tag_relative_cw(), self.turret.angle)
        self.turret.periodic(dt, self.heading_ccw)
        if self.manual_dir != 0:
            self.turret.nudge_clockwise(self.manual_dir * MANUAL_SPEED * dt)
        elif self.auto_aim:
            if self.camera.visible and self.camera.new_frame:
                self.turret.aim_at_tag(self.camera.tx, self.heading_ccw)

        if self.camera.visible:
            self.shot_rpm, self.pollen, self.nectar = lookup_shot(self.distance)
        target_rpm = self.shot_rpm if self.shoot else 0.0
        self.rpm += (target_rpm - self.rpm) * min(1.0, FLYWHEEL_RATE * dt)
        self.gate_open = self.flywheel_ready() and self.shoot_aim_ok()

        flower_stowed = self.flower_state == "STOWED" and self.flower_timer >= SETTLE
        if self.flower_wanted or not flower_stowed:
            pivot_cmd = "DEPLOYED"
        elif self.battle:
            pivot_cmd = "FULLY_STOWED"
        elif self.intake:
            pivot_cmd = "DEPLOYED"
        else:
            pivot_cmd = "STOWED"
        if pivot_cmd != self.pivot:
            self.pivot = pivot_cmd
            self.pivot_timer = 0.0
        else:
            self.pivot_timer += dt

        pivot_deployed = self.pivot == "DEPLOYED" and self.pivot_timer >= SETTLE
        flower_cmd = "DEPLOYED" if self.flower_wanted and pivot_deployed else "STOWED"
        if flower_cmd != self.flower_state:
            self.flower_state = flower_cmd
            self.flower_timer = 0.0
        else:
            self.flower_timer += dt

        if self.outtake:
            self.wheels = "OUTTAKE"
        elif self.gate_open:
            self.wheels = "FEEDING"
        elif self.intake and not self.battle:
            self.wheels = "INTAKE"
        else:
            self.wheels = "IDLE"


def flower_selftest():
    """Walk through the flower scenario: toggle on, intake a bit, toggle off."""
    sim = Sim()
    dt = 1.0 / 60.0

    def run(seconds, intake=False):
        sim.set_inputs(intake, False, False, False)
        for _ in range(int(seconds / dt)):
            sim.step(dt, 0.0)

    ok = True

    def check(label, condition):
        nonlocal ok
        ok = ok and condition
        print("  %-52s %s" % (label, "ok" if condition else "WRONG"))

    sim.battle = True  # starts fully stowed
    sim.set_inputs(False, False, False, False, True)
    sim.step(dt, 0.0)
    check("toggle on from battle mode leaves battle mode", not sim.battle)
    check("intake starts deploying right away", sim.pivot == "DEPLOYED")
    check("flower waits for the intake to settle", sim.flower_state == "STOWED")
    run(1.0)
    check("flower deployed once the intake is down", sim.flower_state == "DEPLOYED")
    run(0.5, intake=True)
    run(0.5, intake=False)
    check("intake stays down after releasing intake", sim.pivot == "DEPLOYED")
    check("wheels idle after releasing intake", sim.wheels == "IDLE")
    sim.set_inputs(False, False, False, False, True)
    sim.step(dt, 0.0)
    check("toggle off stows the flower first", sim.flower_state == "STOWED")
    check("intake waits for the flower", sim.pivot == "DEPLOYED")
    run(1.0)
    check("then the intake stows too", sim.pivot == "STOWED")
    print("flower scenario:", "PASS" if ok else "FAIL")
    return ok


def controls_selftest():
    """Auto aim, home, manual turning and the two kinds of shooting."""
    sim = Sim()
    dt = 1.0 / 60.0
    ok = True

    def check(label, condition):
        nonlocal ok
        ok = ok and condition
        print("  %-52s %s" % (label, "ok" if condition else "WRONG"))

    def run(seconds, **held):
        params = dict(intake=False, outtake=False, shoot=False, battle_pressed=False)
        params.update(held)
        for _ in range(int(seconds / dt)):
            sim.set_inputs(**params)
            sim.step(dt, 0.0)

    run(1.0)
    check("auto aim locks the tag", sim.aim_ready())
    run(3.0, shoot=True)
    check("right trigger opens the gate when locked", sim.gate_open)

    sim.heading_cw = 180.0  # tag is now behind us, camera can't see it
    run(1.0)
    check("camera loses the tag", not sim.camera.visible)
    run(3.0, shoot=True)
    check("right trigger waits while auto aim is on and no lock", not sim.gate_open)

    sim.set_inputs(False, False, False, False, home_pressed=True)
    check("home turns auto aim off", not sim.auto_aim)
    check("home targets the home angle", sim.turret.target == FORWARD_ANGLE)
    run(3.0, shoot=True)
    check("right trigger shoots with auto aim off", sim.gate_open)

    start = sim.turret.angle
    run(0.5, manual_dir=1)
    run(0.5)
    moved = sim.turret.angle - start
    check("dpad right turns the turret clockwise (%.0f deg)" % moved, 20 < moved < 60)
    check("it holds where it stopped", abs(sim.turret.target - sim.turret.angle) < 5)
    run(0.5, manual_dir=-1)
    run(0.5)
    check("dpad left turns it back", sim.turret.angle < start + moved - 20)

    sim.set_inputs(False, False, False, False, auto_toggle_pressed=True)
    check("dpad down turns auto aim back on", sim.auto_aim)
    print("controls scenario:", "PASS" if ok else "FAIL")
    return ok


def hood_selftest():
    """Hoods follow distance all the time, the flywheel only when shooting."""
    sim = Sim()
    dt = 1.0 / 60.0
    ok = True

    def check(label, condition):
        nonlocal ok
        ok = ok and condition
        print("  %-52s %s" % (label, "ok" if condition else "WRONG"))

    def run(seconds, distance_input=0.0, **held):
        params = dict(intake=False, outtake=False, shoot=False, battle_pressed=False)
        params.update(held)
        for _ in range(int(seconds / dt)):
            sim.set_inputs(**params)
            sim.step(dt, 0.0, distance_input)

    run(1.0)
    near = (sim.pollen, sim.nectar)
    run(3.0, distance_input=1.0)
    far = (sim.pollen, sim.nectar)
    check("hoods move with distance while not shooting", far[0] > near[0] and far[1] < near[1])
    check("flywheel stays off while not shooting", sim.rpm < 1.0)
    run(4.0, shoot=True)
    check("flywheel spins up to the table rpm when shooting", abs(sim.rpm - sim.shot_rpm) < 100)
    check("the table asks for more rpm farther away", sim.shot_rpm > 2910.0)
    print("hood scenario:", "PASS" if ok else "FAIL")
    return ok


def selftest():
    """Spin the robot clockwise and see how long the camera loses the tag."""
    results = {}
    for name, sign in (("sign matches the tx convention", 1.0), ("sign flipped", -1.0)):
        sim = Sim(sign)
        dt = 1.0 / 120.0
        lost = 0.0
        unwinds = 0
        was_unwinding = False
        for _ in range(int(14 / dt)):
            sim.step(dt, 1.0)
            if not sim.camera.visible:
                lost += dt
            if sim.turret.unwinding and not was_unwinding:
                unwinds += 1
            was_unwinding = sim.turret.unwinding
        error = abs(signed(sim.tag_relative_cw() - sim.turret.angle))
        results[name] = lost
        print("%-32s lost %.2fs over 14s, %d unwinds, final error %.1f deg"
              % (name, lost, unwinds, error))
    ok = results["sign matches the tx convention"] < results["sign flipped"]
    print("PASS" if ok else "FAIL")
    flower_ok = flower_selftest()
    controls_ok = controls_selftest()
    hood_ok = hood_selftest()
    return 0 if ok and flower_ok and controls_ok and hood_ok else 1


def run(debug_controller=False):
    import pygame

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((1000, 640))
    pygame.display.set_caption("Mechanism sim")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("arial", 16)
    small = pygame.font.SysFont("arial", 13)
    big = pygame.font.SysFont("arial", 20, bold=True)

    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
    x_mode = joystick is not None and joystick.get_numaxes() >= 6
    trigger_signed = {}

    def axis(i):
        return joystick.get_axis(i) if joystick and i < joystick.get_numaxes() else 0.0

    def button(i):
        return bool(joystick.get_button(i)) if joystick and i < joystick.get_numbuttons() else False

    def trigger(axis_index):
        v = axis(axis_index)
        if axis_index not in trigger_signed:
            trigger_signed[axis_index] = v < -0.9
        norm = (v + 1.0) / 2.0 if trigger_signed[axis_index] else v
        return norm > 0.3

    sim = Sim(1.0 if ENCODER_CLOCKWISE_POSITIVE else -1.0)
    previous_battle = previous_flower = previous_up = previous_down = False
    pivot_angle, wheel_angle, fly_angle, gate_angle = 25.0, 0.0, 0.0, 0.0
    flower_angle = 100.0
    pivot_goal = {"DEPLOYED": -40.0, "STOWED": 25.0, "FULLY_STOWED": 80.0}
    wheel_speed = {"INTAKE": 720.0, "FEEDING": 720.0, "OUTTAKE": -720.0, "IDLE": 120.0}
    debug_timer = 0.0

    BG, PANEL, LINE, TEXT, DIM = (30, 32, 36), (42, 45, 51), (90, 94, 102), (230, 230, 230), (150, 154, 162)
    TEAL, PURPLE, CORAL, AMBER, GREEN = (29, 158, 117), (127, 119, 221), (216, 90, 48), (239, 159, 39), (99, 153, 34)

    def text(surface, s, pos, f=font, color=TEXT, center=False):
        img = f.render(s, True, color)
        rect = img.get_rect()
        if center:
            rect.center = pos
        else:
            rect.topleft = pos
        surface.blit(img, rect)

    def robot_to_screen(cx, cy, x, y, h_cw):
        h = math.radians(h_cw)
        return (cx + x * math.cos(h) + y * math.sin(h), cy + x * math.sin(h) - y * math.cos(h))

    running = True
    while running:
        dt = min(clock.tick(60) / 1000.0, 0.05)
        key_b = False
        key_f = False
        key_h = False
        key_t = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_x:
                    key_b = True
                elif event.key == pygame.K_f:
                    key_f = True
                elif event.key == pygame.K_h:
                    key_h = True
                elif event.key == pygame.K_t:
                    key_t = True
                elif event.key == pygame.K_r:
                    sim = Sim(1.0 if ENCODER_CLOCKWISE_POSITIVE else -1.0)

        keys = pygame.key.get_pressed()
        hat = joystick.get_hat(0) if joystick and joystick.get_numhats() > 0 else (0, 0)
        if x_mode:
            pad_intake, pad_shoot = trigger(4), trigger(5)
            pad_flower, pad_outtake, pad_battle = button(0), button(1), button(2)
            pad_turn = axis(2)
        elif joystick:
            pad_intake, pad_shoot = button(6), button(7)
            pad_battle, pad_flower, pad_outtake = button(0), button(1), button(2)
            pad_turn = axis(2)
        else:
            pad_intake = pad_shoot = pad_flower = pad_outtake = pad_battle = False
            pad_turn = 0.0
        if abs(pad_turn) < 0.1:
            pad_turn = 0.0

        pad_up, pad_down = hat[1] == 1, hat[1] == -1
        pad_left, pad_right = hat[0] == -1, hat[0] == 1
        battle_pressed = key_b or (pad_battle and not previous_battle)
        flower_pressed = key_f or (pad_flower and not previous_flower)
        home_pressed = key_h or (pad_up and not previous_up)
        auto_toggle_pressed = key_t or (pad_down and not previous_down)
        previous_battle, previous_flower = pad_battle, pad_flower
        previous_up, previous_down = pad_up, pad_down

        intake = keys[pygame.K_i] or pad_intake
        outtake = keys[pygame.K_o] or pad_outtake
        shoot = keys[pygame.K_s] or pad_shoot
        turn = pad_turn + (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT])
        left = keys[pygame.K_q] or pad_left
        right = keys[pygame.K_e] or pad_right
        manual_dir = (1 if right else 0) - (1 if left else 0)

        sim.set_inputs(intake, outtake, shoot, battle_pressed, flower_pressed,
                       home_pressed, auto_toggle_pressed, manual_dir)
        distance_input = (-axis(1) if abs(axis(1)) > 0.1 else 0.0) + (keys[pygame.K_UP] - keys[pygame.K_DOWN])
        sim.step(dt, clip(turn, -1.0, 1.0), clip(distance_input, -1.0, 1.0))

        if debug_controller and joystick:
            debug_timer += dt
            if debug_timer > 0.5:
                debug_timer = 0.0
                print("axes", [round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())],
                      "buttons", [i for i in range(joystick.get_numbuttons()) if joystick.get_button(i)])

        pivot_angle += clip(pivot_goal[sim.pivot] - pivot_angle, -300 * dt, 300 * dt)
        wheel_angle += wheel_speed[sim.wheels] * dt
        fly_angle += sim.rpm * 6.0 * 0.05 * dt
        gate_goal = -85.0 if sim.gate_open else 0.0
        gate_angle += clip(gate_goal - gate_angle, -500 * dt, 500 * dt)
        flower_goal = 10.0 if sim.flower_state == "DEPLOYED" else 100.0
        flower_angle += clip(flower_goal - flower_angle, -250 * dt, 250 * dt)

        screen.fill(BG)

        # ---- top down view ----
        cx, cy = 250, 235
        pygame.draw.circle(screen, LINE, (cx, cy), 190, 1)
        tag_h = math.radians(TAG_WORLD_BEARING)
        tag_pos = (cx + 190 * math.sin(tag_h), cy - 190 * math.cos(tag_h))
        pygame.draw.rect(screen, CORAL, (tag_pos[0] - 10, tag_pos[1] - 10, 20, 20), border_radius=3)
        text(screen, "tag", (tag_pos[0], tag_pos[1] + 24), small, DIM, True)

        body = [robot_to_screen(cx, cy, x, y, sim.heading_cw) for x, y in ((-36, -44), (36, -44), (36, 44), (-36, 44))]
        pygame.draw.polygon(screen, PANEL, body)
        pygame.draw.polygon(screen, LINE, body, 2)
        nose = [robot_to_screen(cx, cy, x, y, sim.heading_cw) for x, y in ((-10, 44), (0, 58), (10, 44))]
        pygame.draw.polygon(screen, DIM, nose)

        overlay = pygame.Surface((1000, 640), pygame.SRCALPHA)
        wedge = [(cx, cy)]
        for d in range(-int(CAMERA_FOV_HALF), int(CAMERA_FOV_HALF) + 1, 10):
            a = math.radians(sim.turret.angle + d)
            wedge.append(robot_to_screen(cx, cy, 150 * math.sin(a), 150 * math.cos(a), sim.heading_cw))
        pygame.draw.polygon(overlay, TEAL + (60,), wedge)
        screen.blit(overlay, (0, 0))
        a = math.radians(sim.turret.angle)
        tip = robot_to_screen(cx, cy, 90 * math.sin(a), 90 * math.cos(a), sim.heading_cw)
        pygame.draw.line(screen, TEAL, (cx, cy), tip, 4)
        pygame.draw.circle(screen, TEAL, (cx, cy), 7)

        # ---- turret angle bar ----
        bx0, bx1, by = 50, 450, 490
        def bar_x(angle):
            return bx0 + angle * (bx1 - bx0) / MAX_ANGLE
        pygame.draw.rect(screen, PANEL, (bx0, by, bx1 - bx0, 14), border_radius=3)
        pygame.draw.rect(screen, TEXT, (bx0 - 4, by - 8, 4, 30))
        pygame.draw.rect(screen, TEXT, (bx1, by - 8, 4, 30))
        pygame.draw.line(screen, DIM, (bar_x(360), by - 4), (bar_x(360), by + 18), 1)
        text(screen, "0", (bx0, by + 28), small, DIM, True)
        text(screen, "360", (bar_x(360), by + 28), small, DIM, True)
        text(screen, "540", (bx1, by + 28), small, DIM, True)
        pygame.draw.circle(screen, AMBER, (int(bar_x(sim.turret.target)), by + 7), 9, 2)
        pygame.draw.polygon(screen, TEAL, [(bar_x(sim.turret.angle) - 7, by - 16), (bar_x(sim.turret.angle) + 7, by - 16), (bar_x(sim.turret.angle), by - 1)])
        text(screen, "turret angle %.0f" % sim.turret.angle, (bx0, by - 46), font)
        if sim.manual_dir != 0:
            state, state_color = "MANUAL", AMBER
        elif not sim.auto_aim:
            state, state_color = "AUTO AIM OFF", AMBER
        elif sim.turret.unwinding:
            state, state_color = "UNWINDING", PURPLE
        elif sim.camera.visible:
            state, state_color = "TRACKING", TEAL
        else:
            state, state_color = "NO TAG", CORAL
        text(screen, state, (bx0 + 200, by - 46), big, state_color)
        text(screen, "orange ring is the target", (bx0, by + 50), small, DIM)

        # ---- intake side view ----
        px, py = 700, 150
        pygame.draw.rect(screen, PANEL, (px, py - 30, 150, 110), border_radius=6)
        pygame.draw.rect(screen, LINE, (px, py - 30, 150, 110), 1, border_radius=6)
        a = math.radians(pivot_angle)
        arm_tip = (px - 100 * math.cos(a), py - 100 * math.sin(a))
        pygame.draw.line(screen, TEXT, (px, py), arm_tip, 6)
        pygame.draw.circle(screen, TEXT, (px, py), 6)
        wheel_color = {"INTAKE": TEAL, "FEEDING": PURPLE, "OUTTAKE": CORAL, "IDLE": DIM}[sim.wheels]
        pygame.draw.circle(screen, wheel_color, (int(arm_tip[0]), int(arm_tip[1])), 24, 3)
        for k in range(4):
            s = math.radians(wheel_angle + k * 90)
            pygame.draw.line(screen, wheel_color, arm_tip, (arm_tip[0] + 24 * math.cos(s), arm_tip[1] + 24 * math.sin(s)), 3)
        text(screen, "intake pivot: " + sim.pivot.lower().replace("_", " "), (px - 150, py + 95), font)
        text(screen, "wheels: " + sim.wheels.lower(), (px - 150, py + 118), font, wheel_color)
        fa = math.radians(flower_angle)
        f_base = (px + 40, py - 30)
        f_tip = (f_base[0] - 80 * math.cos(fa), f_base[1] - 80 * math.sin(fa))
        pygame.draw.line(screen, AMBER, f_base, f_tip, 5)
        for k in range(3):
            pa = fa + math.radians(60 * (k - 1))
            pygame.draw.circle(screen, AMBER, (int(f_tip[0] - 14 * math.cos(pa)), int(f_tip[1] - 14 * math.sin(pa))), 8, 2)
        pygame.draw.circle(screen, AMBER, f_base, 5)
        text(screen, "flower: " + sim.flower_state.lower(), (px - 150, py + 141), font, AMBER)

        # ---- shooter gate and flywheel ----
        gx, gy = 700, 330
        pygame.draw.line(screen, LINE, (gx, gy - 50), (gx, gy + 50), 3)
        pygame.draw.line(screen, LINE, (gx + 44, gy - 50), (gx + 44, gy + 50), 3)
        lever_a = math.radians(gate_angle)
        pygame.draw.line(screen, GREEN if sim.gate_open else CORAL, (gx, gy),
                         (gx + 44 * math.cos(lever_a), gy - 44 * math.sin(lever_a)), 6)
        text(screen, "gate " + ("open" if sim.gate_open else "closed"), (gx - 20, gy + 66), font)
        fx, fy = 860, 330
        pygame.draw.circle(screen, TEAL if sim.flywheel_ready() else AMBER, (fx, fy), 48, 4)
        for k in range(3):
            s = math.radians(fly_angle + k * 120)
            pygame.draw.line(screen, DIM, (fx, fy), (fx + 48 * math.cos(s), fy + 48 * math.sin(s)), 3)
        text(screen, "%d rpm" % sim.rpm, (fx, fy + 70), font, TEXT, True)
        for base_x, direction, position, color in ((fx - 40, 1, sim.pollen, AMBER), (fx + 40, -1, sim.nectar, PURPLE)):
            hood_a = math.radians(15 + 70 * position)
            hood_tip = (base_x + direction * 52 * math.cos(hood_a), fy - 58 - 52 * math.sin(hood_a))
            pygame.draw.line(screen, color, (base_x, fy - 58), hood_tip, 5)
            pygame.draw.circle(screen, color, (int(base_x), fy - 58), 5)
        text(screen, "pollen %.2f" % sim.pollen, (fx - 70, fy + 92), small, AMBER, True)
        text(screen, "nectar %.2f" % sim.nectar, (fx + 50, fy + 92), small, PURPLE, True)

        # ---- readout and controls ----
        text(screen, "battle: " + ("on" if sim.battle else "off"), (540, 436), big)
        text(screen, "flower: " + ("on" if sim.flower_wanted else "off"), (690, 436), big)
        text(screen, "tag distance %d in, shot %d rpm" % (sim.distance, sim.shot_rpm), (540, 462), small, DIM)
        text(screen, "auto aim: " + ("on" if sim.auto_aim else "off"), (830, 436), big)
        pad_name = joystick.get_name() if joystick else "no controller, keyboard only"
        text(screen, "input: " + pad_name + (" (X mode)" if x_mode else ""), (540, 488), small, DIM)
        row1 = (("LT intake", intake), ("B outtake", outtake), ("RT shoot", shoot),
                ("X battle", pad_battle or key_b), ("A flower", pad_flower or key_f))
        row2 = (("Up home", pad_up or key_h), ("Down auto", pad_down or key_t),
                ("Left ccw", left), ("Right cw", right))
        for r, row in enumerate((row1, row2)):
            for i, (label, on) in enumerate(row):
                rect = pygame.Rect(516 + i * 80, 508 + r * 38, 76, 30)
                pygame.draw.rect(screen, TEAL if on else PANEL, rect, border_radius=6)
                text(screen, label, rect.center, small, BG if on else DIM, True)
        text(screen, "keys: I O S X F H T Q E, arrows turn, R reset, Esc quit", (516, 590), small, DIM)
        text(screen, "right stick x turns the robot, left stick y or up/down sets distance", (516, 610), small, DIM)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    run("--debug-controller" in sys.argv)
