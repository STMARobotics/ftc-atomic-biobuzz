package sim;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConstants.Flower;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakePivot;
import org.firstinspires.ftc.teamcode.RobotConstants.ShooterGate;
import org.firstinspires.ftc.teamcode.RobotConstants.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Simple physics for the robot. Not exact, just close enough that the real code behaves like it
 * does on the robot: the drivetrain has weight, the flywheel takes time to spin up and sags when
 * the intake runs, servos take time to move.
 *
 * Everything here is a guess at the real robot (TUNE), change the numbers to match what you see.
 * Field coordinates are Pedro's: inches, 0-144, heading radians counterclockwise.
 */
public final class SimPhysics {

    // ---- drivetrain: every wheel is a DC motor pushing on the floor, nothing is scripted ----
    // motor: goBILDA 5203 19.2:1 (312 rpm). TUNE: stall numbers are from the spec sheet, check them
    static final double MOTOR_STALL_NM = 2.38;
    static final double MOTOR_STALL_AMPS = 9.2;
    static final double MOTOR_FREE_RPM = 312;
    // from the CAD and the scale
    static final double WHEEL_RADIUS_M = 0.052;
    static final double ROBOT_MASS_KG = 16.33; // 36 lb
    static final double ROBOT_YAW_INERTIA = ROBOT_MASS_KG * (0.457 * 0.457 * 2) / 12; // 18 in square box
    static final double TURN_RADIUS_M = (13.8 + 9.76) / 2 * 0.0254; // (track + wheelbase) / 2
    // floor. TUNE against the match logs: these three set the feel
    static final double TRACTION_MU = 0.7; // mecanum on foam tile, wheels slip past this
    static final double ROLLING_RESIST = 0.05; // fraction of the weight lost to rollers and carpet
    static final double STRAFE_EFFICIENCY = 0.8; // rollers waste some going sideways
    static final double GRAVITY = 9.81;
    static final double INTAKE_STOW_DEG = 100; // same as robot.json, TUNE
    static final double BATTERY_OHMS = 0.06; // battery + wiring, sets the sag when the wheels pull hard. TUNE
    static final double WHEEL_FREE_SPEED = MOTOR_FREE_RPM / 60 * 2 * Math.PI * WHEEL_RADIUS_M; // ~1.70 m/s
    static final double IN_PER_M = 39.37;
    static final double HIVE_AXLE_IN = 43.95; // manual 9.6.1
    // FL FR BL BR: how strafing and turning spin each wheel (Pedro's mix)
    private static final double[] STRAFE_SIGN = {-1, 1, 1, -1};
    private static final double[] TURN_SIGN = {-1, 1, -1, 1};

    // ---- flywheel: 2x 6000rpm 5203, it tops out around 4000 motor rpm under all that drag ----
    static final double FLYWHEEL_TOP_RPM = 4150; // at full power on a 12.6V battery
    static final double FLYWHEEL_TAU = 0.45;
    static final double SHOT_RPM_LOSS = 350; // what one ball takes out of it
    static final double SHOT_MIN_RPM = 1500; // below this the ball doesn't make it out
    static final double FEED_SECONDS_PER_BALL = 0.3;

    // ---- intake: 312 rpm motor ----
    static final double INTAKE_MOTOR_RPM = 312;
    static final double INTAKE_TAU = 0.08;
    static final int MAX_BALLS = 4; // G407, no more than 4 at a time
    static final int PRELOAD_BALLS = 4; // the manual: every robot starts with 4 pollen
    static final double CAPTURE_COOLDOWN_SEC = 0.12;
    // the mouth of the deployed intake, inches in the robot frame. Reaches a bit past the intake box
    static final double MOUTH_X_MIN = 10;
    static final double MOUTH_X_MAX = 14.4;
    static final double MOUTH_HALF_WIDTH = 8.5;
    static final double MOUTH_Z_MAX = 6;
    static final double FLOWER_REACH_X_MAX = 18; // from the CAD: the bar's tip lands 14.9 in out, plus a ball's radius
    // Limelight 3A, on the turret: from the CAD with the turret facing forward. Field of view is the 3A's spec
    static final double CAMERA_FORWARD = 6.2; // ahead of the turret axis
    static final double CAMERA_HEIGHT = 12.3;
    static final double CAMERA_HALF_FOV_X = 27.25;
    static final double CAMERA_HALF_FOV_Y = 21;
    static final double CAMERA_MAX_RANGE = 130; // 3.25 in tags get too small past about here. TUNE
    // the AprilTag clusters, on the bottom face of each cell (field_info.json, inches: x, y, z) with the
    // Hives as staged, 1 = red, then the first tag ID (manual 9.9: audience is low y). They all face the way
    // the Hive's underside does, and turn with it. Each is 4 tags in a row (manual figure 9-15)
    private static final double[][] CELL_TAGS = {
            {59.26, 59.11, 49.67, 1, 34}, {59.26, 83.39, 35.65, 1, 30},
            {84.76, 60.61, 35.65, 0, 38}, {84.76, 84.89, 49.67, 0, 42}};
    private static final double[] TAG_OFFSETS_X = {-6.5, -2.75, 2.75, 6.5};
    private static final double TAG_FACING_MIN = 0.25; // a tag edge-on past about 75 deg can't be read
    // shots. Each barrel's exit relative to the turret axis with the turret facing forward, from the
    // CAD: pollen to the right, nectar to the left. The ball starts clear of the tower box
    static final double TURRET_AXIS_X = -1.85;
    static final double BARREL_FORWARD = 7.0;
    static final double BARREL_HEIGHT = 15.5;
    static final double POLLEN_BARREL_LEFT = -1.42;
    static final double NECTAR_BARREL_LEFT = 1.0;
    static final double FLYWHEEL_RATIO = 16.0 / 24.0; // motor pulley to wheel pulley, from the CAD
    static final double FLYWHEEL_DIAMETER_M = 3 * 0.0254;
    // ball speed vs wheel rim speed. A guess: 0.55 peaks at 60 in and can't reach the cell opening at
    // 63 in. TUNE: film one real shot, its peak height sets this
    static final double BALL_EXIT_FRACTION = 0.65;
    static final double HOOD_ANGLE_AT_0 = 65; // the 65-80 deg range you described. Direction is a guess, TUNE
    static final double HOOD_ANGLE_AT_1 = 80;

    // ---- turret: 312 rpm at 4:1 ----
    static final double TURRET_DEG_PER_SEC = 312 / 4.0 * 360 / 60.0;
    static final double TURRET_START_DEG = Turret.FORWARD_ANGLE_DEG;

    // ---- battery: running the intake pulls it down ~2V ----
    static final double BATTERY_REST_VOLTS = 12.7;
    static final double INTAKE_SAG_VOLTS = 2.0;
    static final double FLYWHEEL_SAG_VOLTS = 0.6;

    static final double START_X = 24;
    static final double START_Y = 72;

    private final SimWorld world;

    // true robot pose on the field
    // starts on open floor, field center is where the A-frame is
    public volatile double x = START_X;
    public volatile double y = START_Y;
    volatile double heading = 0;
    // body velocity: forward, left (in/s), counterclockwise (rad/s)
    volatile double vForward = 0;
    volatile double vLeft = 0;
    volatile double vTurn = 0;

    volatile double batteryVolts = BATTERY_REST_VOLTS;
    // fault injection: a tired battery rests lower
    public volatile double batteryRestVolts = BATTERY_REST_VOLTS;
    // fault injection: the next ball fed into the shooter sticks until the intake reverses briefly
    public volatile boolean jamNextBall = false;
    public volatile boolean jammed = false;
    private double reverseTimer = 0;

    final java.util.ArrayDeque<Integer> held = new java.util.ArrayDeque<>();
    final SimCollisions collisions =
            new SimCollisions(ROBOT_MASS_KG, START_X / IN_PER_M, START_Y / IN_PER_M, 0);
    private double stepBacklog = 0;
    private double driveAmps = 0;
    public volatile double flywheelRpm = 0;
    volatile double intakeRpm = 0;
    volatile double turretDeg = TURRET_START_DEG;
    volatile boolean turretJammed = false; // fault injection
    volatile boolean cameraDead = false; // fault injection, the camera stops sending frames
    volatile boolean limelightHidden = false; // for tests: frames keep coming but there's never a tag in them

    public volatile int balls = PRELOAD_BALLS;
    public volatile int shotsLaunched = 0;
    private double feedTimer = 0;
    private double captureCooldown = 0;
    private double publishTimer = 0;
    private double outtakeTimer = 0;

    SimPhysics(SimWorld world) {
        this.world = world;
        loadPreloads();
    }

    // ---- how each motor is physically mounted, so the current code's directions are right ----

    static int mount(String name) {
        if (name.equals(Constants.drivetrainConfig.frontLeftName.get())
                || name.equals(Constants.drivetrainConfig.backLeftName.get())) return -1;
        if (name.equals(HardwareNames.FLYWHEEL_MOTOR)) return -1;
        if (name.equals(HardwareNames.INTAKE_MOTOR_RIGHT)) return -1;
        return 1;
    }

    static double ticksPerRev(String name) {
        if (name.equals(HardwareNames.FLYWHEEL_MOTOR) || name.equals(HardwareNames.FLYWHEEL_MOTOR_2)) return 28;
        return 537.7;
    }

    static double servoRate(String name) {
        if (name.equals(HardwareNames.PIVOT_SERVO)) return 1.5; // Axon MAX through TPU gears, slowish
        if (name.equals(HardwareNames.GATE_SERVO)) return 4.0;
        return 2.0;
    }

    static double servoStart(String name) {
        if (name.equals(HardwareNames.PIVOT_SERVO)) return IntakePivot.FULLY_STOWED;
        if (name.equals(HardwareNames.GATE_SERVO)) return ShooterGate.CLOSED;
        if (name.equals(HardwareNames.FLOWER_SERVO)) return Flower.STOWED;
        return 0.5;
    }

    // ---- sensors ----

    double turretEncoderVolts() {
        return turretDeg / Turret.ENCODER_RANGE_DEG * Turret.ENCODER_MAX_VOLTS;
    }

    void placeRobot(double newX, double newY, double newHeading) {
        collisions.setPose(newX / IN_PER_M, newY / IN_PER_M, newHeading);
        x = newX;
        y = newY;
        heading = newHeading;
        vForward = 0;
        vLeft = 0;
        vTurn = 0;
    }

    void resetRobot() {
        placeRobot(START_X, START_Y, 0);
        collisions.resetPieces();
        loadPreloads();
        shotsLaunched = 0;
        jamNextBall = false;
        jammed = false;
        batteryRestVolts = BATTERY_REST_VOLTS;
        flywheelRpm = 0;
        intakeRpm = 0;
        world.otos().resetTracking();
    }

    // ---- the step ----

    void step(double dt) {
        double voltScale = batteryVolts / 12.0;

        // drivetrain + collisions, in the fixed steps the physics engine needs
        SimServo intakeServo = world.servo(HardwareNames.PIVOT_SERVO);
        // deployed (the CAD pose) to stowed, same travel as robot.json. TUNE them together
        collisions.setIntakeAngle(INTAKE_STOW_DEG * (intakeServo.physical - IntakePivot.DEPLOYED)
                / (IntakePivot.FULLY_STOWED - IntakePivot.DEPLOYED));
        stepBacklog = Math.min(stepBacklog + dt, 0.05);
        while (stepBacklog >= SimCollisions.STEP_SEC) {
            stepDrive(SimCollisions.STEP_SEC);
            stepBacklog -= SimCollisions.STEP_SEC;
        }

        // flywheel, both motors on one shaft. if they fight (direction bug) it doesn't spin
        SimMotor fly1 = world.motor(HardwareNames.FLYWHEEL_MOTOR);
        SimMotor fly2 = world.motor(HardwareNames.FLYWHEEL_MOTOR_2);
        double flyPower = (fly1.mechanismPower() + fly2.mechanismPower()) / 2;
        double flyTarget = FLYWHEEL_TOP_RPM * flyPower * batteryVolts / 12.6;
        flywheelRpm += (flyTarget - flywheelRpm) * (1 - Math.exp(-dt / FLYWHEEL_TAU));
        fly1.mechanismRpm = flywheelRpm;
        fly2.mechanismRpm = flywheelRpm;

        // intake
        SimMotor intake = world.motor(HardwareNames.INTAKE_MOTOR_RIGHT);
        double intakePower = intake.mechanismPower();
        intakeRpm += (INTAKE_MOTOR_RPM * intakePower * voltScale - intakeRpm) * (1 - Math.exp(-dt / INTAKE_TAU));
        intake.mechanismRpm = intakeRpm;
        intake.currentAmps = 9.0 * Math.abs(intakePower); // it's working hard, TUNE against a real reading

        // turret, hardstops at both ends
        SimMotor turret = world.motor(HardwareNames.TURRET_MOTOR);
        double turretPower = turret.mechanismPower();
        if (!turretJammed) {
            turretDeg = Range.clip(turretDeg + TURRET_DEG_PER_SEC * turretPower * voltScale * dt,
                    0, Turret.ENCODER_RANGE_DEG);
        }
        turret.mechanismRpm = TURRET_DEG_PER_SEC * turretPower / 6.0 * 4;

        // servos
        for (SimServo servo : world.servos()) servo.step(dt);

        // battery
        double sag = INTAKE_SAG_VOLTS * Math.abs(intakePower)
                + FLYWHEEL_SAG_VOLTS * Math.abs(flyPower)
                + BATTERY_OHMS * driveAmps;
        batteryVolts += (batteryRestVolts - sag - batteryVolts) * (1 - Math.exp(-dt / 0.05));

        stepBalls(dt);
        publishTimer += dt;
        if (publishTimer >= 0.015) {
            publishTimer = 0;
            collisions.publish();
        }
    }

    // wheel speed from the robot's motion -> motor force -> traction limit -> the physics world
    private void stepDrive(double h) {
        SimMotor[] wheels = {
                world.motor(Constants.drivetrainConfig.frontLeftName.get()),
                world.motor(Constants.drivetrainConfig.frontRightName.get()),
                world.motor(Constants.drivetrainConfig.backLeftName.get()),
                world.motor(Constants.drivetrainConfig.backRightName.get())};
        double yaw = collisions.heading();
        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);
        double forwardMs = collisions.vx() * cos + collisions.vy() * sin;
        double leftMs = -collisions.vx() * sin + collisions.vy() * cos;
        double spin = collisions.vTurn();

        double stallForce = MOTOR_STALL_NM / WHEEL_RADIUS_M;
        double maxTraction = TRACTION_MU * ROBOT_MASS_KG * GRAVITY / 4;
        double forceForward = 0, forceLeft = 0, torque = 0, amps = 0;
        for (int i = 0; i < 4; i++) {
            SimMotor wheel = wheels[i];
            double duty = wheel.mechanismPower();
            double wheelSpeed = forwardMs + STRAFE_SIGN[i] * leftMs / STRAFE_EFFICIENCY
                    + TURN_SIGN[i] * spin * TURN_RADIUS_M;
            double speedRatio = wheelSpeed / WHEEL_FREE_SPEED;
            wheel.mechanismRpm = speedRatio * MOTOR_FREE_RPM;
            // a DC motor: full force at stall, none at free speed. At zero power BRAKE shorts the motor
            // (it fights the motion), FLOAT doesn't
            boolean floating = duty == 0 && wheel.zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT;
            double drive = duty * batteryVolts / 12.0 - speedRatio;
            double force = floating ? 0 : Range.clip(stallForce * drive, -maxTraction, maxTraction);
            forceForward += force;
            forceLeft += STRAFE_SIGN[i] * force * STRAFE_EFFICIENCY;
            torque += TURN_SIGN[i] * force * TURN_RADIUS_M;
            wheel.currentAmps = floating ? 0 : MOTOR_STALL_AMPS * Math.abs(drive);
            amps += Math.abs(duty) * wheel.currentAmps;
        }
        driveAmps = amps;

        // rollers and carpet: opposes motion, can stop the robot, can't push it backwards
        double rollForce = ROLLING_RESIST * ROBOT_MASS_KG * GRAVITY;
        forceForward = withDrag(forceForward, forwardMs, rollForce, ROBOT_MASS_KG / h);
        forceLeft = withDrag(forceLeft, leftMs, rollForce, ROBOT_MASS_KG / h);
        torque = withDrag(torque, spin, rollForce * TURN_RADIUS_M, ROBOT_YAW_INERTIA / h);

        collisions.step(forceForward * cos - forceLeft * sin, forceForward * sin + forceLeft * cos, torque);

        // what actually happened after contacts. The OTOS only sees real motion, so pushing a wall doesn't count
        double newYaw = collisions.heading();
        double c = Math.cos(newYaw);
        double s = Math.sin(newYaw);
        double actualForward = collisions.vx() * c + collisions.vy() * s;
        double actualLeft = -collisions.vx() * s + collisions.vy() * c;
        vForward = actualForward * IN_PER_M;
        vLeft = actualLeft * IN_PER_M;
        vTurn = collisions.vTurn();
        x = collisions.x() * IN_PER_M;
        y = collisions.y() * IN_PER_M;
        heading = newYaw;
        world.otos().integrate(vForward, vLeft, vTurn, h);
    }

    // friction force on an axis: never bigger than what it takes to stop it this step
    private static double withDrag(double force, double v, double drag, double massPerStep) {
        double toStop = force + v * massPerStep;
        if (Math.abs(toStop) <= drag) return -v * massPerStep;
        return force - Math.copySign(drag, toStop);
    }

    private void loadPreloads() {
        held.clear();
        for (int i = 0; i < PRELOAD_BALLS; i++) held.add(SimCollisions.POLLEN);
        balls = held.size();
    }

    // real balls: the deployed intake picks them up, the shooter launches them in the physics world
    private void stepBalls(double dt) {
        double feed = intakeRpm / INTAKE_MOTOR_RPM; // -1 to 1
        SimServo gate = world.servo(HardwareNames.GATE_SERVO);
        SimServo pivot = world.servo(HardwareNames.PIVOT_SERVO);
        boolean gateOpen = Math.abs(gate.physical - ShooterGate.OPEN) < 0.15;
        boolean pivotDown = Math.abs(pivot.physical - IntakePivot.DEPLOYED) < 0.15;

        if (gateOpen && feed > 0.3 && !held.isEmpty()) {
            feedTimer += dt * feed;
            if (feedTimer >= FEED_SECONDS_PER_BALL && jamNextBall) {
                jamNextBall = false;
                jammed = true;
            }
            if (feedTimer >= FEED_SECONDS_PER_BALL && !jammed) {
                feedTimer = 0;
                launch(held.pollFirst());
                balls = held.size();
                if (flywheelRpm > SHOT_MIN_RPM) {
                    flywheelRpm = Math.max(0, flywheelRpm - SHOT_RPM_LOSS);
                    shotsLaunched++;
                } else {
                    world.warn("A ball got fed into a flywheel that wasn't spinning");
                }
            }
        } else {
            feedTimer = 0;
        }

        captureCooldown = Math.max(0, captureCooldown - dt);
        if (pivotDown && feed > 0.3 && held.size() < MAX_BALLS && captureCooldown == 0) {
            int type = collisions.capture(MOUTH_X_MIN, MOUTH_X_MAX, MOUTH_HALF_WIDTH, MOUTH_Z_MAX);
            if (type >= 0) {
                held.addLast(type);
                balls = held.size();
                captureCooldown = CAPTURE_COOLDOWN_SEC;
            }
        }

        // the flower bar out with the intake running pulls pollen from the bottom of a flower
        SimServo flowerBar = world.servo(HardwareNames.FLOWER_SERVO);
        boolean barOut = Math.abs(flowerBar.physical - Flower.DEPLOYED) < 0.15;
        if (pivotDown && barOut && feed > 0.3 && held.size() < MAX_BALLS && captureCooldown == 0
                && collisions.takeFromFlower(MOUTH_X_MIN, FLOWER_REACH_X_MAX, MOUTH_HALF_WIDTH)) {
            held.addLast(SimCollisions.POLLEN);
            balls = held.size();
            captureCooldown = 0.5; // the stack has to drop down and the next one roll out
        }

        // a short reverse frees a stuck ball
        reverseTimer = feed < -0.2 ? reverseTimer + dt : 0;
        if (jammed && reverseTimer >= 0.08) {
            jammed = false;
            feedTimer = 0;
        }

        if (feed < -0.3 && !held.isEmpty()) {
            outtakeTimer += dt;
            if (outtakeTimer >= 0.4) {
                outtakeTimer = 0;
                eject(held.pollLast());
                balls = held.size();
            }
        } else {
            outtakeTimer = 0;
        }
    }

    // out of the barrel the sorter sends this type to, at the angle that barrel's hood is set to
    void launch(int type) {
        boolean isPollen = type == SimCollisions.POLLEN;
        double left = isPollen ? POLLEN_BARREL_LEFT : NECTAR_BARREL_LEFT;
        SimServo hood = world.servo(isPollen ? HardwareNames.POLLEN_HOOD_SERVO : HardwareNames.NECTAR_HOOD_SERVO);
        double elevation = Math.toRadians(HOOD_ANGLE_AT_0 + (HOOD_ANGLE_AT_1 - HOOD_ANGLE_AT_0) * hood.physical);
        double turn = Math.toRadians(turretDeg - TURRET_START_DEG) * (Turret.ENCODER_CLOCKWISE_POSITIVE ? -1 : 1);
        double aim = heading + turn;

        double originX = x + TURRET_AXIS_X * Math.cos(heading);
        double originY = y + TURRET_AXIS_X * Math.sin(heading);
        double px = originX + BARREL_FORWARD * Math.cos(aim) - left * Math.sin(aim);
        double py = originY + BARREL_FORWARD * Math.sin(aim) + left * Math.cos(aim);

        double rim = flywheelRpm * FLYWHEEL_RATIO / 60 * Math.PI * FLYWHEEL_DIAMETER_M;
        double speed = rim * BALL_EXIT_FRACTION;
        double flat = speed * Math.cos(elevation);
        collisions.spawn(type, px / IN_PER_M, py / IN_PER_M, BARREL_HEIGHT / IN_PER_M,
                flat * Math.cos(aim) + collisions.vx(), flat * Math.sin(aim) + collisions.vy(), speed * Math.sin(elevation));
    }

    // back out of the intake mouth, rolling
    private void eject(int type) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double out = 1.5;
        collisions.spawn(type, (x + (MOUTH_X_MAX + 2) * c) / IN_PER_M, (y + (MOUTH_X_MAX + 2) * s) / IN_PER_M, 3 / IN_PER_M,
                out * c + collisions.vx(), out * s + collisions.vy(), 0);
    }

    /**
     * What the Limelight sees right now, each tag it can read as {id, tx, ty, ta}, biggest first.
     * A tag has to be in view, in range and facing the camera. A cell tipped down toward you has its
     * bottom face turned away, which is how you can tell not to shoot there. Things in the way aren't checked.
     */
    java.util.List<double[]> limelightTags() {
        if (limelightHidden) return new java.util.ArrayList<>();
        double aim = heading + Math.toRadians(turretDeg - TURRET_START_DEG) * (Turret.ENCODER_CLOCKWISE_POSITIVE ? -1 : 1);
        double camX = x + TURRET_AXIS_X * Math.cos(heading) + CAMERA_FORWARD * Math.cos(aim);
        double camY = y + TURRET_AXIS_X * Math.sin(heading) + CAMERA_FORWARD * Math.sin(aim);
        double tilt = Math.toRadians(org.firstinspires.ftc.teamcode.RobotConstants.Limelight.CAMERA_ANGLE_DEG);
        java.util.List<double[]> seen = new java.util.ArrayList<>();
        for (double[] tag : CELL_TAGS) for (int n = 0; n < TAG_OFFSETS_X.length; n++) {
            boolean red = tag[3] == 1;
            double[] m = hiveMirror(red);
            double hive = red ? collisions.redHive.angle : collisions.blueHive.angle;
            double c = Math.cos(hive), s = Math.sin(hive);
            double dy = tag[1] - 72, dz = tag[2] - HIVE_AXLE_IN;
            double tagY = 72 + dy * c - dz * s;
            double tagZ = HIVE_AXLE_IN + dy * s + dz * c;
            double normalY = m[0] * c - m[1] * s;
            double normalZ = m[0] * s + m[1] * c;

            double toX = tag[0] + TAG_OFFSETS_X[n] - camX, toY = tagY - camY, toZ = tagZ - CAMERA_HEIGHT;
            double range = Math.sqrt(toX * toX + toY * toY + toZ * toZ);
            double facing = -(toY * normalY + toZ * normalZ) / range;
            if (range > CAMERA_MAX_RANGE || facing < TAG_FACING_MIN) continue;

            double flat = Math.hypot(toX, toY);
            double tx = Math.toDegrees(wrap(aim - Math.atan2(toY, toX))); // right of center is positive
            double ty = Math.toDegrees(Math.atan2(toZ, flat) - tilt);
            if (Math.abs(tx) > CAMERA_HALF_FOV_X || Math.abs(ty) > CAMERA_HALF_FOV_Y) continue;

            double area = 100 * (3.25 * 3.25 * facing) / (range * range);
            seen.add(new double[]{tag[4] + n, tx, ty, area});
        }
        seen.sort((a, b) -> Double.compare(b[3], a[3]));
        return seen;
    }

    // which way the bottom of a Hive faces as staged (y, z): 30 deg off straight down, the red one toward
    // low y and the blue one toward high y, since they're tilted opposite ways
    private static double[] hiveMirror(boolean red) {
        return new double[]{red ? -0.5 : 0.5, -0.866};
    }

    /**
     * Drops a ball on the field, inches. For tests
     */
    void spawnBall(int type, double xIn, double yIn, double zIn) {
        collisions.spawn(type, xIn / IN_PER_M, yIn / IN_PER_M, zIn / IN_PER_M, 0, 0, 0);
    }

    private static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
