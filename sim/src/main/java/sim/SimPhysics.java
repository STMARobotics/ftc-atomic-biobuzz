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

    // ---- drivetrain: 312 rpm 5203 at 1:1, 104mm goBILDA mecanum, 36 lb ----
    static final double DRIVE_MAX_IN_PER_SEC = 312 / 60.0 * Math.PI * 4.094 * 0.85; // ~57 in/s loaded
    static final double STRAFE_EFFICIENCY = 0.8;
    static final double TURN_RADIUS_IN = 12.0; // (track width + wheelbase) / 2
    static final double DRIVE_TAU = 0.22; // seconds to reach ~63% of the target speed, it's heavy
    static final double BRAKE_TAU = 0.12;
    static final double COAST_TAU = 0.6;
    static final double ROBOT_HALF_SIZE_IN = 9;

    // ---- flywheel: 2x 6000rpm 5203, it tops out around 4000 motor rpm under all that drag ----
    static final double FLYWHEEL_TOP_RPM = 4150; // at full power on a 12.6V battery
    static final double FLYWHEEL_TAU = 0.45;
    static final double SHOT_RPM_LOSS = 350; // what one ball takes out of it
    static final double SHOT_MIN_RPM = 1500; // below this the ball doesn't make it out
    static final double FEED_SECONDS_PER_BALL = 0.3;

    // ---- intake: 312 rpm motor ----
    static final double INTAKE_MOTOR_RPM = 312;
    static final double INTAKE_TAU = 0.08;
    static final double PICKUP_SECONDS = 1.0; // toy model: one ball per second while intaking
    static final int MAX_BALLS = 4;
    static final int PRELOAD_BALLS = 3;

    // ---- turret: 312 rpm at 4:1 ----
    static final double TURRET_DEG_PER_SEC = 312 / 4.0 * 360 / 60.0;
    static final double TURRET_START_DEG = Turret.FORWARD_ANGLE_DEG;

    // ---- battery: running the intake pulls it down ~2V ----
    static final double BATTERY_REST_VOLTS = 12.7;
    static final double INTAKE_SAG_VOLTS = 2.0;
    static final double FLYWHEEL_SAG_VOLTS = 0.6;
    static final double DRIVE_SAG_VOLTS = 0.8;

    private final SimWorld world;

    // true robot pose on the field
    volatile double x = 72;
    volatile double y = 72;
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
    public volatile double flywheelRpm = 0;
    volatile double intakeRpm = 0;
    volatile double turretDeg = TURRET_START_DEG;

    public volatile int balls = PRELOAD_BALLS;
    public volatile int shotsLaunched = 0;
    private double feedTimer = 0;
    private double pickupTimer = 0;
    private double outtakeTimer = 0;

    SimPhysics(SimWorld world) {
        this.world = world;
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
        x = newX;
        y = newY;
        heading = newHeading;
        vForward = 0;
        vLeft = 0;
        vTurn = 0;
    }

    void resetRobot() {
        placeRobot(72, 72, 0);
        balls = PRELOAD_BALLS;
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

        // drivetrain, Pedro's mecanum: fl = f - s - t, fr = f + s + t, bl = f + s - t, br = f - s + t
        SimMotor fl = world.motor(Constants.drivetrainConfig.frontLeftName.get());
        SimMotor fr = world.motor(Constants.drivetrainConfig.frontRightName.get());
        SimMotor bl = world.motor(Constants.drivetrainConfig.backLeftName.get());
        SimMotor br = world.motor(Constants.drivetrainConfig.backRightName.get());
        double pfl = fl.mechanismPower() * voltScale;
        double pfr = fr.mechanismPower() * voltScale;
        double pbl = bl.mechanismPower() * voltScale;
        double pbr = br.mechanismPower() * voltScale;

        double forward = (pfl + pfr + pbl + pbr) / 4;
        double left = (-pfl + pfr + pbl - pbr) / 4;
        double ccw = (-pfl + pfr - pbl + pbr) / 4;

        boolean idle = Math.abs(pfl) + Math.abs(pfr) + Math.abs(pbl) + Math.abs(pbr) < 1e-3;
        double tau = !idle ? DRIVE_TAU
                : fl.zeroPowerBehavior == DcMotor.ZeroPowerBehavior.BRAKE ? BRAKE_TAU : COAST_TAU;
        double k = 1 - Math.exp(-dt / tau);
        vForward += (forward * DRIVE_MAX_IN_PER_SEC - vForward) * k;
        vLeft += (left * DRIVE_MAX_IN_PER_SEC * STRAFE_EFFICIENCY - vLeft) * k;
        vTurn += (ccw * DRIVE_MAX_IN_PER_SEC / TURN_RADIUS_IN - vTurn) * k;

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double nx = x + (vForward * cos - vLeft * sin) * dt;
        double ny = y + (vForward * sin + vLeft * cos) * dt;
        double lo = ROBOT_HALF_SIZE_IN;
        double hi = 144 - ROBOT_HALF_SIZE_IN;
        double cx = Range.clip(nx, lo, hi);
        double cy = Range.clip(ny, lo, hi);
        // the OTOS sees how far the robot actually moved, so pushing into a wall doesn't count
        double movedX = cx - x;
        double movedY = cy - y;
        double actualForward = dt > 0 ? (movedX * cos + movedY * sin) / dt : 0;
        double actualLeft = dt > 0 ? (-movedX * sin + movedY * cos) / dt : 0;
        if (cx != nx || cy != ny) {
            // stuck on the wall, it stops instead of building up speed
            vForward = actualForward;
            vLeft = actualLeft;
        }
        x = cx;
        y = cy;
        heading = wrap(heading + vTurn * dt);
        world.otos().integrate(actualForward, actualLeft, vTurn, dt);

        double wheelRpm = 312 * (vForward / DRIVE_MAX_IN_PER_SEC);
        fl.mechanismRpm = wheelRpm;
        fr.mechanismRpm = wheelRpm;
        bl.mechanismRpm = wheelRpm;
        br.mechanismRpm = wheelRpm;
        double drivePower = (Math.abs(pfl) + Math.abs(pfr) + Math.abs(pbl) + Math.abs(pbr)) / 4;

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
        turretDeg = Range.clip(turretDeg + TURRET_DEG_PER_SEC * turretPower * voltScale * dt,
                0, Turret.ENCODER_RANGE_DEG);
        turret.mechanismRpm = TURRET_DEG_PER_SEC * turretPower / 6.0 * 4;

        // servos
        for (SimServo servo : world.servos()) servo.step(dt);

        // battery
        double sag = INTAKE_SAG_VOLTS * Math.abs(intakePower)
                + FLYWHEEL_SAG_VOLTS * Math.abs(flyPower)
                + DRIVE_SAG_VOLTS * drivePower;
        batteryVolts += (batteryRestVolts - sag - batteryVolts) * (1 - Math.exp(-dt / 0.05));

        stepBalls(dt);
    }

    // toy game piece model, enough to see feeding, shots and the rpm dip
    private void stepBalls(double dt) {
        double feed = intakeRpm / INTAKE_MOTOR_RPM; // -1 to 1
        SimServo gate = world.servo(HardwareNames.GATE_SERVO);
        SimServo pivot = world.servo(HardwareNames.PIVOT_SERVO);
        boolean gateOpen = Math.abs(gate.physical - ShooterGate.OPEN) < 0.15;
        boolean pivotDown = Math.abs(pivot.physical - IntakePivot.DEPLOYED) < 0.15;

        if (gateOpen && feed > 0.3 && balls > 0) {
            feedTimer += dt * feed;
            if (feedTimer >= FEED_SECONDS_PER_BALL && jamNextBall) {
                jamNextBall = false;
                jammed = true;
            }
            if (feedTimer >= FEED_SECONDS_PER_BALL && !jammed) {
                feedTimer = 0;
                balls--;
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

        if (pivotDown && !gateOpen && feed > 0.3 && balls < MAX_BALLS) {
            pickupTimer += dt;
            if (pickupTimer >= PICKUP_SECONDS) {
                pickupTimer = 0;
                balls++;
            }
        } else {
            pickupTimer = 0;
        }

        // a short reverse frees a stuck ball
        reverseTimer = feed < -0.2 ? reverseTimer + dt : 0;
        if (jammed && reverseTimer >= 0.08) {
            jammed = false;
            feedTimer = 0;
        }

        if (feed < -0.3 && balls > 0) {
            outtakeTimer += dt;
            if (outtakeTimer >= 0.4) {
                outtakeTimer = 0;
                balls--;
            }
        } else {
            outtakeTimer = 0;
        }
    }

    private static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
