package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.Turret;
import org.firstinspires.ftc.teamcode.util.Devices;

import java.util.function.DoubleSupplier;

/**
 * 540 degree turret. Angles are degrees from the absolute encoder, periodic() pids to the target.
 * If the tag is past a hardstop it unwinds 360 the other way, adding however much the robot
 * rotated so we still end up on the tag. Seeing a tag after 90 degrees of travel ends it early.
 *
 * Safety: an unplugged analog encoder reads ~0V, which looks exactly like a real 0 degrees, so
 * instead we watch for things that can't physically happen and shut the turret off (latched until
 * the OpMode restarts) when we see them:
 *   the angle jumps more than the motor could move it in one loop
 *   we're pushing but it isn't moving (dead encoder, jammed, cable caught)
 */
public class TurretSubsystem extends SubsystemBase {

    private final DcMotorEx turretMotor; // null if missing from the config
    private final AnalogInput yawEncoder; // null if missing from the config
    private final PIDController controller = new PIDController(Turret.kP, Turret.kI, Turret.kD);

    private final DoubleSupplier headingRad;

    // flips tx and the heading compensation if the encoder counts the other way
    private static final double DIR = Turret.ENCODER_CLOCKWISE_POSITIVE ? 1.0 : -1.0;

    private double targetAngle;
    private boolean unwinding = false;
    private double unwindTarget;
    private double headingAtUnwind;
    private double angleAtUnwind;

    private double writtenPower = Double.NaN;
    private String fault = null;
    private double lastAngle = Double.NaN;
    private long lastNanos = 0;
    private long pushingSinceNanos = 0;
    private double angleWhenPushStarted = 0;

    /**
     * @param headingRad robot heading in radians, counterclockwise positive (drivetrain::heading)
     */
    public TurretSubsystem(HardwareMap hardwareMap, DoubleSupplier headingRad) {
        this.headingRad = headingRad;
        turretMotor = Devices.optional(hardwareMap, DcMotorEx.class, HardwareNames.TURRET_MOTOR);
        yawEncoder = Devices.optional(hardwareMap, AnalogInput.class, HardwareNames.TURRET_ENCODER);
        if (turretMotor != null) turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (turretMotor == null || yawEncoder == null) fault = "missing from config";
        controller.setTolerance(Turret.TOLERANCE_DEG);
        targetAngle = getAngle(); // hold where we start instead of swinging to home
    }

    @Override
    public void periodic() {
        if (!Turret.ENABLED || fault != null) {
            setPower(0);
            return;
        }
        double angle = getAngle();
        if (unwinding) {
            double headingChange = Math.toDegrees(headingRad.getAsDouble() - headingAtUnwind);
            setTarget(unwindTarget + DIR * headingChange);
            if (isAtTarget()) {
                unwinding = false;
            }
        }
        double power = Range.clip(controller.calculate(angle, targetAngle), -Turret.MAX_POWER, Turret.MAX_POWER);
        checkHealth(angle, power);
        setPower(fault == null ? power : 0);
    }

    private void checkHealth(double angle, double power) {
        long now = System.nanoTime();
        if (!Double.isNaN(lastAngle) && lastNanos != 0) {
            double dt = (now - lastNanos) / 1e9;
            double possible = Turret.MAX_DEG_PER_SEC * dt + Turret.JUMP_SLACK_DEG;
            if (Math.abs(angle - lastAngle) > possible) {
                fault = String.format("encoder jumped %.0f deg", Math.abs(angle - lastAngle));
            }
        }
        lastAngle = angle;
        lastNanos = now;

        if (Math.abs(power) >= Turret.STALL_MIN_POWER) {
            if (pushingSinceNanos == 0) {
                pushingSinceNanos = now;
                angleWhenPushStarted = angle;
            } else if ((now - pushingSinceNanos) / 1e6 >= Turret.STALL_MS
                    && Math.abs(angle - angleWhenPushStarted) < Turret.STALL_MIN_TRAVEL_DEG) {
                fault = "not moving (encoder, jam or cable)";
            }
        } else {
            pushingSinceNanos = 0;
        }
    }

    private void setPower(double power) {
        if (turretMotor == null || power == writtenPower) return;
        writtenPower = power;
        turretMotor.setPower(power);
    }

    /**
     * Null when healthy, otherwise why the turret shut itself off
     */
    public String getFault() {
        return Turret.ENABLED ? fault : null;
    }

    /**
     * Aims at an angle, clamped to the hardstops. Cancels any unwind
     */
    public void aimAtAngle(double angleDeg) {
        unwinding = false;
        setTarget(angleDeg);
    }

    /**
     * Aims using the limelight tx, call this once per new camera frame (the target holds in between).
     * While unwinding it's ignored until we've moved far enough, then we take over from there.
     */
    public void aimAtTag(double xOffsetDeg) {
        if (unwinding) {
            if (Math.abs(getAngle() - angleAtUnwind) < Turret.UNWIND_MIN_TRAVEL_DEG) return;
            unwinding = false;
        }

        double desired = getAngle() + DIR * xOffsetDeg;
        double reachable = chooseAngle(desired);

        if (Math.abs(reachable - desired) > 1) {
            // tag is past a hardstop, whip around to the same direction the other way
            unwinding = true;
            unwindTarget = reachable;
            headingAtUnwind = headingRad.getAsDouble();
            angleAtUnwind = getAngle();
        }
        setTarget(reachable);
    }

    private void setTarget(double angleDeg) {
        targetAngle = Range.clip(angleDeg, Turret.MIN_ANGLE_DEG, Turret.MAX_ANGLE_DEG);
    }

    /**
     * Moves the target clockwise (negative is counterclockwise), clamped to the hardstops.
     * Cancels any unwind
     */
    public void nudgeClockwise(double deltaDeg) {
        aimAtAngle(targetAngle + DIR * deltaDeg);
    }

    public void aimForward() {
        aimAtAngle(Turret.FORWARD_ANGLE_DEG);
    }

    public void holdCurrentAngle() {
        aimAtAngle(getAngle());
    }

    /**
     * The turret has 540 degrees so most directions can be reached 2 ways (90 or 450).
     * Gives back the reachable one closest to where we are now, or the clamped angle if none fit.
     */
    public double chooseAngle(double desiredDeg) {
        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double candidate = desiredDeg + 360 * k;
            if (candidate < Turret.MIN_ANGLE_DEG || candidate > Turret.MAX_ANGLE_DEG) continue;
            if (Double.isNaN(best) || Math.abs(candidate - getAngle()) < Math.abs(best - getAngle())) {
                best = candidate;
            }
        }
        return Double.isNaN(best) ? Range.clip(desiredDeg, Turret.MIN_ANGLE_DEG, Turret.MAX_ANGLE_DEG) : best;
    }

    public boolean isUnwinding() {
        return unwinding;
    }

    public boolean isAtTarget() {
        return Math.abs(targetAngle - getAngle()) <= Turret.TOLERANCE_DEG;
    }

    public double getAngle() {
        if (yawEncoder == null) return Turret.FORWARD_ANGLE_DEG;
        return yawEncoder.getVoltage() * (Turret.ENCODER_RANGE_DEG / Turret.ENCODER_MAX_VOLTS);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void telemetrize(Telemetry telemetry) {
        if (yawEncoder != null) telemetry.addData("Encoder voltage", yawEncoder.getVoltage());
        telemetry.addData("Turret angle (deg)", "%.1f", getAngle());
        telemetry.addData("Turret target (deg)", "%.1f", targetAngle);
        telemetry.addData("Turret at target", isAtTarget());
        telemetry.addData("Turret unwinding", unwinding);
        if (getFault() != null) telemetry.addData("TURRET OFF", getFault());
    }
}
