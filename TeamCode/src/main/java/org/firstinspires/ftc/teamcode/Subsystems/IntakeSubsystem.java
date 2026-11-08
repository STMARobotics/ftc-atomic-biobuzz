package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants.Debug;
import org.firstinspires.ftc.teamcode.RobotConstants.Flower;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakePivot;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeWheels;
import org.firstinspires.ftc.teamcode.util.Devices;

/**
 * Intake pivot, intake wheels and the flower bar in one subsystem, because they share hardware:
 * the wheels also feed the shooter, and the intake is the flower's hardstop.
 *
 * Commands don't move anything directly. They turn requests on and off (intaking, feeding,
 * outtaking, flower) and periodic() works out what the hardware does:
 *   wheels: outtake beats feeding beats intaking, otherwise stopped
 *   pivot:  down while intaking or using the flower, fully stowed otherwise (battle mode)
 *   flower: out only once the pivot has had time to get down
 * So holding LT while shooting doesn't cancel anything, both just ask for what they need.
 *
 * Nothing moves until periodic() runs, and the scheduler doesn't run during teleop init.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final Servo pivotServo; // null when IntakePivot.ENABLED is false, then it stays limp
    private final Servo flowerServo;
    private final DcMotorEx wheelMotor;

    private boolean intakeRequested = false;
    private boolean feedRequested = false;
    private boolean outtakeRequested = false;
    private boolean unjamRequested = false;
    private boolean flowerRequested = false;

    private double wheelPower = 0; // ramped power the motor is actually getting
    private long lastNanos = 0;

    private boolean pivotDown = false;
    private long pivotDownSinceNanos = 0;
    private boolean flowerOut = false;

    // last values sent to the hardware, so we only write when something changes
    private double writtenPivot = Double.NaN;
    private double writtenFlower = Double.NaN;
    private double writtenWheels = Double.NaN;

    // any of these can be missing from the config, then that part just doesn't run (see Devices)
    public IntakeSubsystem(HardwareMap hardwareMap) {
        pivotServo = IntakePivot.ENABLED
                ? Devices.optional(hardwareMap, Servo.class, HardwareNames.PIVOT_SERVO)
                : null;
        flowerServo = Devices.optional(hardwareMap, Servo.class, HardwareNames.FLOWER_SERVO);

        wheelMotor = Devices.optional(hardwareMap, DcMotorEx.class, HardwareNames.INTAKE_MOTOR_RIGHT);
        if (wheelMotor != null) {
            wheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // TUNE: flip this until the wheels pull balls in
            wheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        // no servo positions here, nothing can move in teleop init. Auto calls stowNow() itself
    }

    @Override
    public void periodic() {
        long now = System.nanoTime();
        double dt = lastNanos == 0 ? 0 : (now - lastNanos) / 1e9;
        lastNanos = now;

        // pivot
        boolean wantDown = intakeRequested || (Flower.ENABLED && flowerRequested);
        if (wantDown && !pivotDown) pivotDownSinceNanos = now;
        pivotDown = wantDown;
        writePivot(pivotDown ? IntakePivot.DEPLOYED : IntakePivot.FULLY_STOWED);

        // flower, the intake is its hardstop so wait for the pivot to actually get down
        boolean pivotThere = pivotDown && (now - pivotDownSinceNanos) / 1e6 >= IntakePivot.TRAVEL_MS;
        flowerOut = Flower.ENABLED && flowerRequested && pivotThere;
        writeFlower(flowerOut ? Flower.DEPLOYED : Flower.STOWED);

        // wheels
        double target;
        if (outtakeRequested) target = IntakeWheels.OUTTAKE_POWER;
        else if (unjamRequested) target = IntakeWheels.UNJAM_POWER;
        else if (feedRequested) target = IntakeWheels.FEED_POWER;
        else if (intakeRequested) target = IntakeWheels.INTAKE_POWER;
        else target = 0;

        // only speeding up is ramped (that's the current spike). Stopping, slowing down and
        // reversing are instant, otherwise a quick unjam pulse would never actually reverse
        if (target == 0 || Math.signum(target) != Math.signum(wheelPower)) wheelPower = 0;
        if (Math.abs(target) <= Math.abs(wheelPower)) {
            wheelPower = target;
        } else {
            double maxStep = IntakeWheels.RAMP_PER_SEC * dt;
            wheelPower += Range.clip(target - wheelPower, -maxStep, maxStep);
        }
        writeWheels(wheelPower);
    }

    // requests, commands turn these on when they start and off when they end
    public void setIntaking(boolean on) {
        intakeRequested = on;
    }

    public void setFeeding(boolean on) {
        feedRequested = on;
    }

    public void setOuttaking(boolean on) {
        outtakeRequested = on;
    }

    /**
     * ShootCommand pulses this to back a stuck ball out, beats feeding but not the driver's outtake
     */
    public void setUnjamming(boolean on) {
        unjamRequested = on;
    }

    public void setFlowerWanted(boolean on) {
        flowerRequested = on;
    }

    public boolean isFlowerWanted() {
        return flowerRequested;
    }

    /**
     * Battle mode: drops every request so everything stows and the wheels stop.
     */
    public void clearRequests() {
        intakeRequested = false;
        feedRequested = false;
        outtakeRequested = false;
        unjamRequested = false;
        flowerRequested = false;
    }

    /**
     * Moves the pivot to fully stowed right now, for auto init only (teleop init can't move).
     */
    public void stowNow() {
        writePivot(IntakePivot.FULLY_STOWED);
    }

    public boolean isPivotDown() {
        return pivotDown;
    }

    public double getWheelPower() {
        return wheelPower;
    }

    private void writePivot(double position) {
        if (pivotServo == null || position == writtenPivot) return;
        writtenPivot = position;
        pivotServo.setPosition(position);
    }

    private void writeFlower(double position) {
        if (flowerServo == null || position == writtenFlower) return;
        writtenFlower = position;
        flowerServo.setPosition(position);
    }

    private void writeWheels(double power) {
        if (wheelMotor == null || power == writtenWheels) return;
        writtenWheels = power;
        wheelMotor.setPower(power);
    }

    public double getWheelRPM() {
        if (wheelMotor == null) return 0;
        return wheelMotor.getVelocity() / IntakeWheels.TICKS_PER_REV * 60.0;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Intake pivot", pivotDown ? "down" : "stowed");
        telemetry.addData("Intake power", "%.2f", wheelPower);
        telemetry.addData("Intake RPM", "%.0f", getWheelRPM());
        if (Debug.EXTRA_TELEMETRY && wheelMotor != null) {
            telemetry.addData("Intake current (A)", "%.1f", wheelMotor.getCurrent(CurrentUnit.AMPS));
        }
        telemetry.addData("Flower", flowerOut ? "out" : "stowed");
    }
}
