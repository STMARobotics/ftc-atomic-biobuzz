package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Two motor flywheel.
 */
public class ShooterSubsystem implements Subsystem {

    public enum Goal {
        OFF,
        SPIN_UP
    }

    private final DcMotorEx flywheelMotor1;
    private final DcMotorEx flywheelMotor2;

    private Goal goal = Goal.OFF;
    private double targetRPM;
    private double currentRPM;
    private double lastPower = Double.NaN;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");

        flywheelMotor1.setVelocityPIDFCoefficients(1000, 0, 0, 100);
        flywheelMotor2.setVelocityPIDFCoefficients(1000, 0, 0, 100);
    }

    /**
     * @param goal what the flywheel should do
     * @param targetRPM the rpm that counts as "at speed", ignored when the goal is OFF
     */
    public void setGoal(Goal goal, double targetRPM) {
        this.goal = goal;
        // autocomplete is LOCKED IN today
        this.targetRPM = goal == Goal.OFF ? 0 : targetRPM;
    }

    public Goal getGoal() {
        return goal;
    }

    /**
     * @return true when spinning up and within tolerance of the target rpm
     */
    public boolean atSpeed() {
        return goal == Goal.SPIN_UP
                && Math.abs(currentRPM - targetRPM) <= Constants.Shooter.READY_TOLERANCE_RPM;
    }

    public double getRPM() {
        return currentRPM;
    }

    @Override
    public void readInputs() {
        currentRPM = (flywheelMotor1.getVelocity() / Constants.Shooter.TICKS_PER_REV) * 60;
    }

    @Override
    public void writeOutputs() {
        // This runs open loop at a fixed power, exactly like protobot did, and targetRPM is only used
        // to decide when to feed. To make the flywheel actually hold targetRPM, tune the PIDF above and
        // replace setPower with setVelocity(targetRPM * Constants.Shooter.TICKS_PER_REV / 60).
        double power = goal == Goal.SPIN_UP ? Constants.Shooter.SPIN_UP_POWER : 0;

        if (power != lastPower) {
            flywheelMotor1.setPower(power);
            flywheelMotor2.setPower(power);
            lastPower = power;
        }
    }

    @Override
    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Shooter goal", goal);
        telemetry.addData("Flywheel RPM", currentRPM);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("At speed", atSpeed());
    }
}
