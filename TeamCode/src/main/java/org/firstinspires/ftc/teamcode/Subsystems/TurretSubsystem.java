package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Rotating turret, driven by a 312 motor and zeroed by an absolute encoder.
 *
 * The superstructure picks the angle to aim at. This subsystem only turns to the angle it's given
 * and says whether it's there.
 */
public class TurretSubsystem implements Subsystem {

    public enum Goal {
        HOLD,
        AIM
    }

    private final DcMotorEx turretMotor;
    private final AnalogInput turretEncoder;

    private Goal goal = Goal.HOLD;
    private double targetAngleDeg;
    private double currentAngleDeg;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
    }

    /**
     * @param goal HOLD stays where it is, AIM turns toward targetAngleDeg
     * @param targetAngleDeg the angle to turn to, ignored when the goal is HOLD
     */
    public void setGoal(Goal goal, double targetAngleDeg) {
        this.goal = goal;
        this.targetAngleDeg = targetAngleDeg;
    }

    public double getAngleDeg() {
        return currentAngleDeg;
    }

    /** @return true when aiming and within tolerance of the target angle */
    public boolean atTarget() {
        return goal == Goal.AIM
                && Math.abs(currentAngleDeg - targetAngleDeg) <= Constants.Turret.TOLERANCE_DEG;
    }

    @Override
    public void readInputs() {
        // TODO: currentAngleDeg from turretEncoder.getVoltage() (max voltage is 3.3)
        //       this also has to account for the gear ratio and a zero offset
    }

    @Override
    public void writeOutputs() {
        // TODO: HOLD should keep the current angle, AIM should drive toward targetAngleDeg.
        //       That needs a controller (a P loop is a fine start) that outputs a motor power.
        turretMotor.setPower(0);
    }

    @Override
    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Turret goal", goal);
        telemetry.addData("Turret angle", currentAngleDeg);
        telemetry.addData("Turret target", targetAngleDeg);
        telemetry.addData("Turret on target", atTarget());
    }
}
