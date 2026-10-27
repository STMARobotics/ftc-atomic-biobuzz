package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Turret.ENCODER_DIRECTION;
import static org.firstinspires.ftc.teamcode.Constants.Turret.ENCODER_FORWARD_DEG;
import static org.firstinspires.ftc.teamcode.Constants.Turret.KP;
import static org.firstinspires.ftc.teamcode.Constants.Turret.MAX_POWER;
import static org.firstinspires.ftc.teamcode.Constants.Turret.SOFT_LIMIT_MARGIN_DEG;
import static org.firstinspires.ftc.teamcode.Constants.Turret.TOLERANCE_DEG;
import static org.firstinspires.ftc.teamcode.Constants.Turret.TRAVEL_DEG;
import static org.firstinspires.ftc.teamcode.Constants.Turret.TURRET_DEG_PER_ENCODER_DEG;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Rotating turret, driven by a 312 motor. Its angle comes only from an absolute encoder geared so one
 * encoder turn covers the turret's whole 540 degrees of travel. The motor's own encoder is not used.
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
    /** Where HOLD keeps the turret. Taken from the real angle when HOLD starts, so it never jumps. */
    private double holdAngleDeg;
    private boolean holdAngleSet = false;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
    }

    /**
     * @param goal HOLD stays where it is, AIM turns toward targetAngleDeg
     * @param targetAngleDeg the angle to turn to, ignored when the goal is HOLD
     */
    public void setGoal(Goal goal, double targetAngleDeg) {
        if (goal == Goal.HOLD && this.goal != Goal.HOLD) {
            holdAngleDeg = currentAngleDeg;
        }
        this.goal = goal;
        this.targetAngleDeg = targetAngleDeg;
    }

    public double getAngleDeg() {
        return currentAngleDeg;
    }

    /** @return true when aiming and within tolerance of the target angle */
    public boolean atTarget() {
        return goal == Goal.AIM
                && Math.abs(currentAngleDeg - targetAngleDeg) <= TOLERANCE_DEG;
    }

    @Override
    public void readInputs() {
        // 0 to max volts is one encoder turn. Measure it from straight forward, wrap it into +/-180 so the
        // seam lands at the turret's limits, then scale up by the gearing to get turret degrees.
        double encoderDeg = turretEncoder.getVoltage() / turretEncoder.getMaxVoltage() * 360.0;
        double fromForward = ENCODER_DIRECTION * (encoderDeg - ENCODER_FORWARD_DEG);
        currentAngleDeg = wrapTo180(fromForward) * TURRET_DEG_PER_ENCODER_DEG;
        if (!holdAngleSet) {
            holdAngleDeg = currentAngleDeg;
            holdAngleSet = true;
        }
    }

    @Override
    public void writeOutputs() {
        // The turret has hard limits and an absolute angle, so there's no wrapping here: the error is
        // just target minus current. Wherever the aim comes from, keep it inside the soft limits.
        double limit = TRAVEL_DEG / 2.0 - SOFT_LIMIT_MARGIN_DEG;
        double target = Range.clip(goal == Goal.AIM ? targetAngleDeg : holdAngleDeg, -limit, limit);

        // A simple P loop: power in proportion to how far off we are, capped. If the turret runs away
        // from the target instead of toward it, flip the motor direction in the constructor.
        double power = Range.clip(KP * (target - currentAngleDeg), -MAX_POWER, MAX_POWER);
        turretMotor.setPower(power);
    }

    /** @return the angle wrapped into [-180, 180) */
    private static double wrapTo180(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    @Override
    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Turret goal", goal);
        telemetry.addData("Turret angle", currentAngleDeg);
        telemetry.addData("Turret target", targetAngleDeg);
        telemetry.addData("Turret on target", atTarget());
    }
}
