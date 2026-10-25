package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Two motor flywheel plus an adjustable hood. The feed wheel is belted to the flywheel motors, so it
 * has no hardware of its own here.
 */
public class ShooterSubsystem implements Subsystem {

    public enum Goal {
        OFF,
        SPIN_UP
    }

    private final DcMotorEx flywheelMotor1;
    private final DcMotorEx flywheelMotor2;
    private final Servo hoodServo;
    /** The Axon servo's analog feedback wire, which tells us where the hood really is. */
    private final AnalogInput hoodEncoder;

    private Goal goal = Goal.OFF;
    private double targetRPM;
    private double currentRPM;
    private double targetHoodAngleDeg;
    private double currentHoodAngleDeg;
    private double lastCommandedRPM = Double.NaN;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodEncoder = hardwareMap.get(AnalogInput.class, "hoodEncoder");

        // setVelocity only works in RUN_USING_ENCODER, and needs an encoder on each motor
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor1.setVelocityPIDFCoefficients(Constants.Shooter.VELOCITY_P, Constants.Shooter.VELOCITY_I,
                Constants.Shooter.VELOCITY_D, Constants.Shooter.VELOCITY_F);
        flywheelMotor2.setVelocityPIDFCoefficients(Constants.Shooter.VELOCITY_P, Constants.Shooter.VELOCITY_I,
                Constants.Shooter.VELOCITY_D, Constants.Shooter.VELOCITY_F);
    }

    /**
     * @param goal what the flywheel should do
     * @param targetRPM the rpm that counts as "at speed", ignored when the goal is OFF
     * @param hoodAngleDeg the angle to put the hood at, ignored when the goal is OFF
     */
    public void setGoal(Goal goal, double targetRPM, double hoodAngleDeg) {
        this.goal = goal;
        // autocomplete is LOCKED IN today
        this.targetRPM = goal == Goal.OFF ? 0 : targetRPM;
        this.targetHoodAngleDeg = hoodAngleDeg;
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

    /**
     * @return true when the hood is within tolerance of the angle it was told to go to
     */
    public boolean hoodAtTarget() {
        return Math.abs(currentHoodAngleDeg - targetHoodAngleDeg) <= Constants.Shooter.HOOD_TOLERANCE_DEG;
    }

    public double getRPM() {
        return currentRPM;
    }

    @Override
    public void readInputs() {
        currentRPM = (flywheelMotor1.getVelocity() / Constants.Shooter.TICKS_PER_REV) * 60;
        // TODO: currentHoodAngleDeg from hoodEncoder.getVoltage() (max voltage is 3.3)
        //       this also has to account for the hood's linkage and a zero offset
    }

    @Override
    public void writeOutputs() {
        // Closed loop: the hub holds the flywheel at targetRPM using the velocity PIDF set above
        double commandedRPM = goal == Goal.SPIN_UP ? targetRPM : 0;

        // Only write when the command changes, since every hardware write costs loop time
        if (commandedRPM != lastCommandedRPM) {
            if (commandedRPM == 0) {
                flywheelMotor1.setPower(0);
                flywheelMotor2.setPower(0);
            } else {
                double ticksPerSecond = commandedRPM * Constants.Shooter.TICKS_PER_REV / 60;
                flywheelMotor1.setVelocity(ticksPerSecond);
                flywheelMotor2.setVelocity(ticksPerSecond);
            }
            lastCommandedRPM = commandedRPM;
        }

        // TODO: hoodServo.setPosition(...) from targetHoodAngleDeg. Needs the hood's angle -> servo position map.
    }

    @Override
    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Shooter goal", goal);
        telemetry.addData("Flywheel RPM", currentRPM);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("At speed", atSpeed());
        telemetry.addData("Hood angle", currentHoodAngleDeg);
        telemetry.addData("Hood target", targetHoodAngleDeg);
        telemetry.addData("Hood at target", hoodAtTarget());
    }
}
