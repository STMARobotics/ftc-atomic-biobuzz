package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Shooter.HOOD_MAX_ANGLE_DEG;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.HOOD_MIN_ANGLE_DEG;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.HOOD_NECTAR_POSITION_AT_MAX;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.HOOD_NECTAR_POSITION_AT_MIN;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.HOOD_POLLEN_POSITION_AT_MAX;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.HOOD_POLLEN_POSITION_AT_MIN;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.READY_TOLERANCE_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.VELOCITY_D;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.VELOCITY_F;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.VELOCITY_I;
import static org.firstinspires.ftc.teamcode.Constants.Shooter.VELOCITY_P;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Two motor flywheel on one shaft feeding two barrels, each with its own hood servo (one for pollen, one
 * for nectar). The hood servos have no feedback, so we can command them but not check where they are.
 * The feed wheel is belted to the flywheel motors, so it has no hardware of its own here.
 */
public class ShooterSubsystem implements Subsystem {

    public enum Goal {
        OFF,
        SPIN_UP
    }

    private final DcMotorEx flywheelMotor1;
    private final DcMotorEx flywheelMotor2;
    private final Servo hoodPollen;
    private final Servo hoodNectar;

    private Goal goal = Goal.OFF;
    private double targetRPM;
    private double currentRPM;
    private double targetHoodPollenDeg;
    private double targetHoodNectarDeg;
    private double lastCommandedRPM = Double.NaN;
    private double lastHoodPollenPosition = Double.NaN;
    private double lastHoodNectarPosition = Double.NaN;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");
        hoodPollen = hardwareMap.get(Servo.class, "hoodPollen");
        hoodNectar = hardwareMap.get(Servo.class, "hoodNectar");

        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor1.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I,
                VELOCITY_D, VELOCITY_F);
        flywheelMotor2.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I,
                VELOCITY_D, VELOCITY_F);
    }

    /**
     * @param goal what the flywheel should do
     * @param targetRPM the rpm that counts as "at speed", ignored when the goal is OFF
     * @param hoodPollenDeg the launch angle for the pollen hood
     * @param hoodNectarDeg the launch angle for the nectar hood
     */
    public void setGoal(Goal goal, double targetRPM, double hoodPollenDeg, double hoodNectarDeg) {
        this.goal = goal;
        this.targetRPM = goal == Goal.OFF ? 0 : targetRPM;
        this.targetHoodPollenDeg = hoodPollenDeg;
        this.targetHoodNectarDeg = hoodNectarDeg;
    }

    public Goal getGoal() {
        return goal;
    }

    /**
     * @return true when spinning up and within tolerance of the target rpm
     */
    public boolean atSpeed() {
        return goal == Goal.SPIN_UP
                && Math.abs(currentRPM - targetRPM) <= READY_TOLERANCE_RPM;
    }

    public double getRPM() {
        return currentRPM;
    }

    @Override
    public void readInputs() {
        currentRPM = (flywheelMotor1.getVelocity() / TICKS_PER_REV) * 60;
    }

    @Override
    public void writeOutputs() {
        double commandedRPM = goal == Goal.SPIN_UP ? targetRPM : 0;

        if (commandedRPM != lastCommandedRPM) {
            if (commandedRPM == 0) {
                flywheelMotor1.setPower(0);
                flywheelMotor2.setPower(0);
            } else {
                double ticksPerSecond = commandedRPM * TICKS_PER_REV / 60;
                flywheelMotor1.setVelocity(ticksPerSecond);
                flywheelMotor2.setVelocity(ticksPerSecond);
            }
            lastCommandedRPM = commandedRPM;
        }

        double pollenPosition = hoodPosition(targetHoodPollenDeg,
                HOOD_POLLEN_POSITION_AT_MIN, HOOD_POLLEN_POSITION_AT_MAX);
        if (pollenPosition != lastHoodPollenPosition) {
            hoodPollen.setPosition(pollenPosition);
            lastHoodPollenPosition = pollenPosition;
        }
        double nectarPosition = hoodPosition(targetHoodNectarDeg,
                HOOD_NECTAR_POSITION_AT_MIN, HOOD_NECTAR_POSITION_AT_MAX);
        if (nectarPosition != lastHoodNectarPosition) {
            hoodNectar.setPosition(nectarPosition);
            lastHoodNectarPosition = nectarPosition;
        }
    }

    /** A straight line from the hood's min and max angles to the servo positions measured at each. */
    private static double hoodPosition(double angleDeg, double positionAtMin, double positionAtMax) {
        double fraction = (angleDeg - HOOD_MIN_ANGLE_DEG)
                / (HOOD_MAX_ANGLE_DEG - HOOD_MIN_ANGLE_DEG);
        fraction = Math.max(0.0, Math.min(1.0, fraction));
        return positionAtMin + fraction * (positionAtMax - positionAtMin);
    }

    @Override
    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Shooter goal", goal);
        telemetry.addData("Flywheel RPM", currentRPM);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("At speed", atSpeed());
        telemetry.addData("Hood pollen target", targetHoodPollenDeg);
        telemetry.addData("Hood nectar target", targetHoodNectarDeg);
    }
}
