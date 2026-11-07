package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;

/**
 * Two flywheel motors on one shaft. We do the velocity control ourselves (feedforward + P) off
 * the first motor's encoder and give both motors the exact same power so they never fight.
 */
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx flywheelMotor;
    private final DcMotorEx flywheelMotor2;
    private double targetRPM;
    private double power;

    // start from the constants, ShotTuner changes them live
    private double kV = Shooter.kV;
    private double kP = Shooter.kP;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.FLYWHEEL_MOTOR);
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, HardwareNames.FLYWHEEL_MOTOR_2);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // we only need the encoder for reading, the power is ours
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void periodic() {
        update();
    }

    /**
     * Sets the flywheel to a desired rpm (motor shaft rpm)
     */
    public void setRPM(double RPM) {
        targetRPM = RPM;
        update();
    }

    /**
     * Cuts power and lets the flywheel coast down
     */
    public void stop() {
        targetRPM = 0;
        update();
    }

    // feedforward gets it close, P cleans up the rest and pushes harder when a ball pulls it down
    private void update() {
        if (targetRPM <= 0) {
            power = 0;
        } else {
            double error = targetRPM - getRPM();
            power = Range.clip(kV * targetRPM + kP * error, 0, 1);
        }
        flywheelMotor.setPower(power);
        flywheelMotor2.setPower(power);
    }

    public void setGains(double kV, double kP) {
        this.kV = kV;
        this.kP = kP;
    }

    public double getKV() {
        return kV;
    }

    public double getKP() {
        return kP;
    }

    /**
     * Returns t/f if the flywheel is ready for shooting
     */
    public boolean flywheelReady() {
        return Math.abs(getRPM() - targetRPM) <= Shooter.READY_TOLERANCE_RPM;
    }

    public double getRPM() {
        return flywheelMotor.getVelocity() / Shooter.TICKS_PER_REV * 60.0;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Flywheel RPM", "%.0f", getRPM());
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("Flywheel power", "%.2f", power);
    }
}
