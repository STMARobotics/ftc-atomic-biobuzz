package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.Battery;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;

import java.util.function.DoubleSupplier;

/**
 * Two flywheel motors on one shaft. We do the velocity control ourselves off the first motor's
 * encoder and give both motors the exact same power so they never fight:
 *   power = (kS + kV * target) * (12V / battery) + kP * error
 * The feedforward gets it close and scales up as the battery drops, P cleans up the rest.
 *
 * It also counts shots: each ball pulls the rpm down and it recovers after.
 */
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx flywheelMotor;
    private final DcMotorEx flywheelMotor2;
    private final DoubleSupplier batteryVolts;

    private double targetRPM = 0;
    private double rpm = 0;
    private double power = 0;
    private double writtenPower = Double.NaN;

    // start from the constants, ShotTuner changes them live
    private double kS = Shooter.kS;
    private double kV = Shooter.kV;
    private double kP = Shooter.kP;

    // shot counting
    private boolean countingShots = false;
    private boolean dipping = false;
    private double peakRPM = 0;
    private double troughRPM = 0;
    private int shotsFired = 0;

    public ShooterSubsystem(HardwareMap hardwareMap, DoubleSupplier batteryVolts) {
        this.batteryVolts = batteryVolts;
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
        rpm = flywheelMotor.getVelocity() / Shooter.TICKS_PER_REV * 60.0;

        if (targetRPM <= 0) {
            power = 0;
        } else {
            double volts = batteryVolts.getAsDouble();
            double voltageScale = volts > 6 ? Battery.NOMINAL_VOLTS / volts : 1.0; // ignore junk readings
            double feedforward = (kS + kV * targetRPM) * voltageScale;
            power = Range.clip(feedforward + kP * (targetRPM - rpm), 0, 1);
        }

        if (power != writtenPower) {
            writtenPower = power;
            flywheelMotor.setPower(power);
            flywheelMotor2.setPower(power);
        }

        countShots();
    }

    // a shot is a dip of SHOT_DIP_RPM below the peak, counted once it climbs SHOT_RECOVER_RPM back up
    private void countShots() {
        if (!countingShots) {
            dipping = false;
            peakRPM = rpm;
            return;
        }
        if (!dipping) {
            peakRPM = Math.max(peakRPM, rpm);
            if (rpm < peakRPM - Shooter.SHOT_DIP_RPM) {
                dipping = true;
                troughRPM = rpm;
            }
        } else {
            troughRPM = Math.min(troughRPM, rpm);
            if (rpm > troughRPM + Shooter.SHOT_RECOVER_RPM) {
                shotsFired++;
                dipping = false;
                peakRPM = rpm;
            }
        }
    }

    /**
     * Sets the flywheel to a desired rpm (motor shaft rpm), takes effect on the next periodic()
     */
    public void setRPM(double RPM) {
        targetRPM = RPM;
    }

    /**
     * Cuts power and lets the flywheel coast down
     */
    public void stop() {
        targetRPM = 0;
    }

    /**
     * Only count while balls are actually being fed (gate open), otherwise spin up looks like shots
     */
    public void setCountingShots(boolean on) {
        countingShots = on;
    }

    public void resetShotCount() {
        shotsFired = 0;
    }

    public int getShotsFired() {
        return shotsFired;
    }

    public void setGains(double kS, double kV, double kP) {
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
    }

    public double getKS() {
        return kS;
    }

    public double getKV() {
        return kV;
    }

    public double getKP() {
        return kP;
    }

    public boolean isReady() {
        return targetRPM > 0 && Math.abs(rpm - targetRPM) <= Shooter.READY_TOLERANCE_RPM;
    }

    public double getRPM() {
        return rpm;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Flywheel RPM", "%.0f / %.0f", rpm, targetRPM);
        telemetry.addData("Flywheel power", "%.2f", power);
        telemetry.addData("Flywheel ready", isReady());
        telemetry.addData("Shots fired", shotsFired);
    }
}
