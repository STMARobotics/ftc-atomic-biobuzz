package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx flywheelMotor1;
    private final DcMotorEx flywheelMotor2;
    private double targetRPM;
    private double targetTPS;
    private double currentRPM;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheelMotor1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheelMotor2");

        flywheelMotor1.setVelocityPIDFCoefficients(1000, 0, 0, 100);
        flywheelMotor2.setVelocityPIDFCoefficients(1000, 0, 0, 100);
    }

    /**
     * Sets the flywheel to a desired rpm
     * @param RPM the rpm that we want the flywheel to run at
     */
    public void setRPM(double RPM) {
        targetRPM = RPM;
        targetTPS = (targetRPM * 28) / 60;
        flywheelMotor1.setPower(1);
        flywheelMotor2.setPower(1);
    }

    /**
     * Sets the flywheel motor power to 0
     */
    public void stopShooter() {
        flywheelMotor1.setPower(0);
        flywheelMotor2.setPower(0);
        targetRPM = 0;
    }

    /**
     * Returns t/f if the flywheel is ready for shooting
     */
    public boolean flywheelReady() {
        return Math.abs(getRPM() - targetRPM) <= 200;
    }

    public double getRPM() {
        currentRPM = (flywheelMotor1.getVelocity() / 28) * 60;
        return currentRPM;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Flywheel RPM", getRPM());
        telemetry.addData("Target RPM", targetRPM);
    }
}