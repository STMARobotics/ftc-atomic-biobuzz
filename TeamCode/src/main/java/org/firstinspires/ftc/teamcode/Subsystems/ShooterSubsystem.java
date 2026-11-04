package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx flywheelMotor;
    private double targetRPM;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.FLYWHEEL_MOTOR);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Sets the flywheel to a desired rpm
     * @param RPM the rpm that we want the flywheel to run at
     */
    public void setRPM(double RPM) {
        flywheelMotor.setVelocity(RPM * Shooter.TICKS_PER_REV / 60.0);
        targetRPM = RPM;
    }

    /**
     * Sets the flywheel motor power to 0
     */
    public void stop() {
        flywheelMotor.setPower(0);
        targetRPM = 0;
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
        telemetry.addData("Flywheel RPM", getRPM());
        telemetry.addData("Target RPM", targetRPM);
    }
}