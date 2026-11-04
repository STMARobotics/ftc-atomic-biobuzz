package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeWheels;

/**
 * Both intake motors run on velocity control so idle actually turns at low speed.
 * Needs both encoder cables plugged in or that side runs away to full speed.
 */
public class IntakeWheelsSubsystem extends SubsystemBase {

    public enum Mode {
        INTAKE,
        OUTTAKE,
        IDLE
    }

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private Mode mode = Mode.IDLE;

    public IntakeWheelsSubsystem(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE_MOTOR_LEFT);
        rightMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE_MOTOR_RIGHT);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // TUNE: left turns clockwise (looking at the intake) to intake, flip these until both pull in
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMode(Mode newMode) {
        mode = newMode;
        switch (newMode) {
            case INTAKE:
                setPower(IntakeWheels.INTAKE_POWER);
                break;
            case OUTTAKE:
                setPower(IntakeWheels.OUTTAKE_POWER);
                break;
            case IDLE:
                setRPM(IntakeWheels.IDLE_RPM);
                break;
        }
    }

    // in RUN_USING_ENCODER this is a fraction of max speed, not raw power
    private void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    private void setRPM(double rpm) {
        double ticksPerSec = rpm * IntakeWheels.TICKS_PER_REV / 60.0;
        leftMotor.setVelocity(ticksPerSec);
        rightMotor.setVelocity(ticksPerSec);
    }

    private double getRPM(DcMotorEx motor) {
        return motor.getVelocity() / IntakeWheels.TICKS_PER_REV * 60.0;
    }

    public Mode getMode() {
        return mode;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Intake wheels", mode);
        telemetry.addData("Intake left RPM", "%.0f", getRPM(leftMotor));
        telemetry.addData("Intake right RPM", "%.0f", getRPM(rightMotor));
    }
}
