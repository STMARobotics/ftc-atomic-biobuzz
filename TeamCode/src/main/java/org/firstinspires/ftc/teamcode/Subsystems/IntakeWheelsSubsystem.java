package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeWheels;

public class IntakeWheelsSubsystem extends SubsystemBase {

    private final DcMotorEx rightMotor;

    public IntakeWheelsSubsystem(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE_MOTOR_RIGHT);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // TUNE: left turns clockwise (looking at the intake) to intake, flip these until both pull in
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // also used for feeding the shooter, same speed
    public void intake() {
        setPower(IntakeWheels.INTAKE_POWER);
    }

    public void outtake() {
        setPower(IntakeWheels.OUTTAKE_POWER);
    }

    public void stop() {
        setPower(0);
    }

    private void setPower(double power) {
        rightMotor.setPower(power);
    }

    private double getRPM(DcMotorEx motor) {
        return motor.getVelocity() / IntakeWheels.TICKS_PER_REV * 60.0;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Intake right RPM", "%.0f", getRPM(rightMotor));
    }
}
