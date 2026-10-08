package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IntakeSubsystem extends SubsystemBase {

    private double targetRPM;
    private final DcMotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
    }

    public void runIntake() {
        intakeMotor.setVelocity((double) 250 /28*60);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}