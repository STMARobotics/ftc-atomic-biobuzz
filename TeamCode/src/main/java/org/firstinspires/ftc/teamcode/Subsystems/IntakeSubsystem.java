package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private double targetRPM;
    private double targetTPS;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runIntake() {
//        targetRPM = 290;
//        targetTPS = (targetRPM * 28) / 60;
//        intakeMotor.setVelocity(targetTPS);
        intakeMotor.setPower(1);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}