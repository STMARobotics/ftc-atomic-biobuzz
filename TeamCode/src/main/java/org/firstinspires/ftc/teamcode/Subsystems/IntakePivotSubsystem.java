package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakePivot;

public class IntakePivotSubsystem extends SubsystemBase {

    private final Servo pivotServo;
    private double position = IntakePivot.FULLY_STOWED; // auto leaves it stowed

    // doesn't move the servo, nothing can move in teleop init. Auto calls fullyStow() itself
    // null when IntakePivot.ENABLED is false, then the servo never gets a position and stays limp
    public IntakePivotSubsystem(HardwareMap hardwareMap) {
        pivotServo = IntakePivot.ENABLED
                ? hardwareMap.get(Servo.class, HardwareNames.PIVOT_SERVO)
                : null;
    }

    public void deploy() {
        setPosition(IntakePivot.DEPLOYED);
    }

    // battle mode
    public void fullyStow() {
        setPosition(IntakePivot.FULLY_STOWED);
    }

    private void setPosition(double newPosition) {
        position = newPosition;
        if (pivotServo != null) pivotServo.setPosition(newPosition);
    }

    public boolean isDeployed() {
        return position == IntakePivot.DEPLOYED;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Intake pivot", "%.2f", position);
    }
}
