package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.Flower;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;

/**
 * Flower mech. The intake is its hardstop so it can only be deployed while the intake is down.
 */
public class FlowerSubsystem extends SubsystemBase {

    private final Servo flowerServo;
    private boolean deployed = false;

    public FlowerSubsystem(HardwareMap hardwareMap) {
        flowerServo = hardwareMap.get(Servo.class, HardwareNames.FLOWER_SERVO);
        stow();
    }

    public void deploy() {
        deployed = true;
        flowerServo.setPosition(Flower.DEPLOYED);
    }

    public void stow() {
        deployed = false;
        flowerServo.setPosition(Flower.STOWED);
    }

    public boolean isDeployed() {
        return deployed;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Flower deployed", deployed);
    }
}
