package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.ShooterGate;

/**
 * Gate that keeps balls out of the shooter until we're ready to shoot.
 */
public class ShooterGateSubsystem extends SubsystemBase {

    private final Servo gateServo;
    private boolean open;

    public ShooterGateSubsystem(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, HardwareNames.GATE_SERVO);
        // no close() here, nothing can move in teleop init
    }

    public void open() {
        open = true;
        gateServo.setPosition(ShooterGate.OPEN);
    }

    public void close() {
        open = false;
        gateServo.setPosition(ShooterGate.CLOSED);
    }

    public boolean isOpen() {
        return open;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Shooter gate open", open);
    }
}
