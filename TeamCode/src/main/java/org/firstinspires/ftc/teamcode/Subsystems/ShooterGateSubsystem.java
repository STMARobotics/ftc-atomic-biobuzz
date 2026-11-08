package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.ShooterGate;
import org.firstinspires.ftc.teamcode.util.Devices;

/**
 * Gate that keeps balls out of the shooter until we're ready to shoot.
 */
public class ShooterGateSubsystem extends SubsystemBase {

    private final Servo gateServo;
    private boolean open = false;
    private double writtenPosition = Double.NaN;

    public ShooterGateSubsystem(HardwareMap hardwareMap) {
        gateServo = Devices.optional(hardwareMap, Servo.class, HardwareNames.GATE_SERVO);
        // no close() here, nothing can move in teleop init
    }

    public void open() {
        open = true;
        write(ShooterGate.OPEN);
    }

    public void close() {
        open = false;
        write(ShooterGate.CLOSED);
    }

    private void write(double position) {
        if (gateServo == null || position == writtenPosition) return;
        writtenPosition = position;
        gateServo.setPosition(position);
    }

    public boolean isOpen() {
        return open;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Gate", open ? "open" : "closed");
    }
}
