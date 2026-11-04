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

    public enum State {
        STOWED,
        DEPLOYED
    }

    private final Servo flowerServo;
    private State state = State.STOWED;
    private long lastChangeNanos = System.nanoTime() - Flower.SETTLE_MS * 2_000_000L;

    public FlowerSubsystem(HardwareMap hardwareMap) {
        flowerServo = hardwareMap.get(Servo.class, HardwareNames.FLOWER_SERVO);
        flowerServo.setPosition(Flower.STOWED);
    }

    public void setState(State newState) {
        if (newState != state) {
            lastChangeNanos = System.nanoTime();
        }
        state = newState;
        flowerServo.setPosition(newState == State.DEPLOYED ? Flower.DEPLOYED : Flower.STOWED);
    }

    public State getState() {
        return state;
    }

    /**
     * True once the servo has had time to finish its last move
     */
    public boolean isSettled() {
        return (System.nanoTime() - lastChangeNanos) / 1_000_000L >= Flower.SETTLE_MS;
    }

    public boolean isStowed() {
        return state == State.STOWED && isSettled();
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Flower", state);
    }
}
