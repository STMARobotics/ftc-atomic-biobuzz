package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakePivot;

public class IntakePivotSubsystem extends SubsystemBase {

    public enum State {
        FULLY_STOWED,
        STOWED,
        DEPLOYED
    }

    private final Servo pivotServo;
    private State state = State.STOWED;
    private long lastChangeNanos = System.nanoTime() - IntakePivot.SETTLE_MS * 2_000_000L;

    public IntakePivotSubsystem(HardwareMap hardwareMap) {
        pivotServo = hardwareMap.get(Servo.class, HardwareNames.PIVOT_SERVO);
        setState(State.STOWED);
    }

    public void setState(State newState) {
        if (newState != state) {
            lastChangeNanos = System.nanoTime();
        }
        state = newState;
        switch (newState) {
            case FULLY_STOWED:
                pivotServo.setPosition(IntakePivot.FULLY_STOWED);
                break;
            case STOWED:
                pivotServo.setPosition(IntakePivot.STOWED);
                break;
            case DEPLOYED:
                pivotServo.setPosition(IntakePivot.DEPLOYED);
                break;
        }
    }

    public State getState() {
        return state;
    }

    /**
     * True once the servo has had time to finish its last move
     */
    public boolean isSettled() {
        return (System.nanoTime() - lastChangeNanos) / 1_000_000L >= IntakePivot.SETTLE_MS;
    }

    public boolean isDeployed() {
        return state == State.DEPLOYED && isSettled();
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Intake pivot", state);
    }
}
