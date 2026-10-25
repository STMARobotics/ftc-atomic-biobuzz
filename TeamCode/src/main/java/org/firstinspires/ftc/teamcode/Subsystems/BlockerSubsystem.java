package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Elevator that extends to block opponent shots (and maybe bounce ours into a flower).
 * Mind the size limit when it's out: 18 in. wide by 24 in. long by 29 in. tall (R105).
 *
 * Like the drivetrain and the flower bar, the driver commands this directly with a button. The
 * superstructure doesn't touch it.
 */
public class BlockerSubsystem implements Subsystem {

    public enum Goal {
        STOWED,
        EXTENDED
    }

    private final Servo blockerServo;

    private Goal goal = Goal.STOWED;

    public BlockerSubsystem(HardwareMap hardwareMap) {
        blockerServo = hardwareMap.get(Servo.class, "blockerServo");
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    public Goal getGoal() {
        return goal;
    }

    @Override
    public void readInputs() {
        // Servos can't report their position, so there's nothing to read.
    }

    @Override
    public void writeOutputs() {
        // TODO: servo position from goal (positions are in Constants.Blocker)
    }

    @Override
    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Blocker goal", goal);
    }
}
