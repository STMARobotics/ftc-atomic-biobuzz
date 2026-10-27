package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.FlowerBar.DEPLOYED_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.FlowerBar.STOWED_POSITION;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The little bar that drops so we can intake from a flower.
 *
 * Like the drivetrain and the blocker, the driver commands this directly with a button. The
 * superstructure doesn't touch it.
 */
public class FlowerBarSubsystem implements Subsystem {

    public enum Goal {
        STOWED,
        DEPLOYED
    }

    private final Servo barServo;

    private Goal goal = Goal.STOWED;

    public FlowerBarSubsystem(HardwareMap hardwareMap) {
        barServo = hardwareMap.get(Servo.class, "flowerBarServo");
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    public Goal getGoal() {
        return goal;
    }

    @Override
    public void readInputs() {
        // Nothing to read
    }

    @Override
    public void writeOutputs() {
        // TODO: servo position from goal (positions are in Constants.FlowerBar)
        if (goal == Goal.DEPLOYED) {
            barServo.setPosition(DEPLOYED_POSITION);
        } else {
            barServo.setPosition(STOWED_POSITION);
        }
    }

    @Override
    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Flower bar goal", goal);
    }
}
