package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Intake roller. The same motor also pushes artifacts into the flywheel, so it doubles as the feeder.
 */
public class IntakeSubsystem implements Subsystem {

    public enum Goal {
        STOP,
        INTAKE,
        FEED
    }

    private final DcMotorEx intakeMotor;

    private Goal goal = Goal.STOP;
    private double lastPower = Double.NaN;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    public Goal getGoal() {
        return goal;
    }

    @Override
    public void readInputs() {
        // No intake sensors yet. A beam break or color sensor would be read here.
    }

    @Override
    public void writeOutputs() {
        double power;
        switch (goal) {
            case INTAKE:
                power = Constants.Intake.INTAKE_POWER;
                break;
            case FEED:
                power = Constants.Intake.FEED_POWER;
                break;
            case STOP:
            default:
                power = 0;
                break;
        }

        // Every hardware write costs loop time, so only write when the power actually changes
        if (power != lastPower) {
            intakeMotor.setPower(power);
            lastPower = power;
        }
    }

    @Override
    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Intake goal", goal);
    }
}
