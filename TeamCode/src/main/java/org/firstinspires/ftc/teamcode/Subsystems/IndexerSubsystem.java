package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Indexer.FEED_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Indexer.GATE_CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.Indexer.GATE_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.Indexer.IDLE_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Indexer.INTAKE_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Indexer.OUTTAKE_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Indexer.PIVOT_DEPLOYED_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.Indexer.PIVOT_IDLE_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.Indexer.PIVOT_STOWED_POSITION;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake and indexer together, because they are belted to one motor. Moves balls from the floor
 * to the turret.
 *
 * It has three things you can command, each with its own goal:
 * the motor, the gate (in front of the turret), and the intake pivot.
 * It never decides anything. The superstructure sets the goals and reads the facts below.
 */
public class IndexerSubsystem implements Subsystem {

    public enum MotorGoal {
        STOP,
        INTAKE,
        FEED,
        IDLE,
        OUTTAKE
    }

    public enum GateGoal {
        CLOSED,
        OPEN
    }

    public enum PivotGoal {
        STOWED,
        IDLE,
        DEPLOYED
    }

    private final DcMotorEx indexerMotor;
    private final Servo gateServo;
    private final Servo intakePivotServo;

    private MotorGoal motorGoal = MotorGoal.STOP;
    private GateGoal gateGoal = GateGoal.CLOSED;
    private PivotGoal pivotGoal = PivotGoal.STOWED;

    private double lastRPM = Double.NaN;
    private GateGoal lastGateGoal = null;
    private PivotGoal lastPivotGoal = null;

    public IndexerSubsystem(HardwareMap hardwareMap) {
        indexerMotor = hardwareMap.get(DcMotorEx.class, "indexerMotor");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        intakePivotServo = hardwareMap.get(Servo.class, "intakePivotServo");

        // TODO: check this direction on the robot. The old intake motor was REVERSE.
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorGoal(MotorGoal goal) {
        this.motorGoal = goal;
    }

    public void setGateGoal(GateGoal goal) {
        this.gateGoal = goal;
    }

    public void setPivotGoal(PivotGoal goal) {
        this.pivotGoal = goal;
    }

    @Override
    public void readInputs() {
        // Nothing to read
    }

    @Override
    public void writeOutputs() {
        double RPM;
        switch (motorGoal) {
            case INTAKE:
                RPM = INTAKE_RPM;
                break;
            case FEED:
                RPM = FEED_RPM;
                break;
            case IDLE:
                RPM = IDLE_RPM;
            case OUTTAKE:
                RPM = OUTTAKE_RPM;
                break;
            case STOP:
            default:
                RPM = 0;
                break;
        }
        if (RPM != lastRPM) {
            indexerMotor.setVelocity(RPM);
            lastRPM = RPM;
        }

        if (gateGoal != lastGateGoal) {
            gateServo.setPosition(gateGoal == GateGoal.OPEN
                    ? GATE_OPEN_POSITION
                    : GATE_CLOSED_POSITION);
            lastGateGoal = gateGoal;
        }

        if (pivotGoal != lastPivotGoal) {
            double position;
            switch (pivotGoal) {
                case DEPLOYED:
                    position = PIVOT_DEPLOYED_POSITION;
                    break;
                case IDLE:
                    position = PIVOT_IDLE_POSITION;
                    break;
                case STOWED:
                default:
                    position = PIVOT_STOWED_POSITION;
                    break;
            }
            intakePivotServo.setPosition(position);
            lastPivotGoal = pivotGoal;
        }
    }

    @Override
    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Motor goal", motorGoal);
        telemetry.addData("Gate goal", gateGoal);
        telemetry.addData("Pivot goal", pivotGoal);
    }
}
