package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Intake and indexer together, because they are belted to one motor. Moves balls from the floor
 * to the turret, holds up to 4 in a line, and kicks out the wrong color nectar.
 *
 * It has four things you can command, each with its own goal:
 * the motor, the gate (in front of the turret), the diverter, and the intake pivot.
 * It never decides anything. The superstructure sets the goals and reads the facts below.
 */
public class IndexerSubsystem implements Subsystem {

    public enum MotorGoal {
        STOP,
        INTAKE,
        FEED,
        OUTTAKE
    }

    public enum GateGoal {
        CLOSED,
        OPEN
    }

    public enum DiverterGoal {
        PASS,
        DIVERT
    }

    public enum PivotGoal {
        STOWED,
        DEPLOYED
    }

    private static final int SLOT_COUNT = Constants.Indexer.MAX_BALLS;

    private final DcMotorEx indexerMotor;
    private final Servo gateServo;
    private final Servo diverterServo;
    private final Servo intakePivotServo;

    /** One per slot, slot 0 is closest to the intake. Only their distance reading is used. */
    private final DistanceSensor[] slotSensors = new DistanceSensor[SLOT_COUNT];
    /** Sits before the diverter. Only its color reading is used. */
    private final NormalizedColorSensor nectarSensor;

    private MotorGoal motorGoal = MotorGoal.STOP;
    private GateGoal gateGoal = GateGoal.CLOSED;
    private DiverterGoal diverterGoal = DiverterGoal.PASS;
    private PivotGoal pivotGoal = PivotGoal.STOWED;

    // Inputs, filled in by readInputs()
    private final boolean[] slotHasBall = new boolean[SLOT_COUNT];
    private int ballCount;
    private boolean wrongNectarSeen;

    public IndexerSubsystem(HardwareMap hardwareMap) {
        indexerMotor = hardwareMap.get(DcMotorEx.class, "indexerMotor");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        diverterServo = hardwareMap.get(Servo.class, "diverterServo");
        intakePivotServo = hardwareMap.get(Servo.class, "intakePivotServo");

        for (int i = 0; i < SLOT_COUNT; i++) {
            slotSensors[i] = hardwareMap.get(DistanceSensor.class, "slotSensor" + (i + 1));
        }
        nectarSensor = hardwareMap.get(NormalizedColorSensor.class, "nectarSensor");

        // TODO: check this direction on the robot. The old intake motor was REVERSE.
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorGoal(MotorGoal goal) {
        this.motorGoal = goal;
    }

    public void setGateGoal(GateGoal goal) {
        this.gateGoal = goal;
    }

    public void setDiverterGoal(DiverterGoal goal) {
        this.diverterGoal = goal;
    }

    public void setPivotGoal(PivotGoal goal) {
        this.pivotGoal = goal;
    }

    /** @return how many slot sensors currently see a ball */
    public int ballCount() {
        return ballCount;
    }

    public boolean isFull() {
        return ballCount >= Constants.Indexer.MAX_BALLS;
    }

    /** @return true when the nectar sensor sees an opponent nectar */
    public boolean wrongNectarSeen() {
        return wrongNectarSeen;
    }

    @Override
    public void readInputs() {
        // TODO: for each slot, slotHasBall[i] = distance < Constants.Indexer.BALL_DETECT_DISTANCE_MM
        // TODO: ballCount = how many slotHasBall entries are true
        // TODO: wrongNectarSeen = the nectar sensor sees the OTHER alliance's color
        //       (this needs to know our alliance, so think about how it gets here)
    }

    @Override
    public void writeOutputs() {
        // TODO: motor power from motorGoal (powers are in Constants.Indexer)
        // TODO: gate position from gateGoal
        // TODO: diverter position from diverterGoal
        // TODO: intake pivot position from pivotGoal
        // Tip: only write when the value changes, like the old IntakeSubsystem did with lastPower.
    }

    @Override
    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Motor goal", motorGoal);
        telemetry.addData("Gate goal", gateGoal);
        telemetry.addData("Diverter goal", diverterGoal);
        telemetry.addData("Pivot goal", pivotGoal);
        telemetry.addData("Ball count", ballCount);
        telemetry.addData("Wrong nectar seen", wrongNectarSeen);
    }
}
