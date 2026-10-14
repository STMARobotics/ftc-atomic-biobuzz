package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

/**
 * The brain for everything that isn't the drivetrain.
 *
 * Drivers and autos only ever say what they WANT ({@link WantedState}). Every loop, update() looks
 * at that plus sensor facts (like "is the flywheel at speed?") to pick what the robot is actually
 * DOING ({@link SystemState}), then gives each subsystem its goal. All of the "when do we feed?"
 * logic lives in this one file instead of being spread across commands.
 */
public class Superstructure {

    /** What the driver or auto is asking for. */
    public enum WantedState {
        IDLE,
        INTAKE,
        SHOOT
    }

    /** What the robot is actually doing right now. */
    // note to self: intaking and feeding currently are the exact same thing
    public enum SystemState {
        IDLE,
        INTAKING,
        SPINNING_UP,
        FEEDING
    }

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    public Superstructure(IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    /**
     * @return seconds since the system state last changed
     */
    public double timeInState() {
        return stateTimer.seconds();
    }

    /**
     * Call once per loop, after inputs are read and before outputs are written.
     */
    public void update() {
        SystemState nextState = nextState();
        if (nextState != systemState) {
            systemState = nextState;
            stateTimer.reset();
        }
        applyGoals();
    }

    /** 1) Decide: transition rules. */
    private SystemState nextState() {
        switch (wantedState) {
            case INTAKE:
                return SystemState.INTAKING;
            case SHOOT:
                // Only feed while the flywheel is at speed. If an artifact drags the rpm down, this
                // drops back to SPINNING_UP and waits for the flywheel to recover before feeding again.
                return shooter.atSpeed() ? SystemState.FEEDING : SystemState.SPINNING_UP;
            case IDLE:
            default:
                return SystemState.IDLE;
        }
    }

    /** 2) Act: what every subsystem should do in each state. */
    private void applyGoals() {
        switch (systemState) {
            case INTAKING:
                intake.setGoal(IntakeSubsystem.Goal.INTAKE);
                shooter.setGoal(ShooterSubsystem.Goal.OFF, 0);
                break;
            case SPINNING_UP:
                intake.setGoal(IntakeSubsystem.Goal.STOP);
                shooter.setGoal(ShooterSubsystem.Goal.SPIN_UP, Constants.Shooter.TARGET_RPM);
                break;
            case FEEDING:
                intake.setGoal(IntakeSubsystem.Goal.FEED);
                shooter.setGoal(ShooterSubsystem.Goal.SPIN_UP, Constants.Shooter.TARGET_RPM);
                break;
            case IDLE:
            default:
                intake.setGoal(IntakeSubsystem.Goal.STOP);
                shooter.setGoal(ShooterSubsystem.Goal.OFF, 0);
                break;
        }
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Wanted state", wantedState);
        telemetry.addData("System state", systemState);
    }
}
