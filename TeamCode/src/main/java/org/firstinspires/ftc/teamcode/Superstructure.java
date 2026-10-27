package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

/**
 * The brain for the indexer, shooter and turret. The drivetrain, flower bar and blocker are
 * commanded directly by the OpMode or auto. The superstructure only READS the drivetrain's pose
 * and velocity (for aiming), it never sets its goals.
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
        SHOOT,
        OUTTAKE
    }

    /** What the robot is actually doing right now. */
    public enum SystemState {
        IDLE,
        INTAKING,
        SPINNING_UP,
        FEEDING,
        OUTTAKING
    }

    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final DrivetrainSubsystem drivetrain;

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;
    private Alliance alliance = Alliance.RED;
    private final ElapsedTime stateTimer = new ElapsedTime();

    /** Turret angle, rpm and both hood angles for the shot we would take right now. Recomputed every loop. */
    private ShotCalculator.Solution shot = new ShotCalculator.Solution(
            0, Constants.Shooter.TARGET_RPM,
            Constants.Shooter.DEFAULT_HOOD_ANGLE_DEG, Constants.Shooter.DEFAULT_HOOD_ANGLE_DEG);

    public Superstructure(IndexerSubsystem indexer, ShooterSubsystem shooter, TurretSubsystem turret,
                          DrivetrainSubsystem drivetrain) {
        this.indexer = indexer;
        this.shooter = shooter;
        this.turret = turret;
        this.drivetrain = drivetrain;
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

    /** Chooses which hive cell the turret aims at. Set it once during INIT. */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
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
        Pose predictedPose = predictPose(drivetrain.pose(), drivetrain.velocity(),
                Constants.Shooter.AIM_LOOKAHEAD_SECONDS);
        shot = ShotCalculator.calculate(predictedPose, drivetrain.velocity(), targetPose());

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
                // TODO: also wait for the turret to be on target (the hood servos have no feedback, so there is nothing to wait for there)
                return shooter.atSpeed() ? SystemState.FEEDING : SystemState.SPINNING_UP;
            case OUTTAKE:
                return SystemState.OUTTAKING;
            case IDLE:
            default:
                return SystemState.IDLE;
        }
    }

    /**
     * 2) Act: what every subsystem should do in each state.
     *
     * Every loop starts from safe "everything off" goals, then each state only overrides what it
     * needs. That way a state you forgot to fill in just sits still instead of running something.
     */
    private void applyGoals() {
        // Safe defaults
        indexer.setMotorGoal(IndexerSubsystem.MotorGoal.STOP);
        indexer.setGateGoal(IndexerSubsystem.GateGoal.CLOSED);
        // TODO: the intake pivot has to be DEPLOYED once the match starts (it starts STOWED so the
        //       robot fits in the 18 in. start cube). Decide where that gets set.
        shooter.setGoal(ShooterSubsystem.Goal.OFF, 0,
                Constants.Shooter.DEFAULT_HOOD_ANGLE_DEG, Constants.Shooter.DEFAULT_HOOD_ANGLE_DEG);
        turret.setGoal(TurretSubsystem.Goal.HOLD, turret.getAngleDeg());

        switch (systemState) {
            case INTAKING:
                indexer.setMotorGoal(IndexerSubsystem.MotorGoal.INTAKE);
                break;
            case SPINNING_UP:
                shooter.setGoal(ShooterSubsystem.Goal.SPIN_UP, shot.rpm, shot.hoodPollenAngleDeg, shot.hoodNectarAngleDeg);
                turret.setGoal(TurretSubsystem.Goal.AIM, shot.turretAngleDeg);
                break;
            case FEEDING:
                indexer.setMotorGoal(IndexerSubsystem.MotorGoal.FEED);
                indexer.setGateGoal(IndexerSubsystem.GateGoal.OPEN);
                shooter.setGoal(ShooterSubsystem.Goal.SPIN_UP, shot.rpm, shot.hoodPollenAngleDeg, shot.hoodNectarAngleDeg);
                turret.setGoal(TurretSubsystem.Goal.AIM, shot.turretAngleDeg);
                break;
            case OUTTAKING:
                indexer.setMotorGoal(IndexerSubsystem.MotorGoal.OUTTAKE);
                break;
            case IDLE:
            default:
                break;
        }
    }

    /**
     * @return where the robot will be {@code delaySeconds} from now, so the turret aims for the moment
     *         its command actually takes effect instead of using a stale pose. Pass the aim lookahead, not
     *         the feed delay (see {@code Constants.Shooter.AIM_LOOKAHEAD_SECONDS}).
     */
    private Pose predictPose(Pose pose, Velocity velocity, double delaySeconds) {
        // TODO: x += vx * delay, y += vy * delay, heading += omega * delay
        //       (check first whether Pedro's velocity is in the field frame or the robot frame)
        return pose;
    }

    /**
     * @return the hive cell to aim at: our alliance's hive, on whichever side of the field the robot
     *         is on. We trust that cell is the one facing up.
     */
    private Pose targetPose() {
        // TODO: pick the cell from the alliance and which side of the field drivetrain.pose() is on.
        //       The cell positions go in Constants. This also needs a real field coordinate frame:
        //       the robot's pose is currently relative to wherever it started.
        return Pose.zero();
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Wanted state", wantedState);
        telemetry.addData("System state", systemState);
    }
}
