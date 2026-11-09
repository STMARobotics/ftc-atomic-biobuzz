package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants.Limelight;
import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;
import org.firstinspires.ftc.teamcode.RobotConstants.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.ShotSolver;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Keeps the turret on the tag, including leading the shot while driving (see ShotSolver), the turret
 * handles hardstops itself. If the tag drops out it keeps pointing the same way on the field using
 * the gyro, so it's already looking at the right place when the tag comes back. If it stays gone it
 * sweeps around looking for one, unless the gate is open (no spraying balls around) or the camera is
 * dead. With auto aim off it does nothing.
 */
public class AimCommand extends CommandBase {

    // a camera frame shows the past, so it has to be matched with where the turret and robot were then
    private static final int HISTORY = 64;
    private final double[] historySec = new double[HISTORY];
    private final double[] historyTurret = new double[HISTORY];
    private final double[] historyHeading = new double[HISTORY];
    private int historyNext = 0;
    private int historyCount = 0;

    private final TurretSubsystem turretSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final ShotSolver solver;
    private final DoubleSupplier headingRad;
    private final BooleanSupplier autoAimEnabled;
    private final BooleanSupplier searchAllowed;

    private boolean searching = false;
    private double sweepCenter, sweepOffset, sweepDirection, sweepLimit;
    private double lastAimSec = Double.NaN;

    private ShotSolver.Aim aim = null;
    private double lastDesired = Double.NaN;
    private double lastSec = 0;
    private double rate = 0;

    public AimCommand(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, ShotSolver solver,
                      DoubleSupplier headingRad, BooleanSupplier autoAimEnabled, BooleanSupplier searchAllowed) {
        this.turretSubsystem = turretSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.solver = solver;
        this.headingRad = headingRad;
        this.autoAimEnabled = autoAimEnabled;
        this.searchAllowed = searchAllowed;
        addRequirements(turretSubsystem);
    }

    /**
     * The current aim, null when we don't have a target
     */
    public ShotSolver.Aim getAim() {
        return aim;
    }

    public boolean isSearching() {
        return searching;
    }

    @Override
    public void execute() {
        if (!autoAimEnabled.getAsBoolean()) {
            return;
        }

        double now = System.nanoTime() / 1e9;
        double heading = Math.toDegrees(headingRad.getAsDouble());
        double bearingNow = turretSubsystem.getBearingDeg();
        remember(now, bearingNow, unwrapHeading(heading));

        // only use a camera frame once, reusing an old one makes the target run away
        if (limelightSubsystem.hasTarget() && limelightSubsystem.isNewFrame()) {
            double frameSec = now - limelightSubsystem.getLatencyMs() / 1000;
            int i = lookup(frameSec);
            // a bad camera angle can leave no distance, aiming still works without a good one
            double distance = limelightSubsystem.getDistance();
            if (Double.isNaN(distance)) distance = Shooter.FIXED_DISTANCE_IN;
            solver.addFrame(limelightSubsystem.getXOffset(), Math.min(distance, Limelight.MAX_TARGET_DISTANCE_IN),
                    historyTurret[i], historyHeading[i], frameSec);
        }

        aim = solver.solve(now, unwrapHeading(heading), Shooter.LEAD_SHOTS);
        if (Double.isNaN(lastAimSec) || aim != null) lastAimSec = now;
        double bearing;
        if (aim != null) {
            searching = false;
            bearing = aim.turretBearingDeg;
        } else {
            double field = whereToLook(now, bearingNow + unwrapHeading(heading));
            if (Double.isNaN(field)) return; // never seen a tag and not looking, stay put
            bearing = field - unwrapHeading(heading);
        }

        // relative to where it is now the long way around doesn't matter, the turret picks the nearest
        double desired = turretSubsystem.getAngle() + TurretSubsystem.toAngleChange(wrap(bearing - bearingNow));
        if (!Double.isNaN(lastDesired) && now > lastSec) {
            double raw = wrap(desired - lastDesired) / (now - lastSec);
            rate += (raw - rate) * Math.min(1, (now - lastSec) / Turret.RATE_FILTER_SEC);
        }
        lastDesired = desired;
        lastSec = now;
        turretSubsystem.track(desired, rate);
    }

    /**
     * The direction to point with no tag, in the gyro's frame, NaN for nowhere in particular.
     * Where the tag last was, then sweeps either side of that, wider each time
     */
    private double whereToLook(double now, double pointingNow) {
        double last = solver.lastBearingDeg();
        boolean mayLook = Turret.SEARCH_FOR_TAGS && limelightSubsystem.isAlive();
        if (!mayLook || now - lastAimSec < Turret.SEARCH_AFTER_SEC) {
            searching = false;
            return last;
        }
        if (!searching) {
            searching = true;
            sweepCenter = Double.isNaN(last) ? pointingNow : last;
            sweepOffset = 0;
            sweepDirection = 1;
            sweepLimit = Turret.SEARCH_FIRST_SWEEP_DEG;
        } else if (searchAllowed.getAsBoolean()) {
            sweepOffset += sweepDirection * Turret.SEARCH_DEG_PER_SEC * (now - lastSec);
            if (Math.abs(sweepOffset) >= sweepLimit) {
                sweepOffset = sweepDirection * sweepLimit;
                sweepDirection = -sweepDirection;
                sweepLimit = Math.min(180, sweepLimit + Turret.SEARCH_WIDER_DEG);
            }
        }
        return sweepCenter + sweepOffset;
    }

    // heading wraps at +-180 but the frame has to stay continuous across a whole spin
    private double unwrappedHeading = Double.NaN;

    private double unwrapHeading(double heading) {
        if (Double.isNaN(unwrappedHeading)) unwrappedHeading = heading;
        unwrappedHeading += wrap(heading - unwrappedHeading);
        return unwrappedHeading;
    }

    private void remember(double now, double turret, double heading) {
        historySec[historyNext] = now;
        historyTurret[historyNext] = turret;
        historyHeading[historyNext] = heading;
        historyNext = (historyNext + 1) % HISTORY;
        historyCount = Math.min(historyCount + 1, HISTORY);
    }

    // the newest sample that isn't after this time, or the oldest we have
    private int lookup(double sec) {
        int newest = (historyNext - 1 + HISTORY) % HISTORY;
        for (int n = 0; n < historyCount; n++) {
            int i = (newest - n + HISTORY) % HISTORY;
            if (historySec[i] <= sec) return i;
        }
        return (historyNext - historyCount + HISTORY) % HISTORY;
    }

    private static double wrap(double deg) {
        return ((deg + 180) % 360 + 360) % 360 - 180;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.holdCurrentAngle();
    }
}
