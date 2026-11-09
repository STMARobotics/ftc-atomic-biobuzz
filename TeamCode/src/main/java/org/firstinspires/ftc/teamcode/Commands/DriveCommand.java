package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Field centric teleop drive. Runs forever as the drivetrain's default command.
 *
 * Heading bias: with the turret doing the aiming nothing needs the robot's heading while it's just driving
 * around, and mecanum at 45 deg is far slower than square on. So it snaps to the nearest 90 when the left
 * stick is well over, you aren't touching the turn stick and biasAllowed says so. Any turn input wins at once.
 */
public class DriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final DoubleSupplier speedScale;
    private final BooleanSupplier biasAllowed;

    private double biasTargetDeg = Double.NaN;
    private long lastTurnNanos = 0;

    public DriveCommand(DrivetrainSubsystem drivetrain,
                        DoubleSupplier forward,
                        DoubleSupplier strafe,
                        DoubleSupplier turn) {
        this(drivetrain, forward, strafe, turn, () -> 1.0, () -> false);
    }

    /**
     * @param forward forward speed in [-1, 1], usually {@code -left_stick_y}
     * @param strafe left speed in [-1, 1], usually {@code -left_stick_x}
     * @param turn counterclockwise speed in [-1, 1], usually {@code -right_stick_x}
     * @param speedScale multiplier on all axes in [0, 1] for slow mode
     * @param biasAllowed false while it shouldn't ease to the nearest 90, like intaking or shooting
     */
    public DriveCommand(DrivetrainSubsystem drivetrain,
                        DoubleSupplier forward,
                        DoubleSupplier strafe,
                        DoubleSupplier turn,
                        DoubleSupplier speedScale,
                        BooleanSupplier biasAllowed) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.speedScale = speedScale;
        this.biasAllowed = biasAllowed;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double f = forward.getAsDouble(), s = strafe.getAsDouble(), t = turn.getAsDouble();
        double scale = speedScale.getAsDouble();

        long now = System.nanoTime();
        if (Math.abs(t) > 0.05) lastTurnNanos = now;
        boolean bias = Drive.HEADING_BIAS
                && biasAllowed.getAsBoolean()
                && Math.hypot(f, s) >= Drive.BIAS_MIN_STICK
                && (now - lastTurnNanos) / 1e9 >= Drive.BIAS_DELAY_SEC;
        if (!bias) {
            biasTargetDeg = Double.NaN;
            drivetrain.drive(f, s, t, scale);
            return;
        }

        double heading = Math.toDegrees(drivetrain.fieldHeading());
        // picked once when it starts, so being right around 45 doesn't flip it back and forth
        if (Double.isNaN(biasTargetDeg)) biasTargetDeg = Math.round(heading / 90) * 90;
        double off = ((heading - biasTargetDeg + 180) % 360 + 360) % 360 - 180;
        double power = Math.abs(off) <= Drive.BIAS_TOLERANCE_DEG ? 0
                : Math.max(-Drive.BIAS_MAX_TURN, Math.min(Drive.BIAS_MAX_TURN, -off * Drive.BIAS_kP));
        drivetrain.driveWithTurnPower(f, s, power, scale);
    }

    @Override
    public boolean isFinished() {
        return false; // Don't stop pls its kinda important
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
