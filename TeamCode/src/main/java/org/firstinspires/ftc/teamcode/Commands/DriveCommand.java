package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Field centric teleop drive. Runs forever as the drivetrain's default command.
 */
public class DriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final DoubleSupplier speedScale;

    public DriveCommand(DrivetrainSubsystem drivetrain,
                        DoubleSupplier forward,
                        DoubleSupplier strafe,
                        DoubleSupplier turn) {
        this(drivetrain, forward, strafe, turn, () -> 1.0);
    }

    /**
     * @param forward forward speed in [-1, 1], usually {@code -left_stick_y}
     * @param strafe left speed in [-1, 1], usually {@code -left_stick_x}
     * @param turn counterclockwise speed in [-1, 1], usually {@code -right_stick_x}
     * @param speedScale multiplier on all axes in [0, 1] for slow mode
     */
    public DriveCommand(DrivetrainSubsystem drivetrain,
                        DoubleSupplier forward,
                        DoubleSupplier strafe,
                        DoubleSupplier turn,
                        DoubleSupplier speedScale) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.speedScale = speedScale;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(
                forward.getAsDouble(),
                strafe.getAsDouble(),
                turn.getAsDouble(),
                speedScale.getAsDouble());
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