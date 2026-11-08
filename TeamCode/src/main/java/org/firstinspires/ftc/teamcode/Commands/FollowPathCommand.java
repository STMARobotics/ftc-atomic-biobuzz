package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;

/**
 * Follows a Pedro path and ends when the follower is done. The drivetrain's periodic() updates
 * the follower, this just starts it and waits.
 */
public class FollowPathCommand extends CommandBase {

    private final DrivetrainSubsystem drive;
    private final Path path;

    public FollowPathCommand(DrivetrainSubsystem drive, Path path) {
        this.drive = drive;
        this.path = path;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followPath(path);
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) drive.stop();
    }
}
