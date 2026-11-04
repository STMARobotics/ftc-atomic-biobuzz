package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;

/**
 * Keeps the turret on the tag, the turret handles hardstops itself. With no tag it just keeps
 * its last target, and with auto aim off it does nothing so home and manual aiming stick.
 */
public class AimCommand extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final BooleanSupplier autoAimEnabled;

    public AimCommand(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem,
                      BooleanSupplier autoAimEnabled) {
        this.turretSubsystem = turretSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.autoAimEnabled = autoAimEnabled;
        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        if (!autoAimEnabled.getAsBoolean()) {
            return;
        }

        // only use tx once per camera frame, reusing an old one makes the target run away
        if (limelightSubsystem.hasTarget() && limelightSubsystem.isNewFrame()) {
            turretSubsystem.aimAtTag(limelightSubsystem.getXOffset());
        }
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
