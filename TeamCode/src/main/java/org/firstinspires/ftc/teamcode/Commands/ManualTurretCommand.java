package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

/**
 * Turns the turret by hand while it runs, takes the turret from AimCommand until it ends.
 */
public class ManualTurretCommand extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private final double direction;
    private long lastNanos;

    /**
     * @param clockwise true to turn clockwise, false for counterclockwise
     */
    public ManualTurretCommand(TurretSubsystem turretSubsystem, boolean clockwise) {
        this.turretSubsystem = turretSubsystem;
        this.direction = clockwise ? 1.0 : -1.0;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        lastNanos = System.nanoTime();
    }

    @Override
    public void execute() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;
        turretSubsystem.nudgeClockwise(direction * Turret.MANUAL_SPEED_DEG_PER_SEC * dt);
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
