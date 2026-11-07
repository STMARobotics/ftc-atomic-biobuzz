package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterGateSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Spins the flywheel up and opens the gate once it's past GATE_OPEN_RPM and the aim is ready.
 * Doesn't touch the intake wheels, they feed while we're shooting and the gate is open.
 */
public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;
    private final ShooterGateSubsystem gateSubsystem;
    private final BooleanSupplier aimReady;
    private final DoubleSupplier rpm;

    /**
     * @param aimReady true when the turret is on target
     * @param rpm flywheel rpm from the shot table, NaN keeps the last one
     */
    public ShootCommand(ShooterSubsystem shooterSubsystem, ShooterGateSubsystem gateSubsystem,
                        BooleanSupplier aimReady, DoubleSupplier rpm) {
        this.shooterSubsystem = shooterSubsystem;
        this.gateSubsystem = gateSubsystem;
        this.aimReady = aimReady;
        this.rpm = rpm;
        addRequirements(shooterSubsystem, gateSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setRPM(Shooter.SHOOT_RPM);
    }

    @Override
    public void execute() {
        double targetRPM = rpm.getAsDouble();
        if (!Double.isNaN(targetRPM)) {
            shooterSubsystem.setRPM(targetRPM);
        }

        // once it opens it stays open until we let go, so the rpm dip from each ball doesn't stall feeding
        if (!gateSubsystem.isOpen()
                && shooterSubsystem.getRPM() >= Shooter.GATE_OPEN_RPM
                && aimReady.getAsBoolean()) {
            gateSubsystem.open();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        gateSubsystem.close();
        shooterSubsystem.stop();
    }
}
