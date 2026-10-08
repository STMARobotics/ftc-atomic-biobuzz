package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

/**
 * Just spins the flywheel to rpm and runs indexer.
 */
public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.setRPM(4000);
        if (shooterSubsystem.flywheelReady()) {
            intakeSubsystem.runIntake();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        shooterSubsystem.stopShooter();
    }
}