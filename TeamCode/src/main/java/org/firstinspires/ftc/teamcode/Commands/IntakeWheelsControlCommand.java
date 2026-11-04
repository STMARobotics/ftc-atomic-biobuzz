package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeWheelsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeWheelsSubsystem.Mode;

import java.util.function.BooleanSupplier;

/**
 * Default command for the intake wheels, the only thing that writes them. Outtake wins, then
 * feeding the shooter, then intake (not in battle mode), otherwise idle.
 */
public class IntakeWheelsControlCommand extends CommandBase {

    private final IntakeWheelsSubsystem wheelsSubsystem;
    private final BooleanSupplier outtakeHeld;
    private final BooleanSupplier feeding;
    private final BooleanSupplier intakeHeld;
    private final BooleanSupplier battleMode;

    public IntakeWheelsControlCommand(IntakeWheelsSubsystem wheelsSubsystem,
                                      BooleanSupplier outtakeHeld,
                                      BooleanSupplier feeding,
                                      BooleanSupplier intakeHeld,
                                      BooleanSupplier battleMode) {
        this.wheelsSubsystem = wheelsSubsystem;
        this.outtakeHeld = outtakeHeld;
        this.feeding = feeding;
        this.intakeHeld = intakeHeld;
        this.battleMode = battleMode;
        addRequirements(wheelsSubsystem);
    }

    @Override
    public void execute() {
        if (outtakeHeld.getAsBoolean()) {
            wheelsSubsystem.setMode(Mode.OUTTAKE);
        } else if (feeding.getAsBoolean()) {
            wheelsSubsystem.setMode(Mode.INTAKE); // feeding is the same speed as intaking
        } else if (intakeHeld.getAsBoolean() && !battleMode.getAsBoolean()) {
            wheelsSubsystem.setMode(Mode.INTAKE);
        } else {
            wheelsSubsystem.setMode(Mode.IDLE);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
