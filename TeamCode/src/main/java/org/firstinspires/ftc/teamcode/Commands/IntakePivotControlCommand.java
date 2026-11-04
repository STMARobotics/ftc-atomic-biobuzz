package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakePivotSubsystem.State;

import java.util.function.BooleanSupplier;

/**
 * Default command for the intake pivot. The flower mech wins (it needs the intake down, and the
 * intake has to wait for it to stow), then battle mode, then intake held, otherwise stowed.
 */
public class IntakePivotControlCommand extends CommandBase {

    private final IntakePivotSubsystem pivotSubsystem;
    private final BooleanSupplier battleMode;
    private final BooleanSupplier intakeHeld;
    private final BooleanSupplier flowerWanted;
    private final BooleanSupplier flowerStowed;

    public IntakePivotControlCommand(IntakePivotSubsystem pivotSubsystem,
                                     BooleanSupplier battleMode,
                                     BooleanSupplier intakeHeld,
                                     BooleanSupplier flowerWanted,
                                     BooleanSupplier flowerStowed) {
        this.pivotSubsystem = pivotSubsystem;
        this.battleMode = battleMode;
        this.intakeHeld = intakeHeld;
        this.flowerWanted = flowerWanted;
        this.flowerStowed = flowerStowed;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() {
        if (flowerWanted.getAsBoolean() || !flowerStowed.getAsBoolean()) {
            pivotSubsystem.setState(State.DEPLOYED);
        } else if (battleMode.getAsBoolean()) {
            pivotSubsystem.setState(State.FULLY_STOWED);
        } else if (intakeHeld.getAsBoolean()) {
            pivotSubsystem.setState(State.DEPLOYED);
        } else {
            pivotSubsystem.setState(State.STOWED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
