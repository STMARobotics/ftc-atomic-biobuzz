package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FlowerSubsystem.State;
import org.firstinspires.ftc.teamcode.Subsystems.IntakePivotSubsystem;

import java.util.function.BooleanSupplier;

/**
 * Default command for the flower mech. It only deploys once the intake is down and settled,
 * and stows any time it isn't wanted.
 */
public class FlowerControlCommand extends CommandBase {

    private final FlowerSubsystem flowerSubsystem;
    private final IntakePivotSubsystem pivotSubsystem;
    private final BooleanSupplier flowerWanted;

    public FlowerControlCommand(FlowerSubsystem flowerSubsystem,
                                IntakePivotSubsystem pivotSubsystem,
                                BooleanSupplier flowerWanted) {
        this.flowerSubsystem = flowerSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.flowerWanted = flowerWanted;
        addRequirements(flowerSubsystem);
    }

    @Override
    public void execute() {
        if (flowerWanted.getAsBoolean() && pivotSubsystem.isDeployed()) {
            flowerSubsystem.setState(State.DEPLOYED);
        } else {
            flowerSubsystem.setState(State.STOWED);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
