package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.util.ShotLookupTable.Setting;

import java.util.function.DoubleSupplier;

/**
 * Default command for the hoods. Keeps both on the shot table for the current distance all the
 * time, shooting or not. With no distance (NaN) they stay where they were.
 */
public class HoodCommand extends CommandBase {

    private final HoodSubsystem hoodSubsystem;
    private final DoubleSupplier distance;

    public HoodCommand(HoodSubsystem hoodSubsystem, DoubleSupplier distance) {
        this.hoodSubsystem = hoodSubsystem;
        this.distance = distance;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void execute() {
        double d = distance.getAsDouble();
        if (Double.isNaN(d)) {
            return;
        }
        Setting setting = Shooter.SHOT_TABLE.get(d);
        hoodSubsystem.setPositions(setting.pollenHood, setting.nectarHood);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
