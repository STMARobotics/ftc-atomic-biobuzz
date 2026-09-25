package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

/**
 * AAAA
 */
public class ShootAtTargetCommand extends CommandBase {

    // The subsystem the command runs on
//    private final GripperSubsystem m_gripperSubsystem;

    public ShootAtTargetCommand() {
//        m_gripperSubsystem = subsystem;
//        addRequirements(m_gripperSubsystem);
    }

    @Override
    public void initialize() {
//        m_gripperSubsystem.grab();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // m_gripperSubsystem.release();
    }

}