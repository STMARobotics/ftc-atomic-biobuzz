package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterGateSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Spins the flywheel up, opens the gate once it's close to speed and the aim is ready, then feeds
 * balls with the intake wheels until it ends. Counts shots so auto can stop after N.
 * Runs until cancelled (teleop: let go of RT). For auto use .interruptOn(() -> shots >= 3).
 */
public class ShootCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final ShooterGateSubsystem gate;
    private final IntakeSubsystem intake;
    private final BooleanSupplier aimReady;
    private final DoubleSupplier rpm;

    /**
     * @param aimReady true when the turret is on target (or we're aiming by hand)
     * @param rpm flywheel rpm from the shot table, NaN keeps the last one
     */
    public ShootCommand(ShooterSubsystem shooter, ShooterGateSubsystem gate, IntakeSubsystem intake,
                        BooleanSupplier aimReady, DoubleSupplier rpm) {
        this.shooter = shooter;
        this.gate = gate;
        this.intake = intake;
        this.aimReady = aimReady;
        this.rpm = rpm;
        // not the intake: it takes a feed request instead, so you can intake while shooting
        addRequirements(shooter, gate);
    }

    @Override
    public void initialize() {
        double target = rpm.getAsDouble();
        shooter.setRPM(Double.isNaN(target) ? Shooter.SHOOT_RPM : target);
        shooter.resetShotCount();
        gate.close();
    }

    @Override
    public void execute() {
        double target = rpm.getAsDouble();
        if (!Double.isNaN(target)) {
            shooter.setRPM(target);
        }

        // once it opens it stays open until we let go, so the rpm dip from each ball doesn't stall feeding
        if (!gate.isOpen()
                && shooter.getRPM() >= shooter.getTargetRPM() - Shooter.GATE_OPEN_MARGIN_RPM
                && aimReady.getAsBoolean()) {
            gate.open();
        }

        intake.setFeeding(gate.isOpen());
        shooter.setCountingShots(gate.isOpen());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setFeeding(false);
        shooter.setCountingShots(false);
        gate.close();
        shooter.stop();
    }
}
