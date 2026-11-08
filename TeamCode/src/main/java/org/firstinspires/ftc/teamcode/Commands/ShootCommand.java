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
 *
 * When things go wrong it keeps shooting instead of sitting there:
 *   can't reach speed (low battery, drag): after SPINUP_TIMEOUT_MS it opens at FALLBACK_MIN_FRACTION
 *   no flywheel encoder: full power, opens after BLIND_SPINUP_MS
 *   jam (feeding but no shot for JAM_TIMEOUT_MS): backs the balls out for UNJAM_MS, MAX_UNJAMS tries
 */
public class ShootCommand extends CommandBase {

    public enum Status { IDLE, SPINNING_UP, SHOOTING, SHOOTING_LOW_RPM, SHOOTING_NO_ENCODER, UNJAMMING, EMPTY }

    private final ShooterSubsystem shooter;
    private final ShooterGateSubsystem gate;
    private final IntakeSubsystem intake;
    private final BooleanSupplier aimReady;
    private final DoubleSupplier rpm;

    private long startNanos;
    private long lastProgressNanos;
    private long unjamUntilNanos;
    private int unjams;
    private int lastShots;
    private Status status = Status.IDLE;

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
        startNanos = System.nanoTime();
        unjamUntilNanos = 0;
        unjams = 0;
        lastShots = 0;
        status = Status.SPINNING_UP;
    }

    @Override
    public void execute() {
        long now = System.nanoTime();
        double target = rpm.getAsDouble();
        if (!Double.isNaN(target)) {
            shooter.setRPM(target);
        }

        // once it opens it stays open until we let go, so the rpm dip from each ball doesn't stall feeding
        if (!gate.isOpen() && aimReady.getAsBoolean()) {
            double elapsedMs = (now - startNanos) / 1e6;
            double goal = shooter.getTargetRPM();
            if (!shooter.canMeasureRpm()) {
                if (elapsedMs >= Shooter.BLIND_SPINUP_MS) open(now, Status.SHOOTING_NO_ENCODER);
            } else if (shooter.getRPM() >= goal - Shooter.GATE_OPEN_MARGIN_RPM) {
                open(now, Status.SHOOTING);
            } else if (elapsedMs >= Shooter.SPINUP_TIMEOUT_MS
                    && shooter.getRPM() >= goal * Shooter.FALLBACK_MIN_FRACTION) {
                open(now, Status.SHOOTING_LOW_RPM);
            }
        }

        if (gate.isOpen()) checkJam(now);

        boolean unjamming = now < unjamUntilNanos;
        intake.setUnjamming(unjamming);
        intake.setFeeding(gate.isOpen() && !unjamming);
        shooter.setCountingShots(gate.isOpen() && !unjamming);
    }

    private void open(long now, Status why) {
        gate.open();
        lastProgressNanos = now;
        status = why;
    }

    private void checkJam(long now) {
        int shots = shooter.getShotsFired();
        if (shots != lastShots) {
            // a ball went through, all good
            lastShots = shots;
            lastProgressNanos = now;
            unjams = 0;
            if (status == Status.UNJAMMING || status == Status.EMPTY) status = Status.SHOOTING;
            return;
        }
        if (now < unjamUntilNanos) return;
        if (status == Status.UNJAMMING) status = Status.SHOOTING;
        if (!Shooter.JAM_DETECTION || !shooter.canMeasureRpm()) return;
        if ((now - lastProgressNanos) / 1e6 < Shooter.JAM_TIMEOUT_MS) return;

        if (unjams < Shooter.MAX_UNJAMS) {
            unjams++;
            unjamUntilNanos = now + (long) (Shooter.UNJAM_MS * 1e6);
            lastProgressNanos = unjamUntilNanos; // give it a full timeout after the pulse
            status = Status.UNJAMMING;
        } else {
            status = Status.EMPTY; // tried, nothing came out, probably out of balls
        }
    }

    public Status getStatus() {
        return isScheduled() ? status : Status.IDLE;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setFeeding(false);
        intake.setUnjamming(false);
        shooter.setCountingShots(false);
        gate.close();
        shooter.stop();
        status = Status.IDLE;
    }
}
