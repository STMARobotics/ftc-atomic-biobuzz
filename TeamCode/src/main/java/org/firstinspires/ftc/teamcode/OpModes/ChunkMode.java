package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.AimCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.RobotConstants.Drive;
import org.firstinspires.ftc.teamcode.RobotConstants.Flower;
import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.ShotLookupTable;
import org.firstinspires.ftc.teamcode.util.ShotSolver;

import java.util.List;

/**
 * Controls (gamepad 1):
 *   sticks          drive (left translate, right turn), field centric
 *   left trigger    intake (hold), the intake is down while held and fully stowed otherwise
 *   B               outtake (hold)
 *   right trigger   shoot (hold), gate opens once the flywheel is up to speed, then it feeds
 *   right bumper    gate override toggle (holds the gate open)
 *   X               battle mode: cancels shooting and everything else, fully stows
 *   A               flower toggle, the intake goes down for it (only when Flower.ENABLED)
 *   dpad up/down    pollen hood +/- 0.02
 *   dpad right/left nectar hood +/- 0.02
 *   start           reset field centric
 *
 * The turret tracks our AprilTags by itself, leading the shot while driving (AimCommand, ShotSolver),
 * and the flywheel and hoods follow the shot table. Dpad nudges are on top of the table.
 * If it can't see a tag it looks for one on its own (see AimCommand).
 * During INIT: B picks red, X picks blue, so it only tracks our Hive. It remembers, and auto picks it too.
 *
 * Rumble: one short buzz when the gate opens (shooting), three blips when it thinks it's empty.
 * If auto ran just before, the robot keeps auto's pose and field centric is already set up.
 */
@TeleOp(name = "ChunkMode", group = "TeleOp")
public class ChunkMode extends RobotOpMode {

    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double HOOD_STEP = 0.02;
    // auto (30s) + the transition + a slow driver, older than this and the pose is stale
    private static final double AUTO_POSE_MAX_AGE_SEC = 120;

    private GamepadEx gamepad;

    private boolean gateOverride = false;
    // without a camera auto aim would block the gate forever
    private boolean autoAimEnabled = false;

    // on top of the shot table, from the dpad
    private double pollenTrim = 0;
    private double nectarTrim = 0;

    private final ShotSolver solver = new ShotSolver(Shooter.SHOT_TABLE);
    private AimCommand aimCommand;
    private ShootCommand shootCommand;
    private Command intakeCommand;
    private Command outtakeCommand;
    private ShootCommand.Status lastShootStatus = ShootCommand.Status.IDLE;

    private Pose autoPose;

    @Override
    protected Pose startPose() {
        // runs before setup(), picks up where auto left the robot if it's recent
        autoPose = PoseStorage.take(AUTO_POSE_MAX_AGE_SEC);
        return autoPose != null ? autoPose : Pose.zero();
    }

    @Override
    protected void setup() {
        gamepad = new GamepadEx(gamepad1);
        autoAimEnabled = robot.limelight.isPresent(); // no camera, no waiting on one
        robot.limelight.setAlliance(PoseStorage.pickedAlliance());

        // field centric: with a known alliance "forward" is away from our drivers, otherwise it's
        // whichever way the robot faces at init (START on the gamepad re-zeroes it either way)
        Alliance alliance = PoseStorage.alliance();
        if (alliance != null) robot.drive.setDriverForwardHeading(alliance.driverForwardRad);

        // Default commands, these run whenever nothing else is using the subsystem
        robot.drive.setDefaultCommand(new DriveCommand(
                robot.drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () -> Drive.FULL_SPEED_SCALE * robot.driveScale(),
                this::headingBiasAllowed
        ));

        aimCommand = new AimCommand(robot.turret, robot.limelight, solver, robot.drive::heading,
                () -> autoAimEnabled, () -> !robot.gate.isOpen());
        robot.turret.setDefaultCommand(aimCommand);

        // gate: closed unless the override is on, ShootCommand takes it over while shooting
        robot.gate.setDefaultCommand(new RunCommand(() -> {
            if (gateOverride) robot.gate.open();
            else robot.gate.close();
        }, robot.gate));

        // hoods follow the table from the first loop after START. Whole percents only, so camera noise
        // doesn't have the servos twitching
        robot.hood.setDefaultCommand(new RunCommand(() -> {
            ShotLookupTable.Setting shot = Shooter.SHOT_TABLE.get(shotDistance());
            robot.hood.setPositions(
                    Math.round((shot.pollenHood + pollenTrim) * 100) / 100.0,
                    Math.round((shot.nectarHood + nectarTrim) * 100) / 100.0);
        }, robot.hood));

        // Telemetry + driver feedback, every loop
        schedule(new RunCommand(() -> {
            rumbleOnShooterChanges();
            robot.limelight.setAlliance(PoseStorage.pickedAlliance());
            List<String> faults = robot.faults();
            telemetry.addData("STATUS", faults.isEmpty() ? "OK" : faults.toString());
            telemetry.addData("Shooter", shootCommand.getStatus());
            telemetry.addData("Loop (ms)", "%.1f%s", loopMs, loopMs > 40 ? "  SLOW" : "");
            telemetry.addData("Gate override", gateOverride);
            telemetry.addData("Auto aim", autoAimEnabled);
            telemetry.addData("Shot distance (in)", "%.1f", shotDistance());
            ShotSolver.Aim aim = aimCommand.getAim();
            telemetry.addData("Robot speed (in/s)", "%.0f", aim != null ? aim.speedInPerSec : 0);
            telemetry.addData("Turret searching", aimCommand.isSearching());
            robot.telemetrize(telemetry);
            telemetry.update();
        }));

        configureButtonBindings();
    }

    @Override
    protected void initTelemetry() {
        telemetry.addLine(autoPose != null
                ? String.format("Pose from auto: (%.1f, %.1f) %.0f deg", autoPose.x(), autoPose.y(), Math.toDegrees(autoPose.heading()))
                : "No auto pose, starting at (0, 0), robot facing = forward");
        if (gamepad1.b) PoseStorage.pickAlliance(Alliance.RED);
        if (gamepad1.x) PoseStorage.pickAlliance(Alliance.BLUE);
        Alliance picked = PoseStorage.pickedAlliance();
        robot.limelight.setAlliance(picked);
        telemetry.addLine("Alliance: " + (picked != null ? picked : "NOT PICKED, tracks any Hive") + " (B red, X blue)");
    }

    private void rumbleOnShooterChanges() {
        ShootCommand.Status status = shootCommand.getStatus();
        if (status != lastShootStatus) {
            boolean nowShooting = status == ShootCommand.Status.SHOOTING
                    || status == ShootCommand.Status.SHOOTING_LOW_RPM
                    || status == ShootCommand.Status.SHOOTING_NO_ENCODER;
            if (nowShooting && lastShootStatus == ShootCommand.Status.SPINNING_UP) gamepad1.rumble(150);
            if (status == ShootCommand.Status.EMPTY) gamepad1.rumbleBlips(3);
            lastShootStatus = status;
        }
    }

    private void configureButtonBindings() {
        // intake and outtake just turn requests on and off, the intake subsystem sorts out the rest
        intakeCommand = new StartEndCommand(
                () -> robot.intake.setIntaking(true),
                () -> robot.intake.setIntaking(false));
        outtakeCommand = new StartEndCommand(
                () -> robot.intake.setOuttaking(true),
                () -> robot.intake.setOuttaking(false));

        // shoot, waits for the aim unless there's no camera to aim with
        shootCommand = new ShootCommand(
                robot.shooter,
                robot.gate,
                robot.intake,
                () -> !autoAimEnabled
                        || !robot.limelight.isAlive()
                        || (robot.turret.isReadyToShoot() && robot.limelight.hasTarget()),
                this::shootRpm);

        new Trigger(this::intakeHeld).whileActiveOnce(intakeCommand);
        new Trigger(this::shootHeld).whileActiveOnce(shootCommand);
        gamepad.getGamepadButton(GamepadKeys.Button.B).whileActiveOnce(outtakeCommand);

        // battle mode
        gamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(this::battleMode);

        // gate override
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> gateOverride = !gateOverride);

        // flower, the intake goes down for it
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    if (Flower.ENABLED) robot.intake.setFlowerWanted(!robot.intake.isFlowerWanted());
                });

        // hoods, dpad up/down is pollen and right/left is nectar
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> pollenTrim = Range.clip(pollenTrim + HOOD_STEP, -1, 1));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> pollenTrim = Range.clip(pollenTrim - HOOD_STEP, -1, 1));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> nectarTrim = Range.clip(nectarTrim + HOOD_STEP, -1, 1));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> nectarTrim = Range.clip(nectarTrim - HOOD_STEP, -1, 1));

        // reset field centric
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(robot.drive::resetFieldCentric);
    }

    /**
     * Cancels shooting, intaking and outtaking, closes the gate and fully stows everything.
     * Holding a trigger through it doesn't restart anything, you have to press it again.
     */
    private void battleMode() {
        shootCommand.cancel();
        intakeCommand.cancel();
        outtakeCommand.cancel();
        gateOverride = false;
        robot.intake.clearRequests();
    }

    private boolean intakeHeld() {
        return gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD;
    }

    // only for getting around the field: when it's picking up or shooting the driver wants the heading it has
    private boolean headingBiasAllowed() {
        return !intakeHeld() && !shootHeld() && !gamepad1.b && !robot.intake.isFlowerWanted();
    }

    private boolean shootHeld() {
        return gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;
    }

    private double shootRpm() {
        return Shooter.SHOT_TABLE.get(shotDistance()).rpm;
    }

    // from the aim (it allows for driving), otherwise the fixed shot
    private double shotDistance() {
        ShotSolver.Aim aim = aimCommand.getAim();
        return aim != null ? aim.distanceIn : Shooter.FIXED_DISTANCE_IN;
    }
}
