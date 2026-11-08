package org.firstinspires.ftc.teamcode.OpModes;

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
import org.firstinspires.ftc.teamcode.RobotConstants.Limelight;
import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;

/**
 * Controls (gamepad 1):
 *   sticks          drive (left translate, right turn), field centric
 *   left trigger    intake (hold), the intake is down while held and fully stowed otherwise
 *   B               outtake (hold)
 *   right trigger   shoot (hold), gate opens once the flywheel is up to speed, then it feeds
 *   right bumper    gate override toggle (holds the gate open)
 *   X               battle mode: cancels shooting and everything else, fully stows
 *   A               flower toggle (only when Flower.ENABLED)
 *   dpad up/down    pollen hood +/- 0.02
 *   dpad right/left nectar hood +/- 0.02
 *   start           reset field centric
 */
@TeleOp(name = "ChunkMode", group = "TeleOp")
public class ChunkMode extends RobotOpMode {

    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double HOOD_STEP = 0.02;

    private GamepadEx gamepad;

    private boolean gateOverride = false;
    // without a camera auto aim would block the gate forever
    private final boolean autoAimEnabled = Limelight.ENABLED;

    // start from the shot table, the dpad nudges them
    private double pollen = Shooter.SHOT_TABLE.get(Shooter.FIXED_DISTANCE_IN).pollenHood;
    private double nectar = Shooter.SHOT_TABLE.get(Shooter.FIXED_DISTANCE_IN).nectarHood;

    private Command shootCommand;
    private Command intakeCommand;
    private Command outtakeCommand;

    @Override
    protected void setup() {
        gamepad = new GamepadEx(gamepad1);

        // Default commands, these run whenever nothing else is using the subsystem
        robot.drive.setDefaultCommand(new DriveCommand(
                robot.drive,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () -> Drive.FULL_SPEED_SCALE
        ));

        robot.turret.setDefaultCommand(new AimCommand(
                robot.turret, robot.limelight, () -> autoAimEnabled));

        // gate: closed unless the override is on, ShootCommand takes it over while shooting
        robot.gate.setDefaultCommand(new RunCommand(() -> {
            if (gateOverride) robot.gate.open();
            else robot.gate.close();
        }, robot.gate));

        // hoods go to their defaults on the first loop after START, then hold whatever the dpad set
        robot.hood.setDefaultCommand(new RunCommand(
                () -> robot.hood.setPositions(pollen, nectar), robot.hood));

        // Telemetry
        schedule(new RunCommand(() -> {
            telemetry.addData("Loop (ms)", "%.1f", loopMs);
            telemetry.addData("Gate override", gateOverride);
            telemetry.addData("Auto aim", autoAimEnabled);
            telemetry.addData("Shot distance (in)", "%.1f", shotDistance());
            robot.telemetrize(telemetry);
            telemetry.update();
        }));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // intake and outtake just turn requests on and off, the intake subsystem sorts out the rest
        intakeCommand = new StartEndCommand(
                () -> robot.intake.setIntaking(true),
                () -> robot.intake.setIntaking(false));
        outtakeCommand = new StartEndCommand(
                () -> robot.intake.setOuttaking(true),
                () -> robot.intake.setOuttaking(false));

        // shoot, waits for the aim unless auto aim is off (then we're aiming by hand)
        shootCommand = new ShootCommand(
                robot.shooter,
                robot.gate,
                robot.intake,
                () -> !autoAimEnabled
                        || (robot.turret.isAtTarget()
                        && !robot.turret.isUnwinding()
                        && robot.limelight.hasTarget()),
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
                .whenPressed(() -> pollen = Range.clip(pollen + HOOD_STEP, 0, 1));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> pollen = Range.clip(pollen - HOOD_STEP, 0, 1));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> nectar = Range.clip(nectar + HOOD_STEP, 0, 1));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> nectar = Range.clip(nectar - HOOD_STEP, 0, 1));

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

    private boolean shootHeld() {
        return gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;
    }

    private double shootRpm() {
        return Shooter.SHOT_TABLE.get(shotDistance()).rpm;
    }

    // limelight distance if it's on and sees the tag, otherwise the fixed shot
    private double shotDistance() {
        double distance = Limelight.ENABLED ? robot.limelight.getDistance() : Double.NaN;
        return Double.isNaN(distance) ? Shooter.FIXED_DISTANCE_IN : distance;
    }
}
