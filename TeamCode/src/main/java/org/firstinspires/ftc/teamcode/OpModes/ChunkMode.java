package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.AimCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.FlowerControlCommand;
import org.firstinspires.ftc.teamcode.Commands.HoodCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakePivotControlCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeWheelsControlCommand;
import org.firstinspires.ftc.teamcode.Commands.ManualTurretCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.RobotConstants.Drive;
import org.firstinspires.ftc.teamcode.RobotConstants.Flower;
import org.firstinspires.ftc.teamcode.RobotConstants.Limelight;
import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeWheelsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterGateSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

/**
 * Controls (gamepad 1):
 *   sticks        drive (left translate, right turn)
 *   left trigger  intake
 *   B             outtake (hold)
 *   right trigger shoot, waits for the aim unless auto aim is off
 *   X             battle mode toggle
 *   A             flower mech toggle
 *   dpad up       turret home (turns auto aim off)
 *   dpad down     auto aim toggle
 *   dpad left     turn turret counterclockwise (hold)
 *   dpad right    turn turret clockwise (hold)
 *   start         reset field centric
 */
@TeleOp(name = "ChunkMode", group = "TeleOp")
public class ChunkMode extends CommandOpMode {

    private static final double TRIGGER_THRESHOLD = 0.1;

    // Subsystems
    private DrivetrainSubsystem drivetrainSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private TurretSubsystem turretSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private ShooterGateSubsystem gateSubsystem;
    private IntakePivotSubsystem pivotSubsystem;
    private IntakeWheelsSubsystem wheelsSubsystem;
    private FlowerSubsystem flowerSubsystem;
    private HoodSubsystem hoodSubsystem;

    private GamepadEx gamepad;

    private boolean battleMode = false;
    private boolean flowerWanted = false;
    // without a camera auto aim would block the gate forever
    private boolean autoAimEnabled = Limelight.ENABLED;

    @Override
    public void initialize() {
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap);
        limelightSubsystem = new LimelightSubsystem(hardwareMap);
        turretSubsystem = new TurretSubsystem(hardwareMap, drivetrainSubsystem::heading);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        gateSubsystem = new ShooterGateSubsystem(hardwareMap);
        pivotSubsystem = new IntakePivotSubsystem(hardwareMap);
        wheelsSubsystem = new IntakeWheelsSubsystem(hardwareMap);
        flowerSubsystem = new FlowerSubsystem(hardwareMap);
        hoodSubsystem = new HoodSubsystem(hardwareMap);

        // Gamepad
        gamepad = new GamepadEx(gamepad1);

        // Drive command
        DriveCommand teleopDriveCommand = new DriveCommand(
                drivetrainSubsystem,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () -> Drive.FULL_SPEED_SCALE
        );

        // Telemetry
        RunCommand telemetryCommand = new RunCommand(() -> {
            telemetry.addData("|-----|Drivetrain Telemetry|-----|", "");
            drivetrainSubsystem.telemetrize(telemetry);
            telemetry.addData("|-----| Turret Telemetry |-----|", "");
            turretSubsystem.telemetrize(telemetry);
            limelightSubsystem.telemetrize(telemetry);
            telemetry.addData("Auto aim", autoAimEnabled);
            telemetry.addData("|-----| Shooter Telemetry |-----|", "");
            shooterSubsystem.telemetrize(telemetry);
            gateSubsystem.telemetrize(telemetry);
            hoodSubsystem.telemetrize(telemetry);
            telemetry.addData("Distance (in)", "%.1f", limelightSubsystem.getDistance());
            telemetry.addData("|-----| Intake Telemetry |-----|", "");
            pivotSubsystem.telemetrize(telemetry);
            wheelsSubsystem.telemetrize(telemetry);
            flowerSubsystem.telemetrize(telemetry);
            telemetry.addData("Battle mode", battleMode);
            telemetry.addData("Flower wanted", flowerWanted);
            telemetry.update();
        });
        schedule(telemetryCommand);

        // Register subsystems
        register(
                drivetrainSubsystem,
                limelightSubsystem,
                turretSubsystem,
                shooterSubsystem,
                gateSubsystem,
                pivotSubsystem,
                wheelsSubsystem,
                flowerSubsystem,
                hoodSubsystem
        );

        // Default commands
        drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);
        turretSubsystem.setDefaultCommand(new AimCommand(
                turretSubsystem, limelightSubsystem, () -> autoAimEnabled));
        pivotSubsystem.setDefaultCommand(new IntakePivotControlCommand(
                pivotSubsystem, () -> battleMode, this::intakeHeld,
                () -> flowerWanted, flowerSubsystem::isStowed));
        hoodSubsystem.setDefaultCommand(new HoodCommand(hoodSubsystem, limelightSubsystem::getDistance));
        flowerSubsystem.setDefaultCommand(new FlowerControlCommand(
                flowerSubsystem, pivotSubsystem, () -> flowerWanted));
        wheelsSubsystem.setDefaultCommand(new IntakeWheelsControlCommand(
                wheelsSubsystem, this::outtakeHeld, gateSubsystem::isOpen, this::intakeHeld,
                () -> battleMode));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Reset field-centric
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(drivetrainSubsystem::resetFieldCentric);

        // Battle mode, X toggles it and pressing intake leaves it (last pressed wins).
        // Turning it on stows the flower first, the pivot waits for it
        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> {
                    battleMode = !battleMode;
                    if (battleMode) flowerWanted = false;
                });
        new Trigger(this::intakeHeld)
                .whenActive(new InstantCommand(() -> battleMode = false));

        // Flower mech, A toggles it and turning it on leaves battle mode
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    if (!Flower.ENABLED) return;
                    flowerWanted = !flowerWanted;
                    if (flowerWanted) battleMode = false;
                });

        // Shoot, waits for the aim unless auto aim is off (then we're aiming by hand)
        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD)
                .whileActiveOnce(new ShootCommand(
                        shooterSubsystem,
                        gateSubsystem,
                        () -> !autoAimEnabled
                                || (turretSubsystem.isAtTarget()
                                && !turretSubsystem.isUnwinding()
                                && limelightSubsystem.hasTarget()),
                        this::shootRpm));

        // Turret home, turns auto aim off or it would pull the turret right back
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> {
                    autoAimEnabled = false;
                    turretSubsystem.aimForward();
                });

        // Auto aim toggle
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> {
                    autoAimEnabled = !autoAimEnabled;
                    if (!autoAimEnabled) turretSubsystem.holdCurrentAngle();
                });

        // Manual turret rotation, takes the turret from auto aim while held
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new ManualTurretCommand(turretSubsystem, false));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(new ManualTurretCommand(turretSubsystem, true));
    }

    private boolean intakeHeld() {
        return gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD;
    }

    private boolean outtakeHeld() {
        return gamepad.getButton(GamepadKeys.Button.B);
    }

    private double shootRpm() {
        double distance = limelightSubsystem.getDistance();
        return Double.isNaN(distance) ? Double.NaN : Shooter.SHOT_TABLE.get(distance).rpm;
    }
}
