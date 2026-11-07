package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
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
 *   left trigger  intake (hold, the intake is down while held and fully stowed otherwise)
 *   B             outtake (hold)
 *   right trigger shoot, gate opens once the flywheel is past GATE_OPEN_RPM
 *   right bumper  gate override toggle (holds the gate open)
 *   A             flower mech toggle
 *   dpad up/down  pollen hood +/- 0.02
 *   dpad right/left nectar hood +/- 0.02
 *   start         reset field centric
 */
@TeleOp(name = "ChunkMode", group = "TeleOp")
public class ChunkMode extends CommandOpMode {

    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double HOOD_STEP = 0.02;

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

    private boolean flowerWanted = false;
    private boolean gateOverride = false;
    // without a camera auto aim would block the gate forever
    private boolean autoAimEnabled = Limelight.ENABLED;

    // start from the shot table, the dpad nudges them
    private double pollen = Shooter.SHOT_TABLE.get(Shooter.FIXED_DISTANCE_IN).pollenHood;
    private double nectar = Shooter.SHOT_TABLE.get(Shooter.FIXED_DISTANCE_IN).nectarHood;

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
            telemetry.addData("Gate override", gateOverride);
            hoodSubsystem.telemetrize(telemetry);
            telemetry.addData("Shot distance (in)", "%.1f", shotDistance());
            telemetry.addData("|-----| Intake Telemetry |-----|", "");
            pivotSubsystem.telemetrize(telemetry);
            wheelsSubsystem.telemetrize(telemetry);
            flowerSubsystem.telemetrize(telemetry);
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

        // Intake wheels: outtake wins, then feeding (shooting with the gate open), then intake, otherwise stopped
        wheelsSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (outtakeHeld()) wheelsSubsystem.outtake();
            else if ((shootHeld() && gateSubsystem.isOpen()) || intakeHeld()) wheelsSubsystem.intake();
            else wheelsSubsystem.stop();
        }, wheelsSubsystem));

        // Gate: closed unless the override is on, ShootCommand takes it over while shooting
        gateSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (gateOverride) gateSubsystem.open();
            else gateSubsystem.close();
        }, gateSubsystem));

        // Intake pivot: down while intaking (or for the flower), fully stowed the rest of the time
        pivotSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (intakeHeld() || flowerWanted) pivotSubsystem.deploy();
            else pivotSubsystem.fullyStow();
        }, pivotSubsystem));

        // Flower: the intake is its hardstop so it only goes out once the intake is down
        flowerSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (flowerWanted && pivotSubsystem.isDeployed()) flowerSubsystem.deploy();
            else flowerSubsystem.stow();
        }, flowerSubsystem));

        // Hoods go to their defaults on the first loop after START, then hold whatever the dpad set
        hoodSubsystem.setDefaultCommand(new RunCommand(
                () -> hoodSubsystem.setPositions(pollen, nectar), hoodSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Reset field-centric
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(drivetrainSubsystem::resetFieldCentric);

        // Flower mech, A toggles it (the pivot puts the intake down for it)
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    if (Flower.ENABLED) flowerWanted = !flowerWanted;
                });

        // Gate override, right bumper toggles the gate open
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> gateOverride = !gateOverride);

        // Shoot, waits for the aim unless auto aim is off (then we're aiming by hand)
        new Trigger(this::shootHeld)
                .whileActiveOnce(new ShootCommand(
                        shooterSubsystem,
                        gateSubsystem,
                        () -> !autoAimEnabled
                                || (turretSubsystem.isAtTarget()
                                && !turretSubsystem.isUnwinding()
                                && limelightSubsystem.hasTarget()),
                        this::shootRpm));

        // Hoods, dpad up/down is pollen and right/left is nectar
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> pollen = Range.clip(pollen + HOOD_STEP, 0, 1));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> pollen = Range.clip(pollen - HOOD_STEP, 0, 1));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> nectar = Range.clip(nectar + HOOD_STEP, 0, 1));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> nectar = Range.clip(nectar - HOOD_STEP, 0, 1));
    }

    private boolean intakeHeld() {
        return gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD;
    }

    private boolean shootHeld() {
        return gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;
    }

    private boolean outtakeHeld() {
        return gamepad.getButton(GamepadKeys.Button.B);
    }

    private double shootRpm() {
        return Shooter.SHOT_TABLE.get(shotDistance()).rpm;
    }

    // limelight distance if it's on and sees the tag, otherwise the fixed shot
    private double shotDistance() {
        double distance = Limelight.ENABLED ? limelightSubsystem.getDistance() : Double.NaN;
        return Double.isNaN(distance) ? Shooter.FIXED_DISTANCE_IN : distance;
    }
}
