package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.IdleCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@TeleOp(name = "ChunkMode", group = "TeleOp")
public class ChunkMode extends CommandOpMode {

    // Subsystems
    private DrivetrainSubsystem drivetrainSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private IdleCommand idleCommand;

    private GamepadEx gamepad;

    private double driveScale = 1;

    @Override
    public void initialize() {
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        idleCommand = new IdleCommand(intakeSubsystem, shooterSubsystem);

        // Gamepad
        gamepad = new GamepadEx(gamepad1);

        // Drive command
        DriveCommand teleopDriveCommand = new DriveCommand(
                drivetrainSubsystem,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () -> driveScale
        );

        // Telemetry
        RunCommand telemetryCommand = new RunCommand(() -> {
            telemetry.addData("|-----|Drivetrain Telemetry|-----|", "");
            drivetrainSubsystem.telemetrize(telemetry);
            telemetry.addData("|-----| Shooter Telemetry |-----|", "");
            shooterSubsystem.telemetrize(telemetry);
            telemetry.update();
        });
        schedule(telemetryCommand);

        // Register subsystems
        register(
                drivetrainSubsystem
        );

        // Default commands
        drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);
//        shooterSubsystem.setDefaultCommand(idleCommand);
//        intakeSubsystem.setDefaultCommand(idleCommand);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Reset field-centric
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(drivetrainSubsystem::resetFieldCentric);

        // Shoot stuff
        Trigger shootTrigger = new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);

        shootTrigger.whileActiveContinuous(
                new ShootCommand(shooterSubsystem, intakeSubsystem));

        // Intake stuff
        Trigger intakeTrigger = new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);

        intakeTrigger.whileActiveContinuous(
                new IntakeCommand(intakeSubsystem));
    }

    private void toggleDriveScale() {
        driveScale = (driveScale == 1) ? 0.5 : 1;
    }
}