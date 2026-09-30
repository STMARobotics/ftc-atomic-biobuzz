package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.TriggerReader;

import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;

@TeleOp(name = "ChunkMode", group = "TeleOp")
public class ChunkMode extends CommandOpMode {

    // Subsystems
    private DrivetrainSubsystem drivetrainSubsystem;

    private GamepadEx gamepad;
    private TriggerReader leftTriggerReader;
    private TriggerReader rightTriggerReader;

    private double driveScale = 1;

    @Override
    public void initialize() {
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap);

        // Gamepad
        gamepad = new GamepadEx(gamepad1);
        leftTriggerReader  = new TriggerReader(gamepad, GamepadKeys.Trigger.LEFT_TRIGGER);
        rightTriggerReader = new TriggerReader(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);

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
            telemetry.update();
        });
        schedule(telemetryCommand);

        // Register subsystems
        register(
                drivetrainSubsystem
        );

        // Default commands
        drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Reset field-centric
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(drivetrainSubsystem::resetFieldCentric);
    }

    private void toggleDriveScale() {
        driveScale = (driveScale == 1) ? 0.5 : 1;
    }
}