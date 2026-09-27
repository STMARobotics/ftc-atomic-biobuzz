package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;

@TeleOp(name = "ChunkMode")
public class ChunkMode extends CommandOpMode {

    // Subsystems
    private DrivetrainSubsystem drivetrainSubsystem;

    private GamepadEx gamepad;

    private double driveScale = 1;

    @Override
    public void initialize() {
        // Subsystem init
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap);

        // Gamepad
        gamepad = new GamepadEx(gamepad1);

        // Drive command
        DriveCommand teleopDriveCommand = new DriveCommand(
                drivetrainSubsystem,
                () -> gamepad1.left_stick_x,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.right_stick_x,
                () -> driveScale
        );

        // Telemetry (later)
        RunCommand telemetryCommand = new RunCommand(() -> {
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
                .whenPressed(drivetrainSubsystem::resetYaw);
    }

//    private void toggleDriveScale() {
//        driveScale = (driveScale == NORMAL_DRIVE_SCALE) ? SLOW_DRIVE_SCALE : NORMAL_DRIVE_SCALE;
//    }
}