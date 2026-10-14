package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Superstructure.WantedState;

/**
 * Driver controls (gamepad 1):
 * <ul>
 *     <li>Left stick: drive (field centric), right stick X: turn</li>
 *     <li>Right trigger (hold): shoot. Spins up, then feeds whenever the flywheel is at speed</li>
 *     <li>Left trigger (hold): intake</li>
 *     <li>Start: make the direction the robot is facing "forward"</li>
 * </ul>
 * Holding both triggers shoots. That priority is decided in {@link #wantedStateFromDriver()}.
 */
@TeleOp(name = "ChunkMode", group = "TeleOp")
public class ChunkMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // 1. Read every sensor once
            robot.readInputs();

            // 2. Turn driver inputs into wanted states
            if (gamepad1.startWasPressed()) {
                robot.drivetrain.resetFieldCentric();
            }
            robot.drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            robot.superstructure.setWantedState(wantedStateFromDriver());

            // 3 + 4. Superstructure decides, subsystems write to hardware
            robot.update();

            robot.telemetrize(telemetry);
            telemetry.update();
        }
    }

    private WantedState wantedStateFromDriver() {
        if (gamepad1.right_trigger > Constants.Driver.TRIGGER_THRESHOLD) {
            return WantedState.SHOOT;
        }
        if (gamepad1.left_trigger > Constants.Driver.TRIGGER_THRESHOLD) {
            return WantedState.INTAKE;
        }
        return WantedState.IDLE;
    }
}
