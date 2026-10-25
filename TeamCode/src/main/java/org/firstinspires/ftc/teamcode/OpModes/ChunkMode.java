package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FlowerBarSubsystem;
import org.firstinspires.ftc.teamcode.Superstructure.WantedState;

/**
 * Driver controls (gamepad 1):
 * <ul>
 *     <li>INIT: B picks the red alliance, X picks the blue alliance</li>
 *     <li>Left stick: drive (field centric), right stick X: turn</li>
 *     <li>Right trigger (hold): shoot. Spins up, then feeds whenever everything is ready</li>
 *     <li>Left trigger (hold): intake</li>
 *     <li>Right bumper (hold): outtake</li>
 *     <li>Left bumper (hold): flower bar down</li>
 *     <li>Y (hold): blocker out</li>
 *     <li>Start: make the direction the robot is facing "forward"</li>
 * </ul>
 * If more than one of shoot, outtake and intake is held, that priority is decided in
 * {@link #wantedStateFromDriver()}. The button layout is a starting point, change it freely.
 */
@TeleOp(name = "ChunkMode", group = "TeleOp")
public class ChunkMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Alliance alliance = Alliance.RED;

        // Pick the alliance while waiting for the match to start
        while (opModeInInit()) {
            if (gamepad1.b) {
                alliance = Alliance.RED;
            } else if (gamepad1.x) {
                alliance = Alliance.BLUE;
            }
            telemetry.addData("Alliance (B = red, X = blue)", alliance);
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }
        robot.superstructure.setAlliance(alliance);

        while (opModeIsActive()) {
            // 1. Read every sensor once
            robot.readInputs();

            // 2. Turn driver inputs into wanted states and goals
            if (gamepad1.startWasPressed()) {
                robot.drivetrain.resetFieldCentric();
            }
            robot.drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            robot.superstructure.setWantedState(wantedStateFromDriver());
            robot.flowerBar.setGoal(gamepad1.left_bumper
                    ? FlowerBarSubsystem.Goal.DEPLOYED
                    : FlowerBarSubsystem.Goal.STOWED);
            robot.blocker.setGoal(gamepad1.y
                    ? BlockerSubsystem.Goal.EXTENDED
                    : BlockerSubsystem.Goal.STOWED);

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
        if (gamepad1.right_bumper) {
            return WantedState.OUTTAKE;
        }
        if (gamepad1.left_trigger > Constants.Driver.TRIGGER_THRESHOLD) {
            return WantedState.INTAKE;
        }
        return WantedState.IDLE;
    }
}
