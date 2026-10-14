package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Superstructure.WantedState;

/**
 * Example auto built as one state machine: shoot the preloads in place, then drive forward to leave.
 *
 * Each state has an entry action (in {@link #enter}) and an exit condition (in {@link #runStateMachine}).
 * To add a step, add an enum value, what happens when you enter it, and when it's done. Auto states
 * only set wanted states/drive goals; the same Superstructure as TeleOp handles the actual shooting.
 *
 * Poses are relative to wherever the robot starts (0, 0, facing +X). Pedro isn't tuned yet, so keep
 * paths short until the tuners have been run.
 */
@Autonomous(name = "Shoot Preload + Leave", group = "Auto", preselectTeleOp = "ChunkMode")
public class ShootAndLeaveAuto extends LinearOpMode {

    private enum State {
        SHOOT_PRELOAD,
        LEAVE,
        DONE
    }

    private static final Pose START_POSE = Pose.zero();
    private static final Pose LEAVE_POSE = new Pose(24, 0, 0);

    private static final double SHOOT_TIME_SECONDS = 4.0;
    /** Give up on the path if it hasn't finished by now, so a bad tune can't stall the auto. */
    private static final double LEAVE_TIMEOUT_SECONDS = 5.0;

    private Robot robot;
    private Path leavePath;
    private State state;
    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, START_POSE);
        leavePath = Paths.line(START_POSE, LEAVE_POSE).linear(START_POSE, LEAVE_POSE);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        enter(State.SHOOT_PRELOAD);

        while (opModeIsActive()) {
            robot.readInputs();
            runStateMachine();
            robot.update();

            telemetry.addData("Auto state", state);
            telemetry.addData("Time in state", "%.1f", stateTimer.seconds());
            robot.telemetrize(telemetry);
            telemetry.update();
        }
    }

    /** Entry actions: run once, on the loop a state starts. */
    private void enter(State next) {
        state = next;
        stateTimer.reset();

        switch (next) {
            case SHOOT_PRELOAD:
                robot.drivetrain.stop();
                robot.superstructure.setWantedState(WantedState.SHOOT);
                break;
            case LEAVE:
                robot.superstructure.setWantedState(WantedState.IDLE);
                robot.drivetrain.followPath(leavePath);
                break;
            case DONE:
                robot.superstructure.setWantedState(WantedState.IDLE);
                robot.drivetrain.stop();
                break;
        }
    }

    /** Transitions: checked every loop. */
    private void runStateMachine() {
        switch (state) {
            case SHOOT_PRELOAD:
                if (stateTimer.seconds() >= SHOOT_TIME_SECONDS) {
                    enter(State.LEAVE);
                }
                break;
            case LEAVE:
                if (!robot.drivetrain.isBusy() || stateTimer.seconds() >= LEAVE_TIMEOUT_SECONDS) {
                    enter(State.DONE);
                }
                break;
            case DONE:
                break;
        }
    }
}
