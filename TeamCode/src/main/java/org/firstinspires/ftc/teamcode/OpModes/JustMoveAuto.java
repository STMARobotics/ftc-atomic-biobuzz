package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;

/**
 * Drives off the start line and parks with the intake stowed.
 */
@Autonomous(name = "JustMoveAuto")
public class JustMoveAuto extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(132.9734, 62.7002, 180);
    private final Pose park = poseFactory.of(127.0775, 40.5856, 180);

    @Override
    protected Pose startPose() {
        return start;
    }

    @Override
    protected void setup() {
        // auto is allowed to move in init, and the intake has to start stowed
        robot.intake.stowNow();
    }

    @Override
    public void preRun() {
        // runs right after START, before the first loop
        schedule(new FollowPathCommand(robot.drive, Paths.line(start, park).linear(start, park)));
    }
}
