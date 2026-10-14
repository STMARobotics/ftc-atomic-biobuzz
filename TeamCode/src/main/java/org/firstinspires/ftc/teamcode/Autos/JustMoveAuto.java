package org.firstinspires.ftc.teamcode.Autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class JustMoveAuto extends OpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(132.9734, 62.7002, 180);
    private final Pose path1 = poseFactory.of(127.0775, 40.5856, 180);

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(start);
    }

    @Override
    public void start() {
        schedule(follow(follower, path1()));

    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }
}