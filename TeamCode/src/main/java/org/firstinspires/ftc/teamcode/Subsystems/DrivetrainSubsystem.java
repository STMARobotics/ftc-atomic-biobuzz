package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Mecanum drivetrain localized with a SparkFun OTOS, driven through the Pedro Pathing Follower.
 *
 * All distances are in inches and all angles are in radians (see {@link Constants}).
 * Drive inputs follow Pedro's convention: forward is +, left is +, counterclockwise is +. From a
 * gamepad that's usually {@code drive(-left_stick_y, -left_stick_x, -right_stick_x)}.
 */
public class DrivetrainSubsystem extends SubsystemBase {

    private final Follower follower;

    private double driverForwardHeading = 0.0;

    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, Pose.zero());
    }

    public DrivetrainSubsystem(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        driverForwardHeading = startPose.heading();
    }

    @Override
    public void periodic() {
        follower.update();
    }

    /**
     * Field centric drive. Inputs are squared for finer control at low speeds (tbd if we keep squared).
     * @param forward speed away from the driver in range [-1, 1]
     * @param strafe speed to the driver's left in range [-1, 1]
     * @param turn rotation speed in range [-1, 1], counterclockwise positive
     * @param speedScale multiplier on all axes in range [0, 1], useful for a slow mode
     */
    public void drive(double forward, double strafe, double turn, double speedScale) {
        DrivePowers powers = scaledPowers(forward, strafe, turn, speedScale);
        follower.manual(ManualDrive.fieldCentric(powers, heading() - driverForwardHeading));
    }

    public void drive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, 1.0);
    }

    /**
     * Robot centric drive. Inputs are squared for finer control at low speeds (just don't use this).
     * @param forward speed toward the front of the robot in range [-1, 1]
     * @param strafe speed toward the left of the robot in range [-1, 1]
     * @param turn rotation speed in range [-1, 1], counterclockwise positive
     * @param speedScale multiplier on all axes in range [0, 1]
     */
    public void driveRobotCentric(double forward, double strafe, double turn, double speedScale) {
        follower.manual(scaledPowers(forward, strafe, turn, speedScale));
    }

    public void driveRobotCentric(double forward, double strafe, double turn) {
        driveRobotCentric(forward, strafe, turn, 1.0);
    }

    /**
     * Makes whatever direction the robot is currently facing "forward" for field centric driving.
     */
    public void resetFieldCentric() {
        driverForwardHeading = heading();
    }

    /**
     * Starts following a path. Build paths with {@code com.pedropathing.api.Paths.line/curve}.
     */
    public void followPath(Path path) {
        follower.follow(path);
    }

    /**
     * Actively holds the robot at a pose.
     */
    public void holdPose(Pose pose) {
        follower.hold(pose);
    }

    public void holdCurrentPose() {
        follower.hold(pose());
    }

    /**
     * Cuts drive power and drops whatever the follower was doing.
     */
    public void stop() {
        follower.stop();
    }

    /**
     * @return true while following a path, useful for ending auto commands
     */
    public boolean isBusy() {
        return follower.isBusy();
    }

    public Pose pose() {
        return follower.pose();
    }

    public double heading() {
        return follower.pose().heading();
    }

    /**
     * Overwrites the localizer's pose. Use at the start of auto or to relocalize.
     */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void resetPose() {
        setPose(Pose.zero());
        driverForwardHeading = 0.0;
    }

    public Follower getFollower() {
        return follower;
    }

    public void telemetrize(Telemetry telemetry) {
        Pose pose = pose();
        telemetry.addData("Drive mode", follower.mode());
        telemetry.addData("X (in)", "%.2f", pose.x());
        telemetry.addData("Y (in)", "%.2f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
    }

    private static DrivePowers scaledPowers(double forward, double strafe, double turn, double speedScale) {
        double scale = Range.clip(speedScale, 0.0, 1.0);
        return new DrivePowers(square(forward) * scale, square(strafe) * scale, square(turn) * scale);
    }

    private static double square(double value) {
        return Math.copySign(value * value, value);
    }
}