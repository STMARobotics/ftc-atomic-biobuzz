package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Base for every match OpMode. Builds the robot, turns on bulk reads (one hardware read per hub
 * per loop instead of one per sensor) and times the loop.
 *
 * The scheduler doesn't run until START, so nothing moves during init unless an auto does it
 * on purpose in setup().
 */
public abstract class RobotOpMode extends CommandOpMode {

    protected Robot robot;
    protected double loopMs = 0;
    private long lastLoopNanos = 0;

    @Override
    public final void initialize() {
        // the scheduler clears the cache at the end of every loop for us
        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot = new Robot(hardwareMap, startPose());
        register(robot.subsystems());
        setup();
    }

    /**
     * Where the robot starts, autos override this
     */
    protected Pose startPose() {
        return Pose.zero();
    }

    /**
     * Default commands, button bindings, auto routine. Called once during init
     */
    protected abstract void setup();

    @Override
    public void initialize_loop() {
        telemetry.addLine("Initialized, nothing moves until START");
        telemetry.update();
        sleep(50);
    }

    @Override
    public void run() {
        super.run();
        long now = System.nanoTime();
        if (lastLoopNanos != 0) loopMs = (now - lastLoopNanos) / 1e6;
        lastLoopNanos = now;
    }
}
