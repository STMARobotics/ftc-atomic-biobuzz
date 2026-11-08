package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.MatchLog;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.List;

/**
 * Base for every match OpMode. Builds the robot, turns on bulk reads (one hardware read per hub
 * per loop instead of one per sensor), times the loop, writes the match log, and shows faults
 * during INIT so problems get caught before the match instead of during it.
 * Autos also save the pose every loop so teleop can pick it up (PoseStorage).
 *
 * The scheduler doesn't run until START, so nothing moves during init unless an auto does it
 * on purpose in setup().
 */
public abstract class RobotOpMode extends CommandOpMode {

    private static final double LOG_EVERY_SEC = 0.1;

    protected Robot robot;
    protected double loopMs = 0;
    private long lastLoopNanos = 0;
    private long lastLogNanos = 0;
    private MatchLog log;
    private final boolean isAuto = getClass().isAnnotationPresent(Autonomous.class);

    @Override
    public final void initialize() {
        // the scheduler clears the cache at the end of every loop for us
        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot = new Robot(hardwareMap, startPose());
        register(robot.subsystems());
        log = new MatchLog(getClass().getSimpleName(), Robot.LOG_COLUMNS);
        setup();
    }

    /**
     * Where the robot starts, autos override this
     */
    protected Pose startPose() {
        return Pose.zero();
    }

    /**
     * Which alliance this OpMode is for, null if it works for both. Autos should say
     */
    protected Alliance alliance() {
        return null;
    }

    /**
     * Default commands, button bindings, auto routine. Called once during init
     */
    protected abstract void setup();

    /**
     * Extra lines for the INIT screen, teleop uses it to say where its pose came from
     */
    protected void initTelemetry() {}

    @Override
    public void initialize_loop() {
        List<String> faults = robot.faults();
        telemetry.addLine(faults.isEmpty() ? "READY, nothing moves until START" : "CHECK THE ROBOT:");
        for (String fault : faults) telemetry.addLine("  " + fault);
        initTelemetry();
        telemetry.update();
        sleep(50);
    }

    @Override
    public void run() {
        super.run();
        long now = System.nanoTime();
        if (lastLoopNanos != 0) loopMs = (now - lastLoopNanos) / 1e6;
        lastLoopNanos = now;

        if (isAuto) PoseStorage.save(robot.drive.pose(), alliance());
        if ((now - lastLogNanos) / 1e9 >= LOG_EVERY_SEC) {
            lastLogNanos = now;
            log.row(robot.logRow(loopMs));
        }
    }

    @Override
    public void end() {
        if (log != null) log.close(); // null if init blew up before the log existed
    }
}
