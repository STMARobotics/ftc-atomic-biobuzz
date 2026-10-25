package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.BlockerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FlowerBarSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.Arrays;
import java.util.List;

/**
 * Owns all the hardware and runs one loop in a fixed order. This replaces the command scheduler.
 *
 * <pre>
 * robot.readInputs();     // 1. read every sensor once
 * ...set wanted states... // 2. OpMode/auto says what it wants
 * robot.update();         // 3. superstructure decides, 4. subsystems write to hardware
 * </pre>
 *
 * Who sets each subsystem's goal:
 * <ul>
 *     <li>The superstructure: indexer, shooter, turret</li>
 *     <li>The OpMode or auto directly: drivetrain, flower bar, blocker</li>
 * </ul>
 */
public class Robot {

    public final DrivetrainSubsystem drivetrain;
    public final IndexerSubsystem indexer;
    public final ShooterSubsystem shooter;
    public final TurretSubsystem turret;
    public final FlowerBarSubsystem flowerBar;
    public final BlockerSubsystem blocker;
    public final Superstructure superstructure;

    private final List<LynxModule> hubs;
    private final List<Subsystem> subsystems;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopTimeMs;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, Pose.zero());
    }

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        // MANUAL bulk caching: every motor/encoder read in a loop comes from one bulk read per hub,
        // as long as the cache is cleared once per loop (readInputs does that)
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new DrivetrainSubsystem(hardwareMap, startPose);
        indexer = new IndexerSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        flowerBar = new FlowerBarSubsystem(hardwareMap);
        blocker = new BlockerSubsystem(hardwareMap);
        superstructure = new Superstructure(indexer, shooter, turret, drivetrain);

        // A subsystem that isn't in this list never gets readInputs() or writeOutputs() called
        subsystems = Arrays.asList(drivetrain, indexer, shooter, turret, flowerBar, blocker);
    }

    /**
     * Step 1 of the loop. Call before setting wanted states so decisions use fresh data.
     */
    public void readInputs() {
        loopTimeMs = loopTimer.milliseconds();
        loopTimer.reset();

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        for (Subsystem subsystem : subsystems) {
            subsystem.readInputs();
        }
    }

    /**
     * Steps 3 and 4 of the loop. Call after setting wanted states.
     */
    public void update() {
        superstructure.update();
        for (Subsystem subsystem : subsystems) {
            subsystem.writeOutputs();
        }
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Loop time (ms)", "%.1f", loopTimeMs);
        telemetry.addData("|-----| Superstructure |-----|", "");
        superstructure.telemetrize(telemetry);
        telemetry.addData("|-----| Drivetrain |-----|", "");
        drivetrain.telemetrize(telemetry);
        telemetry.addData("|-----| Indexer |-----|", "");
        indexer.telemetrize(telemetry);
        telemetry.addData("|-----| Shooter |-----|", "");
        shooter.telemetrize(telemetry);
        telemetry.addData("|-----| Turret |-----|", "");
        turret.telemetrize(telemetry);
        telemetry.addData("|-----| Flower bar |-----|", "");
        flowerBar.telemetrize(telemetry);
        telemetry.addData("|-----| Blocker |-----|", "");
        blocker.telemetrize(telemetry);
    }
}
