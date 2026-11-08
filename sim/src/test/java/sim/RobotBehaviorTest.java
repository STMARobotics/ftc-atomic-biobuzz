package sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakePivot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Set;

/**
 * Runs the real OpModes in the sim the way a drive team would and checks what physically happened.
 * Each test is one thing that has to keep working. Run before every comp:
 *   .\gradlew -p sim test
 * They run in real time, so the whole thing takes about a minute.
 */
class RobotBehaviorTest {

    @BeforeEach
    void clearStatics() {
        PoseStorage.clear();
    }

    @Test
    void teleopInitMovesNothing() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.init("ChunkMode");
            SimHarness.sleep(700);
            assertEquals(List.of(), sim.world.warnings(), "something moved during TeleOp INIT (penalty)");
            assertTrue(sim.telemetryLines().stream().anyMatch(l -> l.startsWith("READY")),
                    "INIT screen should say READY with nothing wrong: " + sim.telemetryLines());
        }
    }

    @Test
    void holdingRtShootsEveryBallAndCountsThem() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.init("ChunkMode").start();
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 0, 6), "balls left: " + sim.physics.balls);
            SimHarness.sleep(500); // the counter needs the flywheel to recover after the last ball
            assertEquals(3, sim.physics.shotsLaunched, "balls that actually left the shooter");
            assertEquals("3", sim.telemetry("Shots fired"), "the code's shot counter");
        }
    }

    @Test
    void intakeDeploysWhileHeldAndBattleModeStowsEverything() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.init("ChunkMode").start();
            SimServo pivot = sim.world.servoState(HardwareNames.PIVOT_SERVO);
            SimMotor intake = sim.world.motorState(HardwareNames.INTAKE_MOTOR_RIGHT);
            SimMotor flywheel = sim.world.motorState(HardwareNames.FLYWHEEL_MOTOR);

            sim.hold("lt", 1.0);
            assertTrue(sim.waitUntil(() -> Math.abs(pivot.physical - IntakePivot.DEPLOYED) < 0.05, 2),
                    "pivot should be down while LT is held, it's at " + pivot.physical);
            assertTrue(intake.power > 0.5, "intake wheels should be running, power " + intake.power);

            sim.hold("rt", 1.0); // shooting while intaking is allowed
            assertTrue(sim.waitUntil(() -> flywheel.power != 0, 1), "flywheel should spin up");

            sim.press("x"); // battle mode, still holding both triggers
            assertTrue(sim.waitUntil(() -> Math.abs(pivot.physical - IntakePivot.FULLY_STOWED) < 0.05, 2),
                    "battle mode should fully stow the pivot, it's at " + pivot.physical);
            assertEquals(0.0, intake.power, "battle mode should stop the intake");
            assertEquals(0.0, flywheel.power, "battle mode should stop the flywheel");
        }
    }

    @Test
    void weakBatteryStillShootsInsteadOfWaitingForever() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.batteryRestVolts = 11.0; // can't reach full speed anymore
            sim.init("ChunkMode").start();
            sim.hold("rt", 1.0);
            Set<String> statuses = sim.watch("Shooter", 8, () -> sim.physics.balls == 0);
            assertTrue(statuses.contains("SHOOTING_LOW_RPM"), "should fall back to low rpm shots: " + statuses);
            assertEquals(0, sim.physics.balls, "every ball should still go out");
        }
    }

    @Test
    void jamGetsClearedAutomatically() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.jamNextBall = true;
            sim.init("ChunkMode").start();
            sim.hold("rt", 1.0);
            Set<String> statuses = sim.watch("Shooter", 8, () -> sim.physics.balls == 0);
            assertTrue(statuses.contains("UNJAMMING"), "should notice the jam and back it out: " + statuses);
            assertTrue(!sim.physics.jammed, "jam should be cleared");
            assertEquals(0, sim.physics.balls, "every ball should still go out after the jam");
        }
    }

    @Test
    void missingDevicesAreReportedAndDontCrash() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.world.unplug(HardwareNames.TURRET_MOTOR, HardwareNames.TURRET_ENCODER,
                    HardwareNames.FLOWER_SERVO, HardwareNames.POLLEN_HOOD_SERVO, HardwareNames.NECTAR_HOOD_SERVO);
            sim.init("ChunkMode");
            List<String> init = sim.telemetryLines();
            assertTrue(init.contains("CHECK THE ROBOT:"), "INIT should warn the drive team: " + init);
            assertTrue(init.stream().anyMatch(l -> l.contains("MISSING") && l.contains(HardwareNames.TURRET_MOTOR)),
                    "INIT should name what's missing: " + init);

            sim.start();
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 0, 6), "should still shoot without them");
            assertTrue(sim.telemetry("STATUS").contains("MISSING"), "STATUS line should keep saying it");
        }
    }

    @Test
    void noFlywheelEncoderStillShoots() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.world.unplug(HardwareNames.FLYWHEEL_MOTOR); // the motor with the encoder
            sim.init("ChunkMode").start();
            sim.hold("rt", 1.0);
            Set<String> statuses = sim.watch("Shooter", 8, () -> sim.physics.balls == 0);
            assertTrue(statuses.contains("SHOOTING_NO_ENCODER"), "should shoot on a timer: " + statuses);
            assertEquals(0, sim.physics.balls, "every ball should still get fed");
            assertTrue(sim.physics.shotsLaunched >= 1, "at least some should make it out on one motor");
        }
    }

    @Test
    void autoDrivesItsPathAndTeleopKeepsThePose() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.init("JustMoveAuto");
            assertTrue(sim.world.warnings().stream().allMatch(w -> w.contains("allowed in auto")),
                    "auto may stow in init, nothing else: " + sim.world.warnings());
            sim.start();
            assertTrue(sim.waitUntil(() -> Math.hypot(sim.physics.x - 127.08, sim.physics.y - 40.59) < 2, 5),
                    String.format("auto should reach (127.1, 40.6), robot at (%.1f, %.1f)", sim.physics.x, sim.physics.y));
            sim.stop();

            double autoEndX = sim.physics.x;
            sim.init("ChunkMode");
            assertTrue(sim.telemetryLines().stream().anyMatch(l -> l.startsWith("Pose from auto: (")),
                    "teleop should pick up auto's pose: " + sim.telemetryLines());
            sim.start();
            double x = Double.parseDouble(sim.telemetry("X (in)"));
            assertEquals(autoEndX, x, 1.0, "teleop's pose should continue from where auto left the robot");
        }
    }
}
