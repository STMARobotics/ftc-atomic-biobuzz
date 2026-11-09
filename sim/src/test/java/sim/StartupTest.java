package sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakePivot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Set;

/**
 * INIT, auto and missing hardware.
 */
class StartupTest extends SimTest {

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
    void missingDevicesAreReportedAndDontCrash() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.world.unplug(HardwareNames.LIMELIGHT, HardwareNames.TURRET_MOTOR, HardwareNames.TURRET_ENCODER,
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
