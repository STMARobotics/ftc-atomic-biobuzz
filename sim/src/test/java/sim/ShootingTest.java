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
 * Getting balls out of the robot, including when things go wrong.
 */
class ShootingTest extends SimTest {

    @Test
    void shotsReallyFly() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(12, 110, 0);
            sim.init("ChunkMode").start();
            int before = sim.physics.collisions.pieces().length / 4;
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls < 4, 4), "a ball should leave");
            double top = 0, far = 0;
            long end = System.nanoTime() + 1_500_000_000L;
            while (System.nanoTime() < end) {
                double[] p = sim.physics.collisions.pieces();
                for (int i = before * 4; i + 3 < p.length; i += 4) {
                    top = Math.max(top, p[i + 3]);
                    far = Math.max(far, p[i + 1]);
                }
                SimHarness.sleep(10);
            }
            System.out.printf("SHOT peak %.0f in, reached x=%.0f from the robot at x=%.0f%n", top, far, sim.physics.x);
            assertTrue(top > 40, "a shot should go up past hive height, peak " + top);
            assertTrue(far > sim.physics.x + 15, "and forward");
        }
    }

    @Test
    void rtStillShootsWithNoTagAfterWaiting() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(30, 30, Math.PI); // facing the wall, nothing to see
            sim.init("ChunkMode").start();
            SimHarness.sleep(300);
            assertEquals("false", sim.telemetry("Limelight has target"));
            sim.physics.limelightHidden = true; // the camera works but never finds one
            sim.hold("rt", 1.0);
            SimHarness.sleep(3500);
            assertEquals(4, sim.physics.balls, "it should give the search time to find a tag first");
            assertTrue(sim.waitUntil(() -> sim.physics.balls < 4, 4), "then shoot anyway instead of sitting there");
        }
    }

    @Test
    void aDeadCameraShootsRightAway() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(72, 30, -Math.PI / 2);
            sim.init("ChunkMode").start();
            sim.physics.cameraDead = true;
            SimHarness.sleep(800);
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls < 4, 3), "nothing to wait for");
        }
    }

    @Test
    void shotsDontGetSprayedWhileSearching() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(72, 30, -Math.PI / 2);
            sim.physics.limelightHidden = true; // never finds anything
            sim.init("ChunkMode").start();
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls < 4, 8), "RT still shoots eventually");
            double at = sim.physics.turretDeg;
            SimHarness.sleep(600);
            assertEquals(at, sim.physics.turretDeg, 2, "the turret holds still while the gate's open");
        }
    }

    @Test
    void holdingRtShootsEveryBallAndCountsThem() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.init("ChunkMode").start();
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 0, 6), "balls left: " + sim.physics.balls);
            SimHarness.sleep(500); // the counter needs the flywheel to recover after the last ball
            assertEquals(4, sim.physics.shotsLaunched, "balls that actually left the shooter");
            assertEquals("4", sim.telemetry("Shots fired"), "the code's shot counter");
        }
    }

    @Test
    void weakBatteryStillShootsInsteadOfWaitingForever() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.batteryRestVolts = 9.3; // can't reach full speed anymore
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
}
