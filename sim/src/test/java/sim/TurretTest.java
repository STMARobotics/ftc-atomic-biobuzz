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
 * The turret tracking, and its safeties.
 */
class TurretTest extends SimTest {

    @Test
    void turretTracksTheTagAndLosesItWhenTheCellTips() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            // in front of the upward red cell, robot turned away a bit so the tag starts off center
            sim.physics.placeRobot(59, 22, Math.PI / 2 + 0.3);
            sim.init("ChunkMode");
            SimHarness.sleep(200);
            double start = sim.physics.turretDeg;
            assertEquals(start, sim.physics.turretDeg, 0.01, "nothing moves during INIT");
            sim.start();
            assertTrue(sim.waitUntil(() -> "true".equals(sim.telemetry("Limelight has target")), 1), "the tag should be in view");
            assertTrue(sim.waitUntil(() -> Math.abs(Double.parseDouble(sim.telemetry("Limelight tx (deg)"))) < 2, 3),
                    "the turret should center the tag, tx " + sim.telemetry("Limelight tx (deg)"));
            System.out.printf("TRACK turned %.0f deg to center the tag%n", sim.physics.turretDeg - start);
            assertTrue(Math.abs(sim.physics.turretDeg - start) > 8, "it should have had to move");
            assertEquals(null, sim.telemetry("TURRET OFF"), "the safety shouldn't trip in normal use");

            sim.hold("rx", 0.35); // turn the robot, the turret should hold on the tag
            SimHarness.sleep(600);
            sim.release("rx");
            SimHarness.sleep(800);
            double tx = Double.parseDouble(sim.telemetry("Limelight tx (deg)"));
            System.out.printf("TRACK after turning the robot: tx %.1f, heading %.0f deg%n", tx, Math.toDegrees(sim.physics.heading));
            assertEquals("true", sim.telemetry("Limelight has target"), "it should still have it");
            assertTrue(Math.abs(tx) < 3, "still centered, tx " + tx);

            // tip the Hive: the tag's face turns away, same as the real thing
            SimCollisions.Hive red = sim.physics.collisions.redHive;
            for (int i = 0; i < 30 && red.tips == 0; i++) {
                sim.physics.spawnBall(SimCollisions.POLLEN, 57, 62, 62);
                SimHarness.sleep(300);
            }
            assertEquals(1, red.tips, "it should have tipped");
            assertTrue(sim.waitUntil(() -> "false".equals(sim.telemetry("Limelight has target")), 2),
                    "with the cell tipped toward us the tag shouldn't be visible");
        }
    }

    // what really shut the turret off: spinning makes it swing out and whip back through where it started,
    // which the stall check took for not moving
    @Test
    void spinningTheRobotDoesntShutTheTurretOff() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(72, 30, Math.PI / 2);
            sim.init("ChunkMode");
            sim.press("b");
            sim.start();
            double[] range = {sim.physics.turretDeg, sim.physics.turretDeg};
            // long enough each way to run it into both ends, that's what makes it whip back
            for (double rx : new double[]{1.0, -1.0, 1.0, -0.7, 0.7}) {
                sim.hold("rx", rx);
                for (int i = 0; i < 80; i++) {
                    SimHarness.sleep(50);
                    range[0] = Math.min(range[0], sim.physics.turretDeg);
                    range[1] = Math.max(range[1], sim.physics.turretDeg);
                    assertEquals(null, sim.telemetry("TURRET OFF"),
                            String.format("the safety tripped on a healthy turret, it had gone %.0f to %.0f", range[0], range[1]));
                }
            }
            sim.release("rx");
            System.out.printf("SPIN turret went from %.0f to %.0f deg%n", range[0], range[1]);
            assertTrue(range[1] > 500 || range[0] < 40, "the spin should have reached the end of its travel");
        }
    }

    // and it still has to catch a real one: a turret that can't move
    @Test
    void aJammedTurretStillShutsOff() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(72, 30, Math.PI / 2);
            sim.init("ChunkMode");
            sim.press("b");
            sim.physics.turretJammed = true;
            sim.start();
            sim.hold("rx", 1.0); // keeps the target moving away from it
            assertTrue(sim.waitUntil(() -> sim.telemetry("TURRET OFF") != null, 4), "a jam should shut it off");
            System.out.println("JAM " + sim.telemetry("TURRET OFF"));
        }
    }

    @Test
    void shotsGoWhereTheTurretPoints() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(59, 22, Math.PI / 2 + 0.3);
            sim.init("ChunkMode").start();
            SimHarness.sleep(1500);
            int before = sim.physics.collisions.pieces().length / 4;
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls < 4, 4), "a ball should leave");
            SimHarness.sleep(350);
            double[] p = sim.physics.collisions.pieces();
            double xNow = p[before * 4 + 1];
            System.out.printf("TRACK shot heading for x=%.0f, the tag is at x=59%n", xNow);
            assertEquals(59, xNow, 6, "the robot is turned away but the turret should still send it at the tag");
        }
    }
}
