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
 * Which tags count, and finding one when there are none in view.
 */
class TagSearchTest extends SimTest {

    @Test
    void eachCellsTagsAreOnlySeenFromItsOwnSide() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(59.26, 16, Math.PI / 2);
            assertNotNull(tagAhead(sim, 34, 37), "red's raised cell (34-37) from low y");
            sim.physics.placeRobot(84.76, 132, -Math.PI / 2);
            double[] blue = tagAhead(sim, 42, 45);
            assertNotNull(blue, "blue's raised cell (42-45) from high y");
            assertEquals(0, blue[1], 8, "and it should be about straight ahead, tx " + blue[1]);
            sim.physics.placeRobot(84.76, 16, Math.PI / 2);
            assertEquals(null, tagAhead(sim, 42, 45), "blue's underside isn't visible from low y");
            sim.physics.placeRobot(59.26, 132, -Math.PI / 2);
            assertEquals(null, tagAhead(sim, 34, 37), "red's underside isn't visible from high y");
        }
    }

    @Test
    void redPicksRedTags() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            placeSeeingOnlyRed(sim);
            assertTrue(sim.physics.limelightTags().size() > 0, "the camera should see something");
            sim.init("ChunkMode");
            sim.press("b");
            SimHarness.sleep(100);
            double start = sim.physics.turretDeg;
            sim.start();
            assertTrue(sim.waitUntil(() -> "true".equals(sim.telemetry("Limelight has target")), 1), "red's tags count");
            assertEquals("RED", sim.telemetry("Tags from"));
            assertTrue(sim.waitUntil(() -> Math.abs(sim.physics.turretDeg - start) > 5, 2), "and the turret goes for them");
        }
    }

    // from the start spot all 4 cells are in view, and the lowered ones are the closest
    @Test
    void itPicksItsOwnRaisedCell() throws Exception {
        for (String side : new String[]{"b", "x"}) {
            try (SimHarness sim = new SimHarness()) {
                org.firstinspires.ftc.teamcode.util.PoseStorage.clear();
                sim.physics.placeRobot(24, 72, 0);
                SimHarness.sleep(300);
                sim.init("ChunkMode");
                sim.press(side);
                sim.start();
                assertTrue(sim.waitUntil(() -> "true".equals(sim.telemetry("Limelight has target")), 2), "it should have a tag");
                SimHarness.sleep(1000);
                // the turret angle goes up clockwise. Red's raised cell is to the right, blue's to the left
                double off = sim.physics.turretDeg - org.firstinspires.ftc.teamcode.RobotConstants.Turret.FORWARD_ANGLE_DEG;
                System.out.printf("PICK %s: turret %.0f deg right of forward, distance %s%n", side, off, sim.telemetry("Shot distance (in)"));
                if (side.equals("b")) assertEquals(20.4, off, 4, "red's raised cell");
                else assertEquals(-12.1, off, 4, "blue's raised cell");
            }
        }
    }

    @Test
    void blueIgnoresRedTags() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            placeSeeingOnlyRed(sim);
            assertTrue(sim.physics.limelightTags().size() > 0, "the camera should see something");
            sim.init("ChunkMode");
            sim.press("x");
            SimHarness.sleep(100);
            double start = sim.physics.turretDeg;
            sim.start();
            assertEquals("BLUE", sim.telemetry("Tags from"));
            // it'll sweep around looking for one of its own, through red's tags, and shouldn't take them
            double[] range = {start, start};
            for (int i = 0; i < 80; i++) {
                SimHarness.sleep(50);
                range[0] = Math.min(range[0], sim.physics.turretDeg);
                range[1] = Math.max(range[1], sim.physics.turretDeg);
                assertEquals("false", sim.telemetry("Limelight has target"), "red's tags don't count for blue");
            }
            assertTrue(range[1] - range[0] > 90, "it should have looked around");
        }
    }

    // nothing in view at the start: facing away from the cell, with the camera on the turret
    @Test
    void turretFindsTheTagWhenItCantSeeOne() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(72, 30, -Math.PI / 2);
            sim.init("ChunkMode");
            sim.press("b");
            SimHarness.sleep(100);
            assertTrue(sim.physics.limelightTags().isEmpty(), "it should start with nothing in view");
            double before = sim.physics.turretDeg;
            SimHarness.sleep(400);
            assertEquals(before, sim.physics.turretDeg, 0.01, "nothing moves during INIT");
            sim.start();
            long from = System.nanoTime();
            assertTrue(sim.waitUntil(() -> "true".equals(sim.telemetry("Limelight has target")), 6), "it should find the tag");
            System.out.printf("SEARCH found a tag in %.1f s%n", (System.nanoTime() - from) / 1e9);
            assertTrue(sim.waitUntil(() -> Math.abs(Double.parseDouble(sim.telemetry("Limelight tx (deg)"))) < 3, 2), "and center it");
            assertEquals("false", sim.telemetry("Turret searching"), "and stop looking");
        }
    }

    // lost it after having it: the robot turned away with the tag blocked. Finds it again, starting close by
    // off to the side and the search starts going the other way: it shouldn't have to go all the way around
    // first. This cost a drive-by every shot, the first ball went 3 s late
    @Test
    void turretFindsATagOffToTheSideQuickly() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(12, 17, Math.PI / 2);
            SimHarness.sleep(200);
            sim.init("ChunkMode");
            sim.press("b");
            SimHarness.sleep(100);
            assertTrue(sim.physics.limelightTags().isEmpty(), "it should start with nothing in view");
            sim.start();
            long from = System.nanoTime();
            assertTrue(sim.waitUntil(() -> "true".equals(sim.telemetry("Limelight has target")), 4), "it should find the tag");
            double took = (System.nanoTime() - from) / 1e9;
            System.out.printf("SEARCH found one off to the side in %.1f s%n", took);
            assertTrue(took < 2.2, "that should be quick, took " + took);
        }
    }

    @Test
    void turretFindsTheTagAgainAfterLosingIt() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(72, 30, Math.PI / 2);
            sim.init("ChunkMode");
            sim.press("b");
            sim.start();
            assertTrue(sim.waitUntil(() -> "true".equals(sim.telemetry("Limelight has target")), 2), "it should have the tag");
            SimHarness.sleep(500);
            // teleporting isn't something a gyro knows about, so as far as it can tell the tag moved
            sim.physics.placeRobot(120, 30, Math.PI / 2);
            assertTrue(sim.waitUntil(() -> "false".equals(sim.telemetry("Limelight has target")), 2), "it should lose it");
            assertTrue(sim.waitUntil(() -> "true".equals(sim.telemetry("Turret searching")), 2), "then start looking");
            assertTrue(sim.waitUntil(() -> "true".equals(sim.telemetry("Limelight has target")), 8), "and find it");
        }
    }

    @Test
    void turretDoesntSearchWithADeadCamera() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(72, 30, -Math.PI / 2);
            sim.init("ChunkMode").start();
            sim.physics.cameraDead = true;
            SimHarness.sleep(1000); // whatever it was doing, the frames have gone stale by now
            double at = sim.physics.turretDeg;
            SimHarness.sleep(1000);
            assertEquals("false", sim.telemetry("Turret searching"));
            assertEquals(at, sim.physics.turretDeg, 2, "no point sweeping a dead camera around");
        }
    }
}
