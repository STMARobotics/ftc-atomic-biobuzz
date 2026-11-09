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
 * What the test classes share. The tests run the real OpModes in the sim the way a drive team would and
 * check what physically happens. Each test is one thing that has to keep working. Run before every comp:
 *   .\\gradlew -p sim test
 * They run in real time. Each class gets its own JVM and they run side by side (see build.gradle), so
 * keep any one class under about 40 s, that's what sets how long the whole thing takes.
 */
abstract class SimTest {

    @BeforeEach
    void clearStatics() {
        PoseStorage.clear();
    }

    static final double RED_TAG_X = 59.26, RED_TAG_Y = 59.11;

    static final double BLUE_TAG_X = 84.76, BLUE_TAG_Y = 84.89;

    // this far from the tag (camera to tag) and facing it, in the gap between the flowers
    static void placeFacingTag(SimHarness sim, double distance, boolean blue) {
        double tagX = blue ? BLUE_TAG_X : RED_TAG_X, tagY = blue ? BLUE_TAG_Y : RED_TAG_Y;
        double fromTurret = distance + SimPhysics.CAMERA_FORWARD;
        double dx = tagX - 72;
        double dy = Math.sqrt(fromTurret * fromTurret - dx * dx) * (blue ? -1 : 1);
        double facing = Math.atan2(dy, dx);
        // from the turret axis back to the robot's center
        sim.physics.placeRobot(tagX - fromTurret * Math.cos(facing) - SimPhysics.TURRET_AXIS_X * Math.cos(facing),
                tagY - fromTurret * Math.sin(facing) - SimPhysics.TURRET_AXIS_X * Math.sin(facing), facing);
    }

    // fires everything from this far (camera to tag). How many stay in the cell
    int scoreFrom(double distance, boolean blue) throws Exception {
        try (SimHarness sim = new SimHarness()) {
            SimCollisions.Hive hive = blue ? sim.physics.collisions.blueHive : sim.physics.collisions.redHive;
            placeFacingTag(sim, distance, blue);
            SimHarness.sleep(600);
            int before = hive.inCell;
            sim.init("ChunkMode");
            sim.press(blue ? "x" : "b");
            sim.start();
            sim.hold("rt", 1.0);
            sim.waitUntil(() -> sim.physics.balls == 0, 6);
            SimHarness.sleep(1500);
            sim.release("rt");
            SimHarness.sleep(800);
            return hive.inCell - before;
        }
    }

    // driving along the wall past the cell, holding RT the whole way. How many stay in it
    int scoreWhileStrafing(double stick, boolean lead) throws Exception {
        return scoreWhileDriving(12, 17, stick, 0, lead);
    }

    int scoreWhileDriving(double startX, double startY, double stick, double backStick, boolean lead) throws Exception {
        boolean was = org.firstinspires.ftc.teamcode.RobotConstants.Shooter.LEAD_SHOTS;
        org.firstinspires.ftc.teamcode.RobotConstants.Shooter.LEAD_SHOTS = lead;
        try (SimHarness sim = new SimHarness()) {
            SimCollisions.Hive red = sim.physics.collisions.redHive;
            sim.physics.placeRobot(startX, startY, Math.PI / 2); // clear of the flowers on the wall
            SimHarness.sleep(500);
            assertEquals(startX, sim.physics.x, 1, "something's in the way of the start spot, the robot got shoved off it");
            int before = red.inCell;
            sim.init("ChunkMode");
            sim.press("b");
            sim.start();
            sim.hold("lx", stick); // field centric from where it's facing, so this is along the wall
            if (backStick != 0) sim.hold("ly", backStick);
            sim.hold("rt", 1.0);
            sim.waitUntil(() -> sim.physics.balls < 4 || sim.physics.x > 125, 8);
            double first = sim.physics.x;
            String tracking = sim.telemetry("Tracking tags") + " at " + sim.telemetry("Shot distance (in)") + " in";
            double firstAt = System.nanoTime() / 1e9;
            sim.waitUntil(() -> sim.physics.balls == 0 || sim.physics.x > 125, 6);
            double speed = (sim.physics.x - first) / Math.max(0.01, System.nanoTime() / 1e9 - firstAt);
            System.out.printf("MOVE %s stick %.2f: %.0f in/s, shots from x=%.0f to x=%.0f, left %d, tracking %s%n",
                    lead ? "lead" : "no lead", stick, speed, first, sim.physics.x, sim.physics.balls, tracking);
            sim.release("lx");
            sim.release("ly");
            SimHarness.sleep(1800);
            sim.release("rt");
            return red.inCell - before;
        } finally {
            org.firstinspires.ftc.teamcode.RobotConstants.Shooter.LEAD_SHOTS = was;
        }
    }

    static void reportMove(String name, int lead, int plain) {
        System.out.printf("MOVE %s: lead %d/4, no lead %d/4%n", name, lead, plain);
    }

    // the tag (id range) closest to straight ahead, null if none of them are
    static double[] tagAhead(SimHarness sim, int firstId, int lastId) {
        for (double[] tag : sim.physics.limelightTags()) {
            if (tag[0] >= firstId && tag[0] <= lastId) return tag;
        }
        return null;
    }

    // from here only red's tags are in view, off to the left. Clear of the flowers, they'd shove it away
    static void placeSeeingOnlyRed(SimHarness sim) {
        sim.physics.placeRobot(72, 16, Math.PI / 2);
    }

    // gets the robot crooked the way a driver would, by turning it, then drives across the field
    double crookedThenDrive(SimHarness sim, double turnSeconds) throws Exception {
        sim.physics.placeRobot(24, 112, 0);
        sim.init("ChunkMode").start();
        sim.hold("rx", -0.5);
        SimHarness.sleep((long) (turnSeconds * 1000));
        sim.release("rx");
        return Math.toDegrees(sim.physics.heading);
    }
}
