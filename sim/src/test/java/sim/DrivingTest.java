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
 * How the drivetrain behaves, and the heading bias.
 */
class DrivingTest extends SimTest {

    // not a spec: prints what the force model does so it can be compared with the real robot
    @Test
    void drivetrainNumbersAreSane() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(12, 110, 0); // open floor, nothing to hit for 100 in
            sim.init("ChunkMode").start();
            long t0 = System.nanoTime();
            sim.hold("ly", -1.0);
            double t50 = -1, t90 = -1, top = 0;
            while ((System.nanoTime() - t0) / 1e9 < 0.9) {
                double v = sim.physics.vForward, t = (System.nanoTime() - t0) / 1e9;
                top = Math.max(top, v);
                if (t50 < 0 && v > 30) t50 = t;
                if (t90 < 0 && v > 54) t90 = t;
                SimHarness.sleep(2);
            }
            double battery = sim.physics.batteryVolts;
            sim.release("ly");
            long t1 = System.nanoTime();
            double xAtRelease = sim.physics.x;
            assertTrue(sim.waitUntil(() -> Math.abs(sim.physics.vForward) < 1, 1.5), "should brake to a stop");
            double stop = (System.nanoTime() - t1) / 1e9;
            System.out.printf("DRIVE to 30in/s %.2fs, to 54in/s %.2fs, top %.0f in/s, battery %.1fV, stop %.2fs in %.1f in%n",
                    t50, t90, top, battery, stop, Math.abs(sim.physics.x - xAtRelease));
            sim.hold("rx", 1.0);
            SimHarness.sleep(600);
            System.out.printf("TURN %.0f deg/s after 0.6s%n", Math.toDegrees(Math.abs(sim.physics.vTurn)));
            assertTrue(top > 45 && top < 70, "top speed " + top);
        }
    }

    @Test
    void robotStopsAgainstTheAFrameFootAndTheWall() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.init("ChunkMode").start(); // starts at (24, 72) facing the A-frame
            double startX = sim.physics.x;

            sim.hold("ly", -1.0);
            SimHarness.sleep(2000);
            double hit = sim.physics.x;
            System.out.printf("FOOT stopped at x=%.1f y=%.1f heading %.1f deg%n", hit, sim.physics.y,
                    Math.toDegrees(sim.physics.heading));
            // the foot bar is at x = 48.3, the chassis reaches 8.8 in front of center
            assertTrue(hit > 35 && hit < 42, "should stop at the A-frame foot, robot at x=" + hit);
            assertEquals(hit - startX, Double.parseDouble(sim.telemetry("X (in)")), 1.5,
                    "the localizer should only count real motion, not wheels pushing the frame");
            assertTrue(sim.physics.batteryVolts < 12.0, "pushing should pull the battery down: " + sim.physics.batteryVolts);

            sim.hold("ly", 1.0);
            SimHarness.sleep(2000);
            System.out.printf("WALL stopped at x=%.1f%n", sim.physics.x);
            // the back of the chassis is 7.8 in behind center, the wall is about an inch in
            assertTrue(sim.physics.x > 7.5 && sim.physics.x < 11, "should stop at the wall, robot at x=" + sim.physics.x);
        }
    }

    @Test
    void topSpeedAtEachHeading() throws Exception {
        org.firstinspires.ftc.teamcode.RobotConstants.Drive.HEADING_BIAS = false; // measuring what it's for
        StringBuilder report = new StringBuilder("HEADING");
        for (double deg : new double[]{0, 45, 90}) {
            double t = Math.toRadians(deg);
            try (SimHarness sim = new SimHarness()) {
                sim.physics.placeRobot(24, 112, t); // field centric zeroes here, so sticks are robot relative
                SimHarness.sleep(300);
                sim.init("ChunkMode").start();
                sim.hold("ly", -Math.cos(t));
                sim.hold("lx", Math.sin(t));
                sim.waitUntil(() -> sim.physics.x > 44, 5);
                double fromX = sim.physics.x, fromY = sim.physics.y, at = System.nanoTime() / 1e9;
                sim.waitUntil(() -> sim.physics.x > 110, 5);
                double speed = Math.hypot(sim.physics.x - fromX, sim.physics.y - fromY) / (System.nanoTime() / 1e9 - at);
                report.append(String.format(" %.0f:%.0f(y%.0f,h%.0f)", deg, speed, sim.physics.y, Math.toDegrees(sim.physics.heading)));
            }
        }
        org.firstinspires.ftc.teamcode.RobotConstants.Drive.HEADING_BIAS = true;
        System.out.println(report + " in/s");
    }

    @Test
    void headingEasesToTheNearest90WhenDriving() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            double start = crookedThenDrive(sim, 0.5);
            assertTrue(Math.abs(start) > 15, "it should be crooked to begin with, " + start);
            sim.hold("ly", -1.0);
            assertTrue(sim.waitUntil(() -> Math.abs(sim.physics.heading % (Math.PI / 2)) < Math.toRadians(4)
                    || Math.abs(sim.physics.heading % (Math.PI / 2)) > Math.toRadians(86), 2),
                    "it should square up, heading " + Math.toDegrees(sim.physics.heading));
            System.out.printf("BIAS from %.0f to %.0f deg%n", start, Math.toDegrees(sim.physics.heading));
        }
    }

    // the settings page really changes what the running OpMode does, and the snap really is off when it's off
    @Test
    void settingsChangeTheRunningRobot() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            double start = crookedThenDrive(sim, 0.5);
            assertTrue(SimSettings.set("Drive.HEADING_BIAS", 0), "it should know that setting");
            SimHarness.sleep(300);
            start = Math.toDegrees(sim.physics.heading);
            sim.hold("ly", -1.0);
            SimHarness.sleep(600);
            assertEquals(start, Math.toDegrees(sim.physics.heading), 5, "off means it leaves the heading alone");
            assertTrue(SimSettings.set("Drive.HEADING_BIAS", 1));
            assertTrue(sim.waitUntil(() -> Math.abs(sim.physics.heading % (Math.PI / 2)) < Math.toRadians(4)
                    || Math.abs(sim.physics.heading % (Math.PI / 2)) > Math.toRadians(86), 2), "back on it snaps, without restarting");
        }
    }

    @Test
    void settingsStayInRangeAndEveryOneIsListed() throws Exception {
        assertTrue(SimSettings.set("Drive.BIAS_MAX_TURN", 99));
        assertEquals(1, SimSettings.get("Drive.BIAS_MAX_TURN"), 0, "it should stop at the top of its range");
        assertEquals(false, SimSettings.set("Drive.NOT_A_THING", 1));
        String json = SimSettings.json();
        for (String name : new String[]{"Drive.BIAS_kP", "Shooter.LEAD_SHOTS", "Turret.SEARCH_DEG_PER_SEC", "SimCollisions.bistableTorque"}) {
            assertTrue(json.contains("\"" + name + "\""), name + " should be on the page");
        }
    }

    @Test
    void headingBiasLeavesTheDriverAlone() throws Exception {
        for (String mode : new String[]{"intake", "shoot", "slow", "turn"}) {
            try (SimHarness sim = new SimHarness()) {
                double start = crookedThenDrive(sim, 0.5);
                SimHarness.sleep(300); // let it settle
                start = Math.toDegrees(sim.physics.heading);
                if (mode.equals("intake")) sim.hold("lt", 1.0);
                if (mode.equals("shoot")) sim.hold("rt", 1.0);
                sim.hold("ly", mode.equals("slow") ? -0.4 : -1.0);
                if (mode.equals("turn")) {
                    sim.hold("rx", -0.7); // turning further away from square, the bias would go the other way
                    SimHarness.sleep(500);
                    assertTrue(Math.toDegrees(sim.physics.heading) > start + 5, "the stick wins, " + Math.toDegrees(sim.physics.heading));
                } else {
                    SimHarness.sleep(700);
                    assertEquals(start, Math.toDegrees(sim.physics.heading), 6, "it should leave the heading alone while " + mode);
                }
            }
        }
    }
}
