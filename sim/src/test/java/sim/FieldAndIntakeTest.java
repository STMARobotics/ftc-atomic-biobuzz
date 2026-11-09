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
 * The Hives, flowers and picking balls up.
 */
class FieldAndIntakeTest extends SimTest {

    @Test
    void fieldStartsLikeTheManual() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            SimHarness.sleep(500);
            int[] counts = new int[3];
            double[] p = sim.physics.collisions.pieces();
            for (int i = 0; i < p.length; i += 4) counts[(int) p[i]]++;
            assertEquals(24, counts[0], "8 pollen in gardens + 16 in flowers");
            assertEquals(8, counts[1], "red nectar: 3 in the cell + 5 in the alliance area");
            assertEquals(8, counts[2], "blue nectar");
            assertEquals(4, sim.physics.balls, "the robot starts with 4 pollen");
        }
    }

    @Test
    void hiveHoldsUnderItsStagedNectarThenTipsWhenFilled() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            SimCollisions.Hive red = sim.physics.collisions.redHive;
            SimHarness.sleep(1500);
            assertEquals(0, red.tips, "3 staged nectar shouldn't tip it");
            assertEquals(0.0, red.angle, 0.02, "it should hold at its end");
            assertTrue(red.inCell >= 2, "the staged nectar should stay in the cell, in cell: " + red.inCell);

            // drop pollen one at a time into the upward cell until it goes over
            int dropped = 0;
            while (red.tips == 0 && dropped < 30) {
                sim.physics.spawnBall(SimCollisions.POLLEN, 57, 62, 62);
                dropped++;
                SimHarness.sleep(350);
            }
            sim.waitUntil(() -> red.tips > 0, 3);
            assertTrue(dropped <= 10, "it shouldn't take much more than about 8 pollen, took " + dropped);
            System.out.printf("HIVE tipped after %d pollen, angle %.0f deg, %d left in cells%n",
                    dropped, Math.toDegrees(red.angle), red.inCell);
            assertEquals(1, red.tips, "it should count one tip");
            int inCellAtTip = red.inCell;
            SimHarness.sleep(3000); // all of it should be out by now, not just most of it
            System.out.printf("SPILL %d in cells when it tipped, %d after%n", inCellAtTip, red.inCell);
            double[] all = sim.physics.collisions.pieces();
            for (int i = 0; i + 3 < all.length; i += 4) {
                if (all[i + 3] > 20 && all[i + 1] > 45 && all[i + 1] < 75) {
                    System.out.printf("SPILL still up: type %.0f at x=%.1f y=%.1f z=%.1f%n", all[i], all[i + 1], all[i + 2], all[i + 3]);
                }
            }
            assertEquals(0, red.inCell, "what was in the cell should spill out when it tips down");
            assertEquals(60, Math.toDegrees(red.angle), 3, "it should end at its other stop");
            assertTrue(dropped > 1, "one ball shouldn't tip it");
        }
    }

    @Test
    void intakePicksUpABallAndRespectsTheLimit() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            sim.physics.placeRobot(40, 110, 0);
            sim.init("ChunkMode").start();
            sim.hold("rt", 1.0);
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 0, 6), "should fire the preloads first");
            sim.release("rt");
            SimHarness.sleep(300);
            // two balls right in front of the intake
            sim.physics.spawnBall(SimCollisions.POLLEN, 40 + 16, 112, 1.5);
            sim.physics.spawnBall(SimCollisions.BLUE_NECTAR, 40 + 16, 106, 2);
            sim.hold("lt", 1.0);
            SimHarness.sleep(800);
            sim.hold("ly", -0.25); // creep forward into them
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 2, 3), "should pick up both, holding " + sim.physics.balls);
            assertTrue(sim.physics.held.contains(SimCollisions.BLUE_NECTAR), "it should know which ones it has");
        }
    }

    @Test
    void flowerBarPullsPollenFromAFlower() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            // nose up to the flower at (140, 95.4)
            sim.physics.placeRobot(140 - 17, 95.4, 0);
            sim.physics.held.clear(); // empty handed, firing at the wall would just bounce back into the intake
            sim.physics.balls = 0;
            sim.init("ChunkMode").start();

            sim.hold("lt", 1.0);
            SimHarness.sleep(1000);
            assertEquals(0, sim.physics.balls, "the intake alone can't get pollen out of a flower");

            sim.press("a"); // bar out
            assertTrue(sim.waitUntil(() -> sim.physics.balls == 2, 5), "the bar should pull pollen out, got " + sim.physics.balls);
            SimServo out = sim.world.servoState(org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames.FLOWER_SERVO);
            assertEquals(org.firstinspires.ftc.teamcode.RobotConstants.Flower.DEPLOYED, out.physical, 0.05, "the bar should be all the way out");
            sim.press("x"); // battle mode brings the bar back
            SimHarness.sleep(1500);
            SimServo bar = sim.world.servoState(org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames.FLOWER_SERVO);
            assertEquals(org.firstinspires.ftc.teamcode.RobotConstants.Flower.STOWED, bar.physical, 0.05, "battle mode stows it");
        }
    }

    // the way a driver would actually do it: drive up to it with everything down
    @Test
    void canDriveUpToAFlowerAndTakeItsPollen() throws Exception {
        try (SimHarness sim = new SimHarness()) {
            // from the middle of the field toward the flower at (48.6, 140), facing it
            sim.physics.placeRobot(48.6, 96, Math.PI / 2);
            sim.physics.held.clear();
            sim.physics.balls = 0;
            sim.init("ChunkMode").start();
            sim.press("a");
            sim.hold("lt", 1.0);
            SimHarness.sleep(1200);
            sim.hold("ly", -0.5);
            assertTrue(sim.waitUntil(() -> sim.physics.balls >= 1, 8),
                    String.format("should collect from the flower, has %d, stopped at y=%.1f", sim.physics.balls, sim.physics.y));
            double first = sim.physics.y;
            assertTrue(sim.waitUntil(() -> sim.physics.balls >= 3, 4), "should keep collecting, has " + sim.physics.balls);
            SimHarness.sleep(1500); // keep pushing, how close does it really get?
            System.out.printf("FLOWER first ball at y=%.1f, got %d, stopped at y=%.1f, flower at y=140%n",
                    first, sim.physics.balls, sim.physics.y);
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
}
