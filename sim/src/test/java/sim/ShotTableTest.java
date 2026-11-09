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
 * The shot table, through the real OpMode: the camera measures and it sets everything itself.
 */
class ShotTableTest extends SimTest {

    // the shot table, through the real OpMode: the camera measures the distance and it sets everything itself
    @Test
    void theShotTableScoresFromAnywhereTheTagIsInView() throws Exception {
        StringBuilder report = new StringBuilder("TABLE");
        int total = 0;
        for (double distance : new double[]{38, 42, 46}) {
            int scored = scoreFrom(distance, false);
            report.append(String.format(" %.0fin:%d/4", distance, scored));
            total += scored;
        }
        System.out.println(report);
        assertTrue(total >= 10, "the table should land most shots: " + report);
    }

    @Test
    void theShotTableWorksForBlueToo() throws Exception {
        int scored = scoreFrom(42, true);
        System.out.println("TABLE blue 42in:" + scored + "/4");
        assertTrue(scored >= 3, "blue from 42 in, got " + scored);
    }

    @Test
    void theHoodFollowsTheDistance() throws Exception {
        double[] hood = new double[2];
        for (int i = 0; i < 2; i++) {
            try (SimHarness sim = new SimHarness()) {
                placeFacingTag(sim, i == 0 ? 37 : 46, false);
                SimHarness.sleep(500); // settle
                sim.init("ChunkMode").start();
                SimHarness.sleep(1000);
                hood[i] = sim.world.servoState(org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames.POLLEN_HOOD_SERVO).physical;
            }
        }
        System.out.printf("HOOD 37in:%.2f 46in:%.2f%n", hood[0], hood[1]);
        assertTrue(hood[0] > hood[1] + 0.1, "closer should use more hood");
    }
}
