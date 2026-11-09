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
 * Shooting while driving: the ones that have to work.
 */
class ShootingOnTheMoveFastTest extends SimTest {

    @Test
    void leadingScoresAlongTheWall() throws Exception {
        int lead = scoreWhileStrafing(0.75, true), plain = scoreWhileStrafing(0.75, false);
        reportMove("22 in/s", lead, plain);
        assertTrue(lead >= 3, "it should score, got " + lead);
        assertTrue(lead > plain, "and beat not leading, " + lead + " vs " + plain);
    }

    @Test
    void leadingScoresOnADiagonal() throws Exception {
        int lead = scoreWhileDriving(24, 34, 0.7, 0.45, true), plain = scoreWhileDriving(24, 34, 0.7, 0.45, false);
        reportMove("diagonal", lead, plain);
        assertTrue(lead >= 3, "it should score, got " + lead);
        assertTrue(lead > plain, "and beat not leading, " + lead + " vs " + plain);
    }
}
