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
 * Shooting while driving: at the edges of what it can do.
 */
class ShootingOnTheMoveLimitsTest extends SimTest {

    @Test
    void slowShotsScoreEitherWay() throws Exception {
        int lead = scoreWhileStrafing(0.5, true), plain = scoreWhileStrafing(0.5, false);
        reportMove("8 in/s", lead, plain);
        assertTrue(lead >= 3, "slow should be easy, got " + lead);
    }

    // for the record, not held to anything: flat out, the flywheel isn't even up to speed until well past the cell
    @Test
    void fullSpeedForTheRecord() throws Exception {
        reportMove("42 in/s", scoreWhileStrafing(1.0, true), scoreWhileStrafing(1.0, false));
    }
}
