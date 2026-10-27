package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/**
 * Shoot-and-move math. Pure numbers in, pure numbers out: no hardware and no subsystems, so it's
 * easy to test and neither the shooter nor the turret needs to know about it.
 *
 * The idea: the ball keeps the robot's own velocity, so aim at where the target would be if the
 * robot were standing still. Guess the flight time from distance, shift the target by
 * robot velocity * flight time, re-measure the distance, and repeat a couple of times.
 */
public final class ShotCalculator {

    private ShotCalculator() {}

    /** Everything the shooter and turret need for one shot. */
    public static final class Solution {
        public final double turretAngleDeg;
        public final double rpm;
        public final double hoodPollenAngleDeg;
        public final double hoodNectarAngleDeg;

        public Solution(double turretAngleDeg, double rpm, double hoodPollenAngleDeg, double hoodNectarAngleDeg) {
            this.turretAngleDeg = turretAngleDeg;
            this.rpm = rpm;
            this.hoodPollenAngleDeg = hoodPollenAngleDeg;
            this.hoodNectarAngleDeg = hoodNectarAngleDeg;
        }
    }

    /**
     * @param robotPose where the robot is, from the drivetrain
     * @param robotVelocity how fast the robot is moving, from the drivetrain
     * @param targetPose the hive cell to hit
     */
    public static Solution calculate(Pose robotPose, Velocity robotVelocity, Pose targetPose) {
        // TODO: 1. distance from the robot to the target
        // TODO: 2. flight time from distance (lookup table), shift the target by -velocity * time,
        //          and repeat 2 or 3 times
        // TODO: 3. turret angle = direction to the shifted target, minus the robot's heading
        // TODO: 4. rpm and both hood angles (pollen, nectar) from the final distance (one lookup table)
        // TODO: 5. physics check: fly the ball once with the real physics (gravity, drag, and the robot's own
        //          velocity added), see where it crosses the cell opening, and nudge the hood angle and turret
        //          yaw until the predicted miss is near zero. Keep the table answer if the refined one is
        //          worse. In simulation this took toward/away shots at 2 ft/s from about 52% to 100%.

        // Until then, a fixed shot so everything else keeps working
        return new Solution(0, Constants.Shooter.TARGET_RPM,
                Constants.Shooter.DEFAULT_HOOD_ANGLE_DEG, Constants.Shooter.DEFAULT_HOOD_ANGLE_DEG);
    }
}
