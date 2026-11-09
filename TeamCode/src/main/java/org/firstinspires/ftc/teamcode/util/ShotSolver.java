package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.RobotConstants.Limelight;
import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;

/**
 * Where to point the turret and how far the shot is, including while driving. It needs no field
 * position: everything is relative to the target.
 *
 * Each camera frame gives the target's position from the turret (tx, distance from ty, turret angle).
 * Those are stored in a frame that doesn't turn with the robot, using the gyro's heading (any zero works,
 * so a drifted or wrong pose doesn't matter). The target doesn't move, so however that position
 * changes is the robot's own velocity, and the same filter smooths the noise out of both.
 *
 * A ball keeps the robot's velocity when it leaves, so instead of the real target we aim at where
 * the target appears to be once the ball gets there: position + (how it's changing) * flight time.
 * Flight time depends on distance and distance on flight time, a few passes settles it.
 *
 * Plain math, no hardware, all angles in degrees counterclockwise and distances in inches.
 */
public final class ShotSolver {

    public static final class Aim {
        /** Which way the turret should point, relative to the robot's front. Counterclockwise is positive */
        public final double turretBearingDeg;
        /** The distance to look up in the shot table */
        public final double distanceIn;
        /** How fast the robot is moving, inches/sec */
        public final double speedInPerSec;

        Aim(double turretBearingDeg, double distanceIn, double speedInPerSec) {
            this.turretBearingDeg = turretBearingDeg;
            this.distanceIn = distanceIn;
            this.speedInPerSec = speedInPerSec;
        }
    }

    private final ShotLookupTable table;

    // the target from the turret, in the gyro's frame, as of lastFrameSec
    private double x, y, vx, vy;
    private boolean tracking = false;
    private double lastFrameSec;

    public ShotSolver(ShotLookupTable table) {
        this.table = table;
    }

    /**
     * A new camera frame.
     * @param txDeg tag right of the camera's center, as the Limelight reports it
     * @param cameraDistanceIn flat distance from the camera to the tag
     * @param turretBearingDeg turret angle relative to the robot when the frame was taken
     * @param headingDeg gyro heading when the frame was taken
     * @param frameSec when the frame was taken
     */
    public void addFrame(double txDeg, double cameraDistanceIn, double turretBearingDeg, double headingDeg, double frameSec) {
        double tx = Math.toRadians(txDeg);
        // from the turret axis: the camera sits out in front of it
        double forward = Limelight.CAMERA_FORWARD_IN + cameraDistanceIn * Math.cos(tx);
        double left = -cameraDistanceIn * Math.sin(tx);
        double bearing = Math.toRadians(headingDeg + turretBearingDeg);
        double measuredX = forward * Math.cos(bearing) - left * Math.sin(bearing);
        double measuredY = forward * Math.sin(bearing) + left * Math.cos(bearing);

        double dt = frameSec - lastFrameSec;
        if (tracking && dt > 0 && dt < Shooter.AIM_COAST_SEC) {
            double predictedX = x + vx * dt;
            double predictedY = y + vy * dt;
            double missX = measuredX - predictedX;
            double missY = measuredY - predictedY;
            if (Math.hypot(missX, missY) < Shooter.AIM_JUMP_IN) {
                x = predictedX + Shooter.AIM_FILTER_POSITION * missX;
                y = predictedY + Shooter.AIM_FILTER_POSITION * missY;
                vx += Shooter.AIM_FILTER_VELOCITY * missX / dt;
                vy += Shooter.AIM_FILTER_VELOCITY * missY / dt;
                double speed = Math.hypot(vx, vy);
                if (speed > Shooter.MAX_ROBOT_SPEED_IN_PER_SEC) {
                    vx *= Shooter.MAX_ROBOT_SPEED_IN_PER_SEC / speed;
                    vy *= Shooter.MAX_ROBOT_SPEED_IN_PER_SEC / speed;
                }
                lastFrameSec = frameSec;
                return;
            }
        }
        // first sighting, a long gap or it jumped (a different tag, a bumped gyro): start over
        if (dt < 0 && tracking) return; // an old frame showed up late
        x = measuredX;
        y = measuredY;
        vx = 0;
        vy = 0;
        tracking = true;
        lastFrameSec = frameSec;
    }

    /**
     * True while the last frame is recent enough to go on
     */
    public boolean isTracking(double nowSec) {
        return tracking && nowSec - lastFrameSec <= Shooter.AIM_COAST_SEC;
    }

    /**
     * The aim right now, null if we haven't seen the target recently.
     * @param lead false ignores the robot's velocity, just point at the target
     */
    public Aim solve(double nowSec, double headingDeg, boolean lead) {
        if (!isTracking(nowSec)) return null;

        double speed = Math.hypot(vx, vy);
        // standing still shouldn't jitter from camera noise
        boolean moving = lead && speed >= Shooter.MIN_LEAD_SPEED_IN_PER_SEC;
        double useVx = moving ? vx : 0;
        double useVy = moving ? vy : 0;

        // catch up from the frame's age, then the ball's trip
        double age = nowSec - lastFrameSec;
        double nowX = x + useVx * age;
        double nowY = y + useVy * age;
        double aimX = nowX, aimY = nowY;
        if (moving) {
            for (int i = 0; i < 3; i++) {
                double flight = Shooter.FEED_DELAY_SEC + table.get(cameraDistance(aimX, aimY)).flightSec;
                aimX = nowX + useVx * flight;
                aimY = nowY + useVy * flight;
            }
        }

        // keep the tags in frame, losing the target would be worse than missing one shot
        double toTarget = Math.toDegrees(Math.atan2(nowY, nowX));
        double leadDeg = wrap(Math.toDegrees(Math.atan2(aimY, aimX)) - toTarget);
        leadDeg = Math.max(-Shooter.MAX_LEAD_DEG, Math.min(Shooter.MAX_LEAD_DEG, leadDeg));
        return new Aim(wrap(toTarget + leadDeg - headingDeg), cameraDistance(aimX, aimY), speed);
    }

    /**
     * Last known direction of the target in the gyro's frame, even if it's old. For keeping the turret
     * pointed that way when the tag drops out. NaN if we've never seen it
     */
    public double lastBearingDeg() {
        return tracking ? Math.toDegrees(Math.atan2(y, x)) : Double.NaN;
    }

    private static double wrap(double deg) {
        return ((deg + 180) % 360 + 360) % 360 - 180;
    }

    // the table goes by distance from the camera with the turret on target
    private static double cameraDistance(double fromTurretX, double fromTurretY) {
        return Math.hypot(fromTurretX, fromTurretY) - Limelight.CAMERA_FORWARD_IN;
    }
}
