package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/**
 * Hands the robot's pose (and alliance) from auto to teleop, so field centric drive and aiming are
 * right from the first second instead of after someone presses START on the gamepad.
 *
 * Static fields survive between OpModes because they run in the same app. Autos save every loop,
 * teleop takes it once (if it's recent) and clears it.
 */
public final class PoseStorage {

    private static Pose pose = null;
    private static Alliance alliance = null;
    private static Alliance pickedAlliance = null;
    private static long savedAtNanos = 0;

    private PoseStorage() {}

    public static synchronized void save(Pose newPose, Alliance newAlliance) {
        pose = newPose;
        if (newAlliance != null) {
            alliance = newAlliance;
            pickedAlliance = newAlliance;
        }
        savedAtNanos = System.nanoTime();
    }

    /**
     * The pose auto left us, or null if there isn't one from the last maxAgeSec. Clears it either way
     */
    public static synchronized Pose take(double maxAgeSec) {
        Pose result = pose != null && (System.nanoTime() - savedAtNanos) / 1e9 <= maxAgeSec ? pose : null;
        pose = null;
        return result;
    }

    /**
     * Alliance from the last auto that set one, null if unknown
     */
    public static synchronized Alliance alliance() {
        return alliance;
    }

    /**
     * Whose tags to track, picked by hand. Separate from alliance(): that also sets field centric's
     * forward, and without auto there's no heading to go on. It lasts until the app restarts or an auto
     * picks, so check it on the INIT screen every match
     */
    public static synchronized void pickAlliance(Alliance picked) {
        pickedAlliance = picked;
    }

    /**
     * Whose tags to track, null if nobody has said
     */
    public static synchronized Alliance pickedAlliance() {
        return pickedAlliance;
    }

    public static synchronized void clear() {
        pickedAlliance = null;
        pose = null;
        alliance = null;
        savedAtNanos = 0;
    }
}
