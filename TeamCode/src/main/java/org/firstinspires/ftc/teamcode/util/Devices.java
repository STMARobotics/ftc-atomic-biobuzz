package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Gets hardware that the robot can live without. If the name isn't in the Driver Hub config (or
 * the device is unplugged and missing from it) you get null and the name goes on the missing
 * list instead of the whole OpMode crashing at INIT. Subsystems treat null as "skip it".
 *
 * Only the drivetrain is required: without it there's nothing to play with anyway.
 */
public final class Devices {

    private static final List<String> missing = new ArrayList<>();

    private Devices() {}

    public static <T> T optional(HardwareMap hardwareMap, Class<? extends T> type, String name) {
        try {
            return hardwareMap.get(type, name);
        } catch (RuntimeException e) {
            synchronized (missing) {
                if (!missing.contains(name)) missing.add(name);
            }
            return null;
        }
    }

    /**
     * Names that weren't found, for telemetry
     */
    public static List<String> missing() {
        synchronized (missing) {
            return new ArrayList<>(missing);
        }
    }

    /**
     * Call when building a new robot so old misses don't stick around
     */
    public static void reset() {
        synchronized (missing) {
            missing.clear();
        }
    }
}
