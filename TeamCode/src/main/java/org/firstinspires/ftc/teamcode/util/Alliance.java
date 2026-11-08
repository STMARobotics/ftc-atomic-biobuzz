package org.firstinspires.ftc.teamcode.util;

/**
 * Which side we're on. Red drivers stand on the x = 0 wall looking +x, blue on the x = 144 wall
 * looking -x (Pedro coordinates, same layout as DECODE). VERIFY on the real field.
 */
public enum Alliance {
    RED(0.0),
    BLUE(Math.PI);

    /**
     * Field heading (rad) that is "away from the drivers", what pushing the stick forward means
     */
    public final double driverForwardRad;

    Alliance(double driverForwardRad) {
        this.driverForwardRad = driverForwardRad;
    }
}
