package org.firstinspires.ftc.teamcode.util;

/**
 * Which side we're on. Red drivers stand on the x = 0 wall looking +x, blue on the x = 144 wall
 * looking -x (Pedro coordinates, same layout as DECODE). VERIFY on the real field.
 */
public enum Alliance {
    RED(0.0, 30),
    BLUE(Math.PI, 38);

    /**
     * Field heading (rad) that is "away from the drivers", what pushing the stick forward means
     */
    public final double driverForwardRad;

    // manual 9.9: each alliance has 2 cells, 4 tags each. Red is 30-37, blue is 38-45
    private final int firstTagId;

    Alliance(double driverForwardRad, int firstTagId) {
        this.driverForwardRad = driverForwardRad;
        this.firstTagId = firstTagId;
    }

    public boolean ownsTag(int id) {
        return id >= firstTagId && id < firstTagId + 8;
    }
}
