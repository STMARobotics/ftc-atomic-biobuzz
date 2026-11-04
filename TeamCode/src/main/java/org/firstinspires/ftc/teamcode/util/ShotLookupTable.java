package org.firstinspires.ftc.teamcode.util;

/**
 * Distance to the goal in, flywheel rpm and both hood positions out.
 */
public class ShotLookupTable {

    public static final class Setting {
        public final double rpm;
        public final double pollenHood;
        public final double nectarHood;

        public Setting(double rpm, double pollenHood, double nectarHood) {
            this.rpm = rpm;
            this.pollenHood = pollenHood;
            this.nectarHood = nectarHood;
        }
    }

    private final InterpolationTable rpm = new InterpolationTable();
    private final InterpolationTable pollenHood = new InterpolationTable();
    private final InterpolationTable nectarHood = new InterpolationTable();

    public ShotLookupTable add(double distance, double rpm, double pollenHood, double nectarHood) {
        this.rpm.add(distance, rpm);
        this.pollenHood.add(distance, pollenHood);
        this.nectarHood.add(distance, nectarHood);
        return this;
    }

    public Setting get(double distance) {
        return new Setting(rpm.get(distance), pollenHood.get(distance), nectarHood.get(distance));
    }
}
