package org.firstinspires.ftc.teamcode.util;

/**
 * Distance to the goal in, flywheel rpm, both hood positions and how long the ball is in the air out.
 */
public class ShotLookupTable {

    public static final class Setting {
        public final double rpm;
        public final double pollenHood;
        public final double nectarHood;
        public final double flightSec;

        public Setting(double rpm, double pollenHood, double nectarHood, double flightSec) {
            this.rpm = rpm;
            this.pollenHood = pollenHood;
            this.nectarHood = nectarHood;
            this.flightSec = flightSec;
        }
    }

    private final InterpolationTable rpm = new InterpolationTable();
    private final InterpolationTable pollenHood = new InterpolationTable();
    private final InterpolationTable nectarHood = new InterpolationTable();
    private final InterpolationTable flightSec = new InterpolationTable();

    public ShotLookupTable add(double distance, double rpm, double pollenHood, double nectarHood, double flightSec) {
        this.rpm.add(distance, rpm);
        this.pollenHood.add(distance, pollenHood);
        this.nectarHood.add(distance, nectarHood);
        this.flightSec.add(distance, flightSec);
        return this;
    }

    public Setting get(double distance) {
        return new Setting(rpm.get(distance), pollenHood.get(distance), nectarHood.get(distance), flightSec.get(distance));
    }
}
