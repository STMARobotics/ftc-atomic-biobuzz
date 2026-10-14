package org.firstinspires.ftc.teamcode;

/**
 * Every tunable number for the robot lives here so nothing is hiding inside a subsystem.
 * Pedro Pathing's drive/localizer tuning is separate, in {@link org.firstinspires.ftc.teamcode.pedro.Constants}.
 */
public final class Constants {

    private Constants() {}

    public static final class Shooter {
        /** RPM the flywheel has to be near before the superstructure will feed. */
        public static final double TARGET_RPM = 2000;
        /** How close (+/-) to TARGET_RPM counts as "at speed". */
        public static final double READY_TOLERANCE_RPM = 200;
        /** Encoder ticks per flywheel motor revolution. */
        public static final double TICKS_PER_REV = 28;
        /** Open loop power while spinning up. See ShooterSubsystem#writeOutputs for closed loop. */
        public static final double SPIN_UP_POWER = 1.0;
    }

    public static final class Intake {
        /** Power while picking up artifacts off the floor. */
        public static final double INTAKE_POWER = 1.0;
        /** Power while pushing artifacts into the flywheel. */
        public static final double FEED_POWER = 1.0;
    }

    public static final class Driver {
        /** How far a trigger has to be pulled to count as pressed. */
        public static final double TRIGGER_THRESHOLD = 0.1;
    }
}
