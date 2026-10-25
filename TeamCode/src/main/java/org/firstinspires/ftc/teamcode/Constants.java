package org.firstinspires.ftc.teamcode;

/**
 * Every tunable number for the robot lives here so nothing is hiding inside a subsystem.
 * Pedro Pathing's drive/localizer tuning is separate, in {@link org.firstinspires.ftc.teamcode.pedro.Constants}.
 *
 * Every value marked TODO is a placeholder. Find the real number by testing on the robot.
 */
public final class Constants {

    private Constants() {}

    public static final class Shooter {
        /** RPM the flywheel holds while shooting. This is what worked last time. */
        public static final double TARGET_RPM = 3000;
        /** How close (+/-) to the target RPM counts as "at speed". */
        public static final double READY_TOLERANCE_RPM = 100;
        /** Encoder ticks per flywheel motor revolution. */
        public static final double TICKS_PER_REV = 28;

        /** Free speed of the 6000 rpm motors, used to work out the velocity feedforward. */
        public static final double MOTOR_MAX_RPM = 6000;
        // Velocity PIDF for the hub's built-in controller. F is 32767 divided by the motor's max
        // ticks per second, and P, I are fractions of F. TODO: tune on the robot, these are a start.
        public static final double VELOCITY_F = 32767.0 / (MOTOR_MAX_RPM * TICKS_PER_REV / 60.0);
        public static final double VELOCITY_P = 0.1 * VELOCITY_F;
        public static final double VELOCITY_I = 0.01 * VELOCITY_F;
        public static final double VELOCITY_D = 0;

        // TODO: hood angle comes from the shot lookup table later, this is just a starting value
        public static final double DEFAULT_HOOD_ANGLE_DEG = 30;
        /**
         * How far ahead to predict the robot's pose when aiming: the age of the pose reading plus the time
         * from sending a command to the turret, hood and flywheel actually moving. This is NOT the feed
         * delay (gate and belt, about 0.15 s): the turret keeps tracking continuously, so a ball that
         * leaves later is aimed by a later command. The sim says about 0.06 to 0.10 s is best, and
         * using the feed delay here makes shots worse at speed. TODO: measure, then tune on the robot.
         */
        public static final double AIM_LOOKAHEAD_SECONDS = 0.08;
        /** How close (+/-) to the commanded angle counts as "hood at angle". */
        public static final double HOOD_TOLERANCE_DEG = 2;
    }

    /** Intake and indexer share one motor, so they share one set of constants. */
    public static final class Indexer {
        public static final double INTAKE_POWER = 1.0;
        public static final double FEED_POWER = 1.0;
        public static final double OUTTAKE_POWER = -1.0;

        /** Most balls the rules let us control at once (G407). */
        public static final int MAX_BALLS = 4;
        /** A slot sensor closer than this sees a ball in its slot. TODO: measure. */
        public static final double BALL_DETECT_DISTANCE_MM = 30;

        // Servo positions, all in [0, 1]. TODO: find the real ones on the robot.
        public static final double GATE_CLOSED_POSITION = 0.0;
        public static final double GATE_OPEN_POSITION = 1.0;
        public static final double DIVERTER_PASS_POSITION = 0.0;
        public static final double DIVERTER_DIVERT_POSITION = 1.0;
        public static final double PIVOT_STOWED_POSITION = 0.0;
        public static final double PIVOT_DEPLOYED_POSITION = 1.0;
    }

    public static final class Turret {
        /** How close (+/-) to the aim angle counts as "on target". */
        public static final double TOLERANCE_DEG = 2;
    }

    /** The little bar that drops so we can intake from a flower. */
    public static final class FlowerBar {
        public static final double STOWED_POSITION = 0.0;
        public static final double DEPLOYED_POSITION = 1.0;
    }

    public static final class Blocker {
        public static final double STOWED_POSITION = 0.0;
        public static final double EXTENDED_POSITION = 1.0;
    }

    public static final class Driver {
        /** How far a trigger has to be pulled to count as pressed. */
        public static final double TRIGGER_THRESHOLD = 0.1;
    }
}
