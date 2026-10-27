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

        // TODO: hood angles come from the shot lookup table later, this is just a starting value for both hoods
        public static final double DEFAULT_HOOD_ANGLE_DEG = 70;
        /**
         * How far ahead to predict the robot's pose when aiming: the age of the pose reading plus the time
         * from sending a command to the turret, hood and flywheel actually moving. This is NOT the feed
         * delay (gate and belt, about 0.15 s): the turret keeps tracking continuously, so a ball that
         * leaves later is aimed by a later command. The sim says about 0.06 to 0.10 s is best, and
         * using the feed delay here makes shots worse at speed. TODO: measure, then tune on the robot.
         */
        public static final double AIM_LOOKAHEAD_SECONDS = 0.08;

        // Hoods: one servo each, no feedback, so an angle is turned into a servo position with a straight
        // line between two measured points. TODO: all of these are placeholders, find them on the robot.
        public static final double HOOD_MIN_ANGLE_DEG = 65;
        public static final double HOOD_MAX_ANGLE_DEG = 80;
        /** Servo position (0 to 1) that puts each hood at HOOD_MIN_ANGLE_DEG and at HOOD_MAX_ANGLE_DEG. */
        public static final double HOOD_POLLEN_POSITION_AT_MIN = 0.0;
        public static final double HOOD_POLLEN_POSITION_AT_MAX = 1.0;
        public static final double HOOD_NECTAR_POSITION_AT_MIN = 0.0;
        public static final double HOOD_NECTAR_POSITION_AT_MAX = 1.0;
    }

    /** Intake and indexer share one motor, so they share one set of constants. */
    public static final class Indexer {
        public static final double INTAKE_RPM = 300;
        public static final double FEED_RPM = 300;
        public static final double IDLE_RPM = 150;
        public static final double OUTTAKE_RPM = -300;

        // Servo positions, all in [0, 1]. TODO: find the real ones on the robot.
        public static final double GATE_CLOSED_POSITION = 0.0;
        public static final double GATE_OPEN_POSITION = 1.0;
        public static final double PIVOT_STOWED_POSITION = 0.0;
        public static final double PIVOT_IDLE_POSITION = 0.0;
        public static final double PIVOT_DEPLOYED_POSITION = 1.0;
    }

    public static final class Turret {
        // The absolute encoder is geared so one full encoder turn is the turret's whole travel,
        // 540 degrees: straight forward in the middle, 270 each way. So the encoder's 0/360 seam sits at
        // the turret's physical limits and is never crossed.
        /** Total turret travel, in degrees. */
        public static final double TRAVEL_DEG = 540;
        /** Turret degrees per encoder degree. */
        public static final double TURRET_DEG_PER_ENCODER_DEG = TRAVEL_DEG / 360.0;
        /** Absolute encoder degrees (0 to 360) when the turret points straight forward. TODO: measure. */
        public static final double ENCODER_FORWARD_DEG = 180;
        /** +1 if a bigger encoder reading turns the turret the way we call positive, otherwise -1. TODO: check on the robot. */
        public static final double ENCODER_DIRECTION = 1.0;
        /** How close (+/-) to the aim angle counts as "on target". */
        public static final double TOLERANCE_DEG = 2;
        /** Motor power per degree of error. 0.02 means a 50 degree error is full power. TODO: tune on the robot. */
        public static final double KP = 0.02;
        /** Most motor power the turret is allowed to use, in [0, 1]. TODO: tune on the robot. */
        public static final double MAX_POWER = 0.8;
        /** Stay this far inside the physical limits (half of TRAVEL_DEG) so the cable and the encoder seam are never reached. */
        public static final double SOFT_LIMIT_MARGIN_DEG = 5;
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
