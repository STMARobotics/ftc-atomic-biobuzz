package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.ShotLookupTable;

// angles in degrees, TUNE = needs to be measured on the robot
public final class RobotConstants {

    private RobotConstants() {}

    public static final class HardwareNames {
        public static final String FLYWHEEL_MOTOR = "flywheelMotor"; // the one with the encoder
        public static final String FLYWHEEL_MOTOR_2 = "flywheelMotor2";
        public static final String TURRET_MOTOR = "turretMotor";
        public static final String TURRET_ENCODER = "yawEncoder";
        public static final String PIVOT_SERVO = "pivotServo";
        public static final String GATE_SERVO = "gateServo";
        public static final String FLOWER_SERVO = "flowerServo";
        public static final String POLLEN_HOOD_SERVO = "pollenHoodServo";
        public static final String NECTAR_HOOD_SERVO = "nectarHoodServo";
        public static final String INTAKE_MOTOR_LEFT = "intakeMotorLeft";
        public static final String INTAKE_MOTOR_RIGHT = "intakeMotorRight";
        public static final String LIMELIGHT = "limelight";

        private HardwareNames() {}
    }

    public static final class Limelight {
        public static final boolean ENABLED = false; // no camera on the robot yet, flip to true once it's wired
        public static final int TAG_PIPELINE = 0; // TUNE
        public static final double MAX_STALENESS_MS = 100;

        // for working out distance from ty, all placeholders, TUNE
        public static final double CAMERA_HEIGHT_IN = 12.0;
        public static final double CAMERA_ANGLE_DEG = 25.0;
        public static final double TAG_HEIGHT_IN = 29.5;

        private Limelight() {}
    }

    public static final class Shooter {
        public static final double TICKS_PER_REV = 28;
        public static final double READY_TOLERANCE_RPM = 75;

        // flywheel control, power = kV * target + kP * error
        // TUNE kV first with kP at 0: bump it until the flywheel settles right on target
        public static final double kV = 1.0 / 5800; // TUNE, about 1 / the motor's real top speed
        public static final double kP = 1.0; // TUNE, 100 rpm low = 5% more power
        public static final double SHOOT_RPM = 4500; // TUNE, motor rpm not wheel rpm
        public static final double GATE_OPEN_RPM = 3800; // gate opens once the flywheel is past this

        // distance to the goal (in) -> flywheel rpm, pollen hood, nectar hood
        // TUNE (add a point every time you test a shot)
        public static final ShotLookupTable SHOT_TABLE = new ShotLookupTable()
                .add(24, SHOOT_RPM, 0.68, 0.5);

        // with the limelight off every shot uses the table at this distance, so it's one fixed shot
        public static final double FIXED_DISTANCE_IN = 24; // TUNE

        private Shooter() {}
    }

    public static final class Turret {
        public static final boolean ENABLED = false; // yaw encoder isn't plugged in, motor stays at 0 until this is true

        public static final double ENCODER_MAX_VOLTS = 3.3;
        public static final double ENCODER_RANGE_DEG = 540;

        // hardstops
        public static final double MIN_ANGLE_DEG = 0; // TUNE
        public static final double MAX_ANGLE_DEG = ENCODER_RANGE_DEG; // TUNE

        public static final double FORWARD_ANGLE_DEG = ENCODER_RANGE_DEG / 2; // TUNE

        // true if the encoder angle goes up when the turret turns clockwise (seen from above)
        public static final boolean ENCODER_CLOCKWISE_POSITIVE = true; // TUNE

        public static final double kP = 0.0; // TUNE
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double TOLERANCE_DEG = 1.5;
        public static final double MAX_POWER = 0.6;

        // how fast the manual turret buttons move it
        public static final double MANUAL_SPEED_DEG_PER_SEC = 90; // TUNE

        // unwind has to travel this far before a tag sighting can end it
        public static final double UNWIND_MIN_TRAVEL_DEG = 90;

        private Turret() {}
    }

    public static final class IntakePivot {
        public static final boolean ENABLED = true; // false leaves the servo limp

        // servo positions, TUNE
        public static final double FULLY_STOWED = 1.0; // battle mode, also where it starts
        public static final double DEPLOYED = 0.0;

        private IntakePivot() {}
    }

    public static final class Flower {
        public static final boolean ENABLED = false; // off for the first comp, A does nothing

        // servo positions, TUNE
        public static final double STOWED = 0.0;
        public static final double DEPLOYED = 1.0;

        private Flower() {}
    }

    public static final class IntakeWheels {
        public static final double TICKS_PER_REV = 537.7; // goBILDA 312 rpm

        // motor powers, TUNE
        public static final double INTAKE_POWER = 1.0;
        public static final double OUTTAKE_POWER = -1.0;

        private IntakeWheels() {}
    }

    public static final class ShooterGate {
        // servo positions, TUNE
        public static final double OPEN = 0.0;
        public static final double CLOSED = 1.0;

        private ShooterGate() {}
    }

    public static final class Drive {
        public static final double FULL_SPEED_SCALE = 1.0;
        public static final double SLOW_SPEED_SCALE = 0.5;

        private Drive() {}
    }
}
