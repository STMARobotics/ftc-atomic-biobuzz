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
        public static final String INTAKE_MOTOR_RIGHT = "intakeMotorRight"; // the left one isn't plugged in
        public static final String LIMELIGHT = "limelight";

        private HardwareNames() {}
    }

    public static final class Debug {
        // extra telemetry that costs loop time (intake current is a separate hardware read)
        public static final boolean EXTRA_TELEMETRY = false;

        private Debug() {}
    }

    public static final class Battery {
        // the shooter feedforward is tuned at this voltage and scaled up as the battery drops
        public static final double NOMINAL_VOLTS = 12.0;
        // reading the battery is its own hardware call, so don't do it every loop
        public static final double READ_INTERVAL_MS = 250;

        // brownout guard: below START the drive slows down linearly, down to MIN_DRIVE_SCALE at
        // FLOOR. The hubs reset around 7V, losing a few inches/sec beats losing the robot. TUNE
        public static final double BROWNOUT_START_VOLTS = 9.0;
        public static final double BROWNOUT_FLOOR_VOLTS = 7.5;
        public static final double BROWNOUT_MIN_DRIVE_SCALE = 0.5;

        private Battery() {}
    }

    public static final class Limelight {
        public static final boolean ENABLED = false; // no camera on the robot yet, flip to true once it's wired
        public static final int TAG_PIPELINE = 0; // TUNE
        public static final double MAX_STALENESS_MS = 100;

        // for working out distance from ty, all placeholders, TUNE
        public static final double CAMERA_HEIGHT_IN = 12.0;
        public static final double CAMERA_ANGLE_DEG = 25.0;
        public static final double TAG_HEIGHT_IN = 29.5;

        // the tag is hidden when the seesaw is tipped toward us (can't score then).
        // it has to be seen this long in a row before we trust that we can score
        public static final double SEESAW_CONFIRM_MS = 300; // TUNE

        private Limelight() {}
    }

    public static final class Shooter {
        public static final double TICKS_PER_REV = 28;
        public static final double READY_TOLERANCE_RPM = 75;

        // flywheel control, power = (kS + kV * target) * (12V / battery) + kP * error
        // TUNE in ShotTuner: kV first with kP at 0, bump it until the flywheel settles right on target
        public static final double kS = 0.0; // TUNE, power it takes just to get past the friction
        public static final double kV = 1.0 / 4100; // TUNE, about 1 / the top speed at 12V (it maxes out around 4000)
        public static final double kP = 0.0005; // TUNE, 100 rpm low = 5% more power

        // motor rpm, not wheel rpm. has to be below what it can actually reach (~4000) or there's
        // no power left to recover after each ball. TUNE, and retune the hoods if you change it
        public static final double SHOOT_RPM = 3700;
        // gate opens once the flywheel is this close to the target
        public static final double GATE_OPEN_MARGIN_RPM = 150;

        // shot counting: a ball going through pulls the flywheel down this far, then it recovers.
        // TUNE by watching "Shots fired" while feeding in ShotTuner
        public static final double SHOT_DIP_RPM = 250;
        public static final double SHOT_RECOVER_RPM = 100;

        // if the flywheel can't get within GATE_OPEN_MARGIN_RPM (low battery, drag), shoot anyway
        // after this long once it's at least FALLBACK_MIN_FRACTION of the target. Short beats nothing
        public static final double SPINUP_TIMEOUT_MS = 2000; // TUNE
        public static final double FALLBACK_MIN_FRACTION = 0.9; // TUNE
        // no encoder at all (motor missing): it runs at full power, open the gate after this long
        public static final double BLIND_SPINUP_MS = 1500; // TUNE

        // jam: gate open and feeding but no shot counted for this long -> back the balls out briefly.
        // stops after MAX_UNJAMS tries in a row (probably just empty). needs SHOT_DIP_RPM tuned
        public static final boolean JAM_DETECTION = true;
        public static final double JAM_TIMEOUT_MS = 1500; // TUNE
        public static final double UNJAM_MS = 250; // TUNE
        public static final int MAX_UNJAMS = 2;

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

        // gear stack: 0V is 0 deg and about 3.3V is about 540 deg, TUNE both
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

        // safety (see TurretSubsystem): 312rpm at 4:1 is ~470 deg/s flat out
        public static final double MAX_DEG_PER_SEC = 470;
        public static final double JUMP_SLACK_DEG = 15; // encoder noise allowance per loop
        public static final double STALL_MIN_POWER = 0.25; // pushing at least this hard...
        public static final double STALL_MS = 400; // ...for this long...
        public static final double STALL_MIN_TRAVEL_DEG = 2; // ...should move it more than this

        private Turret() {}
    }

    public static final class IntakePivot {
        public static final boolean ENABLED = true; // false leaves the servo limp

        // servo positions, TUNE
        public static final double FULLY_STOWED = 1.0; // battle mode, also where it starts
        public static final double DEPLOYED = 0.0;

        // how long the pivot takes to get down, the flower waits this long before it deploys
        public static final double TRAVEL_MS = 400; // TUNE

        private IntakePivot() {}
    }

    public static final class Flower {
        public static final boolean ENABLED = false; // off for now, A does nothing

        // servo positions, TUNE
        public static final double STOWED = 0.0;
        public static final double DEPLOYED = 1.0;

        private Flower() {}
    }

    public static final class IntakeWheels {
        public static final double TICKS_PER_REV = 537.7; // goBILDA 312 rpm

        // motor powers, TUNE. the intake pulls the battery down ~2V, drop these if the robot browns out
        public static final double INTAKE_POWER = 1.0;
        public static final double FEED_POWER = 1.0; // pushing balls through the gate into the shooter
        public static final double OUTTAKE_POWER = -1.0;
        public static final double UNJAM_POWER = -0.6; // the shooter's short back-out pulse, TUNE

        // power ramps up at this rate instead of jumping, the jump is what sags the battery the most.
        // 4 = 0 to full in a quarter second. stopping is always instant
        public static final double RAMP_PER_SEC = 4.0; // TUNE

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

        // sticks get raised to this power: 2 = half stick is quarter speed (fine control),
        // 1 = linear (snappier). TUNE with the driver
        public static final double STICK_EXPONENT = 2.0;

        private Drive() {}
    }
}
