package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.ShotLookupTable;

// angles in degrees, TUNE = needs to be measured on the robot
// Anything here that isn't final can be changed live on the sim's settings page
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
        public static final boolean ENABLED = true; // missing from the config just turns auto aim off
        public static final int TAG_PIPELINE = 0; // TUNE
        public static final double MAX_STALENESS_MS = 100;
        public static final double ALIVE_STALENESS_MS = 500;

        // for working out distance from ty. Heights from the CAD (tag on an upward cell, Limelight on the
        // turret). The tilt is a guess, TUNE: it should be measured
        public static final double CAMERA_HEIGHT_IN = 12.3;
        public static final double CAMERA_ANGLE_DEG = 25.0;
        public static final double TAG_HEIGHT_IN = 49.7;
        // the distance math assumes a raised cell's tag, so a Hive's lowered cell reads much too far away.
        // Its direction is still good though, turning toward it is often how the raised cell comes into view.
        // So distances stop here, past the end of the shot table anyway
        public static final double MAX_TARGET_DISTANCE_IN = 100;
        // how far the camera is out in front of the turret's axis, from the CAD
        public static final double CAMERA_FORWARD_IN = 6.2;

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
        // waiting for the turret to be on a tag, then shoot anyway: a camera that can't see tags shouldn't cost
        // the match. Long enough for it to finish looking, a sweep can take over 3 s. A dead camera doesn't wait
        public static double AIM_WAIT_MS = 5000; // TUNE
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

        // distance from the camera to the tag (in) -> flywheel rpm, pollen hood, nectar hood, flight time (sec)
        // From the sim's physics (gradlew -p sim shotTable), so it's only as true as the sim's guesses.
        // TUNE: replace these with real shots in ShotTuner. Past either end it uses the end point
        public static final ShotLookupTable SHOT_TABLE = new ShotLookupTable()
                .add(24, 3100, 0.80, 0.90, 0.66)
                .add(30, 3100, 0.60, 0.70, 0.66)
                .add(36, 3100, 0.40, 0.40, 0.62)
                .add(42, 3100, 0.30, 0.30, 0.60)
                .add(48, 3100, 0.20, 0.20, 0.56)
                .add(54, 3300, 0.00, 0.20, 0.80)
                .add(60, 3300, 0.10, 0.20, 0.68) // from here on only some angles go in
                .add(66, 3400, 0.10, 0.20, 0.72)
                .add(72, 3500, 0.10, 0.10, 0.76)
                .add(80, 3600, 0.00, 0.10, 0.80);

        // no tag in view: every shot uses the table at this distance. The camera can't see the tag from
        // up close, so close is the likely reason
        public static final double FIXED_DISTANCE_IN = 30; // TUNE

        // shooting on the move (see ShotSolver). Not final so the sim can compare with it off, false just
        // points at the target
        public static boolean LEAD_SHOTS = true;
        // the camera turns with the turret, so too much lead puts the tags out of frame and we'd lose the
        // target. Past this the shot misses instead. Half the 3A's view plus a bit, since a tag past the
        // middle of the sticker is still in frame
        public static double MAX_LEAD_DEG = 28;
        // slower than this counts as standing still, so camera noise doesn't wiggle the aim
        public static double MIN_LEAD_SPEED_IN_PER_SEC = 4; // TUNE
        public static final double MAX_ROBOT_SPEED_IN_PER_SEC = 80; // nothing past this is real
        // gate open to the ball leaving, on top of the table's flight time
        public static final double FEED_DELAY_SEC = 0.05; // TUNE
        // no new frame for this long and we stop trusting the old one
        public static final double AIM_COAST_SEC = 0.3;
        // a frame this far from what we expected means something changed, start tracking over
        public static final double AIM_JUMP_IN = 15;
        // how much of each frame's surprise goes into position and velocity. More is faster and noisier, TUNE
        public static final double AIM_FILTER_POSITION = 0.45;
        public static final double AIM_FILTER_VELOCITY = 0.12;

        private Shooter() {}
    }

    public static final class Turret {
        // false keeps the motor at 0. Turn this off if the turret isn't wired at a comp: it can shut
        // itself off on a dead encoder (see TurretSubsystem) but it can't know a chain isn't installed
        public static final boolean ENABLED = true;

        // gear stack: 0V is 0 deg and about 3.3V is about 540 deg, TUNE both
        public static final double ENCODER_MAX_VOLTS = 3.3;
        public static final double ENCODER_RANGE_DEG = 540;

        // hardstops
        public static final double MIN_ANGLE_DEG = 0; // TUNE
        public static final double MAX_ANGLE_DEG = ENCODER_RANGE_DEG; // TUNE

        public static final double FORWARD_ANGLE_DEG = ENCODER_RANGE_DEG / 2; // TUNE

        // true if the encoder angle goes up when the turret turns clockwise (seen from above)
        public static final boolean ENCODER_CLOCKWISE_POSITIVE = true; // TUNE

        public static final double kP = 0.02; // TUNE on the robot, 30 deg off = full MAX_POWER. From the sim
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        // power per deg/sec the target is moving, so it keeps up instead of trailing behind
        public static final double kV = 1.0 / 470; // TUNE, from the top speed below
        public static final double RATE_FILTER_SEC = 0.08;

        // no tag: wait a moment in case it's a blip, then sweep around where it was, wider each pass
        public static boolean SEARCH_FOR_TAGS = true;
        public static double SEARCH_AFTER_SEC = 0.75; // TUNE
        // the camera has to be able to read a tag going past, too fast and it blurs. TUNE on the robot
        public static double SEARCH_DEG_PER_SEC = 150;
        // close by first, the tag's rarely behind us. With the camera's view that's most of the way around in
        // about 1.5 s. Going all the way around first can mean a 3 s trip the wrong way
        public static final double SEARCH_FIRST_SWEEP_DEG = 75;
        public static final double SEARCH_WIDER_DEG = 105;

        public static final double TOLERANCE_DEG = 1.5;
        // close enough to start shooting. A cell is about +-10 deg wide from normal range, and a target that's
        // moving (shooting on the move) can sit a couple of degrees behind forever. TUNE
        public static double SHOOT_TOLERANCE_DEG = 4;
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
        public static final boolean ENABLED = true; // false and A does nothing, the servo holds stowed

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
        public static double STICK_EXPONENT = 2.0;

        // mecanum is fastest square to the way it's going, so when just driving around it eases to the nearest
        // 90 from field centric forward. Not while rotating by hand, intaking or shooting (see DriveCommand)
        public static boolean HEADING_BIAS = true;
        public static double BIAS_MIN_STICK = 0.6; // only when really going somewhere
        public static double BIAS_DELAY_SEC = 0.4; // after letting go of the turn stick
        // from the sim: square from 47 deg in 0.5 s, and past 0.7 turn power it doesn't get across the field any
        // sooner since turning takes speed from driving. TUNE with the driver
        public static double BIAS_kP = 0.05; // turn power per deg off
        public static double BIAS_MAX_TURN = 0.7;
        public static double BIAS_TOLERANCE_DEG = 1;

        private Drive() {}
    }
}
