package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.Battery;
import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterGateSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.Devices;

import java.util.ArrayList;
import java.util.List;

/**
 * Every subsystem on the robot, built once so teleop, auto and the tuners all get the same robot.
 * Building it doesn't move anything.
 *
 * Only the drivetrain is required. Anything else missing from the config just turns that part off
 * and shows up in faults() (see Devices), so one bad cable doesn't cost the whole match.
 */
public class Robot {

    public final DrivetrainSubsystem drive;
    public final LimelightSubsystem limelight;
    public final TurretSubsystem turret;
    public final ShooterSubsystem shooter;
    public final ShooterGateSubsystem gate;
    public final HoodSubsystem hood;
    public final IntakeSubsystem intake;

    private final VoltageSensor batterySensor;
    private double batteryVolts = Battery.NOMINAL_VOLTS;
    private long lastBatteryReadNanos = 0;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, Pose.zero());
    }

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        Devices.reset();
        batterySensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        drive = new DrivetrainSubsystem(hardwareMap, startPose);
        limelight = new LimelightSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap, drive::heading);
        shooter = new ShooterSubsystem(hardwareMap, this::batteryVolts);
        gate = new ShooterGateSubsystem(hardwareMap);
        hood = new HoodSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
    }

    /**
     * In the order their periodic() runs: sensors first so everything after sees fresh values.
     */
    public Subsystem[] subsystems() {
        return new Subsystem[]{drive, limelight, turret, shooter, gate, hood, intake};
    }

    /**
     * Battery voltage, only actually read every Battery.READ_INTERVAL_MS
     */
    public double batteryVolts() {
        if (batterySensor == null) return Battery.NOMINAL_VOLTS;
        long now = System.nanoTime();
        if ((now - lastBatteryReadNanos) / 1e6 >= Battery.READ_INTERVAL_MS) {
            lastBatteryReadNanos = now;
            double volts = batterySensor.getVoltage();
            if (volts > 1) batteryVolts = volts; // ignore junk reads
        }
        return batteryVolts;
    }

    /**
     * 1 normally, less when the battery sags toward a brownout, so the drive backs off first
     */
    public double driveScale() {
        double volts = batteryVolts();
        if (volts >= Battery.BROWNOUT_START_VOLTS) return 1.0;
        double t = (volts - Battery.BROWNOUT_FLOOR_VOLTS)
                / (Battery.BROWNOUT_START_VOLTS - Battery.BROWNOUT_FLOOR_VOLTS);
        return Range.clip(Battery.BROWNOUT_MIN_DRIVE_SCALE + t * (1 - Battery.BROWNOUT_MIN_DRIVE_SCALE),
                Battery.BROWNOUT_MIN_DRIVE_SCALE, 1.0);
    }

    /**
     * Everything the drive team should know is wrong, empty when all good
     */
    public List<String> faults() {
        List<String> faults = new ArrayList<>();
        List<String> missing = Devices.missing();
        if (!missing.isEmpty()) faults.add("MISSING " + missing);
        if (turret.getFault() != null) faults.add("TURRET OFF: " + turret.getFault());
        if (driveScale() < 1.0) {
            faults.add(String.format("LOW BATTERY %.1fV, drive at %.0f%%", batteryVolts(), driveScale() * 100));
        }
        return faults;
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Battery (V)", "%.2f", batteryVolts());
        telemetry.addData("|-----| Drivetrain |-----|", "");
        drive.telemetrize(telemetry);
        telemetry.addData("|-----| Shooter |-----|", "");
        shooter.telemetrize(telemetry);
        gate.telemetrize(telemetry);
        hood.telemetrize(telemetry);
        telemetry.addData("|-----| Intake |-----|", "");
        intake.telemetrize(telemetry);
        telemetry.addData("|-----| Turret + camera |-----|", "");
        turret.telemetrize(telemetry);
        limelight.telemetrize(telemetry);
    }

    public static final String[] LOG_COLUMNS = {
            "loop_ms", "battery", "x", "y", "heading_deg", "fly_rpm", "fly_target", "shots",
            "gate_open", "intake_power", "pivot_down", "turret_deg", "faults"
    };

    /**
     * One row for the match log, same order as LOG_COLUMNS
     */
    public Object[] logRow(double loopMs) {
        Pose pose = drive.pose();
        return new Object[]{
                loopMs, batteryVolts(), pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                shooter.getRPM(), shooter.getTargetRPM(), shooter.getShotsFired(),
                gate.isOpen(), intake.getWheelPower(), intake.isPivotDown(), turret.getAngle(),
                faults().isEmpty() ? "" : faults().toString()
        };
    }
}
