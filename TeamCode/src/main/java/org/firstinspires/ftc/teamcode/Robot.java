package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

/**
 * Every subsystem on the robot, built once so teleop, auto and the tuners all get the same robot.
 * Building it doesn't move anything.
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
        batterySensor = hardwareMap.voltageSensor.iterator().next();

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
        long now = System.nanoTime();
        if ((now - lastBatteryReadNanos) / 1e6 >= Battery.READ_INTERVAL_MS) {
            lastBatteryReadNanos = now;
            batteryVolts = batterySensor.getVoltage();
        }
        return batteryVolts;
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
}
