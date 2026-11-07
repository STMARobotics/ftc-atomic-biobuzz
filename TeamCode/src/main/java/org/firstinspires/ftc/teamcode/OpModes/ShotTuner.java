package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeWheelsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterGateSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

/**
 * For filling in the shot table. Put the robot at a measured distance, shoot, adjust, repeat.
 * When the shots go in, copy the .add(...) line from telemetry into Shooter.SHOT_TABLE.
 *
 * Controls (gamepad 1):
 *   right trigger  shoot (spins up, gate opens once the flywheel is at speed, wheels feed)
 *   left trigger   intake
 *   left bumper    outtake
 *   dpad up/down   rpm +/- 50
 *   dpad right/left rpm +/- 10
 *   Y / A          pollen hood +/- 0.02
 *   X / B          nectar hood +/- 0.02
 *   right bumper   distance + 6 in, back resets it to 24
 *
 * Flywheel gains (gamepad 2), copy them into Shooter.kV and Shooter.kP when it holds speed:
 *   dpad up/down    kV +/- 0.000005
 *   dpad right/left kV +/- 0.000001
 *   Y / A           kP +/- 0.0001
 *   X               kP to 0 (tune kV first with kP at 0)
 */
@TeleOp(name = "ShotTuner", group = "Tuning")
public class ShotTuner extends LinearOpMode {

    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double HOOD_STEP = 0.02;

    @Override
    public void runOpMode() {

        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        ShooterGateSubsystem gate = new ShooterGateSubsystem(hardwareMap);
        IntakeWheelsSubsystem wheels = new IntakeWheelsSubsystem(hardwareMap);
        HoodSubsystem hood = new HoodSubsystem(hardwareMap);
        GamepadEx gamepad = new GamepadEx(gamepad1);
        GamepadEx gainPad = new GamepadEx(gamepad2);

        double rpm = Shooter.SHOOT_RPM;
        double pollen = 0.5;
        double nectar = 0.5;
        double distance = 24;
        double lowestRPM = 0; // how far the flywheel dips while feeding, lower = worse recovery
        boolean wasShooting = false;

        telemetry.addLine("Ready, measure the distance from the goal and set it with right bumper");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            gamepad.readButtons();
            gainPad.readButtons();

            double kV = shooter.getKV();
            double kP = shooter.getKP();
            if (gainPad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) kV += 0.000005;
            if (gainPad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) kV -= 0.000005;
            if (gainPad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) kV += 0.000001;
            if (gainPad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) kV -= 0.000001;
            if (gainPad.wasJustPressed(GamepadKeys.Button.Y)) kP += 0.0001;
            if (gainPad.wasJustPressed(GamepadKeys.Button.A)) kP -= 0.0001;
            if (gainPad.wasJustPressed(GamepadKeys.Button.X)) kP = 0;
            shooter.setGains(Math.max(kV, 0), Math.max(kP, 0));

            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) rpm += 50;
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) rpm -= 50;
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) rpm += 10;
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) rpm -= 10;
            rpm = Math.max(rpm, 0);

            if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) pollen += HOOD_STEP;
            if (gamepad.wasJustPressed(GamepadKeys.Button.A)) pollen -= HOOD_STEP;
            if (gamepad.wasJustPressed(GamepadKeys.Button.X)) nectar += HOOD_STEP;
            if (gamepad.wasJustPressed(GamepadKeys.Button.B)) nectar -= HOOD_STEP;
            pollen = Range.clip(pollen, 0, 1);
            nectar = Range.clip(nectar, 0, 1);
            hood.setPositions(pollen, nectar);

            if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) distance += 6;
            if (gamepad.wasJustPressed(GamepadKeys.Button.BACK)) distance = 24;

            boolean shooting = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;
            if (shooting) {
                shooter.setRPM(rpm);
                if (!wasShooting) lowestRPM = rpm;
                if (shooter.flywheelReady()) gate.open();
                if (gate.isOpen()) lowestRPM = Math.min(lowestRPM, shooter.getRPM());
            } else {
                shooter.stop();
                gate.close();
            }
            wasShooting = shooting;

            // same priority as the match: outtake, then feeding, then intake, otherwise idle
            if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) wheels.outtake();
            else if (gate.isOpen() || gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD) wheels.intake();
            else wheels.stop();

            telemetry.addData("Distance (in)", "%.0f", distance);
            telemetry.addData("Flywheel ready", shooter.flywheelReady());
            telemetry.addData("Lowest RPM while feeding", "%.0f", lowestRPM);
            shooter.telemetrize(telemetry);
            telemetry.addData("kV", "%.6f (top speed about %.0f rpm)", shooter.getKV(),
                    shooter.getKV() > 0 ? 1 / shooter.getKV() : 0);
            telemetry.addData("kP", "%.4f", shooter.getKP());
            gate.telemetrize(telemetry);
            hood.telemetrize(telemetry);
            wheels.telemetrize(telemetry);
            telemetry.addLine();
            telemetry.addLine(String.format(".add(%.0f, %.0f, %.2f, %.2f)", distance, rpm, pollen, nectar));
            telemetry.update();
        }

        shooter.stop();
        gate.close();
        CommandScheduler.getInstance().reset();
    }
}
