package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotConstants.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
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
 * Flywheel gains (gamepad 2), copy them into Shooter.kS, kV and kP when it holds speed:
 *   dpad up/down    kV +/- 0.000005
 *   dpad right/left kV +/- 0.000001
 *   Y / A           kP +/- 0.0001
 *   X               kP to 0 (tune kV first with kP at 0)
 *   bumpers         kS +/- 0.01
 *
 * "Shots fired" should go up by exactly one per ball, if it doesn't tune SHOT_DIP_RPM.
 */
@TeleOp(name = "ShotTuner", group = "Tuning")
public class ShotTuner extends LinearOpMode {

    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double HOOD_STEP = 0.02;

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);
        ShooterSubsystem shooter = robot.shooter;
        ShooterGateSubsystem gate = robot.gate;
        IntakeSubsystem intake = robot.intake;
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

            double kS = shooter.getKS();
            double kV = shooter.getKV();
            double kP = shooter.getKP();
            if (gainPad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) kV += 0.000005;
            if (gainPad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) kV -= 0.000005;
            if (gainPad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) kV += 0.000001;
            if (gainPad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) kV -= 0.000001;
            if (gainPad.wasJustPressed(GamepadKeys.Button.Y)) kP += 0.0001;
            if (gainPad.wasJustPressed(GamepadKeys.Button.A)) kP -= 0.0001;
            if (gainPad.wasJustPressed(GamepadKeys.Button.X)) kP = 0;
            if (gainPad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) kS += 0.01;
            if (gainPad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) kS -= 0.01;
            shooter.setGains(Math.max(kS, 0), Math.max(kV, 0), Math.max(kP, 0));

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
            robot.hood.setPositions(pollen, nectar);

            if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) distance += 6;
            if (gamepad.wasJustPressed(GamepadKeys.Button.BACK)) distance = 24;

            // same logic as ShootCommand
            boolean shooting = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_THRESHOLD;
            if (shooting) {
                shooter.setRPM(rpm);
                if (!wasShooting) {
                    lowestRPM = rpm;
                    shooter.resetShotCount();
                }
                if (shooter.getRPM() >= rpm - Shooter.GATE_OPEN_MARGIN_RPM) gate.open();
                if (gate.isOpen()) lowestRPM = Math.min(lowestRPM, shooter.getRPM());
            } else {
                shooter.stop();
                gate.close();
            }
            wasShooting = shooting;

            intake.setFeeding(gate.isOpen());
            shooter.setCountingShots(gate.isOpen());
            intake.setIntaking(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_THRESHOLD);
            intake.setOuttaking(gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER));

            // no scheduler here, so run the subsystems ourselves
            shooter.periodic();
            intake.periodic();

            telemetry.addData("Distance (in)", "%.0f", distance);
            telemetry.addData("Lowest RPM while feeding", "%.0f", lowestRPM);
            telemetry.addData("Battery (V)", "%.2f", robot.batteryVolts());
            shooter.telemetrize(telemetry);
            telemetry.addData("kS", "%.3f", shooter.getKS());
            telemetry.addData("kV", "%.6f (top speed about %.0f rpm)", shooter.getKV(),
                    shooter.getKV() > 0 ? 1 / shooter.getKV() : 0);
            telemetry.addData("kP", "%.4f", shooter.getKP());
            gate.telemetrize(telemetry);
            robot.hood.telemetrize(telemetry);
            intake.telemetrize(telemetry);
            telemetry.addLine();
            telemetry.addLine(String.format(".add(%.0f, %.0f, %.2f, %.2f)", distance, rpm, pollen, nectar));
            telemetry.update();
        }

        shooter.stop();
        shooter.periodic();
        gate.close();
        CommandScheduler.getInstance().reset();
    }
}
