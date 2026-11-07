package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;

/**
 * For finding servo positions. Copy the numbers into ShooterGate and IntakePivot.
 * A servo doesn't move until you nudge it, so nothing jumps when you hit start.
 *
 * Controls (gamepad 1):
 *   A               switch between gate and intake pivot
 *   dpad up/down    +/- 0.05
 *   dpad right/left +/- 0.01
 */
@TeleOp(name = "ServoTuner", group = "Tuning")
public class ServoTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        String[] names = {"Gate", "Intake pivot"};
        Servo[] servos = {
                hardwareMap.get(Servo.class, HardwareNames.GATE_SERVO),
                hardwareMap.get(Servo.class, HardwareNames.PIVOT_SERVO)
        };
        double[] positions = {0.5, 0.5};
        boolean[] moved = {false, false};
        int selected = 0;

        GamepadEx gamepad = new GamepadEx(gamepad1);

        telemetry.addLine("A switches servo, dpad moves it");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            gamepad.readButtons();

            if (gamepad.wasJustPressed(GamepadKeys.Button.A)) selected = (selected + 1) % servos.length;

            double step = 0;
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) step = 0.05;
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) step = -0.05;
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) step = 0.01;
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) step = -0.01;

            if (step != 0) {
                positions[selected] = Range.clip(positions[selected] + step, 0, 1);
                servos[selected].setPosition(positions[selected]);
                moved[selected] = true;
            }

            telemetry.addData("Selected", names[selected]);
            for (int i = 0; i < servos.length; i++) {
                telemetry.addData(names[i], moved[i] ? String.format("%.2f", positions[i]) : "not moved yet");
            }
            telemetry.update();
        }
    }
}
