package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.HardwareNames;

/**
 * The pollen and nectar hoods. They move together so one subsystem owns both.
 */
public class HoodSubsystem extends SubsystemBase {

    private final Servo pollenServo;
    private final Servo nectarServo;
    private double pollenPosition;
    private double nectarPosition;

    public HoodSubsystem(HardwareMap hardwareMap) {
        pollenServo = hardwareMap.get(Servo.class, HardwareNames.POLLEN_HOOD_SERVO);
        nectarServo = hardwareMap.get(Servo.class, HardwareNames.NECTAR_HOOD_SERVO);
    }

    public void setPositions(double pollen, double nectar) {
        pollenPosition = Range.clip(pollen, 0.0, 1.0);
        nectarPosition = Range.clip(nectar, 0.0, 1.0);
        pollenServo.setPosition(pollenPosition);
        nectarServo.setPosition(nectarPosition);
    }

    public void telemetrize(Telemetry telemetry) {
        telemetry.addData("Pollen hood", "%.2f", pollenPosition);
        telemetry.addData("Nectar hood", "%.2f", nectarPosition);
    }
}
