package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Subsystem for driving ill do comments later
 */
public class DrivetrainSubsystem extends SubsystemBase {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    IMU imu;

    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    // Drives the robot field-relative, with squared inputs for finer low-speed control
    public void driveFieldRelative(double translationX, double translationY, double rotation, double reductionFactor) {
        double clampedReduction = MathUtils.clamp(reductionFactor, /*low:*/ 0.0, /*high:*/ 1.0);

        // Convert requested direction to polar coordinates
        double theta = Math.atan2(translationY, translationX);
        double r = Math.hypot(translationX, translationY);

        // Rotate by the angle the robot is pointing, so input stays field-relative
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Convert back to cartesian, now in robot frame
        double forward = r * Math.sin(theta);
        double right = r * Math.cos(theta);

        // Square inputs for finer control at low speeds, preserving sign
        double modifiedY = squareSigned(forward) * clampedReduction;
        double modifiedX = squareSigned(right) * clampedReduction;
        double modifiedRotation = squareSigned(rotation) * clampedReduction;

        double frontLeftPower = modifiedY + modifiedX + modifiedRotation;
        double frontRightPower = modifiedY - modifiedX - modifiedRotation;
        double backRightPower = modifiedY + modifiedX - modifiedRotation;
        double backLeftPower = modifiedY - modifiedX + modifiedRotation;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private double squareSigned(double x) {
        return Math.signum(x) * x * x;
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void resetYaw() {
        imu.resetYaw();
    }
}
