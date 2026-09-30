package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing config. Every value in here is a placeholder: run the matching tuner from
 * {@link Tuning} and paste the code it spits out over the config below.
 */
public class Constants {

    // TODO: replace with Mecanum Tuner output
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // TODO: replace with OTOS Tuner output
    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("otos");
        c.linearScalar.set(1.0);
        c.angularScalar.set(1.0);
        c.offset.set(new Pose(0, 0));
        c.linearUnit.set(DistanceUnit.INCH);
    });

    // TODO: replace with Foresight Tuner output
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1);
                Controller secondaryTranslationalForward = Controller.proportional(0.05);
                Controller primaryTranslationalLateral = Controller.proportional(0.1);
                Controller secondaryTranslationalLateral = Controller.proportional(0.05);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01));
                c.brake.set(Controller.proportionalFeedforward(0.01));

                c.headingFeedback.set(Controller.proportional(1.0));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0, 0.0));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0, 0.0));

                c.maxAchievableForwardVelocity.set(60.0);
                c.maxAchievableStrafeVelocity.set(50.0);
                c.naturalForwardDeceleration.set(50.0);
                c.naturalStrafeDeceleration.set(50.0);
            }
    );

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new OTOSLocalizer(hardwareMap, localizerConfig);
    }

    public static Algorithm createAlgorithm() {
        return new Foresight(foresightConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap), createAlgorithm());
    }
}
