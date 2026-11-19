package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FLM");
        c.frontRightName.set("FRM");
        c.backLeftName.set("BLM");
        c.backRightName.set("BRM");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

        c.manualBrakeMode.set(true);
    });


    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-7.090507567398191);
        c.yPodOffset.set(-1.1083111049622063);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });



    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.32351655765573833);
                Controller secondaryTranslationalForward = Controller.proportional(0.11953072353216632);
                Controller primaryTranslationalLateral = Controller.proportional(0.45635807490951374);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1686121145667216);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010267298571769043));
                c.brake.set(Controller.proportionalFeedforward(0.008727203786003687));

                c.headingFeedback.set(Controller.proportional(5.017255679149969));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06274423375909666, 0.0057917666078591435));

                c.linearBrakeCoefficients.set(Matrix.diag(0.13555956377452213, 0.08625813358529318));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016066309041797501, 0.0015716330679988095));

                c.maxAchievableForwardVelocity.set(90.89017883554168);
                c.maxAchievableStrafeVelocity.set(72.45012313844512);
                c.naturalForwardDeceleration.set(25.471203231997947);
                c.naturalStrafeDeceleration.set(62.19413667884652);
            }
    );


    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}