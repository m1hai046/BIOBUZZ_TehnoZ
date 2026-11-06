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


//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.30278238375935984);
//                Controller secondaryTranslationalForward = Controller.proportional(0.11187000030478465);
//                Controller primaryTranslationalLateral = Controller.proportional(0.4881863939409707);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.18037182798049708);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.010468520142970604));
//                c.brake.set(Controller.proportionalFeedforward(0.008898242121525013));
//
//                c.headingFeedback.set(Controller.proportional(4.775712190343766));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06295060172559165, 0.004208712339527633));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.12582757802581415, 0.09355745245025313));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012966236065444343, 0.0014370305788951848));
//
//                c.maxAchievableForwardVelocity.set(87.36599897029274);
//                c.maxAchievableStrafeVelocity.set(70.48733221794666);
//                c.naturalForwardDeceleration.set(26.808490489603624);
//                c.naturalStrafeDeceleration.set(65.23973076393409);
//            }
//    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.32622429100777356);
                Controller secondaryTranslationalForward = Controller.proportional(0.12053115865377563);
                Controller primaryTranslationalLateral = Controller.proportional(0.45780192718793017);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16914557940315297);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010571326326908824));
                c.brake.set(Controller.proportionalFeedforward(0.0089856273778725));

                c.headingFeedback.set(Controller.proportional(4.842412761528398));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0733786231662938, 0.0033394350253477386));

                c.linearBrakeCoefficients.set(Matrix.diag(0.16101633395840997, 0.08934731331643102));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.19212013811183E-4, 0.0014492879296137612));

                c.maxAchievableForwardVelocity.set(89.17904962393);
                c.maxAchievableStrafeVelocity.set(71.02080394208252);
                c.naturalForwardDeceleration.set(22.2319847840865);
                c.naturalStrafeDeceleration.set(53.40609876927376);
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