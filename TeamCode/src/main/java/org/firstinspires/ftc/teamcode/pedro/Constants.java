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
        c.frontLeftName.set("LFMot");       // E2
        c.frontRightName.set("RFMot");      // C1
        c.backLeftName.set("LBMot");        // E1
        c.backRightName.set("RBMot");       // C2
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-13.48120741956816);
        c.yPodOffset.set(12.6648105411079);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(1.7690239059665873);
                Controller secondaryTranslationalForward = Controller.proportional(0.6536070640653174);
                Controller primaryTranslationalLateral = Controller.proportional(1.7690239059665873);
                Controller secondaryTranslationalLateral = Controller.proportional(0.6536070640653174);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.0313970192686813));
                c.brake.set(Controller.proportionalFeedforward(0.026687466378379102));

                c.headingFeedback.set(Controller.proportional(17.61616141678706));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.053204842356953684, 0.00776660203508651));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0691102382808568, 0.04341209717977595));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0021295797440916545, 0.002201505189003758));

                c.maxAchievableForwardVelocity.set(59.03433346354528);
                c.maxAchievableStrafeVelocity.set(34.94987967580295);
                c.naturalForwardDeceleration.set(43.48466572464938);
                c.naturalStrafeDeceleration.set(95.69533046340105);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}
