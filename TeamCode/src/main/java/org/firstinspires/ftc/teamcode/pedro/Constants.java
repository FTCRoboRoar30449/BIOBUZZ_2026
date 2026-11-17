package org.firstinspires.ftc.teamcode.pedro;

import static java.lang.Double.NaN;

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
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(6.6407480314960629921259842519685);//12.212947334830218
        c.yPodOffset.set(7.467519685039370078740157480315);//11.773001603254185
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.9666517823617802);
                Controller secondaryTranslationalForward = Controller.proportional(0.357152004171346);
                Controller primaryTranslationalLateral = Controller.proportional(0.9666517823617802); //Failed in Auto tune
                Controller secondaryTranslationalLateral = Controller.proportional(0.357152004171346); //Failed in Auto tune

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.019640793150820245));
                c.brake.set(Controller.proportionalFeedforward(0.016694674178197207));

                c.headingFeedback.set(Controller.proportional(21.04314578264762));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04706473311366131, 0.014164106851742835));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0679368072787105, 0.039789334374364535));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001564495578625028, 0.0012979586318519743));

                c.maxAchievableForwardVelocity.set(78.3652854281816);
                c.maxAchievableStrafeVelocity.set(50.424809628472396);
                c.naturalForwardDeceleration.set(67.51456825824887);
                c.naturalStrafeDeceleration.set(125.9960301832918);
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
