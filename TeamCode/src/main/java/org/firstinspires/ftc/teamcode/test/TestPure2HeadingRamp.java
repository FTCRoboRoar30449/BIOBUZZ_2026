package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

// Replaces the old P2 square-tangent run.
//
// Straight lines only, but the commanded heading CHANGES along most legs:
// 90 -> 90, 90 -> 180, 180 -> 270, 270 -> 180, 180 -> 90, 90 -> 90, 90 -> 90.
// Every leg's end pose is the next leg's start pose, heading included, so
// the chain is continuous and nothing steps between legs.
//
// P1/P3 held a fixed heading and were clean. P4 tracked a moving heading and
// was not. This run puts a moving heading on straight lines: if it shakes on
// legs 2-5 and not on 1, 6, 7, the heading ramp is the fault and curves are
// innocent.
//
// Follower only: no helper classes, no robot code.
@Autonomous(name = "P2 heading ramp", group = "ZTest pure")
public class TestPure2HeadingRamp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose[] pts = {
                new Pose(72.0, 72.0, Math.toRadians(90)),
                new Pose(96.0, 72.0, Math.toRadians(90)),
                new Pose(96.0, 96.0, Math.toRadians(180)),
                new Pose(48.0, 96.0, Math.toRadians(270)),
                new Pose(48.0, 48.0, Math.toRadians(180)),
                new Pose(96.0, 48.0, Math.toRadians(90)),
                new Pose(96.0, 72.0, Math.toRadians(90)),
                new Pose(72.0, 72.0, Math.toRadians(90)),
        };

        Follower follower = Constants.create(hardwareMap);
        follower.setPose(pts[0]);

        telemetry.addLine("P2 heading ramp, 7 legs");
        telemetry.addLine("legs 2-5 change heading, 1/6/7 hold 90");
        telemetry.update();
        waitForStart();

        long worst = 0;

        for (int i = 0; i < pts.length - 1 && opModeIsActive(); i++) {
            Pose a = pts[i];
            Pose b = pts[i + 1];
            double fromDeg = Math.toDegrees(a.heading());
            double toDeg = Math.toDegrees(b.heading());

            follower.follow(Paths.line(a, b).linear(a, b));

            while (opModeIsActive() && follower.isBusy()) {
                long t = System.nanoTime();
                follower.update();
                Pose p = follower.pose();
                long ms = (System.nanoTime() - t) / 1000000L;
                if (ms > worst) worst = ms;
                telemetry.addData("leg", "%d / 7", i + 1);
                telemetry.addData("heading cmd", "%.0f -> %.0f  %s",
                        fromDeg, toDeg, fromDeg == toDeg ? "HOLD" : "RAMP");
                telemetry.addData("pose", "%.1f, %.1f @ %.1f",
                        p.x(), p.y(), Math.toDegrees(p.heading()));
                telemetry.addData("worst update ms", worst);
                telemetry.update();
            }
        }

        follower.manual(0, 0, 0);
        telemetry.addLine("done");
        telemetry.addData("worst update ms", worst);
        telemetry.update();
    }
}
