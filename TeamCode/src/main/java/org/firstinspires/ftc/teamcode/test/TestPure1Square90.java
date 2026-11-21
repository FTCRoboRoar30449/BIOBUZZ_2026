package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

// 48 in square CENTRED on 72,72 -- same centre as the circle runs, so both
// shapes cover the same ground. Corners at 48 and 96 on both axes. Starts on
// the field centre, so leg 1 is a 33.9 in diagonal lead-in to the first
// corner. Counter-clockwise, heading held at 90 throughout.
// Follower only: no RobotHardware, no MechController, no vision, no turret,
// no helper classes.
@Autonomous(name = "P1 square 90", group = "ZTest pure")
public class TestPure1Square90 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose[] pts = {
                new Pose(72.0, 72.0, Math.toRadians(90)),   // centre, start
                new Pose(96.0, 48.0, Math.toRadians(90)),   // lead-in corner
                new Pose(96.0, 96.0, Math.toRadians(90)),
                new Pose(48.0, 96.0, Math.toRadians(90)),
                new Pose(48.0, 48.0, Math.toRadians(90)),
                new Pose(96.0, 48.0, Math.toRadians(90)),   // closes the square
                new Pose(72.0, 72.0, Math.toRadians(90)),   // back to the centre
        };

        Follower follower = Constants.create(hardwareMap);
        follower.setPose(pts[0]);

        telemetry.addLine("P1 square 90, lead-in + 4 sides + return");
        telemetry.update();
        waitForStart();

        long worst = 0;

        for (int i = 0; i < pts.length - 1 && opModeIsActive(); i++) {
            Pose a = pts[i];
            Pose b = pts[i + 1];

            follower.follow(Paths.line(a, b).linear(a, b));

            while (opModeIsActive() && follower.isBusy()) {
                long t = System.nanoTime();
                follower.update();
                Pose p = follower.pose();
                long ms = (System.nanoTime() - t) / 1000000L;
                if (ms > worst) worst = ms;
                telemetry.addData("leg", "%d / 6  %s", i + 1, i == 0 ? "lead-in" : "side");
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
