package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

// 48 x 48 square from the field centre, heading follows the path.
// Leg 1 runs along +X, so the robot pivots 90 at the start: the start pose
// is 90 but a tangent path wants 0. Expected, not a fault.
// Follower only: no RobotHardware, no MechController, no vision, no turret,
// no helper classes.
@Autonomous(name = "P2 square tangent", group = "ZTest pure")
public class TestPure2SquareTangent extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose p0 = new Pose(72.0,  72.0,  Math.toRadians(90));
        Pose p1 = new Pose(120.0, 72.0,  Math.toRadians(90));
        Pose p2 = new Pose(120.0, 120.0, Math.toRadians(90));
        Pose p3 = new Pose(72.0,  120.0, Math.toRadians(90));

        Follower follower = Constants.create(hardwareMap);
        follower.setPose(p0);

        telemetry.addLine("P2 square tangent, 4 legs");
        telemetry.update();
        waitForStart();

        long worst = 0;

        follower.follow(Paths.line(p0, p1).tangent());
        while (opModeIsActive() && follower.isBusy()) {
            long t = System.nanoTime();
            follower.update();
            Pose p = follower.pose();
            long ms = (System.nanoTime() - t) / 1000000L;
            if (ms > worst) worst = ms;
            telemetry.addData("leg", "1 / 4");
            telemetry.addData("pose", "%.1f, %.1f @ %.1f",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("worst update ms", worst);
            telemetry.update();
        }

        follower.follow(Paths.line(p1, p2).tangent());
        while (opModeIsActive() && follower.isBusy()) {
            long t = System.nanoTime();
            follower.update();
            Pose p = follower.pose();
            long ms = (System.nanoTime() - t) / 1000000L;
            if (ms > worst) worst = ms;
            telemetry.addData("leg", "2 / 4");
            telemetry.addData("pose", "%.1f, %.1f @ %.1f",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("worst update ms", worst);
            telemetry.update();
        }

        follower.follow(Paths.line(p2, p3).tangent());
        while (opModeIsActive() && follower.isBusy()) {
            long t = System.nanoTime();
            follower.update();
            Pose p = follower.pose();
            long ms = (System.nanoTime() - t) / 1000000L;
            if (ms > worst) worst = ms;
            telemetry.addData("leg", "3 / 4");
            telemetry.addData("pose", "%.1f, %.1f @ %.1f",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("worst update ms", worst);
            telemetry.update();
        }

        follower.follow(Paths.line(p3, p0).tangent());
        while (opModeIsActive() && follower.isBusy()) {
            long t = System.nanoTime();
            follower.update();
            Pose p = follower.pose();
            long ms = (System.nanoTime() - t) / 1000000L;
            if (ms > worst) worst = ms;
            telemetry.addData("leg", "4 / 4");
            telemetry.addData("pose", "%.1f, %.1f @ %.1f",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("worst update ms", worst);
            telemetry.update();
        }

        follower.manual(0, 0, 0);
        telemetry.addLine("done");
        telemetry.addData("worst update ms", worst);
        telemetry.update();
    }
}
