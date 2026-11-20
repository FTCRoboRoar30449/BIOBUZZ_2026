package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

// Dia 48 circle centred on 72,72, as four quarter arcs, heading follows
// the path. The lead-in stays at 90, which IS the CCW tangent at 96,72,
// so arc 1 starts with no heading step.
// Starts at the field centre, so leg 1 is a 24 in lead-in out to the circle.
// Each control point is the corner of the bounding square, where the two end
// tangents meet. Follower only: no helper classes.
@Autonomous(name = "P4 circle tangent", group = "ZTest pure")
public class TestPure4CircleTangent extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose start = new Pose(72.0, 72.0, Math.toRadians(90));

        Pose e = new Pose(96.0, 72.0, Math.toRadians(90));
        Pose n = new Pose(72.0, 96.0, Math.toRadians(90));
        Pose w = new Pose(48.0, 72.0, Math.toRadians(90));
        Pose s = new Pose(72.0, 48.0, Math.toRadians(90));

        Pose cne = new Pose(96.0, 96.0, Math.toRadians(90));
        Pose cnw = new Pose(48.0, 96.0, Math.toRadians(90));
        Pose csw = new Pose(48.0, 48.0, Math.toRadians(90));
        Pose cse = new Pose(96.0, 48.0, Math.toRadians(90));

        Follower follower = Constants.create(hardwareMap);
        follower.setPose(start);

        telemetry.addLine("P4 circle tangent, lead-in + 4 arcs");
        telemetry.update();
        waitForStart();

        long worst = 0;

        follower.follow(Paths.line(start, e).linear(start, e));
        while (opModeIsActive() && follower.isBusy()) {
            long t = System.nanoTime();
            follower.update();
            Pose p = follower.pose();
            long ms = (System.nanoTime() - t) / 1000000L;
            if (ms > worst) worst = ms;
            telemetry.addData("leg", "lead-in");
            telemetry.addData("pose", "%.1f, %.1f @ %.1f",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("worst update ms", worst);
            telemetry.update();
        }

        follower.follow(Paths.curve(e, cne, n).tangent());
        while (opModeIsActive() && follower.isBusy()) {
            long t = System.nanoTime();
            follower.update();
            Pose p = follower.pose();
            long ms = (System.nanoTime() - t) / 1000000L;
            if (ms > worst) worst = ms;
            telemetry.addData("leg", "arc 1 / 4");
            telemetry.addData("pose", "%.1f, %.1f @ %.1f",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("worst update ms", worst);
            telemetry.update();
        }

        follower.follow(Paths.curve(n, cnw, w).tangent());
        while (opModeIsActive() && follower.isBusy()) {
            long t = System.nanoTime();
            follower.update();
            Pose p = follower.pose();
            long ms = (System.nanoTime() - t) / 1000000L;
            if (ms > worst) worst = ms;
            telemetry.addData("leg", "arc 2 / 4");
            telemetry.addData("pose", "%.1f, %.1f @ %.1f",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("worst update ms", worst);
            telemetry.update();
        }

        follower.follow(Paths.curve(w, csw, s).tangent());
        while (opModeIsActive() && follower.isBusy()) {
            long t = System.nanoTime();
            follower.update();
            Pose p = follower.pose();
            long ms = (System.nanoTime() - t) / 1000000L;
            if (ms > worst) worst = ms;
            telemetry.addData("leg", "arc 3 / 4");
            telemetry.addData("pose", "%.1f, %.1f @ %.1f",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("worst update ms", worst);
            telemetry.update();
        }

        follower.follow(Paths.curve(s, cse, e).tangent());
        while (opModeIsActive() && follower.isBusy()) {
            long t = System.nanoTime();
            follower.update();
            Pose p = follower.pose();
            long ms = (System.nanoTime() - t) / 1000000L;
            if (ms > worst) worst = ms;
            telemetry.addData("leg", "arc 4 / 4");
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
