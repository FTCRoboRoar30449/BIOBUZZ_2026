package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

// Same 7 legs as TestPure2HeadingRamp, through Auto_Base.
// Replaces the old B2 square-tangent run.
@Autonomous(name = "B2 heading ramp", group = "ZTest base")
public class TestBase_2_HeadingRamp extends Auto_Base {

    private static final Pose P0 = new Pose(72.0, 72.0, Math.toRadians(90));
    private static final Pose P1 = new Pose(96.0, 72.0, Math.toRadians(90));
    private static final Pose P2 = new Pose(96.0, 96.0, Math.toRadians(180));
    private static final Pose P3 = new Pose(48.0, 96.0, Math.toRadians(270));
    private static final Pose P4 = new Pose(48.0, 48.0, Math.toRadians(180));
    private static final Pose P5 = new Pose(96.0, 48.0, Math.toRadians(90));
    private static final Pose P6 = new Pose(96.0, 72.0, Math.toRadians(90));
    private static final Pose P7 = new Pose(72.0, 72.0, Math.toRadians(90));

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return P0; }

    private static Path ln(Pose a, Pose b) { return Paths.line(a, b).linear(a, b); }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            new Step(ln(P0, P1), MechState.IDLE, null),   // hold 90
            new Step(ln(P1, P2), MechState.IDLE, null),   // 90  -> 180
            new Step(ln(P2, P3), MechState.IDLE, null),   // 180 -> 270
            new Step(ln(P3, P4), MechState.IDLE, null),   // 270 -> 180
            new Step(ln(P4, P5), MechState.IDLE, null),   // 180 -> 90
            new Step(ln(P5, P6), MechState.IDLE, null),   // hold 90
            new Step(ln(P6, P7), MechState.IDLE, null),   // hold 90
        };
    }
}
