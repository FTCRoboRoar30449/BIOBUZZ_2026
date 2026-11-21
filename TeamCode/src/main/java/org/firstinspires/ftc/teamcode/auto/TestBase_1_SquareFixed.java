package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.test.TestGeom;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

// Same geometry as the matching TestPure_ run, through Auto_Base, so the
// mech, vision and turret are constructed and updated every loop.
@Autonomous(name = "B1 square, heading 90", group = "ZTest base")
public class TestBase_1_SquareFixed extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return TestGeom.CENTER; }

    private static Path ln(Pose a, Pose b)  { return Paths.line(a, b).linear(a, b); }
    private static Path lnT(Pose a, Pose b) { return Paths.line(a, b).tangent(); }
    private static Path cv(Pose a, Pose c, Pose b)  { return Paths.curve(a, c, b).linear(a, b); }
    private static Path cvT(Pose a, Pose c, Pose b) { return Paths.curve(a, c, b).tangent(); }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            new Step(ln(TestGeom.CENTER, TestGeom.SQ0), MechState.IDLE, null),   // lead-in
            new Step(ln(TestGeom.SQ0, TestGeom.SQ1), MechState.IDLE, null),
            new Step(ln(TestGeom.SQ1, TestGeom.SQ2), MechState.IDLE, null),
            new Step(ln(TestGeom.SQ2, TestGeom.SQ3), MechState.IDLE, null),
            new Step(ln(TestGeom.SQ3, TestGeom.SQ0), MechState.IDLE, null),
            new Step(ln(TestGeom.SQ0, TestGeom.CENTER), MechState.IDLE, null),   // return
        };
    }
}