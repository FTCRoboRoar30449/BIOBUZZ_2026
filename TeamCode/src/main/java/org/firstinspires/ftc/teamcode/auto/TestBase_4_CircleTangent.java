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
@Autonomous(name = "B4 circle, heading tangent", group = "ZTest base")
public class TestBase_4_CircleTangent extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return TestGeom.CENTER; }

    private static Path ln(Pose a, Pose b)  { return Paths.line(a, b).linear(a, b); }
    private static Path lnT(Pose a, Pose b) { return Paths.line(a, b).tangent(); }
    private static Path cv(Pose a, Pose c, Pose b)  { return Paths.curve(a, c, b).linear(a, b); }
    private static Path cvT(Pose a, Pose c, Pose b) { return Paths.curve(a, c, b).tangent(); }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            // lead-in, holds 90 deg = the CCW tangent at CIR_E
            new Step(ln(TestGeom.CENTER, TestGeom.CIR_E), MechState.IDLE, null),
            new Step(cvT(TestGeom.CIR_E, TestGeom.C_NE, TestGeom.CIR_N), MechState.IDLE, null),
            new Step(cvT(TestGeom.CIR_N, TestGeom.C_NW, TestGeom.CIR_W), MechState.IDLE, null),
            new Step(cvT(TestGeom.CIR_W, TestGeom.C_SW, TestGeom.CIR_S), MechState.IDLE, null),
            new Step(cvT(TestGeom.CIR_S, TestGeom.C_SE, TestGeom.CIR_E), MechState.IDLE, null),
        };
    }
}