package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.math.Pose;

// Shared geometry for the eight drivetrain tests. No mech, no vision.
public final class TestGeom {

    private TestGeom() {}

    private static Pose at(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    // Every test starts here.
    public static final Pose CENTER = at(72.0, 72.0, 90);

    public static final double SIDE_IN = 48.0;
    public static final double RADIUS_IN = 24.0;

    // Square, one corner at field centre. 72..120 in both axes, so the far
    // corner sits 24 in off the wall.
    public static final Pose SQ0 = CENTER;   // the square starts on the centre
    public static final Pose SQ1 = at(120.0, 72.0, 90);
    public static final Pose SQ2 = at(120.0, 120.0, 90);
    public static final Pose SQ3 = at(72.0,  120.0, 90);

    // Circle of dia 48 CENTRED on 72,72. The robot cannot start on its own
    // centre, so a 24 in lead-in runs CENTER -> CIR_E first. That lead-in
    // holds 90 deg, which is exactly the CCW tangent at CIR_E, so the
    // tangent-heading runs join it without a step.
    public static final Pose CIR_E = at(96.0, 72.0, 90);
    public static final Pose CIR_N = at(72.0, 96.0, 90);
    public static final Pose CIR_W = at(48.0, 72.0, 90);
    public static final Pose CIR_S = at(72.0, 48.0, 90);

    // Control point of each quarter arc: where the two end tangents meet,
    // which is the corner of the bounding square. A single control point is
    // a quadratic, so the arc bulges about 5% of the radius inside a true
    // circle -- close enough to judge smoothness.
    public static final Pose C_NE = at(96.0, 96.0, 90);
    public static final Pose C_NW = at(48.0, 96.0, 90);
    public static final Pose C_SW = at(48.0, 48.0, 90);
    public static final Pose C_SE = at(96.0, 48.0, 90);
}
