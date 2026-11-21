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

    // 48 in square CENTRED on 72,72, same centre as the circle, so the two
    // shapes cover the same ground. Corners at 48 and 96 on both axes. The
    // robot starts on the centre, so a diagonal lead-in runs CENTER -> SQ0
    // first, 33.9 in. Counter-clockwise, like the circle.
    public static final Pose SQ0 = at(96.0, 48.0, 90);
    public static final Pose SQ1 = at(96.0, 96.0, 90);
    public static final Pose SQ2 = at(48.0, 96.0, 90);
    public static final Pose SQ3 = at(48.0, 48.0, 90);

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
