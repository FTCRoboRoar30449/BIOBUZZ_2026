package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

// EVERY field pose. Origin at the RED garden corner, x to BLUE, y to SCORING.
public final class Red {

    private Red() {}

    /** Which side we are. Pass to MatchState.isFresh / summary. */
    public static final MatchState.Alliance ALLIANCE = MatchState.Alliance.RED;

    private static Pose at(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    // ---- starts ----
    public static final Pose START            = at(56.0,  12.0,  90);
    public static final Pose START_FAR        = at(60.2, 131.7,  90);

    // ---- shooting ----
    // Three per CELL, spread across that CELL's manual RANGE box so a
    // blocked spot has two alternatives. All 31 in out, all 92.9% on the
    // weaker ball, all divergence 0.0 so both balls go together.
    public static final Pose SHOOT_AUD_WEST   = at(43.0,  33.0, 336);
    public static final Pose SHOOT_AUD_MID    = at(53.0,  29.0, 356);
    public static final Pose SHOOT_AUD_EAST   = at(74.0,  32.0,  36);

    public static final Pose SHOOT_SCO_WEST   = at(40.0, 109.0, 226);
    public static final Pose SHOOT_SCO_MID    = at(59.0, 116.0, 188);
    public static final Pose SHOOT_SCO_EAST   = at(80.0, 108.0, 146);

    // Names the autos already use.
    public static final Pose SHOOT_NEAR       = SHOOT_AUD_MID;
    public static final Pose SHOOT_MID        = SHOOT_SCO_MID;
    public static final Pose SHOOT_FAR        = SHOOT_SCO_EAST;

    // ---- collecting ----
    public static final Pose GARDEN_APPROACH  = at(11.5,  24.0,  90);
    public static final Pose GARDEN           = at( 8.5,  11.5,  90);
    public static final Pose GARDEN_SWEEP     = at(20.0,   8.0, 180);
    public static final Pose GARDEN_FAR       = at(10.7, 133.2,  90);

    // Intake sits FLOWER_COLLECT_IN ahead of centre, so these put it on the
    // flowers at 4,48 and 48,140.
    public static final Pose FLOWER_NEAR      = at(18.5,  48.0, 180);
    public static final Pose FLOWER_FAR       = at(48.0, 125.5,  90);

    // ---- parks ----
    public static final Pose PARK             = at(12.0,  97.0,  99.3);
    public static final Pose PARK_FAR         = at(12.0, 115.0, 220);
    public static final Pose PARK_SOUTH       = at(10.6,  42.8, 180);

    // ---- bezier control points ----
    public static final Pose C_START_GARDEN   = at(56.0,  20.0,   0);
    public static final Pose C_GARDEN_PARK    = at(24.0,  24.0,   0);
    public static final Pose C_GARDEN_SHOOT   = at(34.0, 142.0,   0);
    public static final Pose C_SHOOT_PARK     = at(12.9,  37.2,   0);
    public static final Pose C_START_FLOWER   = at(60.0, 120.0, 270);
    public static final Pose C_FLOWER_SHOOT   = at(48.0, 120.0, 270);
    public static final Pose C_SHOOT_SOUTH    = at( 8.0, 102.0,   0);
    // Keeps SHOOT_SCO_EAST -> PARK_FAR north of the hive frame.
    public static final Pose C_FAR_PARK       = at(46.0, 125.0,   0);
}
