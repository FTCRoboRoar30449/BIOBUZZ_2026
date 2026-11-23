package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

public final class Red {

    private Red() {}

    /** Which side we are. Pass to MatchState.isFresh / summary. */
    public static final MatchState.Alliance ALLIANCE = MatchState.Alliance.RED;

    private static Pose at(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    // ---- starts ----
    public static final Pose START            = at(56.0,  12.0,  90);
    public static final Pose START_FAR        = at(60.2, 131.7, -90);

    // ---- shooting ----
    public static final Pose SHOOT_AUD_WEST   = at(43.0,  33.0, 336);
    public static final Pose SHOOT_AUD_MID    = at(53.0,  29.0, 356);
    public static final Pose SHOOT_AUD_EAST   = at(74.0,  32.0,  36);

    public static final Pose SHOOT_SCO_WEST   = at(40.0, 109.0, 226);
    public static final Pose SHOOT_SCO_MID    = at(59.0, 116.0, 188);
    public static final Pose SHOOT_SCO_EAST   = at(80.0, 108.0, 146);

    // ---- the shot that actually scores, 22 Nov ----
    public static final Pose SHOOT_CLOSE_AUD  = at(55.31,  31.25,  90);
    public static final Pose SHOOT_CLOSE_SCO  = at(63.19, 112.75, -90);

    public static final Pose SHOOT_NEAR       = SHOOT_CLOSE_AUD;
    public static final Pose SHOOT_MID        = SHOOT_CLOSE_SCO;
    public static final Pose SHOOT_FAR        = SHOOT_CLOSE_SCO;

    // ---- collecting ----
    public static final Pose GARDEN_TURN      = at(48.0,  16.0,  90);
    public static final Pose FLOWER_NEAR_TURN = at(40.0,  36.0, 180);
    public static final Pose GARDEN_VIA       = at(32.0,  26.5,  90);
    public static final Pose GARDEN_VIA_FAR   = at(35.0, 124.5,  90);

    public static final Pose GARDEN_APPROACH  = at(11.5,  24.0,  90);
    public static final Pose GARDEN           = at( 8.5,  11.5,  90);
    public static final Pose GARDEN_SWEEP     = at(20.0,   8.0, 180);
    public static final Pose GARDEN_FAR       = at(10.7, 133.2,  90);

    public static final Pose FLOWER_NEAR      = at(18.5,  48.0, 180);
    public static final Pose FLOWER_RED_AUD   = at( 18.5,  48.0, 180);
    public static final Pose FLOWER_RED_SCO   = at( 48.0, 125.5,  90);
    public static final Pose FLOWER_BLU_AUD   = at( 96.0,  18.5, 270);
    public static final Pose FLOWER_BLU_SCO   = at(125.5,  96.0,   0);
    public static final Pose FLOWER_FAR       = at(48.0, 125.5,  90);

    // ---- parks ----
    public static final Pose PARK             = at(12.0,  97.0,  99.3);
    public static final Pose PARK_FAR         = at(12.0, 115.0, 220);
    public static final Pose PARK_SOUTH       = at(10.6,  42.8, 180);

    public static final Pose PRE_AUD_PARK     = at(14.0,  22.0,  90);
    public static final Pose PRE_SCO_MID      = at(29.0, 103.0, -90);
    public static final Pose PRE_SCO_PARK     = at(14.0,  45.0, -90);

    // ---- bezier control points ----
    public static final Pose C_START_GARDEN   = at(56.0,  20.0,   0);
    public static final Pose C_GARDEN_PARK    = at(24.0,  24.0,   0);
    public static final Pose C_GARDEN_SHOOT   = at(34.0, 142.0,   0);
    public static final Pose C_SHOOT_PARK     = at(12.9,  37.2,   0);
    public static final Pose C_START_FLOWER   = at(60.0, 120.0, 270);
    public static final Pose C_FLOWER_SHOOT   = at(48.0, 120.0, 270);
    public static final Pose C_SHOOT_SOUTH    = at( 8.0, 102.0,   0);
    public static final Pose C_FAR_PARK       = at(46.0, 125.0,   0);
}
