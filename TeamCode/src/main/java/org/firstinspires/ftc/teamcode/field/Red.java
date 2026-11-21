package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

// EVERY field pose.
public final class Red {

    private Red() {}

    /** Which side we are. Pass to MatchState.isFresh / summary. */
    public static final MatchState.Alliance ALLIANCE = MatchState.Alliance.RED;

    private static Pose at(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    // ---- starts ----
    public static final Pose START            = at(56.0,  12.0,  90);

    // Was (60, 134, 270): intake at y = 144.2, through the far wall.
    public static final Pose START_FAR        = at(60.2, 131.7,  90);

    // ---- shooting ----
    public static final Pose SHOOT_NEAR       = at(56.0,  35.0,  90);   // 16.4 in, AUDIENCE

    // Was y = 104.5 -- 3.5 in SHORT of the scoring zone, which starts
    // 352, not 270. At 270 the two barrels sit side-on to the goal and their
    // bearings diverge 17.1 deg, far over the 6 deg dual limit, so the volley
    // is sequential. At 352 the barrels line up with the goal: divergence
    // 0.1 deg, both shots simultaneous. Turret needs 90.3 deg, inside the
    // 105 deg limit. 187 is the mirror option if 352 points somewhere awkward.
    public static final Pose SHOOT_MID        = at(59.5, 120.5, 352);   // 26.6 in, SCORING

    // Was y = 134 -- in the zone, but 40.0 in against a table that stops
    public static final Pose SHOOT_FAR        = at(60.0, 131.0, 270);   // 37.0 in, SCORING

    // ---- collecting ----
    public static final Pose GARDEN_APPROACH  = at(11.5,  24.0,  90);
    public static final Pose GARDEN           = at( 8.5,  11.5,  90);
    public static final Pose GARDEN_SWEEP     = at(20.0,   8.0, 180);
    public static final Pose GARDEN_FAR       = at(10.7, 133.2,  90);

    // Both flower poses were INSIDE the flower -- 6.3 in and 9.0
    public static final Pose FLOWER_NEAR      = at(16.8,  41.0,   0);
    public static final Pose FLOWER_FAR       = at(48.0, 125.5,  90);

    // ---- parks ----
    public static final Pose PARK             = at(12.0,  97.0,  99.3);
    public static final Pose PARK_FAR         = at(12.0, 115.0, 220);

    // Sits 8.4 in from the near flower.
    public static final Pose PARK_SOUTH       = at(10.6,  42.8, 180);

    // ---- bezier control points ----
    public static final Pose C_START_GARDEN   = at(56.0,  20.0,   0);
    public static final Pose C_GARDEN_PARK    = at(24.0,  24.0,   0);
    public static final Pose C_GARDEN_SHOOT   = at(40.5, 140.8,   0);
    public static final Pose C_SHOOT_PARK     = at(12.9,  37.2,   0);
    public static final Pose C_START_FLOWER   = at(60.0, 120.0, 270);
    public static final Pose C_FLOWER_SHOOT   = at(48.0, 120.0, 270);
    public static final Pose C_SHOOT_SOUTH    = at(13.5,  95.6,   0);
}
