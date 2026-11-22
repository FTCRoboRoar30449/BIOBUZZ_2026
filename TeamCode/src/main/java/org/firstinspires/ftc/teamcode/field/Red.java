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
    // Faces -90, down the field. It used to face +90, into a wall 12 in
    // away, which forced a 180 deg step on the first leg of every far route.
    // PLACE THE ROBOT FACING THE HIVE, not the wall behind it.
    public static final Pose START_FAR        = at(60.2, 131.7, -90);

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

    // ---- the shot that actually scores, 22 Nov ----
    // Dead in line with the cell, square to the field, 27.7 in back from the
    // point beneath it. Both balls land on 24.5 in, hood 75, turret +-9.5,
    // and that is the only range measured at 0.88 -- 26.5 in gave 0.44 and
    // 36.4 in gave 0.33. Every auto and the gamepad 2 A macro aim here.
    public static final Pose SHOOT_CLOSE_AUD  = at(59.25,  31.6,  90);
    public static final Pose SHOOT_CLOSE_SCO  = at(59.25, 112.4, -90);

    // Names the cycle autos use. REVERTED 22 Nov after a field test: pointing
    // these at SHOOT_CLOSE_SCO put a 180 deg heading step into six routes,
    // because leg() holds the TARGET heading from the first instant and this
    // drivetrain lurches on a step that large. The only auto that still ran
    // clean was the one route with no turn on any leg. The close pose is
    // worth 0.88 against 0.33, but not at the price of not arriving.
    public static final Pose SHOOT_NEAR       = SHOOT_AUD_MID;
    public static final Pose SHOOT_MID        = SHOOT_SCO_MID;
    public static final Pose SHOOT_FAR        = SHOOT_SCO_EAST;

    // ---- collecting ----
    // Waypoints the garden runs route through. They take the heading of the
    // pose AFTER them, so the short leg does the turning and the long leg is
    // pure translation -- this drivetrain lurches when asked to do both.
    // RED's sit on the direct line and change nothing; BLUE's are what keep
    // its routes off the hive. See Blue.GARDEN_VIA.
    // A short leg whose only job is to carry a big heading change, so the
    // long leg after it is pure translation.
    public static final Pose GARDEN_TURN      = at(48.0,  16.0,  90);
    public static final Pose FLOWER_NEAR_TURN = at(40.0,  36.0, 180);
    public static final Pose GARDEN_VIA       = at(32.0,  26.5,  90);
    public static final Pose GARDEN_VIA_FAR   = at(35.0, 124.5,  90);

    public static final Pose GARDEN_APPROACH  = at(11.5,  24.0,  90);
    public static final Pose GARDEN           = at( 8.5,  11.5,  90);
    public static final Pose GARDEN_SWEEP     = at(20.0,   8.0, 180);
    public static final Pose GARDEN_FAR       = at(10.7, 133.2,  90);

    // Intake sits FLOWER_COLLECT_IN ahead of centre, so these put it on the
    // flowers at 4,48 and 48,140.
    public static final Pose FLOWER_NEAR      = at(18.5,  48.0, 180);
    // All four flower approaches, in FIELD coordinates -- identical in both
    // alliance classes, because a flower does not move when you change sides.
    // ShooterAdapter's flowerZones already cover all four; the Y macro used
    // to look at only this alliance's two, which is why it kept going to the
    // same one. Each sits where the INTAKE lands on the flower.
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
    // Keeps SHOOT_SCO_EAST -> PARK_FAR north of the hive frame.
    public static final Pose C_FAR_PARK       = at(46.0, 125.0,   0);
}
