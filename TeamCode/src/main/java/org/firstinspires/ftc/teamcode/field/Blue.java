package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

public final class Blue {

    private Blue() {}

    /** Which side we are. Pass to MatchState.isFresh / summary. */
    public static final MatchState.Alliance ALLIANCE = MatchState.Alliance.BLUE;

    private static final double FIELD_IN = 144.0;

    private static Pose mirror(Pose red) {
        double h = Math.PI - red.heading();
        while (h >  Math.PI) h -= 2 * Math.PI;
        while (h <= -Math.PI) h += 2 * Math.PI;
        return new Pose(FIELD_IN - red.x(), red.y(), h);
    }

    private static Pose rotate(Pose red) {
        double h = red.heading() + Math.PI;
        while (h >  Math.PI) h -= 2 * Math.PI;
        while (h <= -Math.PI) h += 2 * Math.PI;
        return new Pose(FIELD_IN - red.x(), FIELD_IN - red.y(), h);
    }

    public static final Pose START            = mirror(Red.START);
    public static final Pose START_FAR        = mirror(Red.START_FAR);

    public static final Pose SHOOT_AUD_WEST   = mirror(Red.SHOOT_AUD_WEST);
    public static final Pose SHOOT_AUD_MID    = mirror(Red.SHOOT_AUD_MID);
    public static final Pose SHOOT_AUD_EAST   = mirror(Red.SHOOT_AUD_EAST);

    public static final Pose SHOOT_SCO_WEST   = mirror(Red.SHOOT_SCO_WEST);
    public static final Pose SHOOT_SCO_MID    = mirror(Red.SHOOT_SCO_MID);
    public static final Pose SHOOT_SCO_EAST   = mirror(Red.SHOOT_SCO_EAST);

    // ---- the shot that actually scores, 22 Nov. See Red for the numbers ----
    public static final Pose SHOOT_CLOSE_AUD  = new Pose(80.81,  31.25, Math.toRadians(90));
    public static final Pose SHOOT_CLOSE_SCO  = new Pose(88.69, 112.75, Math.toRadians(-90));

    public static final Pose SHOOT_NEAR       = SHOOT_CLOSE_AUD;
    public static final Pose SHOOT_MID        = SHOOT_CLOSE_SCO;
    public static final Pose SHOOT_FAR        = SHOOT_CLOSE_SCO;

    public static final Pose GARDEN_TURN      = new Pose( 96.0,  20.0, Math.toRadians(-90));
    public static final Pose FLOWER_NEAR_TURN = new Pose( 93.0,  24.0, Math.toRadians(270));
    public static final Pose GARDEN_VIA       = new Pose(120.0,  46.0, Math.toRadians(-90));
    public static final Pose GARDEN_VIA_FAR   = new Pose(120.0, 104.0, Math.toRadians(-90));

    public static final Pose GARDEN_APPROACH  = rotate(Red.GARDEN_APPROACH);
    public static final Pose GARDEN           = rotate(Red.GARDEN);
    public static final Pose GARDEN_SWEEP     = rotate(Red.GARDEN_SWEEP);
    public static final Pose GARDEN_FAR       = rotate(Red.GARDEN_FAR);

    public static final Pose PRE_AUD_PARK     = new Pose(130.0, 22.0, Math.toRadians(90));
    public static final Pose PRE_SCO_MID      = new Pose(115.0, 103.0, Math.toRadians(-90));
    public static final Pose PRE_SCO_PARK     = new Pose(130.0, 45.0, Math.toRadians(-90));

    public static final Pose FLOWER_NEAR      = new Pose(96.0,  18.5, Math.toRadians(270));
    public static final Pose FLOWER_RED_AUD   = new Pose(18.5, 48.0, Math.toRadians(180));
    public static final Pose FLOWER_RED_SCO   = new Pose(48.0, 125.5, Math.toRadians(90));
    public static final Pose FLOWER_BLU_AUD   = new Pose(96.0, 18.5, Math.toRadians(270));
    public static final Pose FLOWER_BLU_SCO   = new Pose(125.5, 96.0, Math.toRadians(0));
    public static final Pose FLOWER_FAR       = new Pose(125.5, 96.0, Math.toRadians(0));

    public static final Pose PARK             = new Pose(132.0, 97.0, Math.toRadians(-90));
    public static final Pose PARK_FAR         = mirror(Red.PARK_FAR);
    public static final Pose PARK_SOUTH       = mirror(Red.PARK_SOUTH);

    public static final Pose C_START_GARDEN   = mirror(Red.C_START_GARDEN);
    public static final Pose C_GARDEN_PARK    = mirror(Red.C_GARDEN_PARK);
    public static final Pose C_GARDEN_SHOOT   = mirror(Red.C_GARDEN_SHOOT);
    public static final Pose C_SHOOT_PARK     = mirror(Red.C_SHOOT_PARK);
    public static final Pose C_START_FLOWER   = mirror(Red.C_START_FLOWER);
    public static final Pose C_FLOWER_SHOOT   = mirror(Red.C_FLOWER_SHOOT);
    public static final Pose C_SHOOT_SOUTH    = mirror(Red.C_SHOOT_SOUTH);
    public static final Pose C_FAR_PARK       = mirror(Red.C_FAR_PARK);
}
