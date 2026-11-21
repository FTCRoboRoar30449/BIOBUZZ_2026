package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

// No data of its own.
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

    public static final Pose SHOOT_NEAR       = SHOOT_AUD_MID;
    public static final Pose SHOOT_MID        = SHOOT_SCO_MID;
    public static final Pose SHOOT_FAR        = SHOOT_SCO_EAST;

    // GARDENs are 180 deg rotationally symmetric: RED's is the bottom-left
    // corner, BLUE's the top-right. mirror() sent BLUE to the bottom-right,
    // which is nobody's garden.
    public static final Pose GARDEN_APPROACH  = rotate(Red.GARDEN_APPROACH);
    public static final Pose GARDEN           = rotate(Red.GARDEN);
    public static final Pose GARDEN_SWEEP     = rotate(Red.GARDEN_SWEEP);
    public static final Pose GARDEN_FAR       = rotate(Red.GARDEN_FAR);

    // NOT mirrored. The FLOWERs are 180 deg rotationally symmetric, not
    // mirrored: BLUE's are at 96,4 and 140,96. mirror() put the intake 48 in
    // from either one. These place it on them.
    public static final Pose FLOWER_NEAR      = new Pose(96.0,  18.5, Math.toRadians(270));
    public static final Pose FLOWER_FAR       = new Pose(125.5, 96.0, Math.toRadians(0));

    public static final Pose PARK             = mirror(Red.PARK);
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
