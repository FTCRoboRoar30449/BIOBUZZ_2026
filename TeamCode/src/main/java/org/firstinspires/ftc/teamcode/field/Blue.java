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

    public static final Pose START            = mirror(Red.START);
    public static final Pose START_FAR        = mirror(Red.START_FAR);

    public static final Pose SHOOT_NEAR       = mirror(Red.SHOOT_NEAR);
    public static final Pose SHOOT_MID        = mirror(Red.SHOOT_MID);
    public static final Pose SHOOT_FAR        = mirror(Red.SHOOT_FAR);

    public static final Pose GARDEN_APPROACH  = mirror(Red.GARDEN_APPROACH);
    public static final Pose GARDEN           = mirror(Red.GARDEN);
    public static final Pose GARDEN_SWEEP     = mirror(Red.GARDEN_SWEEP);
    public static final Pose GARDEN_FAR       = mirror(Red.GARDEN_FAR);

    public static final Pose FLOWER_NEAR      = mirror(Red.FLOWER_NEAR);
    public static final Pose FLOWER_FAR       = mirror(Red.FLOWER_FAR);

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
}
