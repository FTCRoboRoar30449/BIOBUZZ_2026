package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

// EVERY field pose. Blue mirrors this file; change a number here and both
// alliances follow.
//
// Three rules every pose has to pass, all checked 13 Nov:
//
//   1. The INTAKE must be inside the field, not just the centre. It trails
//      10.181 in behind centre (RobotHardware.INTAKE_BEHIND_IN), so a pose
//      with the centre against a wall puts the intake through it.
//   2. A SHOOTING pose needs BOTH a hive zone (rectangle) AND the 15-39 in
//      shot table, measured from the TURRET AXIS, not the centre. The zone is
//      much the larger; being in it is not enough.
//   3. A FLOWER pose sits 14.535 in from the flower CENTRE
//      (RobotHardware.FLOWER_COLLECT_IN) -- that is where the robot collects.
//      Closer means it has driven into the flower, and nothing in the FSM
//      detects that: flowerRangeIn() is a distance, so any smaller number
//      still reads as "at the flower".
public final class Red {

    private Red() {}

    /** Which side we are. Pass to MatchState.isFresh / summary. */
    public static final MatchState.Alliance ALLIANCE = MatchState.Alliance.RED;

    private static Pose at(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    // ---- starts ----
    // y was 8.0, which put the intake at y = -2.2, through the near wall.
    // 12.0 clears it by 1.8 in and happens to bring the audience goal to
    // 38.4 in, just inside the table.
    public static final Pose START            = at(56.0,  12.0,  90);

    // Was (60, 134, 270): intake at y = 144.2, through the far wall. These
    // are the visualizer's numbers from 13 Nov.
    public static final Pose START_FAR        = at(60.2, 131.7,  90);

    // ---- shooting ----
    // Ranges are turret axis to goal. Table is 15-39; mid-table tolerates
    // odometry drift in both directions, the edges do not.
    public static final Pose SHOOT_NEAR       = at(56.0,  35.0,  90);   // 16.4 in, AUDIENCE

    // Was y = 104.5 -- 3.5 in SHORT of the scoring zone, which starts at
    // y = 108, so it could never fire however well it aimed.
    public static final Pose SHOOT_MID        = at(59.5, 120.5, 270);   // 26.6 in, SCORING

    // Was y = 134 -- in the zone, but 40.0 in against a table that stops at
    // 39.0, and the intake was through the far wall.
    public static final Pose SHOOT_FAR        = at(60.0, 131.0, 270);   // 37.0 in, SCORING

    // ---- collecting ----
    public static final Pose GARDEN_APPROACH  = at(11.5,  24.0,  90);
    public static final Pose GARDEN           = at( 8.5,  11.5,  90);
    public static final Pose GARDEN_SWEEP     = at(20.0,   8.0, 180);
    public static final Pose GARDEN_FAR       = at(10.7, 133.2,  90);

    // Both flower poses were INSIDE the flower -- 6.3 in and 9.0 in from the
    // centre against a 14.535 in collecting pose. Backed out along their own
    // approach lines.
    //
    // HEADING IS UNVERIFIED. flowerRangeIn() ignores it, so the FSM runs the
    // intake whichever way the robot faces; which end actually collects is a
    // question for the built robot. If it is the BACK (same side as the
    // intake and the C270), FLOWER_NEAR wants 331 and FLOWER_FAR wants 270.
    public static final Pose FLOWER_NEAR      = at(16.8,  41.0,   0);
    public static final Pose FLOWER_FAR       = at(48.0, 125.5,  90);

    // ---- parks ----
    // Was (10.5, 101, 0): intake at x = 0.3, a third of an inch from the
    // wall. The visualizer's pose has 13.6 in of clearance.
    public static final Pose PARK             = at(12.0,  97.0,  99.3);
    public static final Pose PARK_FAR         = at(12.0, 115.0, 220);

    // Sits 8.4 in from the near flower. Legal, but it is a flower, not a
    // park zone -- worth re-drawing once the park zones are marked out.
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
