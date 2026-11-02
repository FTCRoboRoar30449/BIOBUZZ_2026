package org.firstinspires.ftc.teamcode.field;

/**
 * Firing solution for the RED alliance, BIOBUZZ.
 *
 * Only ONE red CELL is scoreable at a time -- the one currently raised. The
 * seesaw puts it either on the far side (+Y) or the audience side (-Y); the
 * down CELL faces below horizontal and cannot retain a ball, so it is never a
 * target. Pick the raised one with {@link Target} and everything else follows.
 *
 * ROBOT GEOMETRY
 *   Two shooters, each on its own turret ring fixed in the ROBOT frame at
 *   105 mm forward of centre and +/-100 mm lateral: POLLEN right, NECTAR left.
 *   Both rings are geared to one servo, so they always share a turret angle.
 *   The ball exit sits ON its ring's axis laterally, set back along the barrel
 *   by 5-20 mm depending on hood, so the shot line passes through the turret
 *   axis and the required bearing is simply axis -> goal.
 *
 *   Because the two axes are fixed in the ROBOT (not on a shared turret), the
 *   angle between the two shooters' required bearings is set by ROBOT YAW. It
 *   is exactly zero when the line joining the axes points at the goal, and
 *   worst when the robot faces the goal squarely. Turn side-on and both
 *   shooters fire together with no aim compromise. See sidewaysYawDeg().
 *
 * COORDINATES
 *   Public API is in PEDRO PATHING inches: 0..144 square, origin bottom-left,
 *   field centre at (72, 72), +X toward the BLUE wall, +Y toward the far side
 *   wall (so lower Y is the audience side). Robot yaw CCW from +X, matching
 *   Pedro's own "counterclockwise rotation is positive" convention. Turret
 *   command is CW-positive with 0 = robot forward.
 *
 *   Feed it follower.getPose() directly. AprilTag poses are the one thing
 *   needing conversion -- see the frame notes on PEDRO_OFFSET_IN below, and
 *   be careful: the SDK's frame is ROTATED from Pedro's, not just shifted.
 *
 * The BLUE mirror of this class is BioBuzzShooterBlue. The two are 180-degree
 * rotations of each other; the ballistics tables below are byte-identical
 * between them (verified against the trajectory model), so if you retune a
 * table or a calibration constant HERE, make the same edit THERE.
 */
public final class ShooterRed {

    private ShooterRed() {}

    public enum Ball { POLLEN, NECTAR }

    /** Which red CELL is currently raised. Indexes every per-target array. */
    public enum Target {
        /** Far side of the field, +Y, opposite the audience. Tags 30-33. */
        FAR_SIDE,
        /** Audience side of the field, -Y. Tags 34-37. */
        AUDIENCE_SIDE
    }

    // ---- coordinate frames ------------------------------------------------
    /**
     * THREE frames are in play. Getting them mixed up is the most likely way
     * to make this class produce confident nonsense, so they are spelled out.
     *
     * 1. PEDRO -- what this class's public API speaks, and what your odometry
     *    and the Pinpoint report. 0..144 in square, origin at the bottom-left
     *    corner, field centre at (72, 72). Pedro's docs: "As a robot moves to
     *    the right, x increases. As a robot moves up on the field, y
     *    increases", heading 0 points along +x and "counterclockwise rotation
     *    is positive rotation, similar to a unit circle."
     *
     * 2. DIAGRAM -- the frame the game manual figures and this class's own
     *    geometry were derived in: origin at field centre, +X toward the BLUE
     *    wall, +Y toward the far side wall (away from the audience). Its axes
     *    point the SAME way as Pedro's, so the two differ by a pure
     *    translation of +72 on each axis. Headings, bearings, turret angles
     *    and ranges therefore read identically in DIAGRAM and PEDRO.
     *
     * 3. FTC SDK STANDARD -- what the SDK's AprilTag pipeline reports, and
     *    very likely what your Limelight reports too. Pedro's own docs call it
     *    "nonstandard to" Pedro and give the conversion as a ROTATION plus a
     *    translation, not a shift:
     *        pedroX = sdkY + 72;  pedroY = 72 - sdkX;  heading -= 90 deg
     *
     * The goal and cluster constants below are already in PEDRO. Tag poses are
     * the only thing that needs converting at runtime -- use sdkPoseToPedro if
     * your vision stack reports SDK-standard, diagramPoseToPedro if it reports
     * in the centre-origin frame of the manual figures. CHECK WHICH.
     */
    public static final double PEDRO_OFFSET_IN = 72.0;

    public static double diagramToPedro(double diagramIn) { return diagramIn + PEDRO_OFFSET_IN; }
    public static double pedroToDiagram(double pedroIn)   { return pedroIn - PEDRO_OFFSET_IN; }

    // ---- targets, PEDRO inches -------------------------------------------
    // field (-12.75, +18.14) -> pedro (59.25, 90.14)   far side
    // field (-12.75, -18.14) -> pedro (59.25, 53.86)   audience side
    private static final double[] GOAL_X = { 59.25, 59.25 };
    private static final double[] GOAL_Y = { 90.14, 53.86 };
    // cluster: field (-12.75, +/-12.89) -> pedro (59.25, 84.89 / 59.11)
    private static final double[] TAG_X  = { 59.25, 59.25 };
    private static final double[] TAG_Y  = { 84.89, 59.11 };
    private static final int[][]  TAG_ID = { {30, 31, 32, 33}, {34, 35, 36, 37} };

    /** Aperture centre height; same for whichever CELL is raised. */
    public static final double GOAL_Z = 59.55;
    /** Tag cluster height; the cluster rides its own CELL. */
    public static final double TAG_Z  = 49.662;

    public static double goalX(Target t) { return GOAL_X[t.ordinal()]; }
    public static double goalY(Target t) { return GOAL_Y[t.ordinal()]; }
    public static double tagX(Target t)  { return TAG_X[t.ordinal()]; }
    public static double tagY(Target t)  { return TAG_Y[t.ordinal()]; }
    public static int[] tagIds(Target t) { return TAG_ID[t.ordinal()].clone(); }

    /** Which red target a detected tag belongs to, or null if not ours. */
    public static Target targetForTagId(int id) {
        if (id >= 30 && id <= 33) return Target.FAR_SIDE;
        if (id >= 34 && id <= 37) return Target.AUDIENCE_SIDE;
        return null;
    }

    // ---- turret rings, ROBOT frame, inches ------------------------------
    public static final double AXIS_FWD = 105.0 / 25.4;   // 4.1339
    public static final double AXIS_LAT = 100.0 / 25.4;   // 3.9370

    // ---- flywheel --------------------------------------------------------
    public static final double WHEEL_DIA_M = 0.072;

    /**
     * Ball exit speed / wheel surface speed. The CAD shows ONE driven wheel
     * against a hood, so ~0.5 is the right regime -- but the exact value is
     * unmeasured and scales every RPM linearly. CALIBRATE THIS.
     */
    public static double efficiency = 0.50;

    public static final double TABLE_MIN_IN = 15.0;
    public static final double TABLE_MAX_IN = 39.0;

    // ---- Limelight 3A, turret-mounted ------------------------------------
    // Rotates with the turret, so its boresight azimuth IS the turret bearing.
    // 54.5 x 42 deg is the 3A's FOV -- the narrowest Limelight made.
    public static final double CAM_BACK_IN   = 15.066 / 25.4;   // 0.5932
    public static final double CAM_HEIGHT_IN = 441.327 / 25.4;  // 17.375
    public static final double CAM_PITCH_DEG = 45.0;
    public static final double CAM_HFOV_HALF = 27.25;
    public static final double CAM_VFOV_HALF = 21.0;

    // ---- tables, one entry per inch of range from 15 in -------------------
    // Hood is discrete and each row's speed belongs to that row's hood, so
    // look the pair up by nearest inch rather than interpolating across a
    // hood change. Nearest-inch costs <= 0.025 m/s against a ~5% tolerance.
    private static final double[] POLLEN_HOOD = {
            80, 80, 80, 80, 80, 80, 80, 80, 80, 75, 75, 75, 75,
            75, 75, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70 };
    private static final double[] POLLEN_V = {
            4.950, 4.925, 4.950, 4.975, 5.025, 5.075, 5.125, 5.225, 5.275,
            5.000, 5.025, 5.050, 5.075, 5.100, 5.125, 5.100, 5.100, 5.150,
            5.150, 5.150, 5.175, 5.200, 5.200, 5.250, 5.250 };
    private static final double[] POLLEN_MAKE = {
            81.1, 83.0, 87.9, 89.2, 92.5, 93.3, 94.2, 91.6, 87.9, 92.9, 94.9, 92.8,
            95.0, 92.7, 94.9, 92.7, 92.7, 92.5, 92.5, 92.5, 94.7, 92.2, 92.2, 91.9, 91.9 };

    private static final double[] NECTAR_HOOD = {
            80, 80, 80, 80, 80, 80, 80, 80, 75, 75, 75, 75, 75,
            75, 75, 75, 70, 70, 70, 70, 70, 70, 70, 70, 70 };
    private static final double[] NECTAR_V = {
            4.925, 4.900, 4.925, 4.975, 5.025, 5.075, 5.150, 5.200, 4.975,
            4.975, 5.000, 5.025, 5.050, 5.075, 5.100, 5.150, 5.075, 5.075,
            5.125, 5.125, 5.150, 5.175, 5.175, 5.225, 5.225 };
    private static final double[] NECTAR_MAKE = {
            68.8, 76.7, 83.0, 86.4, 89.0, 90.8, 86.3, 83.4, 89.5, 89.9, 92.9, 90.0,
            92.9, 89.8, 92.7, 92.5, 89.9, 89.9, 89.6, 89.6, 92.5, 89.3, 89.3, 88.9, 88.9 };

    /** Exit setback behind the turret axis, inches. Linear in hood (CAD). */
    public static double setbackIn(Ball ball, double hoodDeg) {
        double a55 = (ball == Ball.NECTAR) ?  6.208 :  5.323;
        double a80 = (ball == Ball.NECTAR) ? 19.579 : 17.011;
        return (a55 + (hoodDeg - 55.0) * (a80 - a55) / 25.0) / 25.4;
    }

    /**
     * Shots modelled below this are rejected. At the default 89 the usable
     * ranges are POLLEN 18-22 and 24-39 in, NECTAR 19-20 and 23-37 in. Lower
     * it to widen the envelope at the cost of confidence.
     */
    public static double MIN_CONFIDENCE_PCT = 89.0;

    public static final class Shot {
        public final boolean valid;
        /** Turret servo command, degrees, CW-positive, 0 = robot forward. */
        public final double turretDeg;
        public final double hoodDeg;
        public final double rpm;
        public final double rangeIn;
        /** Field bearing of the shot line, degrees CCW from +X. */
        public final double bearingDeg;
        /** Modelled make rate at this range, percent. */
        public final double confidencePct;

        private Shot(boolean v, double t, double h, double r, double rng, double b, double c) {
            valid = v; turretDeg = t; hoodDeg = h; rpm = r; rangeIn = rng; bearingDeg = b;
            confidencePct = c;
        }
        static Shot invalid() { return new Shot(false, 0, 0, 0, 0, 0, 0); }
    }

    /**
     * Solve for one ball type against the currently raised CELL.
     *
     * Robot yaw matters here: it places the turret ring, which is fixed in the
     * robot frame. The bearing has no lateral correction term -- the shot line
     * runs through the ring axis.
     *
     * @param x,y          robot CENTRE, field inches
     * @param robotYawDeg  robot heading, degrees CCW from +X
     */
    public static Shot solve(Ball ball, Target tgt, double x, double y, double robotYawDeg) {
        final double psi = Math.toRadians(robotYawDeg);
        final double lat = (ball == Ball.POLLEN) ? -AXIS_LAT : AXIS_LAT;

        final double ax = x + AXIS_FWD * Math.cos(psi) - lat * Math.sin(psi);
        final double ay = y + AXIS_FWD * Math.sin(psi) + lat * Math.cos(psi);

        final double dx = goalX(tgt) - ax, dy = goalY(tgt) - ay;
        final double dAxis = Math.hypot(dx, dy);
        if (dAxis < 1e-6) return Shot.invalid();

        final double bearing = Math.atan2(dy, dx);

        // Range depends on hood through the setback, and hood comes from range.
        // One refinement pass is ample: the setback never exceeds 0.8 in.
        double range = dAxis + setbackIn(ball, 70.0);
        double hood  = hoodAt(ball, index(range));
        range = dAxis + setbackIn(ball, hood);
        if (range < TABLE_MIN_IN || range > TABLE_MAX_IN) return Shot.invalid();

        final int i = index(range);
        hood = hoodAt(ball, i);
        final double v = speedAt(ball, i);
        final double conf = (ball == Ball.POLLEN) ? POLLEN_MAKE[i] : NECTAR_MAKE[i];
        if (conf < MIN_CONFIDENCE_PCT) return Shot.invalid();

        final double rpm = 60.0 * v / (Math.PI * WHEEL_DIA_M * efficiency);
        final double bearingDeg = Math.toDegrees(bearing);
        return new Shot(true, wrap180(robotYawDeg - bearingDeg), hood, rpm, range, bearingDeg, conf);
    }

    /**
     * The two robot headings that put BOTH turret axes in line with the goal.
     * At either one the shooters need the identical turret angle (+/-90 deg),
     * so they fire together with no aim compromise at all.
     *
     * Requiring the goal to lie on the line through both axes gives
     *     D * cos(psi - beta) = AXIS_FWD  ->  psi = beta +/- acos(AXIS_FWD/D).
     *
     * @return {yawA, yawB} degrees, or null if the goal is too close.
     */
    public static double[] sidewaysYawDeg(Target tgt, double x, double y) {
        final double dx = goalX(tgt) - x, dy = goalY(tgt) - y;
        final double d = Math.hypot(dx, dy);
        if (d <= AXIS_FWD) return null;
        final double beta = Math.atan2(dy, dx);
        final double half = Math.acos(AXIS_FWD / d);
        return new double[] { Math.toDegrees(beta + half), Math.toDegrees(beta - half) };
    }

    /**
     * Below this near-slot range, POLLEN should take the near slot. NECTAR is
     * the bigger ball (3.6 in vs 2.8 in) so it has less aperture clearance,
     * and short range is where entry is tightest -- it loses more there.
     */
    public static final double NEAR_SLOT_SWAP_IN = 25.0;

    /**
     * Pick the better of the two side-on headings.
     *
     * The two shooters sit ~7.9 in apart in range, and the two headings swap
     * WHICH ball gets the near slot -- that swap happens at every position
     * tested, so the choice is always live. Measured over 494 positions:
     * POLLEN-near is worth +5.5 pts of make rate at a 15-20 in near slot and
     * +9.6 pts at 20-25 in, but at 25-30 in NECTAR-near wins by 2.6 and past
     * 30 in it is a wash. Hence the single threshold.
     *
     * NOTE: choosing instead by comparing modelled make rates directly agreed
     * with the simulation only 49% of the time -- the table is too noisy at
     * the 1-2 point level to rank two near-identical options. This coarser
     * rule is the one that holds up.
     */
    public static double bestSidewaysYawDeg(Target tgt, double x, double y, double currentYawDeg) {
        double[] opts = sidewaysYawDeg(tgt, x, y);
        if (opts == null) return Double.NaN;

        int pollenNear = -1, anyValid = -1;
        double nearRange = Double.NaN;
        for (int i = 0; i < 2; i++) {
            Shot p = solve(Ball.POLLEN, tgt, x, y, opts[i]);
            Shot n = solve(Ball.NECTAR, tgt, x, y, opts[i]);
            if (!p.valid || !n.valid) continue;
            anyValid = i;
            if (p.rangeIn < n.rangeIn) { pollenNear = i; nearRange = p.rangeIn; }
        }
        if (pollenNear < 0) {
            if (anyValid >= 0) return opts[anyValid];
            return Math.abs(wrap180(opts[0] - currentYawDeg))
                    <= Math.abs(wrap180(opts[1] - currentYawDeg)) ? opts[0] : opts[1];
        }
        return (nearRange < NEAR_SLOT_SWAP_IN) ? opts[pollenNear] : opts[1 - pollenNear];
    }

    /** Angle between the two shooters' required bearings, degrees. */
    public static double divergenceDeg(Target tgt, double x, double y, double robotYawDeg) {
        Shot p = solve(Ball.POLLEN, tgt, x, y, robotYawDeg);
        Shot n = solve(Ball.NECTAR, tgt, x, y, robotYawDeg);
        if (!p.valid || !n.valid) return Double.NaN;
        return Math.abs(wrap180(p.bearingDeg - n.bearingDeg));
    }

    /** Half of this lands on each shooter as fixed aim error. */
    public static final double DUAL_MAX_DIVERGENCE_DEG = 6.0;

    /** True when one turret angle serves both shooters well enough to fire together. */
    public static boolean inDualZone(Target tgt, double x, double y, double robotYawDeg) {
        double d = divergenceDeg(tgt, x, y, robotYawDeg);
        return !Double.isNaN(d) && d <= DUAL_MAX_DIVERGENCE_DEG;
    }

    /** Shared turret command for simultaneous fire: split the difference. */
    public static double dualTurretDeg(Target tgt, double x, double y, double robotYawDeg) {
        Shot p = solve(Ball.POLLEN, tgt, x, y, robotYawDeg);
        Shot n = solve(Ball.NECTAR, tgt, x, y, robotYawDeg);
        if (!p.valid || !n.valid) return Double.NaN;
        return wrap180(robotYawDeg - 0.5 * (p.bearingDeg + n.bearingDeg));
    }

    // ---- firing modes ----------------------------------------------------

    /**
     * How to put both balls away from one spot. The two modes want DIFFERENT
     * robot headings, so choose the mode before you turn.
     *
     *   SIMULTANEOUS -- robot side-on, both shooters share one turret angle
     *                   (divergence is exactly zero there) and fire together.
     *                   One turret move. But the two axes then lie along the
     *                   sight line, so their ranges differ by ~7.9 in and both
     *                   halves of that split must land in a >=89% band.
     *
     *   SEQUENTIAL   -- robot facing the goal, so the two axes sit symmetric
     *                   about the sight line and are at the SAME range. Much
     *                   easier to get both into a high-confidence band, and it
     *                   reaches further out. Costs a turret move between shots.
     *
     * Measured over 2263 positions at the 89% floor: SIMULTANEOUS is available
     * at 40.5%, SEQUENTIAL at 55.4%, and one or the other at 63.5%. Carrying
     * both modes is worth about half again as many shooting positions.
     */
    public enum FireMode { SIMULTANEOUS, SEQUENTIAL }

    /** Heading that points the robot straight at the goal. */
    public static double faceOnYawDeg(Target tgt, double x, double y) {
        return Math.toDegrees(Math.atan2(goalY(tgt) - y, goalX(tgt) - x));
    }

    /** The heading this mode needs. Drive to it before firing. */
    public static double yawForModeDeg(Target tgt, FireMode mode,
                                       double x, double y, double currentYawDeg) {
        return (mode == FireMode.SIMULTANEOUS)
                ? bestSidewaysYawDeg(tgt, x, y, currentYawDeg)
                : faceOnYawDeg(tgt, x, y);
    }

    /**
     * Whether a mode works from this POSITION once you have turned to its
     * heading. Current yaw only breaks ties between the two side-on options.
     */
    public static boolean modeAchievable(Target tgt, FireMode mode, double x, double y) {
        double yaw = yawForModeDeg(tgt, mode, x, y, 0.0);
        if (Double.isNaN(yaw)) return false;
        return solve(Ball.POLLEN, tgt, x, y, yaw).valid
                && solve(Ball.NECTAR, tgt, x, y, yaw).valid;
    }

    /**
     * Preferred mode from this position: SIMULTANEOUS when it works (one
     * turret move, both balls away at once), otherwise SEQUENTIAL, otherwise
     * null -- move before shooting.
     */
    public static FireMode bestMode(Target tgt, double x, double y) {
        if (modeAchievable(tgt, FireMode.SIMULTANEOUS, x, y)) return FireMode.SIMULTANEOUS;
        if (modeAchievable(tgt, FireMode.SEQUENTIAL,   x, y)) return FireMode.SEQUENTIAL;
        return null;
    }

    /** Everything needed to take the shot, in one object. */
    public static final class FirePlan {
        public final boolean valid;
        public final FireMode mode;
        /** Heading to drive to before firing. */
        public final double yawDeg;
        /** Shared turret command for SIMULTANEOUS; NaN for SEQUENTIAL. */
        public final double sharedTurretDeg;
        /** For SEQUENTIAL each Shot carries its own turretDeg -- aim twice. */
        public final Shot pollen, nectar;

        private FirePlan(boolean v, FireMode m, double yaw, double turret, Shot p, Shot n) {
            valid = v; mode = m; yawDeg = yaw; sharedTurretDeg = turret; pollen = p; nectar = n;
        }
        static FirePlan invalid() {
            return new FirePlan(false, null, Double.NaN, Double.NaN, Shot.invalid(), Shot.invalid());
        }
    }

    /**
     * Plan the shot from this position, picking the mode automatically.
     * The returned yawDeg is the heading to adopt -- it generally is NOT your
     * current heading, so turn first, then re-solve at the yaw you land on.
     */
    public static FirePlan plan(Target tgt, double x, double y, double currentYawDeg) {
        FireMode mode = bestMode(tgt, x, y);
        if (mode == null) return FirePlan.invalid();
        return planFor(tgt, mode, x, y, currentYawDeg);
    }

    /** Plan in a mode you have chosen yourself. */
    public static FirePlan planFor(Target tgt, FireMode mode,
                                   double x, double y, double currentYawDeg) {
        double yaw = yawForModeDeg(tgt, mode, x, y, currentYawDeg);
        if (Double.isNaN(yaw)) return FirePlan.invalid();
        Shot p = solve(Ball.POLLEN, tgt, x, y, yaw);
        Shot n = solve(Ball.NECTAR, tgt, x, y, yaw);
        if (!p.valid || !n.valid) return FirePlan.invalid();
        double shared = (mode == FireMode.SIMULTANEOUS)
                ? dualTurretDeg(tgt, x, y, yaw) : Double.NaN;
        return new FirePlan(true, mode, yaw, shared, p, n);
    }

    // ---- vision ----------------------------------------------------------

    /**
     * Convert a pose from the FTC SDK STANDARD field frame into Pedro.
     *
     * Straight out of Pedro's own coordinates reference:
     *     pedroX = sdkY + 72;   pedroY = 72 - sdkX;   heading -= 90 deg
     * It is a rotation as well as a shift, because Pedro's axes are turned
     * 90 degrees from the SDK's.
     *
     * Limelight botpose is in METRES unless you have configured otherwise --
     * convert units before calling this.
     *
     * @return {pedroX, pedroY, pedroHeadingDeg}
     */
    public static double[] sdkPoseToPedro(double sdkXIn, double sdkYIn, double sdkHeadingDeg) {
        return new double[] {
                sdkYIn + PEDRO_OFFSET_IN,
                PEDRO_OFFSET_IN - sdkXIn,
                wrap180(sdkHeadingDeg - 90.0)
        };
    }

    /** Inverse of {@link #sdkPoseToPedro}. */
    public static double[] pedroPoseToSdk(double pedroXIn, double pedroYIn, double pedroHeadingDeg) {
        return new double[] {
                PEDRO_OFFSET_IN - pedroYIn,
                pedroXIn - PEDRO_OFFSET_IN,
                wrap180(pedroHeadingDeg + 90.0)
        };
    }

    /**
     * Convert a pose from the DIAGRAM frame (centre origin, +X toward BLUE,
     * +Y toward the far wall) into Pedro. Those axes already match Pedro's, so
     * this is a pure translation and the heading passes through untouched.
     */
    public static double[] diagramPoseToPedro(double xIn, double yIn, double headingDeg) {
        return new double[] { xIn + PEDRO_OFFSET_IN, yIn + PEDRO_OFFSET_IN, headingDeg };
    }

    /**
     * Horizontal angle between the camera boresight and the tag cluster.
     * Boresight follows the turret, so pass the turret command you are using.
     */
    public static double tagOffsetDeg(Target tgt, double x, double y,
                                      double robotYawDeg, double turretDeg) {
        double psi = Math.toRadians(robotYawDeg);
        double cx = x - CAM_BACK_IN * Math.cos(psi);
        double cy = y - CAM_BACK_IN * Math.sin(psi);
        double bore = robotYawDeg - turretDeg;
        double toTag = Math.toDegrees(Math.atan2(tagY(tgt) - cy, tagX(tgt) - cx));
        return wrap180(toTag - bore);
    }

    /** Elevation from the camera up to the tag cluster, degrees. */
    public static double tagElevationDeg(Target tgt, double x, double y, double robotYawDeg) {
        double psi = Math.toRadians(robotYawDeg);
        double cx = x - CAM_BACK_IN * Math.cos(psi);
        double cy = y - CAM_BACK_IN * Math.sin(psi);
        double d = Math.hypot(tagX(tgt) - cx, tagY(tgt) - cy);
        return Math.toDegrees(Math.atan2(TAG_Z - CAM_HEIGHT_IN, d));
    }

    /**
     * True when the cluster is framed WHILE the turret is aimed to fire -- so
     * you can localise and shoot in the same instant, no extra rotation.
     * Holds at every position in the shooting box: worst horizontal offset is
     * 23.8 deg against a 27.25 deg limit, and the vertical never leaves the
     * 24-66 deg window. Checked anyway, because it is cheap.
     */
    public static boolean tagInView(Target tgt, double x, double y,
                                    double robotYawDeg, double turretDeg) {
        if (Math.abs(tagOffsetDeg(tgt, x, y, robotYawDeg, turretDeg)) > CAM_HFOV_HALF) return false;
        double el = tagElevationDeg(tgt, x, y, robotYawDeg);
        return Math.abs(el - CAM_PITCH_DEG) <= CAM_VFOV_HALF;
    }

    /**
     * Modelled make rate at a given range, for telemetry. Good for "am I in a
     * sensible range band"; NOT reliable for ranking two options within a
     * couple of points of each other.
     */
    public static double expectedMakePct(Ball ball, double rangeIn) {
        int i = index(rangeIn);
        return (ball == Ball.POLLEN) ? POLLEN_MAKE[i] : NECTAR_MAKE[i];
    }

    private static double hoodAt(Ball b, int i)  { return (b == Ball.POLLEN) ? POLLEN_HOOD[i] : NECTAR_HOOD[i]; }
    private static double speedAt(Ball b, int i) { return (b == Ball.POLLEN) ? POLLEN_V[i]    : NECTAR_V[i];    }

    private static int index(double rangeIn) {
        int i = (int) Math.round(rangeIn - TABLE_MIN_IN);
        if (i < 0) i = 0;
        if (i >= POLLEN_HOOD.length) i = POLLEN_HOOD.length - 1;
        return i;
    }

    public static double wrap180(double deg) {
        double a = (deg + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }
}

/*
 * ---------------------------------------------------------------------------
 * USAGE
 *
 *   Target tgt = BioBuzzShooterRed.Target.FAR_SIDE;     // whichever CELL is up
 *   // or resolve it from a detection:
 *   // Target tgt = BioBuzzShooterRed.targetForTagId(det.id);
 *
 *   Target tgt = BioBuzzShooterRed.Target.FAR_SIDE;   // whichever CELL is up
 *
 *   // x, y, yawNow come straight from Pedro / the Pinpoint -- no conversion.
 *   Pose pose = follower.getPose();
 *   double x = pose.getX(), y = pose.getY(), yawNow = Math.toDegrees(pose.getHeading());
 *
 *   FirePlan plan = BioBuzzShooterRed.plan(tgt, x, y, yawNow);
 *   if (!plan.valid) { driveSomewhereBetter(); return; }
 *
 *   driveToHeading(plan.yawDeg);        // mode-specific: side-on or face-on
 *   // re-solve once you are actually there
 *   plan = BioBuzzShooterRed.planFor(tgt, plan.mode, x, y, yawNow);
 *
 *   if (plan.mode == FireMode.SIMULTANEOUS) {
 *       aimTurret(plan.sharedTurretDeg);                       // ~ +/-90 deg
 *       if (tagInView(tgt, x, y, yawNow, plan.sharedTurretDeg)) {
 *           // Limelight reports SDK-standard -- rotate AND shift into Pedro.
 *           // (If yours reports centre-origin diagram coords, use
 *           //  diagramPoseToPedro instead -- it is a plain +72 shift.)
 *           double[] p = sdkPoseToPedro(tagSdkXIn, tagSdkYIn, tagSdkHeadingDeg);
 *           follower.setPose(new Pose(p[0], p[1], Math.toRadians(p[2])));
 *       }
 *       setPollen(plan.pollen.hoodDeg, plan.pollen.rpm);
 *       setNectar(plan.nectar.hoodDeg, plan.nectar.rpm);
 *       fireBoth();
 *   } else {
 *       aimTurret(plan.pollen.turretDeg);
 *       if (tagInView(tgt, x, y, yawNow, plan.pollen.turretDeg)) updatePoseFromTag();
 *       setPollen(plan.pollen.hoodDeg, plan.pollen.rpm);  firePollen();
 *       aimTurret(plan.nectar.turretDeg);
 *       setNectar(plan.nectar.hoodDeg, plan.nectar.rpm);  fireNectar();
 *   }
 *
 * SIMULTANEOUS puts the two shooters ~7.9 in apart in range, so they get
 * different hoods and RPMs -- that costs nothing, the hoods and motors are
 * independent. SEQUENTIAL puts them at the SAME range, which is why it covers
 * more of the field at a 89% floor. Only the turret is ever shared.
 *
 * ---------------------------------------------------------------------------
 * CALIBRATED vs NOT
 *
 * Geometry: exact. Exit heights, ball diameters, flywheel diameter, orbit
 * radii and hood angles reproduce the CAD to 0.003 in / 0.001 deg.
 *
 * Hood and speed: from a drag model whose Cd = 0.55 is an ESTIMATE (0.4-0.8
 * plausible). That spread moves required speed by more than the +/-5% speed
 * tolerance. Fit Cd from video.
 *
 * efficiency: a guess. Scales every RPM linearly. Measure it.
 *
 * NECTAR mass was assumed, never confirmed from the game manual.
 *
 * Tag IDs are as supplied (30-33 red far side, 34-37 red audience side). The
 * far/audience mapping assumes +Y is the far side wall, per the simulator's
 * axis note. If that is backwards, swap the two ID rows in TAG_ID -- nothing
 * else changes.
 * ---------------------------------------------------------------------------
 */