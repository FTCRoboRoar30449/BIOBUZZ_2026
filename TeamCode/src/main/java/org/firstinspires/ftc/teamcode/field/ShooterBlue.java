package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.robot.HoodMap;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;

public final class ShooterBlue {

    private ShooterBlue() {}

    public enum Ball { POLLEN, NECTAR }

    public enum Target {
        SCORING_SIDE,
        AUDIENCE_SIDE
    }

    // ---- coordinate frames ------------------------------------------------
    public static final double PEDRO_OFFSET_IN = 72.0;

    public static double diagramToPedro(double diagramIn) { return diagramIn + PEDRO_OFFSET_IN; }
    public static double pedroToDiagram(double pedroIn)   { return pedroIn - PEDRO_OFFSET_IN; }

    // ---- targets, PEDRO inches -------------------------------------------
    private static final double[] GOAL_X = { 84.75, 84.75 };
    // See ShooterRed: the four CELLS form a 25.4 in square centred on 72,72.
    private static final double[] GOAL_Y = { 84.7, 59.3 };
    // cluster: diagram (+12.75, +/-12.89) -> pedro (84.75, 84.89 / 59.11)
    private static final double[] TAG_X  = { 84.75, 84.75 };
    private static final double[] TAG_Y  = { 84.89, 59.11 };
    private static final int[][]  TAG_ID = { {42, 43, 44, 45}, {38, 39, 40, 41} };

    public static final double GOAL_Z = 59.55;
    public static final double TAG_Z  = 49.662;

    public static double goalX(Target t) { return GOAL_X[t.ordinal()]; }
    public static double goalY(Target t) { return GOAL_Y[t.ordinal()]; }
    public static double tagX(Target t)  { return TAG_X[t.ordinal()]; }
    public static double tagY(Target t)  { return TAG_Y[t.ordinal()]; }
    public static int[] tagIds(Target t) { return TAG_ID[t.ordinal()].clone(); }

    public static Target targetForTagId(int id) {
        if (id >= 42 && id <= 45) return Target.SCORING_SIDE;
        if (id >= 38 && id <= 41) return Target.AUDIENCE_SIDE;
        return null;
    }

    // ---- turret rings, ROBOT frame, inches ------------------------------
    public static final double AXIS_FWD = 105.0 / 25.4;   // 4.1339
    public static final double AXIS_LAT = 100.0 / 25.4;   // 3.9370

    // ---- flywheel --------------------------------------------------------

    public static final double TABLE_MIN_IN = 15.0;
    public static final double TABLE_MAX_IN = 39.0;

    // ---- Limelight 3A, turret-mounted ------------------------------------

    // ---- tables, one entry per inch of range from 15 in -------------------
    // RESHAPED 22 Nov from the measured CSV. The old table was a flat 80/75/70
    // and needed a hood trim that swung from -2 at 24 in to -8 at 36 in; that
    // slope is now in the table itself, so one trim of zero works everywhere.
    // Fitted trim(range) = 13.484 - 0.5795*range over the four measured
    // stands, clamped to the measured span so the ends are not extrapolated.
    private static final double[] POLLEN_HOOD = {
            80, 80, 80, 80, 80, 80, 80, 80, 80, 75, 74, 73, 73,
            72, 72, 66, 66, 65, 64, 64, 63, 63, 62, 61, 61 };
    // Rescaled so SHOOTER_EFF_POLLEN = 0.465 reproduces the rpm actually
    // fired at every measured range, within 0.4%. Before this the efficiency
    // had to be walked from 0.470 at 24 in to 0.445 at 36 in, which is the
    // table's shape being wrong, not the constant.
    private static final double[] POLLEN_V = {
            4.712, 4.708, 4.752, 4.796, 4.865, 4.935, 5.005, 5.124, 5.196,
            4.947, 4.993, 5.040, 5.088, 5.135, 5.184, 5.182, 5.205, 5.280,
            5.304, 5.328, 5.379, 5.429, 5.455, 5.533, 5.559 };
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

    public static double setbackIn(Ball ball, double hoodDeg) {
        double a55 = (ball == Ball.NECTAR) ?  6.208 :  5.323;
        double a80 = (ball == Ball.NECTAR) ? 19.579 : 17.011;
        return (a55 + (hoodDeg - 55.0) * (a80 - a55) / 25.0) / 25.4;
    }

    public static double MIN_CONFIDENCE_PCT = 89.0;

    public static final class Shot {
        public final boolean valid;
        public final double turretDeg;
        public final double hoodDeg;
        public final double rpm;
        public final double rangeIn;
        public final double bearingDeg;
        public final double confidencePct;

        private Shot(boolean v, double t, double h, double r, double rng, double b, double c) {
            valid = v; turretDeg = t; hoodDeg = h; rpm = r; rangeIn = rng; bearingDeg = b;
            confidencePct = c;
        }
        static Shot invalid() { return new Shot(false, 0, 0, 0, 0, 0, 0); }
    }

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
        double range = dAxis + setbackIn(ball, 70.0);
        double hood  = hoodAt(ball, index(range));
        range = dAxis + setbackIn(ball, hood);
        if (range < TABLE_MIN_IN || range > TABLE_MAX_IN) return Shot.invalid();

        final int i = index(range);
        hood = HoodMap.clampHood(hoodAt(ball, i)
                + (ball == Ball.NECTAR ? ShooterRed.HOOD_TRIM_NECTAR
                                       : ShooterRed.HOOD_TRIM_POLLEN));
        final double v = speedAt(ball, i);
        final double conf = (ball == Ball.POLLEN) ? POLLEN_MAKE[i] : NECTAR_MAKE[i];
        if (conf < MIN_CONFIDENCE_PCT) return Shot.invalid();

        final double eff = RobotHardware.shooterEfficiency(ball == Ball.NECTAR);
        final double rpm = 60.0 * v / (Math.PI * RobotHardware.SHOOTER_WHEEL_DIA_M * eff);
        final double bearingDeg = Math.toDegrees(bearing);
        return new Shot(true, wrap180(ShooterRed.AIM_SIGN * (robotYawDeg - bearingDeg)),
                hood, rpm, range, bearingDeg, conf);
    }

    public static double[] sidewaysYawDeg(Target tgt, double x, double y) {
        final double dx = goalX(tgt) - x, dy = goalY(tgt) - y;
        final double d = Math.hypot(dx, dy);
        if (d <= AXIS_FWD) return null;
        final double beta = Math.atan2(dy, dx);
        final double half = Math.acos(AXIS_FWD / d);
        return new double[] { Math.toDegrees(beta + half), Math.toDegrees(beta - half) };
    }

    public static final double NEAR_SLOT_SWAP_IN = 25.0;

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

    public static double divergenceDeg(Target tgt, double x, double y, double robotYawDeg) {
        Shot p = solve(Ball.POLLEN, tgt, x, y, robotYawDeg);
        Shot n = solve(Ball.NECTAR, tgt, x, y, robotYawDeg);
        if (!p.valid || !n.valid) return Double.NaN;
        return Math.abs(wrap180(p.bearingDeg - n.bearingDeg));
    }

    public static final double DUAL_MAX_DIVERGENCE_DEG = 6.0;

    public static boolean inDualZone(Target tgt, double x, double y, double robotYawDeg) {
        double d = divergenceDeg(tgt, x, y, robotYawDeg);
        return !Double.isNaN(d) && d <= DUAL_MAX_DIVERGENCE_DEG;
    }

    public static double dualTurretDeg(Target tgt, double x, double y, double robotYawDeg) {
        Shot p = solve(Ball.POLLEN, tgt, x, y, robotYawDeg);
        Shot n = solve(Ball.NECTAR, tgt, x, y, robotYawDeg);
        if (!p.valid || !n.valid) return Double.NaN;
        return wrap180(ShooterRed.AIM_SIGN
                * (robotYawDeg - 0.5 * (p.bearingDeg + n.bearingDeg)));
    }

    // ---- firing modes ----------------------------------------------------

    public enum FireMode { SIMULTANEOUS, SEQUENTIAL }

    public static double faceOnYawDeg(Target tgt, double x, double y) {
        return Math.toDegrees(Math.atan2(goalY(tgt) - y, goalX(tgt) - x));
    }

    public static double yawForModeDeg(Target tgt, FireMode mode,
                                       double x, double y, double currentYawDeg) {
        return (mode == FireMode.SIMULTANEOUS)
                ? bestSidewaysYawDeg(tgt, x, y, currentYawDeg)
                : faceOnYawDeg(tgt, x, y);
    }

    public static boolean modeAchievable(Target tgt, FireMode mode, double x, double y) {
        double yaw = yawForModeDeg(tgt, mode, x, y, 0.0);
        if (Double.isNaN(yaw)) return false;
        return solve(Ball.POLLEN, tgt, x, y, yaw).valid
                && solve(Ball.NECTAR, tgt, x, y, yaw).valid;
    }

    public static FireMode bestMode(Target tgt, double x, double y) {
        if (modeAchievable(tgt, FireMode.SIMULTANEOUS, x, y)) return FireMode.SIMULTANEOUS;
        if (modeAchievable(tgt, FireMode.SEQUENTIAL,   x, y)) return FireMode.SEQUENTIAL;
        return null;
    }

    public static final class FirePlan {
        public final boolean valid;
        public final FireMode mode;
        public final double yawDeg;
        public final double sharedTurretDeg;
        public final Shot pollen, nectar;

        private FirePlan(boolean v, FireMode m, double yaw, double turret, Shot p, Shot n) {
            valid = v; mode = m; yawDeg = yaw; sharedTurretDeg = turret; pollen = p; nectar = n;
        }
        static FirePlan invalid() {
            return new FirePlan(false, null, Double.NaN, Double.NaN, Shot.invalid(), Shot.invalid());
        }
    }

    public static FirePlan plan(Target tgt, double x, double y, double currentYawDeg) {
        FireMode mode = bestMode(tgt, x, y);
        if (mode == null) return FirePlan.invalid();
        return planFor(tgt, mode, x, y, currentYawDeg);
    }

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

    public static double[] sdkPoseToPedro(double sdkXIn, double sdkYIn, double sdkHeadingDeg) {
        return new double[] {
                sdkYIn + PEDRO_OFFSET_IN,
                PEDRO_OFFSET_IN - sdkXIn,
                wrap180(sdkHeadingDeg - 90.0)
        };
    }

    public static double[] pedroPoseToSdk(double pedroXIn, double pedroYIn, double pedroHeadingDeg) {
        return new double[] {
                PEDRO_OFFSET_IN - pedroYIn,
                pedroXIn - PEDRO_OFFSET_IN,
                wrap180(pedroHeadingDeg + 90.0)
        };
    }

    public static double[] diagramPoseToPedro(double xIn, double yIn, double headingDeg) {
        return new double[] { xIn + PEDRO_OFFSET_IN, yIn + PEDRO_OFFSET_IN, headingDeg };
    }

    public static double tagOffsetDeg(Target tgt, double x, double y,
                                      double robotYawDeg, double turretDeg) {
        double psi = Math.toRadians(robotYawDeg);
        double cx = x - RobotHardware.LIMELIGHT_BACK_IN * Math.cos(psi);
        double cy = y - RobotHardware.LIMELIGHT_BACK_IN * Math.sin(psi);
        double bore = robotYawDeg - ShooterRed.AIM_SIGN * turretDeg;
        double toTag = Math.toDegrees(Math.atan2(tagY(tgt) - cy, tagX(tgt) - cx));
        return wrap180(toTag - bore);
    }

    public static double tagElevationDeg(Target tgt, double x, double y, double robotYawDeg) {
        double psi = Math.toRadians(robotYawDeg);
        double cx = x - RobotHardware.LIMELIGHT_BACK_IN * Math.cos(psi);
        double cy = y - RobotHardware.LIMELIGHT_BACK_IN * Math.sin(psi);
        double d = Math.hypot(tagX(tgt) - cx, tagY(tgt) - cy);
        return Math.toDegrees(Math.atan2(TAG_Z - RobotHardware.LIMELIGHT_HEIGHT_IN, d));
    }

    public static boolean tagInView(Target tgt, double x, double y,
                                    double robotYawDeg, double turretDeg) {
        if (Math.abs(tagOffsetDeg(tgt, x, y, robotYawDeg, turretDeg)) > RobotHardware.LIMELIGHT_HFOV_HALF) return false;
        double el = tagElevationDeg(tgt, x, y, robotYawDeg);
        return Math.abs(el - RobotHardware.LIMELIGHT_PITCH_DEG) <= RobotHardware.LIMELIGHT_VFOV_HALF;
    }

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
