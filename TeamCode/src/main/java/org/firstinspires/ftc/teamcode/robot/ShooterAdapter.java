package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.field.ShooterBlue;
import org.firstinspires.ftc.teamcode.field.ShooterRed;

/**
 * Alliance-neutral front end for ShooterRed / ShooterBlue.
 *
 * Those two are separate final classes with their own nested enums, so they
 * cannot be used interchangeably as types even though their APIs are identical.
 * This wraps whichever one applies and re-exposes the results in neutral types,
 * so MechController is written once instead of twice.
 *
 * All coordinates are PEDRO inches -- feed follower.getPose() straight in.
 *
 * Nothing here second-guesses the shooter classes; it only forwards and
 * translates. If a number looks wrong, fix it there, not here.
 */
public final class ShooterAdapter {

    public enum Alliance { RED, BLUE }
    public enum Ball { POLLEN, NECTAR }
    /** Which CELL of our alliance is currently raised. */
    public enum Target { SCORING_SIDE, AUDIENCE_SIDE }
    /**
     * SIMULTANEOUS means one turret angle serves both shooters at once. With
     * both conveyor beds up, a pollen and a nectar leave on the SAME feed
     * stroke -- one aim, one spin-up, one conveyor move, two balls scored.
     *
     * SEQUENTIAL means the turret has to move between the two shots, so each
     * ball costs its own aim, spin-up and feed.
     *
     * That is why carrying both modes matters: SIMULTANEOUS roughly halves the
     * cycle time when you hold one of each ball, and SEQUENTIAL reaches the
     * positions where the 7.9 in range split would push one shooter out of a
     * high-confidence band.
     */
    public enum FireMode { SIMULTANEOUS, SEQUENTIAL }

    public static final class Shot {
        public final boolean valid;
        public final double turretDeg, hoodDeg, rpm, rangeIn, bearingDeg, confidencePct;
        Shot(boolean v, double t, double h, double r, double rng, double b, double c) {
            valid = v; turretDeg = t; hoodDeg = h; rpm = r; rangeIn = rng;
            bearingDeg = b; confidencePct = c;
        }
        static Shot invalid() { return new Shot(false, 0, 0, 0, 0, 0, 0); }
    }

    public static final class FirePlan {
        public final boolean valid;
        public final FireMode mode;
        public final double yawDeg;            // heading to adopt before firing
        public final double sharedTurretDeg;   // SIMULTANEOUS only; NaN otherwise
        public final Shot pollen, nectar;
        FirePlan(boolean v, FireMode m, double yaw, double turret, Shot p, Shot n) {
            valid = v; mode = m; yawDeg = yaw; sharedTurretDeg = turret; pollen = p; nectar = n;
        }
        static FirePlan invalid() {
            return new FirePlan(false, null, Double.NaN, Double.NaN,
                                Shot.invalid(), Shot.invalid());
        }
    }

    // ---- auto-aim zones, PEDRO inches ------------------------------------
    // Supplied by the team as ZONE DEFINITIONS ONLY -- the aim points come
    // from the shooter classes' aperture constants, which sit 5.25 in outboard
    // of the tag clusters these numbers were taken from.
    public static final class Zone {
        public final double x0, y0, x1, y1;
        public final double targetX, targetY;
        Zone(double x0, double y0, double x1, double y1, double tx, double ty) {
            this.x0 = x0; this.y0 = y0; this.x1 = x1; this.y1 = y1;
            this.targetX = tx; this.targetY = ty;
        }
        public boolean contains(double x, double y) {
            return x >= x0 && x <= x1 && y >= y0 && y <= y1;
        }
    }

    private final Alliance alliance;

    /** Flower zones, in the order {audience, scoring}. */
    private final Zone[] flowerZones;
    /** Hive zones, same order as Target: {SCORING_SIDE, AUDIENCE_SIDE}. */
    private final Zone[] hiveZones;

    public ShooterAdapter(Alliance alliance) {
        this.alliance = alliance;

        /*
         * ALL FOUR FLOWERS, BOTH ALLIANCES.
         *
         * Flowers are not alliance property -- either team may collect from
         * any of the four during either match. The zones used to be split two
         * and two, which silently made half the field unusable: a red robot
         * at a "blue" flower got null from flowerTargetForPosition(), so
         * flowerRangeIn() returned NaN, the scoop fell back to its
         * unknown-range behaviour and the flower shot refused with "not in a
         * flower zone".
         *
         * Same list for both alliances now. HIVE zones stay split, because
         * those really are alliance-specific.
         */
        flowerZones = new Zone[] {
            new Zone(0,   0,  48,  96,    4,  48),   // red side, audience
            new Zone(0,  96,  96, 144,   48, 140),   // red side, scoring
            new Zone(48,  0, 144,  48,   96,   4),   // blue side, audience
            new Zone(96, 48, 144, 144,  140,  96)    // blue side, scoring
        };

        if (alliance == Alliance.RED) {
            hiveZones = new Zone[] {
                new Zone(24, 108, 96, 144, ShooterRed.goalX(ShooterRed.Target.SCORING_SIDE),
                                            ShooterRed.goalY(ShooterRed.Target.SCORING_SIDE)),
                new Zone(24,   0, 96,  36, ShooterRed.goalX(ShooterRed.Target.AUDIENCE_SIDE),
                                            ShooterRed.goalY(ShooterRed.Target.AUDIENCE_SIDE))
            };
        } else {
            hiveZones = new Zone[] {
                new Zone(48, 108, 120, 144, ShooterBlue.goalX(ShooterBlue.Target.SCORING_SIDE),
                                             ShooterBlue.goalY(ShooterBlue.Target.SCORING_SIDE)),
                new Zone(48,   0, 120,  36, ShooterBlue.goalX(ShooterBlue.Target.AUDIENCE_SIDE),
                                             ShooterBlue.goalY(ShooterBlue.Target.AUDIENCE_SIDE))
            };
        }
    }

    public Alliance getAlliance() { return alliance; }

    /** 2 for red nectar, 3 for blue nectar -- matches the colour sensor IDs. */
    public int allianceNectarColorId() { return alliance == Alliance.RED ? 2 : 3; }

    // ---- target identification -------------------------------------------

    /**
     * Which of OUR cells a detected tag belongs to, or null if the tag is not
     * ours. Only the RAISED cell's cluster faces the tiles, so seeing a tag at
     * all is what tells you which cell is up.
     */
    public Target targetForTagId(int id) {
        if (alliance == Alliance.RED) {
            ShooterRed.Target t = ShooterRed.targetForTagId(id);
            return t == null ? null : (t == ShooterRed.Target.SCORING_SIDE
                    ? Target.SCORING_SIDE : Target.AUDIENCE_SIDE);
        }
        ShooterBlue.Target t = ShooterBlue.targetForTagId(id);
        return t == null ? null : (t == ShooterBlue.Target.SCORING_SIDE
                ? Target.SCORING_SIDE : Target.AUDIENCE_SIDE);
    }

    /** Fallback when no tag is visible: pick the hive target by position. */
    public Target hiveTargetForPosition(double x, double y) {
        if (hiveZones[0].contains(x, y)) return Target.SCORING_SIDE;
        if (hiveZones[1].contains(x, y)) return Target.AUDIENCE_SIDE;
        return null;
    }

    /**
     * Nearest flower, from ANYWHERE on the field.
     *
     * flowerTargetForPosition() only answers once you are already standing in
     * a flower zone, which is no use for "drive me to the nearest flower".
     * This scans every zone and returns the closest aim point regardless.
     *
     * The aim point IS the flower's physical location, so this is also what
     * MechController measures scoop clearance against.
     *
     * @return {x, y} PEDRO inches. Never null -- there is always a nearest.
     */
    public double[] nearestFlowerTarget(double x, double y) {
        double[] best = null;
        double bestD2 = Double.MAX_VALUE;
        for (Zone z : flowerZones) {
            double dx = z.targetX - x, dy = z.targetY - y;
            double d2 = dx * dx + dy * dy;
            if (d2 < bestD2) { bestD2 = d2; best = new double[] { z.targetX, z.targetY }; }
        }
        return best;
    }

    /**
     * Centre of the shooting zone for a hive target -- a known-good place to
     * stand, for a follower to drive to.
     *
     * This is deliberately NOT an optimised firing position. Searching the
     * field for the best spot is a different problem, and the zone centre is
     * inside a region already verified to produce valid shots.
     *
     * @return {x, y} PEDRO inches.
     */
    public double[] hiveZoneCentre(Target t) {
        Zone z = hiveZones[t == Target.SCORING_SIDE ? 0 : 1];
        return new double[] { (z.x0 + z.x1) / 2.0, (z.y0 + z.y1) / 2.0 };
    }

    /** True when this position is inside the shooting zone for that target. */
    public boolean inHiveZone(Target t, double x, double y) {
        return hiveZones[t == Target.SCORING_SIDE ? 0 : 1].contains(x, y);
    }

    /** True when this position is inside any flower zone. */
    public boolean inFlowerZone(double x, double y) {
        for (Zone z : flowerZones) if (z.contains(x, y)) return true;
        return false;
    }

    /** Flower aim point for this position, or null if outside both zones. */
    public double[] flowerTargetForPosition(double x, double y) {
        /*
         * NEAREST containing zone, not the first.
         *
         * With two zones per alliance the first match was unambiguous. With
         * all four live they share edges, so a robot standing on a boundary
         * could be handed the flower further away -- and MechController
         * measures scoop clearance against whatever comes back. Picking the
         * nearest makes the answer independent of array order.
         */
        double[] best = null;
        double bestD2 = Double.MAX_VALUE;
        for (Zone z : flowerZones) {
            if (!z.contains(x, y)) continue;
            double dx = z.targetX - x, dy = z.targetY - y;
            double d2 = dx * dx + dy * dy;
            if (d2 < bestD2) { bestD2 = d2; best = new double[] { z.targetX, z.targetY }; }
        }
        return best;
    }

    public double goalX(Target t) {
        return alliance == Alliance.RED ? ShooterRed.goalX(red(t)) : ShooterBlue.goalX(blue(t));
    }
    public double goalY(Target t) {
        return alliance == Alliance.RED ? ShooterRed.goalY(red(t)) : ShooterBlue.goalY(blue(t));
    }

    // ---- planning ---------------------------------------------------------

    public FirePlan plan(Target t, double x, double y, double yawDeg) {
        if (alliance == Alliance.RED) {
            return convert(ShooterRed.plan(red(t), x, y, yawDeg));
        }
        return convert(ShooterBlue.plan(blue(t), x, y, yawDeg));
    }

    /**
     * Plan a shot from the heading the robot IS at, not one it should turn to.
     *
     * plan() and planFor() both answer a different question: they pick a mode,
     * work out the heading that mode wants, and solve THERE. Their own docs say
     * "the returned yawDeg is the heading to adopt -- it generally is NOT your
     * current heading, so turn first, then re-solve". A turretDeg from that
     * plan is only correct once the robot has actually turned.
     *
     * MechController never drives -- in TeleOp the driver owns the heading, and
     * in autonomous Pedro has already finished its move. Firing plan()'s
     * turretDeg from an unturned robot mis-aims by the whole difference between
     * the two headings, which is tens of degrees at many positions.
     *
     * So this solves both balls at the CURRENT yaw and picks the mode from the
     * divergence that actually applies there. The returned yawDeg is the yaw
     * passed in, because no turn is required.
     *
     * Use plan() when something is going to drive to the heading it returns.
     * Use this when you are shooting from where you stand.
     */
    public FirePlan planHere(Target t, double x, double y, double yawDeg) {
        Shot p = solve(Ball.POLLEN, t, x, y, yawDeg);
        Shot n = solve(Ball.NECTAR, t, x, y, yawDeg);
        if (!p.valid || !n.valid) return FirePlan.invalid();

        boolean dual = alliance == Alliance.RED
                ? ShooterRed.inDualZone(red(t), x, y, yawDeg)
                : ShooterBlue.inDualZone(blue(t), x, y, yawDeg);

        double shared = Double.NaN;
        if (dual) {
            shared = alliance == Alliance.RED
                    ? ShooterRed.dualTurretDeg(red(t), x, y, yawDeg)
                    : ShooterBlue.dualTurretDeg(blue(t), x, y, yawDeg);
            if (Double.isNaN(shared)) dual = false;
        }
        return new FirePlan(true, dual ? FireMode.SIMULTANEOUS : FireMode.SEQUENTIAL,
                            yawDeg, shared, p, n);
    }

    /** Angle between the two shooters' bearings at this heading, degrees. */
    public double divergenceDeg(Target t, double x, double y, double yawDeg) {
        return alliance == Alliance.RED
                ? ShooterRed.divergenceDeg(red(t), x, y, yawDeg)
                : ShooterBlue.divergenceDeg(blue(t), x, y, yawDeg);
    }

    public FirePlan planFor(Target t, FireMode m, double x, double y, double yawDeg) {
        if (alliance == Alliance.RED) {
            return convert(ShooterRed.planFor(red(t), redMode(m), x, y, yawDeg));
        }
        return convert(ShooterBlue.planFor(blue(t), blueMode(m), x, y, yawDeg));
    }

    public Shot solve(Ball b, Target t, double x, double y, double yawDeg) {
        if (alliance == Alliance.RED) {
            return convert(ShooterRed.solve(redBall(b), red(t), x, y, yawDeg));
        }
        return convert(ShooterBlue.solve(blueBall(b), blue(t), x, y, yawDeg));
    }

    /** Heading to drive to before firing in this mode. */
    public double yawForModeDeg(Target t, FireMode m, double x, double y, double currentYawDeg) {
        if (alliance == Alliance.RED) {
            return ShooterRed.yawForModeDeg(red(t), redMode(m), x, y, currentYawDeg);
        }
        return ShooterBlue.yawForModeDeg(blue(t), blueMode(m), x, y, currentYawDeg);
    }

    public boolean tagInView(Target t, double x, double y, double yawDeg, double turretDeg) {
        if (alliance == Alliance.RED) {
            return ShooterRed.tagInView(red(t), x, y, yawDeg, turretDeg);
        }
        return ShooterBlue.tagInView(blue(t), x, y, yawDeg, turretDeg);
    }

    /** Turret angle that points the camera/shooter at an arbitrary field point. */
    public static double turretDegToPoint(double x, double y, double yawDeg,
                                          double targetX, double targetY) {
        double bearing = Math.toDegrees(Math.atan2(targetY - y, targetX - x));
        return wrap180(yawDeg - bearing);
    }

    public static double wrap180(double deg) {
        double a = (deg + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }

    // ---- conversions ------------------------------------------------------

    private static ShooterRed.Target  red(Target t)  {
        return t == Target.SCORING_SIDE ? ShooterRed.Target.SCORING_SIDE : ShooterRed.Target.AUDIENCE_SIDE;
    }
    private static ShooterBlue.Target blue(Target t) {
        return t == Target.SCORING_SIDE ? ShooterBlue.Target.SCORING_SIDE : ShooterBlue.Target.AUDIENCE_SIDE;
    }
    private static ShooterRed.Ball  redBall(Ball b)  {
        return b == Ball.POLLEN ? ShooterRed.Ball.POLLEN : ShooterRed.Ball.NECTAR;
    }
    private static ShooterBlue.Ball blueBall(Ball b) {
        return b == Ball.POLLEN ? ShooterBlue.Ball.POLLEN : ShooterBlue.Ball.NECTAR;
    }
    private static ShooterRed.FireMode  redMode(FireMode m)  {
        return m == FireMode.SIMULTANEOUS
                ? ShooterRed.FireMode.SIMULTANEOUS : ShooterRed.FireMode.SEQUENTIAL;
    }
    private static ShooterBlue.FireMode blueMode(FireMode m) {
        return m == FireMode.SIMULTANEOUS
                ? ShooterBlue.FireMode.SIMULTANEOUS : ShooterBlue.FireMode.SEQUENTIAL;
    }

    private static Shot convert(ShooterRed.Shot s) {
        return s.valid ? new Shot(true, s.turretDeg, s.hoodDeg, s.rpm, s.rangeIn,
                                  s.bearingDeg, s.confidencePct) : Shot.invalid();
    }
    private static Shot convert(ShooterBlue.Shot s) {
        return s.valid ? new Shot(true, s.turretDeg, s.hoodDeg, s.rpm, s.rangeIn,
                                  s.bearingDeg, s.confidencePct) : Shot.invalid();
    }
    private static FirePlan convert(ShooterRed.FirePlan p) {
        if (!p.valid) return FirePlan.invalid();
        return new FirePlan(true,
                p.mode == ShooterRed.FireMode.SIMULTANEOUS
                        ? FireMode.SIMULTANEOUS : FireMode.SEQUENTIAL,
                p.yawDeg, p.sharedTurretDeg, convert(p.pollen), convert(p.nectar));
    }
    private static FirePlan convert(ShooterBlue.FirePlan p) {
        if (!p.valid) return FirePlan.invalid();
        return new FirePlan(true,
                p.mode == ShooterBlue.FireMode.SIMULTANEOUS
                        ? FireMode.SIMULTANEOUS : FireMode.SEQUENTIAL,
                p.yawDeg, p.sharedTurretDeg, convert(p.pollen), convert(p.nectar));
    }
}
