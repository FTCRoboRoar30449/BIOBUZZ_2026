package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.field.ShooterBlue;
import org.firstinspires.ftc.teamcode.field.ShooterRed;

public final class ShooterAdapter {

    public enum Alliance { RED, BLUE }
    public enum Ball { POLLEN, NECTAR }
    public enum Target { SCORING_SIDE, AUDIENCE_SIDE }
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

    private final Zone[] flowerZones;
    private final Zone[] hiveZones;

    public ShooterAdapter(Alliance alliance) {
        this.alliance = alliance;

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

    public int allianceNectarColorId() { return alliance == Alliance.RED ? 2 : 3; }

    // ---- target identification -------------------------------------------

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

    public Target hiveTargetForPosition(double x, double y) {
        if (hiveZones[0].contains(x, y)) return Target.SCORING_SIDE;
        if (hiveZones[1].contains(x, y)) return Target.AUDIENCE_SIDE;
        return null;
    }

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

    public double[] hiveZoneCentre(Target t) {
        Zone z = hiveZones[t == Target.SCORING_SIDE ? 0 : 1];
        return new double[] { (z.x0 + z.x1) / 2.0, (z.y0 + z.y1) / 2.0 };
    }

    public boolean inHiveZone(Target t, double x, double y) {
        return hiveZones[t == Target.SCORING_SIDE ? 0 : 1].contains(x, y);
    }

    public boolean inFlowerZone(double x, double y) {
        for (Zone z : flowerZones) if (z.contains(x, y)) return true;
        return false;
    }

    public double[] flowerTargetForPosition(double x, double y) {
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
