package org.firstinspires.ftc.teamcode.robot;

// Hood angle -> servo position, for both hoods.
//
// THE ANGLE HERE IS THE BALL'S EXIT ANGLE, not the angle of the hood plate.
// That is what the shot table is indexed by -- ShooterRed/Blue's POLLEN_HOOD
// and NECTAR_HOOD rows are launch angles per inch of range, and setbackIn()
// interpolates the exit geometry across the same 55..80. Measure the ball's
// line out of the shooter, not the plate.
//
// Calibrated 14 Nov from ONE measured angle per hood plus the gear ratios.
// That is the right shape for this: the slope comes from gearing, which is
// known exactly, so only an offset needs measuring -- and measuring one angle
// accurately on a built robot is far easier than measuring two.
public final class HoodMap {

    private HoodMap() {}

    public static final double SERVO_TRAVEL_DEG = 1800.0;

    public static final double POLLEN_GEAR_RATIO = 158.0 / 20.0;   // 7.900
    public static final double NECTAR_GEAR_RATIO = 176.0 / 19.0;   // 9.2632

    // Slope, from the gearing. Confirmed correct 14 Nov.
    //
    // These were briefly overridden with 0.00560 / 0.00760, taken as
    // span-over-25 from the mechanical stops on the belief that the stops
    // were 55 and 80 deg apart. They are not -- the tops are about 45 and 43
    // deg -- so that override was 28% and 48% HIGH and is reverted.
    public static final double POLLEN_POS_PER_DEG = POLLEN_GEAR_RATIO / SERVO_TRAVEL_DEG; // 0.0043889
    public static final double NECTAR_POS_PER_DEG = NECTAR_GEAR_RATIO / SERVO_TRAVEL_DEG; // 0.0051462

    public static final double HOOD_MIN_DEG = 55.0;
    public static final double HOOD_MAX_DEG = 80.0;

    // Per-hood ceiling, because they are not the same.
    //
    // The POLLEN hood runs out of travel at 77 deg: its bottom stop is 0.355,
    // and 80 deg would need 0.3418 -- 0.0132 PAST it. Clamping both hoods to
    // a shared 80 would command the pollen servo into its stop and hold it
    // there on every long shot.
    //
    // The shot table still asks for up to 80. Pollen simply cannot deliver
    // the last 3 deg, so its longest shots will fall short until the hood
    // gains travel. That is mechanical, not a constant to tune.
    public static final double POLLEN_MAX_DEG = 77.0;
    public static final double NECTAR_MAX_DEG = 80.0;

    // Offset, back-solved from one measured EXIT ANGLE per hood, taken with
    // the hood down -- its deepest position, which is its largest angle:
    //     pollen  bottom stop 0.355 exits at 77 deg -> 55 deg at 0.4516
    //     nectar  bottom stop 0.706 exits at 80 deg -> 55 deg at 0.5773
    //
    // Sanity: that puts the TOP stops at about 45 and 43 deg. Both hoods can
    // therefore go shallower than the table ever asks, which is why treating
    // a top stop as "55 deg" was wrong by roughly 10 deg.
    public static double POLLEN_POS_AT_55 = 0.4516;
    public static double NECTAR_POS_AT_55 = 0.5773;

    public static int POLLEN_SIGN = -1;
    public static int NECTAR_SIGN = +1;

    // INIT: each hood at its BOTTOM stop, which is its deepest reachable
    // angle -- 77 deg for pollen, 80 for nectar.
    public static double POLLEN_INIT_POS = 0.355;
    public static double NECTAR_INIT_POS = 0.706;

    public static double pollenPos(double hoodDeg) {
        double h = clampPollen(hoodDeg);
        return clamp01(POLLEN_POS_AT_55 + POLLEN_SIGN * (h - HOOD_MIN_DEG) * POLLEN_POS_PER_DEG);
    }

    public static double nectarPos(double hoodDeg) {
        double h = clampNectar(hoodDeg);
        return clamp01(NECTAR_POS_AT_55 + NECTAR_SIGN * (h - HOOD_MIN_DEG) * NECTAR_POS_PER_DEG);
    }

    public static double pollenDeg(double pos) {
        return HOOD_MIN_DEG + POLLEN_SIGN * (pos - POLLEN_POS_AT_55) / POLLEN_POS_PER_DEG;
    }

    public static double nectarDeg(double pos) {
        return HOOD_MIN_DEG + NECTAR_SIGN * (pos - NECTAR_POS_AT_55) / NECTAR_POS_PER_DEG;
    }

    public static boolean pollenRangeFits() {
        double a = POLLEN_POS_AT_55;
        double b = pollenPos(POLLEN_MAX_DEG);
        return inRange01(a) && inRange01(b);
    }

    public static boolean nectarRangeFits() {
        double a = NECTAR_POS_AT_55;
        double b = nectarPos(NECTAR_MAX_DEG);
        return inRange01(a) && inRange01(b);
    }

    // True when the table is asking for more than this hood has. The shot
    // planner cannot see a clamp, so this is how it finds out.
    public static boolean pollenClamped(double hoodDeg) { return hoodDeg > POLLEN_MAX_DEG; }
    public static boolean nectarClamped(double hoodDeg) { return hoodDeg > NECTAR_MAX_DEG; }

    public static double clampPollen(double d) {
        return d < HOOD_MIN_DEG ? HOOD_MIN_DEG : (d > POLLEN_MAX_DEG ? POLLEN_MAX_DEG : d);
    }

    public static double clampNectar(double d) {
        return d < HOOD_MIN_DEG ? HOOD_MIN_DEG : (d > NECTAR_MAX_DEG ? NECTAR_MAX_DEG : d);
    }

    // Kept for callers that do not know which hood they are talking about.
    // Uses the SHALLOWER ceiling so it can never over-command the pollen hood.
    public static double clampHood(double d) {
        double hi = Math.min(POLLEN_MAX_DEG, NECTAR_MAX_DEG);
        return d < HOOD_MIN_DEG ? HOOD_MIN_DEG : (d > hi ? hi : d);
    }

    private static double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
    private static boolean inRange01(double v) { return v >= 0.0 && v <= 1.0; }
}
