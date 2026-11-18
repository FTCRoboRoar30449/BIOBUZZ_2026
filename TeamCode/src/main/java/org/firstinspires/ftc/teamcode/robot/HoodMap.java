package org.firstinspires.ftc.teamcode.robot;

// Hood angle -> servo position, for both hoods.
public final class HoodMap {

    private HoodMap() {}

    public static final double SERVO_TRAVEL_DEG = 1800.0;

    public static final double POLLEN_GEAR_RATIO = 158.0 / 20.0;   // 7.900
    public static final double NECTAR_GEAR_RATIO = 176.0 / 19.0;   // 9.2632

    // Slope, from the gearing.
    public static final double POLLEN_POS_PER_DEG = POLLEN_GEAR_RATIO / SERVO_TRAVEL_DEG; // 0.0043889
    public static final double NECTAR_POS_PER_DEG = NECTAR_GEAR_RATIO / SERVO_TRAVEL_DEG; // 0.0051462

    public static final double HOOD_MIN_DEG = 55.0;
    public static final double HOOD_MAX_DEG = 80.0;

    // Stops MEASURED 17 Nov, RobotSetup step 3:
    //   pollen  0.332 = 80 deg, 0.472 = 48.1 deg
    //   nectar  0.707 = 80 deg, 0.537 = 47.0 deg
    public static final double POLLEN_STOP_POS = 0.343;
    public static final double NECTAR_STOP_POS = 0.7050;

    // Never command the stop itself. A servo told to hold AT a hard limit
    // can never null its error, so it drives into the stop and buzzes --
    // which is what both hoods did at init on 17 Nov. 0.010 of position is
    // 2.3 deg of pollen hood, 1.9 deg of nectar.
    // Re-measured 17 Nov with degree-stepped jogging, backlash taken up on
    // the way in, so these ARE the positions that give 80 deg -- no extra
    // margin is applied. Nectar moved 0.043 (8.4 deg) from the first pass;
    // the coarse jog had stopped short of real travel.
    public static double POLLEN_MAX_DEG = 80.0;
    public static double NECTAR_MAX_DEG = 80.0;

    // Back-solved from the 80 deg stop using the gear ratio.
    public static double POLLEN_POS_AT_55 = 0.4527;
    public static double NECTAR_POS_AT_55 = 0.5763;

    public static int POLLEN_SIGN = -1;
    public static int NECTAR_SIGN = +1;

    // Rest 1 deg short of the stop. Deep enough to be out of the way, far
    // enough off the limit that the servo is not holding against it.
    public static double HOOD_INIT_DEG = 79.0;
    public static double POLLEN_INIT_POS = pollenPos(HOOD_INIT_DEG);
    public static double NECTAR_INIT_POS = nectarPos(HOOD_INIT_DEG);

    // The stops were found jogging UP off the limit, so the map is true for
    // a hood arriving from below. A shot drives the hood DOWN, and the lash
    // makes it fall short by this much -- so ask for 80.5 to land on 80.
    public static double HOOD_LASH_DEG = 0.5;

    public static double pollenPos(double hoodDeg) {
        double h = clampPollen(hoodDeg) + HOOD_LASH_DEG;
        return clamp01(POLLEN_POS_AT_55 + POLLEN_SIGN * (h - HOOD_MIN_DEG) * POLLEN_POS_PER_DEG);
    }

    public static double nectarPos(double hoodDeg) {
        double h = clampNectar(hoodDeg) + HOOD_LASH_DEG;
        return clamp01(NECTAR_POS_AT_55 + NECTAR_SIGN * (h - HOOD_MIN_DEG) * NECTAR_POS_PER_DEG);
    }

    // Inverse of the above, lash and all, so the angle RobotSetup step 3
    // shows is the angle a shot at that position would actually produce.
    public static double pollenDeg(double pos) {
        return HOOD_MIN_DEG - HOOD_LASH_DEG
                + POLLEN_SIGN * (pos - POLLEN_POS_AT_55) / POLLEN_POS_PER_DEG;
    }

    public static double nectarDeg(double pos) {
        return HOOD_MIN_DEG - HOOD_LASH_DEG
                + NECTAR_SIGN * (pos - NECTAR_POS_AT_55) / NECTAR_POS_PER_DEG;
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

    // True when the table is asking for more than this hood has.
    public static boolean pollenClamped(double hoodDeg) { return hoodDeg > POLLEN_MAX_DEG; }
    public static boolean nectarClamped(double hoodDeg) { return hoodDeg > NECTAR_MAX_DEG; }

    public static double clampPollen(double d) {
        return d < HOOD_MIN_DEG ? HOOD_MIN_DEG : (d > POLLEN_MAX_DEG ? POLLEN_MAX_DEG : d);
    }

    public static double clampNectar(double d) {
        return d < HOOD_MIN_DEG ? HOOD_MIN_DEG : (d > NECTAR_MAX_DEG ? NECTAR_MAX_DEG : d);
    }

    // Kept for callers that do not know which hood they are talking about.
    public static double clampHood(double d) {
        double hi = Math.min(POLLEN_MAX_DEG, NECTAR_MAX_DEG);
        return d < HOOD_MIN_DEG ? HOOD_MIN_DEG : (d > hi ? hi : d);
    }

    private static double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
    private static boolean inRange01(double v) { return v >= 0.0 && v <= 1.0; }
}
