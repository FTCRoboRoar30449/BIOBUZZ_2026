package org.firstinspires.ftc.teamcode.robot;

public final class HoodMap {

    private HoodMap() {}

    public static final double SERVO_TRAVEL_DEG = 1800.0;

    public static final double POLLEN_GEAR_RATIO = 158.0 / 20.0;
    public static final double NECTAR_GEAR_RATIO = 176.0 / 19.0;

    public static final double POLLEN_POS_PER_DEG = POLLEN_GEAR_RATIO / SERVO_TRAVEL_DEG;
    public static final double NECTAR_POS_PER_DEG = NECTAR_GEAR_RATIO / SERVO_TRAVEL_DEG;

    public static final double HOOD_MIN_DEG = 55.0;
    public static final double HOOD_MAX_DEG = 80.0;

    public static final double POLLEN_STOP_POS = 0.343;
    public static final double NECTAR_STOP_POS = 0.7050;

    public static double POLLEN_MAX_DEG = 80.0;
    public static double NECTAR_MAX_DEG = 80.0;

    public static double POLLEN_POS_AT_55 = 0.4527;
    public static double NECTAR_POS_AT_55 = 0.5763;

    public static int POLLEN_SIGN = -1;
    public static int NECTAR_SIGN = +1;

    public static double HOOD_INIT_DEG = 79.0;
    public static double POLLEN_INIT_POS = pollenPos(HOOD_INIT_DEG);
    public static double NECTAR_INIT_POS = nectarPos(HOOD_INIT_DEG);

    public static double HOOD_LASH_DEG = 0.5;

    public static double pollenPos(double hoodDeg) {
        double h = clampPollen(hoodDeg) + HOOD_LASH_DEG;
        return clamp01(POLLEN_POS_AT_55 + POLLEN_SIGN * (h - HOOD_MIN_DEG) * POLLEN_POS_PER_DEG);
    }

    public static double nectarPos(double hoodDeg) {
        double h = clampNectar(hoodDeg) + HOOD_LASH_DEG;
        return clamp01(NECTAR_POS_AT_55 + NECTAR_SIGN * (h - HOOD_MIN_DEG) * NECTAR_POS_PER_DEG);
    }

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
