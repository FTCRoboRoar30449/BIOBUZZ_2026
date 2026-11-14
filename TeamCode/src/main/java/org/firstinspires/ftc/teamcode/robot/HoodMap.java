package org.firstinspires.ftc.teamcode.robot;

public final class HoodMap {

    private HoodMap() {}

    public static final double SERVO_TRAVEL_DEG = 1800.0;

    public static final double POLLEN_GEAR_RATIO = 158.0 / 20.0;   // 7.900
    public static final double NECTAR_GEAR_RATIO = 176.0 / 19.0;   // 9.2632

    // PROVISIONAL, from the mechanical limits measured 13 Nov:
    //     pollen  top 0.495 .. bottom 0.355   span 0.140
    //     nectar  top 0.516 .. bottom 0.706   span 0.190
    //
    // The limits are solid. The ANGLES at them are not -- they were eyeballed
    // as roughly 55 and 80, so these ratios are span/25 on an assumption, and
    // carry whatever error is in that 25. They are still better than the old
    // derived values (0.00439 and 0.00515, which were 28% and 48% lower),
    // because those came from assumed gearing and an assumed 1800 deg servo
    // and were never checked against anything.
    //
    // TO FINISH: put a protractor on each hood, find the position at a real
    // 55 and a real 80, and set POS_AT_55 and these from that. Until then
    // every hood angle carries the error in the eyeballed endpoints.
    //
    // Keep the gear ratios above for reference, but do not compute from them.
    public static double POLLEN_POS_PER_DEG = 0.00560;
    public static double NECTAR_POS_PER_DEG = 0.00760;

    public static final double HOOD_MIN_DEG = 55.0;
    public static final double HOOD_MAX_DEG = 80.0;

    // ---- CALIBRATE with RobotSetup steps 9-10 ----------------------------
    //
    // Measured stops, 13 Nov:  pollen 0.335..0.475   nectar 0.466..0.636
    // Spans 31.9 and 33.0 deg of hood -- both clear the 25 deg the 55-80
    // table needs.
    //
    // The old values (0.555 / 0.436) were OUTSIDE those ranges, so every hood
    // command clamped against a stop and the whole table was unreachable.
    //
    // The TOP mechanical limit of each hood, taken as 55 deg provisionally.
    // Treating a hard stop as the shallow end of the table is a guess: the
    // stop is wherever the mechanism ends, not wherever 55 deg happens to be.
    // Good enough to command from, not good enough to shoot on.
    //
    // Two earlier stop sweeps disagree with these, and the disagreement is
    // worth resolving before trusting any of it:
    //     pollen  step 3 gave 0.335..0.475, this gave 0.355..0.495
    //             -- same span 0.140, shifted 0.020. An offset, so probably
    //             one sweep stopped short at both ends.
    //     nectar  step 3 gave 0.466..0.636 (span 0.170), this 0.516..0.706
    //             (span 0.190). DIFFERENT span, which an offset cannot
    //             explain -- one of the two found a stop the other did not.
    public static double POLLEN_POS_AT_55 = 0.495;
    public static double NECTAR_POS_AT_55 = 0.516;

    public static int POLLEN_SIGN = -1;
    public static int NECTAR_SIGN = +1;

    // Where each hood is commanded at INIT: its BOTTOM limit, 80 deg.
    // Confirmed 14 Nov, and each equals pollenPos(80) / nectarPos(80) exactly,
    // so the table and the init agree by construction rather than by luck.
    public static double POLLEN_INIT_POS = 0.355;
    public static double NECTAR_INIT_POS = 0.706;

    public static double pollenPos(double hoodDeg) {
        double h = clampHood(hoodDeg);
        return clamp01(POLLEN_POS_AT_55 + POLLEN_SIGN * (h - HOOD_MIN_DEG) * POLLEN_POS_PER_DEG);
    }

    public static double nectarPos(double hoodDeg) {
        double h = clampHood(hoodDeg);
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
        double b = POLLEN_POS_AT_55 + POLLEN_SIGN * (HOOD_MAX_DEG - HOOD_MIN_DEG) * POLLEN_POS_PER_DEG;
        return inRange01(a) && inRange01(b);
    }

    public static boolean nectarRangeFits() {
        double a = NECTAR_POS_AT_55;
        double b = NECTAR_POS_AT_55 + NECTAR_SIGN * (HOOD_MAX_DEG - HOOD_MIN_DEG) * NECTAR_POS_PER_DEG;
        return inRange01(a) && inRange01(b);
    }

    public static double clampHood(double d) {
        return d < HOOD_MIN_DEG ? HOOD_MIN_DEG : (d > HOOD_MAX_DEG ? HOOD_MAX_DEG : d);
    }
    private static double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
    private static boolean inRange01(double v) { return v >= 0.0 && v <= 1.0; }
}
