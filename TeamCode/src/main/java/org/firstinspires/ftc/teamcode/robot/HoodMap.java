package org.firstinspires.ftc.teamcode.robot;

/**
 * Hood angle to servo position.
 *
 * Both hoods are goBILDA 2000-series 5-turn servos (1800 deg of travel) driving
 * herringbone gear pairs:
 *
 *     POLLEN   158 hood teeth : 20 servo teeth  ->  7.900 servo turns per hood turn
 *     NECTAR   176 hood teeth : 19 servo teeth  ->  9.263 servo turns per hood turn
 *
 * So sweeping the hood 55 -> 80 deg takes 197.5 deg of pollen servo and 231.6 deg
 * of nectar servo -- 11.0% and 12.9% of the servo's travel. At roughly 2000 PWM
 * steps that is about 0.11 deg of hood per step, against a hood tolerance of
 * +/-4-6 deg. Resolution is a non-issue.
 *
 * ONE constant per hood needs measuring: the servo position at hood 55 deg. The
 * span is fixed by the gear ratio, so BenchCalibration only has to find that
 * single reference, not both ends.
 *
 * Hood angle is measured FROM HORIZONTAL, matching ShooterRed/ShooterBlue and
 * ShooterCalibration. 80 deg is the steep lob used at short range; 70 deg is the
 * flattest the tables ever ask for. Do not "correct" this to from-vertical -- the
 * tables put hood 80 at 15-23 in and hood 70 at 30-39 in, which only makes sense
 * measured from horizontal.
 */
public final class HoodMap {

    private HoodMap() {}

    public static final double SERVO_TRAVEL_DEG = 1800.0;

    public static final double POLLEN_GEAR_RATIO = 158.0 / 20.0;   // 7.900
    public static final double NECTAR_GEAR_RATIO = 176.0 / 19.0;   // 9.2632

    /** Servo position change per degree of hood. */
    public static final double POLLEN_POS_PER_DEG = POLLEN_GEAR_RATIO / SERVO_TRAVEL_DEG; // 0.0043889
    public static final double NECTAR_POS_PER_DEG = NECTAR_GEAR_RATIO / SERVO_TRAVEL_DEG; // 0.0051462

    public static final double HOOD_MIN_DEG = 55.0;
    public static final double HOOD_MAX_DEG = 80.0;

    // ---- CALIBRATE with BenchCalibration ---------------------------------
    /**
     * STILL UNMEASURED -- the servo position that puts each hood at exactly
     * 55 deg. These are placeholders chosen only to CENTRE each sweep in the
     * servo's 0..1 range, so nothing clamps before the real numbers arrive:
     *
     *   pollen  sign -1, span 0.110  ->  55 at 0.555, 80 at 0.445
     *   nectar  sign +1, span 0.129  ->  55 at 0.436, 80 at 0.564
     *
     * The old 0.100 for pollen does not even fit: with sign -1 it puts 80 deg
     * at -0.010 and every steep shot silently clamps to 0.
     */
    public static double POLLEN_POS_AT_55 = 0.555;
    public static double NECTAR_POS_AT_55 = 0.436;

    /**
     * MEASURED on the bench. +1 if increasing servo position increases hood
     * angle. The two hoods are mirrored on the robot and genuinely disagree,
     * which is exactly why this was worth measuring rather than assuming:
     *
     *   pollen  raising the servo drives 80 -> 55, angle DECREASES  -> -1
     *   nectar  raising the servo drives 55 -> 80, angle INCREASES  -> +1
     */
    public static int POLLEN_SIGN = -1;
    public static int NECTAR_SIGN = +1;

    /** Servo position for a pollen hood angle, clamped to the servo's range. */
    public static double pollenPos(double hoodDeg) {
        double h = clampHood(hoodDeg);
        return clamp01(POLLEN_POS_AT_55 + POLLEN_SIGN * (h - HOOD_MIN_DEG) * POLLEN_POS_PER_DEG);
    }

    /** Servo position for a nectar hood angle, clamped to the servo's range. */
    public static double nectarPos(double hoodDeg) {
        double h = clampHood(hoodDeg);
        return clamp01(NECTAR_POS_AT_55 + NECTAR_SIGN * (h - HOOD_MIN_DEG) * NECTAR_POS_PER_DEG);
    }

    /** Inverse, for telemetry: what hood angle does this pollen servo position mean? */
    public static double pollenDeg(double pos) {
        return HOOD_MIN_DEG + POLLEN_SIGN * (pos - POLLEN_POS_AT_55) / POLLEN_POS_PER_DEG;
    }

    public static double nectarDeg(double pos) {
        return HOOD_MIN_DEG + NECTAR_SIGN * (pos - NECTAR_POS_AT_55) / NECTAR_POS_PER_DEG;
    }

    /** True when the whole 55-80 sweep fits inside the servo's 0..1 range. */
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
