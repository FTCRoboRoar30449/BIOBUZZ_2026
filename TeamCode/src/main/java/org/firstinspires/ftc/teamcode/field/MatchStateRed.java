package org.firstinspires.ftc.teamcode.field;

/**
 * Carries robot state from the RED autonomous into the RED TeleOp.
 *
 * WHY A SEPARATE CLASS PER ALLIANCE
 *   One shared holder with an alliance field would compile just as well and
 *   then, one day, hand a blue match red's inventory because somebody passed
 *   the wrong enum. Two classes make that mistake impossible to write rather
 *   than merely unlikely: RED TeleOp reads MatchStateRed and nothing else.
 *   The duplication is forty lines and it buys structural safety.
 *
 * WHY STATIC
 *   Autonomous and TeleOp are separate OpModes in the same process. Statics
 *   survive between them, and nothing else does. They do NOT survive a Robot
 *   Controller restart, which is exactly the case `valid` guards.
 *
 * WHAT TO DO WITH IT
 *   At the END of autonomous:      MatchStateRed.save(...)
 *   At the START of TeleOp:        if (MatchStateRed.isFresh()) { ... }
 *
 * ALWAYS check isFresh() before trusting any field. A TeleOp started without
 * its autonomous -- a re-init, a restart, a practice run -- finds stale or
 * absent values here, and acting on a pose from the last match is worse than
 * having no pose at all.
 */
public final class MatchStateRed {

    private MatchStateRed() {}

    /**
     * How long after autonomous these values are still believable.
     *
     * The gap between autonomous ending and TeleOp starting is well under a
     * minute in a real match. Anything older is a different run.
     */
    public static final long FRESH_FOR_MS = 120_000;

    /** False until autonomous has written something. */
    public static boolean valid = false;
    /** When save() was called, from System.currentTimeMillis(). */
    public static long savedAtMs = 0;

    /** Final pose, PEDRO inches and degrees CCW from +X. */
    public static double poseX = 72.0, poseY = 72.0, headingDeg = 0.0;

    /** Inventory at the end of autonomous. */
    public static int countPollen = 0, countNectar = 0;

    /** True if the intake arm had already been deployed. */
    public static boolean deployed = false;

    /** Running estimate of what we put in the CELL. Provisional -- see notes. */
    public static int cellPollenIn = 0, cellNectarIn = 0;

    /** Call once at the end of autonomous. */
    public static void save(double x, double y, double heading,
                            int pollen, int nectar,
                            boolean armDeployed,
                            int cellPollen, int cellNectar) {
        poseX = x; poseY = y; headingDeg = heading;
        countPollen = pollen; countNectar = nectar;
        deployed = armDeployed;
        cellPollenIn = cellPollen; cellNectarIn = cellNectar;
        savedAtMs = System.currentTimeMillis();
        valid = true;
    }

    /**
     * True when autonomous wrote this recently enough to trust.
     *
     * Checks BOTH the flag and the age. The flag alone survives a stale
     * process; the age alone cannot tell "never written" from "written at
     * time zero".
     */
    public static boolean isFresh() {
        return valid && (System.currentTimeMillis() - savedAtMs) < FRESH_FOR_MS;
    }

    /** Forget everything. Call if a run is abandoned. */
    public static void clear() {
        valid = false; savedAtMs = 0;
        poseX = 72.0; poseY = 72.0; headingDeg = 0.0;
        countPollen = 0; countNectar = 0;
        deployed = false;
        cellPollenIn = 0; cellNectarIn = 0;
    }

    /** One line for telemetry, so a driver can see what TeleOp inherited. */
    public static String summary() {
        if (!valid) return "no autonomous data";
        if (!isFresh()) return "STALE - ignoring (saved "
                + ((System.currentTimeMillis() - savedAtMs) / 1000) + "s ago)";
        return String.format(java.util.Locale.US,
                "%.1f, %.1f @ %.0f deg | %dP %dN | arm %s",
                poseX, poseY, headingDeg, countPollen, countNectar,
                deployed ? "out" : "stowed");
    }
}
