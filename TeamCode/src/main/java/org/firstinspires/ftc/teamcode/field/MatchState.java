package org.firstinspires.ftc.teamcode.field;

public final class MatchState {

    public enum Alliance { RED, BLUE }

    private MatchState() {}

    public static final long FRESH_FOR_MS = 120_000;

    public static Alliance alliance = null;
    public static long savedAtMs = 0;

    public static double poseX = 72.0, poseY = 72.0, headingDeg = 0.0;
    public static int countPollen = 0, countNectar = 0;
    public static boolean deployed = false;
    public static int cellPollenIn = 0, cellNectarIn = 0;

    public static void save(Alliance a, double x, double y, double heading,
                            int pollen, int nectar, boolean armDeployed,
                            int cellPollen, int cellNectar) {
        alliance = a;
        poseX = x; poseY = y; headingDeg = heading;
        countPollen = pollen; countNectar = nectar;
        deployed = armDeployed;
        cellPollenIn = cellPollen; cellNectarIn = cellNectar;
        savedAtMs = System.currentTimeMillis();
    }

    public static boolean isFresh(Alliance a) {
        return alliance == a
                && (System.currentTimeMillis() - savedAtMs) < FRESH_FOR_MS;
    }

    public static void clear() {
        alliance = null; savedAtMs = 0;
        poseX = 72.0; poseY = 72.0; headingDeg = 0.0;
        countPollen = 0; countNectar = 0;
        deployed = false;
        cellPollenIn = 0; cellNectarIn = 0;
    }

    public static String summary(Alliance a) {
        if (alliance == null) return "no autonomous data";
        if (alliance != a) return "data is from " + alliance + " - ignoring";
        if (!isFresh(a)) return "STALE - ignoring (saved "
                + ((System.currentTimeMillis() - savedAtMs) / 1000) + "s ago)";
        return String.format(java.util.Locale.US,
                "%.1f, %.1f @ %.0f deg | %dP %dN | arm %s",
                poseX, poseY, headingDeg, countPollen, countNectar,
                deployed ? "out" : "stowed");
    }
}
