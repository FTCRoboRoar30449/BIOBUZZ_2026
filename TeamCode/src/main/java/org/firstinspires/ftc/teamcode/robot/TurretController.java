package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class TurretController {

    // ---- mechanism, fixed ------------------------------------------------
    public static final double SERVO_TRAVEL_DEG  = 1800.0;
    public static final double SERVO_PER_TURRET  = 2.0;

    public static double TURRET_TRAVEL_DEG = 745.0;

    public static double TURRET_LIMIT_DEG = 105.0;

    public static double ENCODER_TICKS_PER_REV = 8192.0;

    public static double SERVO_AT_ZERO = 0.39795;

    public static int SERVO_SIGN = -1;

    public static int ENCODER_SIGN = +1;
    public static boolean LIMIT_TRIGGERED_STATE = false;

    // ---- tuning ----------------------------------------------------------
    public static int SEARCH_FIRST_DIR = +1;

    public static boolean AUTO_SEARCH_ON_INIT = false;
    public static double SEARCH_SPAN_DEG = 85.0;
    public static double SEARCH_STEP_DEG = 0.5;
    public static long SEARCH_STEP_MS = 40;
    public static long VERIFY_SETTLE_MS = 700;
    public static double SLIP_TOLERANCE_DEG = 6.0;

    public enum HomeState { NOT_HOMED, VERIFYING, NEEDS_SEARCH, SEARCHING, HOMED, FAILED }

    private final RobotHardware robot;
    private final Servo servo;
    private final DigitalChannel limit;

    private HomeState homeState = HomeState.NOT_HOMED;
    private double commandedDeg = 0.0;
    private int encoderZeroTicks = 0;
    private boolean encoderZeroed = false;

    private long phaseStartMs = 0;
    private double searchOffsetDeg = 0.0;
    private int searchStep = 0;
    private long lastStepMs = 0;

    public TurretController(RobotHardware robot) {
        this.robot = robot;
        this.servo = robot.turretRot;
        this.limit = robot.magneticLimitSwitch;

        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    // ---- aiming ----------------------------------------------------------

    public static double HOME_OFFSET_DEG = 0.0;

    public static double servoPositionFor(double turretDeg) {
        double d = clampTurret(turretDeg);
        double pos = SERVO_AT_ZERO + SERVO_SIGN * d / TURRET_TRAVEL_DEG;
        return pos < 0 ? 0 : (pos > 1 ? 1 : pos);
    }

    public static double turretDegFor(double servoPos) {
        return SERVO_SIGN * (servoPos - SERVO_AT_ZERO) * TURRET_TRAVEL_DEG;
    }

    public static double clampTurret(double deg) {
        return deg < -TURRET_LIMIT_DEG ? -TURRET_LIMIT_DEG
             : (deg > TURRET_LIMIT_DEG ? TURRET_LIMIT_DEG : deg);
    }

    public static long SETTLE_BASE_MS = 120;
    public static double SERVO_MS_PER_TURRET_DEG = 3.40;
    public static double ARRIVED_TOLERANCE_DEG = 2.0;

    public static boolean ENCODER_COUPLED = false;

    // ---- closed loop on the through-bore ----------------------------------
    public static boolean CLOSED_LOOP = false;
    public static double CL_KP        = 0.35;
    public static double CL_TOL_DEG   = 0.75;
    public static double CL_MAX_TRIM  = 0.080;

    private double servoTrim = 0.0;
    private boolean clSaturated = false;

    private long moveStartMs = 0;
    private double lastMoveDeg = 0;

    public void setTurretDeg(double turretDeg) {
        double next = clampTurret(turretDeg);
        double delta = Math.abs(next - commandedDeg);
        if (delta > 0.25) {
            lashDir = next > commandedDeg ? +1 : -1;
            lastMoveDeg = delta;
            moveStartMs = System.currentTimeMillis();
            servoTrim = 0.0;
            clSaturated = false;
        }
        commandedDeg = next;
        writeServo();
    }

    public static boolean LASH_COMPENSATE = true;
    public static double LASH_DEG = 7.97;
    private int lashDir = +1;

    public int getLashDir() { return lashDir; }

    private void writeServo() {
        double eff = commandedDeg;
        if (LASH_COMPENSATE) {
            eff = clampTurret(commandedDeg + lashDir * LASH_DEG / 2.0);
        }
        double pos = servoPositionFor(eff) + servoTrim;
        servo.setPosition(pos < 0 ? 0 : (pos > 1 ? 1 : pos));
    }

    private void serviceClosedLoop() {
        if (!CLOSED_LOOP || !ENCODER_COUPLED || !encoderZeroed) return;
        if (homeState != HomeState.HOMED) return;

        double m = getMeasuredDeg();
        if (Double.isNaN(m)) return;

        double err = commandedDeg - m;
        if (Math.abs(err) <= CL_TOL_DEG) return;

        servoTrim += SERVO_SIGN * CL_KP * err / TURRET_TRAVEL_DEG;
        if (servoTrim >  CL_MAX_TRIM) { servoTrim =  CL_MAX_TRIM; clSaturated = true; }
        else if (servoTrim < -CL_MAX_TRIM) { servoTrim = -CL_MAX_TRIM; clSaturated = true; }
        else clSaturated = false;

        writeServo();
    }

    public double getServoTrim() { return servoTrim; }
    public boolean isTrimSaturated() { return clSaturated; }

    public long settleTimeMs() {
        return SETTLE_BASE_MS + (long) (lastMoveDeg * SERVO_MS_PER_TURRET_DEG);
    }

    public boolean isSettled() {
        if (System.currentTimeMillis() - moveStartMs < settleTimeMs()) return false;
        if (CLOSED_LOOP && ENCODER_COUPLED && encoderZeroed) {
            double s = getSlipDeg();
            return Double.isNaN(s) || Math.abs(s) <= ARRIVED_TOLERANCE_DEG;
        }
        if (!ENCODER_COUPLED) return true;
        double slip = getSlipDeg();
        if (Double.isNaN(slip)) return true;
        return Math.abs(slip) <= ARRIVED_TOLERANCE_DEG;
    }

    public double getCommandedDeg() { return commandedDeg; }

    public double getMeasuredDeg() {
        if (!encoderZeroed) return Double.NaN;
        int ticks = robot.intakeMot.getCurrentPosition() - encoderZeroTicks;
        return ENCODER_SIGN * ticks * 360.0 / ENCODER_TICKS_PER_REV
                + HOME_OFFSET_DEG;
    }

    public int getRawEncoderTicks() { return robot.intakeMot.getCurrentPosition(); }

    public double getSlipDeg() {
        double m = getMeasuredDeg();
        return Double.isNaN(m) ? Double.NaN : m - commandedDeg;
    }

    public boolean isSlipping() {
        double s = getSlipDeg();
        return !Double.isNaN(s) && Math.abs(s) > SLIP_TOLERANCE_DEG;
    }

    public boolean isAtHomeSwitch() { return limit.getState() == LIMIT_TRIGGERED_STATE; }

    // ---- homing ----------------------------------------------------------

    public void beginHoming() {
        homeState = HomeState.VERIFYING;
        phaseStartMs = System.currentTimeMillis();
        searchOffsetDeg = 0.0;
        searchStep = 0;
        setTurretDeg(0.0);
        zeroEncoderHere();
    }

    // Trust the stored SERVO_AT_ZERO instead of searching for the magnet.
    // Match code uses this: the turret goes to 0 and is treated as homed, so
    // nothing in a match can move the zero. Only RobotSetup still homes, and
    // only RobotSetup writes SERVO_AT_ZERO.
    public void assumeHomed() {
        homeState = HomeState.HOMED;
        phaseStartMs = System.currentTimeMillis();
        searchOffsetDeg = 0.0;
        searchStep = 0;
        setTurretDeg(0.0);
        zeroEncoderHere();
    }

    public void beginSearch() {
        homeState = HomeState.SEARCHING;
        phaseStartMs = System.currentTimeMillis();
        lastStepMs = 0;
        searchOffsetDeg = 0.0;
        searchStep = 0;
    }

    public void update() {
        long now = System.currentTimeMillis();
        switch (homeState) {
            case VERIFYING:
                if (now - phaseStartMs < VERIFY_SETTLE_MS) break;
                if (isAtHomeSwitch()) {
                    adoptHomeAt(servoPositionFor(0.0));
                } else {
                    homeState = HomeState.NEEDS_SEARCH;
                    if (AUTO_SEARCH_ON_INIT) beginSearch();
                }
                break;

            case SEARCHING: {
                if (now - lastStepMs < SEARCH_STEP_MS) break;
                lastStepMs = now;

                if (isAtHomeSwitch()) {
                    adoptHomeAt(servoPositionFor(searchOffsetDeg));
                    break;
                }

                searchStep++;
                int ring = (searchStep + 1) / 2;
                int side = (searchStep % 2 == 1)
                        ? (SEARCH_FIRST_DIR >= 0 ? +1 : -1)
                        : (SEARCH_FIRST_DIR >= 0 ? -1 : +1);
                searchOffsetDeg = side * ring * SEARCH_STEP_DEG;
                if (Math.abs(searchOffsetDeg) > SEARCH_SPAN_DEG) {
                    homeState = HomeState.FAILED;
                    break;
                }
                servo.setPosition(servoPositionFor(searchOffsetDeg));
                break;
            }

            default:
                break;
        }
        serviceClosedLoop();
    }

    private void adoptHomeAt(double servoPosAtSwitch) {
        SERVO_AT_ZERO = servoPosAtSwitch
                - SERVO_SIGN * HOME_OFFSET_DEG / TURRET_TRAVEL_DEG;
        commandedDeg = HOME_OFFSET_DEG;
        zeroEncoderHere();
        homeState = HomeState.HOMED;
    }

    private void zeroEncoderHere() {
        encoderZeroTicks = robot.intakeMot.getCurrentPosition();
        encoderZeroed = true;
    }

    public HomeState getHomeState() { return homeState; }
    public boolean isHomed() { return homeState == HomeState.HOMED; }

    public String getHomeMessage() {
        switch (homeState) {
            case NOT_HOMED:    return "turret not homed - call beginHoming()";
            case VERIFYING:    return "turret moving to forward, confirming...";
            case NEEDS_SEARCH: return "turret NOT at home - park it forward, or press RB to search";
            case SEARCHING:    return String.format("searching %+.1f deg...", searchOffsetDeg);
            case HOMED:        return "turret homed";
            case FAILED:       return "HOME FAILED - check switch polarity and SERVO_AT_ZERO";
        }
        return "";
    }

    public void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry t) {
        t.addData("Turret home", getHomeMessage());
        t.addData("Turret cmd (deg)", "%.1f", commandedDeg);
        double m = getMeasuredDeg();
        t.addData("Turret enc (deg)", Double.isNaN(m) ? "not zeroed" : String.format("%.1f", m));
        double s = getSlipDeg();
        t.addData("Turret slip (deg)", Double.isNaN(s) ? "-" : String.format("%.1f%s",
                s, !ENCODER_COUPLED ? "  (ignored, encoder not coupled)"
                   : (isSlipping() ? "  <-- SLIP" : "")));
        t.addData("Turret settled", isSettled());
        t.addData("Turret servo pos", "%.4f", servoPositionFor(commandedDeg));
        t.addData("Home switch", isAtHomeSwitch() ? "AT HOME" : "off");
    }
}
