package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Turret aiming and homing.
 *
 * MECHANISM
 *   turretRot is a goBILDA 2000-series 5-turn servo in POSITION mode: 1800 deg
 *   of travel. The servo turns 2x the turret, so the turret has 900 deg of
 *   range -- +/-450 deg about centre. The +/-180 deg the shooter classes ask
 *   for uses 40% of that, so travel is never the constraint.
 *
 *   A REV Through Bore Encoder V2 reads turret rotation 1:1, wired into the
 *   intakeMot encoder port (intakeMot's own encoder is disconnected). A
 *   magnetic limit switch sits at turret 0, shooters facing robot-forward.
 *
 * DRIVE TRAIN -- why the obvious sign guess is wrong twice over
 *   The servo is mounted UPSIDE-DOWN and drives both turret rings through a
 *   herringbone mesh. That is TWO reversals, and they cancel:
 *
 *     servo CW (seen from underneath, the way it is mounted)
 *       -> herringbone mesh reverses -> rings CCW seen from underneath
 *       -> viewed from ABOVE that same motion reads CW
 *
 *   So the rings turn CW from above when the servo turns CW. Anyone
 *   re-deriving SERVO_SIGN from "there is a gear in it, so it must invert"
 *   will get the wrong answer -- the mounting undoes the mesh.
 *
 * WHY HOMING DOES NOT SWEEP
 *   A servo in position mode is ABSOLUTE: setPosition(0.5) drives to its own
 *   centre no matter where it started. So the turret's angle is known from the
 *   command, not from a search. What actually needs establishing is the
 *   ASSEMBLY OFFSET -- which servo position corresponds to turret-forward --
 *   and that is a one-time calibration constant, not a per-match procedure.
 *
 *   At init we command SERVO_AT_ZERO and expect the limit switch to confirm.
 *   If it does not, a BOUNDED nudge sweeps the command a few degrees either
 *   side and reports the corrected offset. The turret never free-runs and
 *   never approaches its mechanical stops.
 *
 *   The encoder is then there to CHECK the servo got where it was told --
 *   catching slip, stall and a mis-set offset -- not to drive a control loop.
 *
 * CONVENTION
 *   turretDeg is CW-positive with 0 = robot forward, matching the turretDeg
 *   that ShooterRed/ShooterBlue emit. Feed their value straight in.
 */
public class TurretController {

    // ---- mechanism, fixed ------------------------------------------------
    public static final double SERVO_TRAVEL_DEG  = 1800.0;   // 5-turn servo
    public static final double SERVO_PER_TURRET  = 2.0;      // servo deg per turret deg
    /** Servo 0..1 spans this much turret rotation. Sets the position scaling. */
    public static final double TURRET_TRAVEL_DEG = SERVO_TRAVEL_DEG / SERVO_PER_TURRET; // 900

    /**
     * USABLE travel either side of forward -- NOT the servo's full 450.
     *
     * MEASURED: the turret reaches about 190 deg each way before the mechanism
     * stops it. The servo could drive 450, so the clamp is what keeps a
     * commanded angle from winding the turret into its own stop. The shooter
     * classes emit +/-180, which fits inside this with 10 deg to spare.
     *
     * Scaling still uses TURRET_TRAVEL_DEG, because servo 0..1 really does
     * span 900 deg -- this only limits what may be COMMANDED.
     */
    public static double TURRET_LIMIT_DEG = 190.0;

    /** REV Through Bore Encoder V2, 1:1 with the turret. */
    public static double ENCODER_TICKS_PER_REV = 8192.0;

    /**
     * STILL UNMEASURED: the servo position that puts the turret at 0 deg
     * (shooters facing robot-forward). This is the ASSEMBLY OFFSET and it can
     * only be found with the turret bolted together.
     *
     * 0.500 is a sound placeholder now that the limit is +/-190: that needs
     * only 0.211 of servo either side, so the whole usable range fits around
     * centre without clamping. Homing will fold in the correction it finds.
     */
    public static double SERVO_AT_ZERO = 0.500;

    /**
     * MEASURED: raising the servo position rotates the turret CLOCKWISE seen
     * from above, and this class defines turretDeg as CW-positive (see the
     * CONVENTION note in the class header). Positive angle therefore wants a
     * higher servo position: +1.
     *
     * !! CHECK THIS ONE AGAINST ShooterRed/ShooterBlue BEFORE COMPETING.
     * Everything here is internally consistent, but it only AIMS correctly if
     * those classes also emit CW-positive turretDeg. The turret's own travel
     * was described as reaching "-190 in CW" -- CW labelled negative, which
     * is CCW-positive, the opposite of what this class assumes. If ShooterRed
     * thinks that way, flip this to -1 and the chain lines up again.
     *
     * Settle it by READING ShooterRed, no robot needed. Find where it builds
     * turretDeg from a bearing and the robot yaw:
     *     bearing - yaw   -> CCW-positive  -> SERVO_SIGN = -1
     *     yaw - bearing   -> CW-positive   -> SERVO_SIGN = +1  (as set)
     * ShooterAdapter.turretDegToPoint() uses the second form, which is what
     * the flower shot already relies on.
     *
     * Or on a built robot: stand a target clearly to the robot's LEFT, call
     * plan(), watch which way the turret goes. Left is correct; right means
     * flip this sign.
     */
    public static int SERVO_SIGN = +1;

    /** MEASURED: the through-bore reads the right sense after intakeMot was
     *  set FORWARD. Re-check if that direction is ever changed again. */
    public static int ENCODER_SIGN = +1;
    /** DigitalChannel reading when the turret is AT home. */
    public static boolean LIMIT_TRIGGERED_STATE = false;

    // ---- tuning ----------------------------------------------------------
    /** How far either side of the expected zero the bounded search looks. */
    public static double SEARCH_SPAN_DEG = 30.0;
    /** Step size of the bounded search, turret degrees. */
    public static double SEARCH_STEP_DEG = 0.5;
    /** Settle time per search step. */
    public static long SEARCH_STEP_MS = 40;
    /** Settle time after the initial home command. */
    public static long VERIFY_SETTLE_MS = 700;
    /** Encoder-vs-command disagreement that counts as slip. */
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
    private int searchDir = +1;
    private long lastStepMs = 0;

    public TurretController(RobotHardware robot) {
        this.robot = robot;
        this.servo = robot.turretRot;
        this.limit = robot.magneticLimitSwitch;

        // A 5-turn servo only reaches its full 1800 deg if the PWM range is
        // widened. With the SDK default it will travel roughly a third of
        // that, and every commanded angle will come up short.
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    // ---- aiming ----------------------------------------------------------

    /** Servo position for a turret angle. Pure maths, no hardware. */
    public static double servoPositionFor(double turretDeg) {
        double d = clampTurret(turretDeg);
        double pos = SERVO_AT_ZERO + SERVO_SIGN * d / TURRET_TRAVEL_DEG;
        return pos < 0 ? 0 : (pos > 1 ? 1 : pos);
    }

    /** Inverse, for telemetry. */
    public static double turretDegFor(double servoPos) {
        return SERVO_SIGN * (servoPos - SERVO_AT_ZERO) * TURRET_TRAVEL_DEG;
    }

    public static double clampTurret(double deg) {
        return deg < -TURRET_LIMIT_DEG ? -TURRET_LIMIT_DEG
             : (deg > TURRET_LIMIT_DEG ? TURRET_LIMIT_DEG : deg);
    }

    /**
     * How long the turret needs to settle, as a function of how far it moved.
     *
     * A fixed wait does not work here. The servo sweeps its full 1800 deg in a
     * fixed time, so a 90 deg turret slew takes 180 deg of servo and a 180 deg
     * slew takes 360 -- several times longer than a small trim. Firing on a
     * fixed 350 ms would launch mid-swing on any large re-aim.
     *
     * MEASURE SERVO_MS_PER_TURRET_DEG on the bench: command a servo 0.0 -> 1.0
     * and time it. That is 900 deg of turret, so ms_per_deg = time_ms / 900.
     */
    public static long SETTLE_BASE_MS = 120;
    /** MEASURED: 900 deg of turret swept in 3060 ms. */
    public static double SERVO_MS_PER_TURRET_DEG = 3.40;
    /** Encoder agreement that counts as arrived, once the encoder is coupled. */
    public static double ARRIVED_TOLERANCE_DEG = 2.0;

    /**
     * Is the through-bore encoder mechanically coupled to the turret yet?
     *
     * FALSE on an electronics bench, where the encoder is wired and zeroes
     * happily but never turns. In that state the slip check sees the command
     * run away from a stationary encoder and isSettled() can NEVER become
     * true -- the AIM phase times out, re-arms, and the shooters pulse
     * forever without a ball ever being fed.
     *
     * Set TRUE once the encoder actually turns with the turret.
     */
    public static boolean ENCODER_COUPLED = false;

    private long moveStartMs = 0;
    private double lastMoveDeg = 0;

    /** Point the turret. Feed ShooterRed/Blue turretDeg straight in. */
    public void setTurretDeg(double turretDeg) {
        double next = clampTurret(turretDeg);
        double delta = Math.abs(next - commandedDeg);
        // Only restart the clock on a real move, so holding a target steady
        // does not keep resetting the settle timer.
        if (delta > 0.25) {
            lastMoveDeg = delta;
            moveStartMs = System.currentTimeMillis();
        }
        commandedDeg = next;
        servo.setPosition(servoPositionFor(commandedDeg));
    }

    /** Milliseconds the last commanded move should need. */
    public long settleTimeMs() {
        return SETTLE_BASE_MS + (long) (lastMoveDeg * SERVO_MS_PER_TURRET_DEG);
    }

    /**
     * True when the turret should be on target. Uses the encoder when it is
     * zeroed and coupled -- that is the real answer -- and falls back to the
     * move-size estimate otherwise, which is the case on a bench.
     */
    public boolean isSettled() {
        if (System.currentTimeMillis() - moveStartMs < settleTimeMs()) return false;
        if (!ENCODER_COUPLED) return true;            // bench: trust the clock
        double slip = getSlipDeg();
        if (Double.isNaN(slip)) return true;          // not zeroed yet
        return Math.abs(slip) <= ARRIVED_TOLERANCE_DEG;
    }

    public double getCommandedDeg() { return commandedDeg; }

    /** Turret angle as the through-bore encoder sees it, degrees. */
    public double getMeasuredDeg() {
        if (!encoderZeroed) return Double.NaN;
        int ticks = robot.intakeMot.getCurrentPosition() - encoderZeroTicks;
        return ENCODER_SIGN * ticks * 360.0 / ENCODER_TICKS_PER_REV;
    }

    /** Raw encoder, for calibration. */
    public int getRawEncoderTicks() { return robot.intakeMot.getCurrentPosition(); }

    /**
     * How far the turret actually is from where it was told to go. NaN until
     * homed. Large values mean slip, stall, or a wrong SERVO_AT_ZERO.
     */
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

    /** Call once at init. Commands turret-forward and waits to confirm. */
    public void beginHoming() {
        homeState = HomeState.VERIFYING;
        phaseStartMs = System.currentTimeMillis();
        searchOffsetDeg = 0.0;
        searchDir = +1;
        setTurretDeg(0.0);
    }

    /** Restart a bounded search. Bind to the re-home button. */
    public void beginSearch() {
        homeState = HomeState.SEARCHING;
        phaseStartMs = System.currentTimeMillis();
        lastStepMs = 0;
        searchOffsetDeg = 0.0;
        searchDir = +1;
    }

    /** Non-blocking. Call every loop. */
    public void update() {
        long now = System.currentTimeMillis();
        switch (homeState) {
            case VERIFYING:
                if (now - phaseStartMs < VERIFY_SETTLE_MS) break;
                if (isAtHomeSwitch()) {
                    zeroEncoderHere();
                    homeState = HomeState.HOMED;
                } else {
                    /*
                     * Start the bounded search straight away.
                     *
                     * This used to stop at NEEDS_SEARCH and wait for a driver
                     * to press RB. In TeleOp that is recoverable; in
                     * AUTONOMOUS there is nobody to press it, so a turret that
                     * failed to confirm home would leave REQUIRE_HOMED
                     * blocking every shooting state for the whole period,
                     * silently.
                     *
                     * It adds no new kind of motion -- beginHoming() has
                     * already commanded the turret to 0 -- and the search is
                     * bounded to SEARCH_SPAN_DEG either side, ending in HOMED
                     * or FAILED. FAILED is terminal, so this cannot loop.
                     * RB still forces a fresh search at any time.
                     */
                    homeState = HomeState.NEEDS_SEARCH;
                    beginSearch();
                }
                break;

            case SEARCHING: {
                if (now - lastStepMs < SEARCH_STEP_MS) break;
                lastStepMs = now;

                if (isAtHomeSwitch()) {
                    // The offset we had to add IS the correction to the
                    // stored constant. Fold it in so it is right from now on.
                    SERVO_AT_ZERO = servoPositionFor(searchOffsetDeg);
                    commandedDeg = 0.0;
                    zeroEncoderHere();
                    homeState = HomeState.HOMED;
                    break;
                }

                // Walk outward alternating sides: 0, +s, -s, +2s, -2s, ...
                if (searchDir > 0) {
                    searchOffsetDeg = Math.abs(searchOffsetDeg) + SEARCH_STEP_DEG;
                    searchDir = -1;
                } else {
                    searchOffsetDeg = -Math.abs(searchOffsetDeg);
                    searchDir = +1;
                }
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
    }

    private void zeroEncoderHere() {
        encoderZeroTicks = robot.intakeMot.getCurrentPosition();
        encoderZeroed = true;
    }

    public HomeState getHomeState() { return homeState; }
    public boolean isHomed() { return homeState == HomeState.HOMED; }

    /** Human-readable prompt for the driver station. */
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


