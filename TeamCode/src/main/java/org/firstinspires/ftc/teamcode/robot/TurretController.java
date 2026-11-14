package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class TurretController {

    // ---- mechanism, fixed ------------------------------------------------
    public static final double SERVO_TRAVEL_DEG  = 1800.0;   // 5-turn servo
    public static final double SERVO_PER_TURRET  = 2.0;      // ASSUMED 2:1

    // Degrees of TURRET per 1.0 of servo position. MEASURED, not derived.
    //
    // It used to be SERVO_TRAVEL_DEG / SERVO_PER_TURRET = 900, which assumed
    // a 5-turn servo and 2:1 gearing. On the built robot, sweeping the
    // commanded band 0.289..0.711 -- 0.422 of servo range -- moves the turret
    // 190 deg, not the 380 that implies. So the real figure is 450: either
    // the gearing is 4:1 or the servo is not 5-turn.
    //
    // Every commanded angle scales with this. At 900 the turret moved HALF
    // what was asked and nothing said so -- a 30 deg sign test produced 15.
    // 13 Nov, second measurement, taken with BOTH servo positions and encoder
    // ticks at the stops: servo 0.24..0.57 (0.330) against 5447 ticks
    // (239.4 deg) gives 725 deg per 1.0 of servo.
    //
    // CONFIRMED 14 Nov by a run that captured all three points together:
    //     + limit (LEFT)    servo 0.23   encoder  4790
    //     zero              servo 0.41   encoder  1816
    //     - limit (RIGHT)   servo 0.56   encoder  -808
    //
    // Each half gives its own ratio, and they agree:
    //     zero -> +limit  2974 ticks = 130.7 deg over 0.180 -> 726 deg/unit
    //     zero -> -limit  2624 ticks = 115.3 deg over 0.150 -> 769 deg/unit
    //
    // Earlier runs looked wildly inconsistent (1150 vs 260 deg/unit) only
    // because the home reading came from a DIFFERENT OpMode run. init does
    // STOP_AND_RESET_ENCODER, so absolute ticks are meaningless across runs
    // -- compare them only within one.
    public static double TURRET_TRAVEL_DEG = 745.0;

    // Half the turret's usable travel. MEASURED on the built robot, 13 Nov,
    // from the encoder at both mechanical stops: 4707 ticks = 206.9 deg total.
    //
    // But that travel is NOT centred on the home switch:
    //     home -> low end    2142 ticks =  94.1 deg
    //     home -> high end   2565 ticks = 112.7 deg
    //
    // This constant is symmetric (+/-LIMIT), so it is capped by the TIGHT
    // side, not by half the span.
    //
    // From zero (servo 0.4100), 14 Nov:
    //     +130.7 deg to the LEFT limit,  -115.3 deg to the RIGHT limit
    //
    // Asymmetric, and this constant is symmetric (+/-LIMIT), so the TIGHT
    // side caps it: 105 leaves 10.3 deg of margin on the right and never
    // loads a stop in a match.
    //
    // It pairs with TURRET_TRAVEL_DEG: the band is SERVO_AT_ZERO +/-
    // LIMIT/TRAVEL, so fixing one without the other moves the ends the wrong
    // way. Re-check after HOME_OFFSET_DEG is set, because moving zero to true
    // forward changes how the asymmetry falls.
    public static double TURRET_LIMIT_DEG = 105.0;

    public static double ENCODER_TICKS_PER_REV = 8192.0;

    // 0.4100: the servo position at which the home magnet trips, measured
    // 13 Nov. Homing adopts whatever it finds, so this is only the starting
    // guess -- but a guess 0.09 out shifted the whole band and made the first
    // search start from the wrong place.
    public static double SERVO_AT_ZERO = 0.4100;

    // -1, from the direction labels taken 14 Nov: the + limit is the turret
    // shooting LEFT, at servo 0.23, which is BELOW zero (0.4100). Positive
    // degrees therefore DECREASE the servo position.
    //
    // This was +1, so every commanded angle aimed at the MIRROR of the target
    // -- the one turret fault that nothing else reveals, because the shot is
    // perfectly executed in the wrong direction.
    public static int SERVO_SIGN = -1;

    // +1: at the + limit (left) the encoder reads 4790, against -808 at the
    // other end, so ticks increase with positive degrees. The -1 measured on
    // step 22 was taken before the travel scale was corrected and against a
    // step that moved half what it claimed; this comes from the limits
    // themselves and supersedes it.
    public static int ENCODER_SIGN = +1;
    public static boolean LIMIT_TRIGGERED_STATE = false;

    // ---- tuning ----------------------------------------------------------
    // Which side the search tries FIRST. It alternates either way --
    // 0, +s, -s, +2s, -2s -- so it finds the magnet whichever this is. But a
    // turret carries its own harness, and sweeping away from the magnet first
    // doubles the travel and winds the cables the wrong way before it comes
    // back. Set this toward the side the switch is actually on.
    // +1 from 13 Nov. On this robot that is COUNTER-CLOCKWISE seen from
    // above, which is the short way to the magnet; -1 swept clockwise and
    // went the long way round, winding the harness first.
    //
    // The sign-to-direction mapping is not fixed: it goes through
    // SERVO_SIGN, which step 8 measures. If SERVO_SIGN is ever flipped, this
    // flips with it. Y on the HOME step changes it live.
    public static int SEARCH_FIRST_DIR = +1;

    // 85: the magnet sits well off the stored zero, and 30 could not reach it
    // -- homing reported FAILED with the cause nowhere on screen.
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

        // A 5-turn servo only reaches its full 1800 deg if the PWM range is
        // widened. With the SDK default it will travel roughly a third of
        // that, and every commanded angle will come up short.
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    // ---- aiming ----------------------------------------------------------

    // The home switch is a repeatable reference MARK, not the zero you want.
    // Nothing guarantees the magnet lands exactly at robot-forward, and on the
    // built robot it does not: homing leaves the turret about 45 deg off.
    //
    // This is the turret angle AT THE SWITCH, measured from true forward.
    // Homing adopts it, so forward stays forward however the magnet is
    // mounted -- and moving the magnet later only changes this one number.
    //
    // Sign follows SERVO_SIGN, so do not derive it: set a magnitude, re-home,
    // and look. If the turret ends up twice as far out, flip the sign.
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
    //
    // Open loop asks the servo for a position and assumes it arrives. On this
    // robot it does not: a fast 95 deg ramp tracks, but a slow 22.5 deg creep
    // delivers about 1 deg -- slip, deadband, compliance, take your pick. All
    // three are invisible to an open loop and all three are corrected by the
    // same thing: there is an encoder on the turret OUTPUT, so measure where
    // it actually is and trim the command until it agrees.
    //
    // servoTrim is that correction. It is an offset on top of the open-loop
    // position, NOT a replacement for it, so a sane command is still issued
    // even if the loop is off or the encoder stops reading.
    public static boolean CLOSED_LOOP = false;      // off until signs are verified
    public static double CL_KP        = 0.35;       // fraction of error per loop
    public static double CL_TOL_DEG   = 0.75;       // inside this, stop correcting
    public static double CL_MAX_TRIM  = 0.080;      // servo units, 36 deg of authority

    private double servoTrim = 0.0;
    private boolean clSaturated = false;

    private long moveStartMs = 0;
    private double lastMoveDeg = 0;

    public void setTurretDeg(double turretDeg) {
        double next = clampTurret(turretDeg);
        double delta = Math.abs(next - commandedDeg);
        // Only restart the clock on a real move, so holding a target steady
        // does not keep resetting the settle timer.
        if (delta > 0.25) {
            lastMoveDeg = delta;
            moveStartMs = System.currentTimeMillis();
            // A new target invalidates the trim earned at the old one. Keeping
            // it would carry one position's slip into the next.
            servoTrim = 0.0;
            clSaturated = false;
        }
        commandedDeg = next;
        writeServo();
    }

    private void writeServo() {
        double pos = servoPositionFor(commandedDeg) + servoTrim;
        servo.setPosition(pos < 0 ? 0 : (pos > 1 ? 1 : pos));
    }

    // Nudge the command until the ENCODER agrees with the target. Bounded by
    // CL_MAX_TRIM so a wrong ENCODER_SIGN cannot run away -- it saturates and
    // says so instead, which is the failure you want from a loop whose sign
    // you have not yet proved.
    private void serviceClosedLoop() {
        if (!CLOSED_LOOP || !ENCODER_COUPLED || !encoderZeroed) return;
        if (homeState != HomeState.HOMED) return;      // homing owns the servo

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
        // With the loop running, "settled" means the ENCODER agrees -- not
        // that a timer expired. A clock cannot tell you the turret slipped.
        if (CLOSED_LOOP && ENCODER_COUPLED && encoderZeroed) {
            double s = getSlipDeg();
            return Double.isNaN(s) || Math.abs(s) <= ARRIVED_TOLERANCE_DEG;
        }
        if (!ENCODER_COUPLED) return true;            // bench: trust the clock
        double slip = getSlipDeg();
        if (Double.isNaN(slip)) return true;          // not zeroed yet
        return Math.abs(slip) <= ARRIVED_TOLERANCE_DEG;
    }

    public double getCommandedDeg() { return commandedDeg; }

    public double getMeasuredDeg() {
        if (!encoderZeroed) return Double.NaN;
        int ticks = robot.intakeMot.getCurrentPosition() - encoderZeroTicks;
        // The encoder was zeroed AT THE SWITCH, where the turret really sits
        // at HOME_OFFSET_DEG. Without this term getSlipDeg() would read a
        // permanent HOME_OFFSET_DEG of slip and isSettled() would never agree.
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
                    beginSearch();
                }
                break;

            case SEARCHING: {
                if (now - lastStepMs < SEARCH_STEP_MS) break;
                lastStepMs = now;

                if (isAtHomeSwitch()) {
                    // The offset we had to add IS the correction to the
                    // stored constant. Fold it in so it is right from now on.
                    adoptHomeAt(servoPositionFor(searchOffsetDeg));
                    break;
                }

                // Walk outward, alternating sides, FIRST_DIR first:
                //   +1 -> +s, -s, +2s, -2s, ...
                //   -1 -> -s, +s, -2s, +2s, ...
                //
                // The old version toggled a flag and negated the running
                // offset, which looks equivalent and is not: starting on the
                // negative side it computed -|0| = 0, burned a step commanding
                // nowhere, and then went positive. Both settings produced the
                // SAME sweep, one step apart -- which is exactly what the
                // robot did on 13 Nov. Deriving each offset from a counter
                // cannot drift like that.
                searchStep++;
                int ring = (searchStep + 1) / 2;          // 1,1,2,2,3,3,...
                int side = (searchStep % 2 == 1)          // first, second, ...
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

    // The switch has been found at servoPosAtSwitch. The turret is therefore
    // physically at HOME_OFFSET_DEG, not at zero -- so back the stored zero
    // off by that much and record that we are standing at the offset, not at
    // forward. Compute servoPosAtSwitch BEFORE calling: it is read with the
    // OLD SERVO_AT_ZERO, which is where the servo actually is.
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
