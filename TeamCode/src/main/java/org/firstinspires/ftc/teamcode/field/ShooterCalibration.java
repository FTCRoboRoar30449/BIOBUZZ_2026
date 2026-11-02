package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.HoodMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Measures the two numbers BioBuzzShooterRed/Blue cannot derive: the shooter
 * EFFICIENCY (ball exit speed / wheel surface speed) and the ball's DRAG
 * COEFFICIENT Cd.
 *
 * WHY BOTH AT ONCE
 *   Efficiency sets how fast the ball leaves for a given RPM; Cd sets how
 *   quickly it slows down. Both move the landing point, so no single shot can
 *   tell them apart -- a slow ball with little drag lands where a fast ball
 *   with lots of drag does. They separate only across a SPREAD of hood angles
 *   and speeds, because drag bites harder on long, fast flights than on short
 *   lobbed ones. Hence: take shots across the whole hood range at two or three
 *   speeds each, then fit both together.
 *
 * WHAT TO EXPECT
 *   Your hood travels 55-80 deg, which is the WORST regime for pinning down
 *   Cd -- steep lobs spend little time at speed, so drag barely shows. Expect
 *   efficiency to come out tight and Cd to stay loose. That is fine: efficiency
 *   is the one that scales every RPM. If Cd refuses to converge, pin it at 0.55
 *   and trust the efficiency number.
 *
 * MEASUREMENT
 *   Fire onto bare tiles and mark where the ball FIRST touches down, not where
 *   it stops. Dust the ball with chalk or flour so the first contact leaves a
 *   mark. Measure from the point on the floor directly below the exit, along
 *   the ground, in inches.
 *
 * CONTROLS
 *   dpad up/down     move between fields
 *   dpad left/right  change the selected field
 *   right bumper     spin up / stop flywheel
 *   A                fire
 *   B                record the sample at the entered distance
 *   X                delete the last sample
 *   Y                run the fit (takes a few seconds)
 *   back             dump all samples as CSV to telemetry
 *
 * This version uses the exact hardware names from RobotHardware. Both shooter
 * motors must be configured with their built-in encoders connected. Samples
 * are recorded using measured encoder RPM, never the commanded RPM.
 */
@TeleOp(name = "BioBuzz Shooter Calibration", group = "calibration")
public class ShooterCalibration extends LinearOpMode {

    // ---- CONFIG: set these to match your robot ---------------------------
    public static String POLLEN_MOTOR = "pollenShootingMot";
    public static String NECTAR_MOTOR = "nectarShootingMot";
    public static String POLLEN_HOOD  = "hoodPollen";
    public static String NECTAR_HOOD  = "hoodNectar";

    /**
     * goBILDA 5000 Series bare motor encoder resolution.
     * The built-in quadrature encoder reports 28 countable events per
     * revolution of this motor's output shaft (no gearbox on this motor).
     * If you add gearing between the motor and flywheel, include that ratio.
     */
    public static double TICKS_PER_REV = 28.0;

    /**
     * MEASURED on the bench, mirroring RobotHardware. These replace P 65 / F
     * 13.6, which were the pre-measurement guesses: P 65 is roughly 50x the
     * standard recipe and made the velocity loop oscillate, so every recorded
     * RPM would have been taken while the wheel was still hunting.
     *
     * Recipe from the measured 2900 ticks/s free-run ceiling:
     *     F = 32767 / 2900 = 11.30    P = 0.1 F    I = 0.1 P    D = 0
     */
    public static double VELOCITY_PIDF_P = 1.13;
    public static double VELOCITY_PIDF_I = 0.113;
    public static double VELOCITY_PIDF_D = 0.0;
    public static double VELOCITY_PIDF_F = 11.30;

    // ---- physical constants (must match the shooter classes) -------------
    public static final double WHEEL_DIA_M = 0.072;
    public static final double AIR_DENSITY = 1.225;
    public static final double G           = 9.81;
    public static final double DT          = 0.0015;
    public static final double IN_PER_M    = 39.3700787;

    /** POLLEN: 2.8 in, 26 g. NECTAR: 3.6 in, 42 g. */
    private static double ballDiaM(boolean nectar)  { return (nectar ? 3.6 : 2.8) / IN_PER_M; }
    private static double ballMassKg(boolean nectar){ return nectar ? 0.042 : 0.026; }

    /** Exit height above the floor, inches, from the CAD hood map. */
    private static double exitHeightIn(boolean nectar, double hoodDeg) {
        double a55 = nectar ? 16.89 : 16.26;
        double a80 = nectar ? 15.62 : 15.15;
        return a55 + (hoodDeg - 55.0) * (a80 - a55) / 25.0;
    }

    // ---- a recorded shot --------------------------------------------------
    private static class Sample {
        final boolean nectar;
        final double hoodDeg, rpm, measuredIn;
        Sample(boolean n, double h, double r, double m) {
            nectar = n; hoodDeg = h; rpm = r; measuredIn = m;
        }
    }

    private final List<Sample> samples = new ArrayList<Sample>();

    // ---- UI state ---------------------------------------------------------
    private int field = 0;                       // 0 ball, 1 hood, 2 rpm, 3 distance, 4 Cd
    private boolean nectar = false;
    private double hoodDeg = 70.0;
    private double targetRpm = 2650.0;
    private double measuredIn = 0.0;
    private double cd = 0.55;   // pinned; see runCdDiagnostic()
    private boolean spinning = false;
    private String fitResult = "(not run)";

    private DcMotorEx pollenMotor, nectarMotor;
    private Servo pollenHood, nectarHood;

    @Override
    public void runOpMode() {
        pollenMotor = tryMotor(POLLEN_MOTOR);
        nectarMotor = tryMotor(NECTAR_MOTOR);
        pollenHood  = tryServo(POLLEN_HOOD);
        nectarHood  = tryServo(NECTAR_HOOD);

        telemetry.addLine("BioBuzz shooter calibration");
        telemetry.addLine(pollenMotor == null
                ? "ERROR: missing motor: " + POLLEN_MOTOR
                : "Pollen motor found: " + POLLEN_MOTOR);
        telemetry.addLine(nectarMotor == null
                ? "ERROR: missing motor: " + NECTAR_MOTOR
                : "Nectar motor found: " + NECTAR_MOTOR);
        telemetry.addLine(pollenHood == null
                ? "WARNING: missing servo: " + POLLEN_HOOD
                : "Pollen hood found: " + POLLEN_HOOD);
        telemetry.addLine(nectarHood == null
                ? "WARNING: missing servo: " + NECTAR_HOOD
                : "Nectar hood found: " + NECTAR_HOOD);

        // Use encoder-based RUN_USING_ENCODER velocity control on both motors.
        configureShooterMotor(pollenMotor);
        configureShooterMotor(nectarMotor);
        telemetry.update();

        waitForStart();

        boolean pUp=false, pDown=false, pLeft=false, pRight=false;
        boolean pA=false, pB=false, pX=false, pY=false, pRB=false, pBack=false, pLB=false;

        while (opModeIsActive()) {
            boolean up=gamepad1.dpad_up, down=gamepad1.dpad_down;
            boolean left=gamepad1.dpad_left, right=gamepad1.dpad_right;
            boolean a=gamepad1.a, b=gamepad1.b, x=gamepad1.x, y=gamepad1.y;
            boolean rb=gamepad1.right_bumper, back=gamepad1.back;
            boolean lb=gamepad1.left_bumper;

            if (up && !pUp)     field = (field + 4) % 5;
            if (down && !pDown) field = (field + 1) % 5;
            if (left && !pLeft)   adjust(-1);
            if (right && !pRight) adjust(+1);

            if (rb && !pRB) { spinning = !spinning; applyFlywheel(); }
            if (a && !pA)   fire();
            if (b && !pB && measuredIn > 0) {
                DcMotorEx selectedMotor = activeMotor();
                double measuredRpm = actualRpm();
                if (selectedMotor == null) {
                    fitResult = "Cannot record: selected shooter motor is missing";
                } else if (!Double.isFinite(measuredRpm) || measuredRpm < 0.0) {
                    fitResult = "Cannot record: encoder RPM is invalid";
                } else {
                    samples.add(new Sample(nectar, hoodDeg, measuredRpm, measuredIn));
                    fitResult = "Recorded actual encoder RPM: " + fmt(measuredRpm);
                }
            }
            if (x && !pX && !samples.isEmpty()) samples.remove(samples.size() - 1);
            if (y && !pY) fitResult = runFit();
            if (lb && !pLB) runCdDiagnostic();
            if (back && !pBack) dumpCsv();

            pUp=up; pDown=down; pLeft=left; pRight=right;
            pA=a; pB=b; pX=x; pY=y; pRB=rb; pBack=back; pLB=lb;

            applyHood();
            draw();
        }
        stopFlywheel();
    }

    // ---- UI ---------------------------------------------------------------

    private void adjust(int dir) {
        switch (field) {
            case 0: nectar = !nectar; break;
            case 1: hoodDeg = clamp(hoodDeg + 5.0 * dir, 55.0, 80.0); break;
            case 2: targetRpm = clamp(targetRpm + 25.0 * dir, 0.0, 6000.0);
                if (spinning) applyFlywheel(); break;
            case 3: measuredIn = clamp(measuredIn + 1.0 * dir, 0.0, 400.0); break;
            case 4: cd = clamp(cd + 0.01 * dir, 0.30, 0.90); break;
        }
    }

    private void draw() {
        telemetry.addLine(sel(0) + "ball      : " + (nectar ? "NECTAR" : "POLLEN"));
        telemetry.addLine(sel(1) + "hood      : " + fmt(hoodDeg) + " deg"
                + "   exit " + fmt(exitHeightIn(nectar, hoodDeg)) + " in");
        double rpmNow = actualRpm();
        telemetry.addLine(sel(2) + "rpm target: " + fmt(targetRpm)
                + "   actual " + (Double.isFinite(rpmNow) ? fmt(rpmNow) : "NO ENCODER"));
        telemetry.addLine(sel(3) + "distance  : " + fmt(measuredIn) + " in");
        telemetry.addLine(sel(4) + "Cd (pinned): " + String.format(Locale.US, "%.2f", cd));
        telemetry.addLine("");
        telemetry.addLine("flywheel " + (spinning ? "SPINNING" : "stopped")
                + "   samples " + samples.size());
        telemetry.addLine("fit: " + fitResult);
        telemetry.addLine("");
        telemetry.addLine("RB spin  A fire  B record  X undo  Y fit  LB Cd-sweep  back CSV");
        telemetry.update();
    }

    private String sel(int i) { return (field == i) ? "> " : "  "; }
    private static String fmt(double v) { return String.format(Locale.US, "%.1f", v); }
    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    // ---- hardware ---------------------------------------------------------

    private DcMotorEx tryMotor(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); } catch (Exception e) { return null; }
    }
    private Servo tryServo(String name) {
        try { return hardwareMap.get(Servo.class, name); } catch (Exception e) { return null; }
    }

    private DcMotorEx activeMotor() { return nectar ? nectarMotor : pollenMotor; }
    private Servo activeHood()      { return nectar ? nectarHood  : pollenHood;  }

    /** Configure encoder-based velocity control and the tunable PIDF values. */
    private void configureShooterMotor(DcMotorEx motor) {
        if (motor == null) return;
        motor.setDirection(DcMotorSimple.Direction.REVERSE); // matches RobotHardware
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(
                VELOCITY_PIDF_P, VELOCITY_PIDF_I,
                VELOCITY_PIDF_D, VELOCITY_PIDF_F);
    }

    /** Actual flywheel RPM from the selected motor's encoder; never substitute target RPM. */
    private double actualRpm() {
        DcMotorEx m = activeMotor();
        if (m == null || TICKS_PER_REV <= 0.0) return Double.NaN;
        return m.getVelocity() / TICKS_PER_REV * 60.0;
    }

    private void applyFlywheel() {
        DcMotorEx m = activeMotor();
        if (m == null) return;
        // DcMotorEx velocity is in encoder ticks/second.
        m.setVelocity(spinning ? targetRpm * TICKS_PER_REV / 60.0 : 0.0);
    }

    private void stopFlywheel() {
        spinning = false;
        if (pollenMotor != null) pollenMotor.setVelocity(0);
        if (nectarMotor != null) nectarMotor.setVelocity(0);
    }

    /**
     * Hood positions come from HoodMap, not from a local 0..1 ramp.
     *
     * This used to interpolate HOOD_SERVO_AT_55=0.00 to HOOD_SERVO_AT_80=1.00
     * for BOTH hoods. Two things were wrong with that. It drove the full servo
     * range across 25 deg of hood, when the gearing needs only 0.110 (pollen)
     * and 0.129 (nectar). And it used ONE direction for both, when the hoods
     * are mirrored and measured OPPOSITE: raising the pollen servo lowers its
     * angle, raising the nectar servo raises its angle.
     *
     * Calibrating against a hood map that disagrees with the one the robot
     * flies would produce an efficiency number that is wrong by whatever the
     * two maps differ by -- the exact failure this OpMode exists to avoid.
     */
    private void applyHood() {
        Servo s = activeHood();
        if (s == null) return;
        s.setPosition(nectar ? HoodMap.nectarPos(hoodDeg) : HoodMap.pollenPos(hoodDeg));
    }

    /** Hook your feeder here. Left as a no-op so the OpMode is safe as shipped. */
    private void fire() {
        // e.g. feederServo.setPosition(FEED); sleep(250); feederServo.setPosition(REST);
    }

    // ---- ballistics -------------------------------------------------------

    /**
     * Horizontal distance, inches, from the exit point to where the ball first
     * reaches floor level. Same integrator and drag law as the shot tables.
     */
    public static double flightDistanceIn(double v0, double hoodDeg, double exitHIn,
                                          double diaM, double massKg, double cd) {
        double th = Math.toRadians(hoodDeg);
        double k  = 0.5 * AIR_DENSITY * cd * Math.PI * Math.pow(diaM / 2.0, 2) / massKg;
        double x = 0.0, z = exitHIn / IN_PER_M;
        double vx = v0 * Math.cos(th), vz = v0 * Math.sin(th);
        double px = 0.0, pz = z;
        for (int i = 0; i < 20000; i++) {
            double sp = Math.hypot(vx, vz);
            vx += -k * sp * vx * DT;
            vz += (-G - k * sp * vz) * DT;
            px = x; pz = z;
            x += vx * DT; z += vz * DT;
            if (z <= 0.0) {
                double f = (pz == z) ? 0.0 : pz / (pz - z);   // interpolate the crossing
                return (px + f * (x - px)) * IN_PER_M;
            }
        }
        return -1;
    }

    /** Ball exit speed, m/s, for a commanded RPM at a given efficiency. */
    public static double exitSpeed(double rpm, double efficiency) {
        return Math.PI * WHEEL_DIA_M * rpm * efficiency / 60.0;
    }

    // ---- the fit ----------------------------------------------------------

    /**
     * Sum of squared range error, inches, for one (efficiency, Cd) pair.
     */
    private double sse(double eff, double cd) {
        double err = 0.0;
        for (int i = 0; i < samples.size(); i++) {
            Sample s = samples.get(i);
            double v = exitSpeed(s.rpm, eff);
            double d = flightDistanceIn(v, s.hoodDeg,
                    exitHeightIn(s.nectar, s.hoodDeg),
                    ballDiaM(s.nectar), ballMassKg(s.nectar), cd);
            if (d < 0) return Double.MAX_VALUE;
            double r = d - s.measuredIn;
            err += r * r;
        }
        return err;
    }

    /**
     * Best efficiency with Cd held fixed. Coarse pass then refinement.
     * @return {efficiency, sumSquaredError}
     */
    public double[] fitEfficiency(double cd) {
        double bE = 0.2, bErr = Double.MAX_VALUE;
        for (double e = 0.20; e <= 1.00 + 1e-9; e += 0.01) {
            double err = sse(e, cd);
            if (err < bErr) { bErr = err; bE = e; }
        }
        double lo = Math.max(0.10, bE - 0.015), hi = bE + 0.015;
        for (double e = lo; e <= hi + 1e-9; e += 0.0005) {
            double err = sse(e, cd);
            if (err < bErr) { bErr = err; bE = e; }
        }
        return new double[] { bE, bErr };
    }

    /**
     * Fit efficiency at the Cd currently selected. This is the number that
     * matters: it scales every RPM, and it comes out within about +/-3% no
     * matter which Cd you pin, so it does not wait on Cd being right.
     */
    private String runFit() {
        if (samples.size() < 3) return "need at least 3 samples";
        double[] f = fitEfficiency(cd);
        double rms = Math.sqrt(f[1] / samples.size());
        return String.format(Locale.US, "eff %.3f at Cd %.2f   rms %.2f in  (n=%d)",
                f[0], cd, rms, samples.size());
    }

    /**
     * Sweep Cd, re-fitting efficiency at each, and show how little the fit
     * quality actually changes. If rms is flat across the sweep, your data
     * cannot identify Cd -- leave it at 0.55 and use the efficiency.
     *
     * On synthetic noise-free data the rms signal across Cd 0.35..0.90 is only
     * about 0.6 in end to end, while +/-1 in of tape-measure noise produces
     * roughly 0.5 in of rms on its own. So expect it to be flat, and do not
     * read a Cd off a flat curve.
     */
    private void runCdDiagnostic() {
        if (samples.size() < 3) { fitResult = "need at least 3 samples"; return; }
        telemetry.clearAll();
        telemetry.addLine("Cd sweep -- efficiency re-fitted at each Cd");
        telemetry.addLine("   Cd     eff     rms(in)");
        double best = Double.MAX_VALUE, worst = 0, bestCd = 0.55;
        StringBuilder sb = new StringBuilder();
        for (double c = 0.35; c <= 0.90 + 1e-9; c += 0.05) {
            double[] f = fitEfficiency(c);
            double rms = Math.sqrt(f[1] / samples.size());
            if (rms < best) { best = rms; bestCd = c; }
            if (rms > worst) worst = rms;
            sb.append(String.format(Locale.US, "  %.2f   %.3f   %.2f\n", c, f[0], rms));
            telemetry.addLine(String.format(Locale.US, "  %.2f   %.3f   %.2f", c, f[0], rms));
            telemetry.update();
            if (!opModeIsActive()) return;
        }
        double spread = worst - best;
        telemetry.addLine("");
        telemetry.addLine(String.format(Locale.US,
                "rms spread %.2f in -> Cd is %s", spread,
                spread < 1.0 ? "NOT identifiable; keep 0.55" : "weakly identifiable"));
        telemetry.update();
        fitResult = String.format(Locale.US, "Cd sweep: best %.2f, rms spread %.2f in",
                bestCd, spread);
        sleep(6000);
    }

    private void dumpCsv() {
        telemetry.clearAll();
        telemetry.addLine("ball,hoodDeg,rpm,measuredIn");
        for (int i = 0; i < samples.size(); i++) {
            Sample s = samples.get(i);
            telemetry.addLine(String.format(Locale.US, "%s,%.0f,%.0f,%.1f",
                    s.nectar ? "NECTAR" : "POLLEN", s.hoodDeg, s.rpm, s.measuredIn));
        }
        telemetry.update();
        sleep(4000);
    }
}

/*
 * ---------------------------------------------------------------------------
 * HOW TO RUN IT
 *
 * 1. Set TICKS_PER_REV and the hood servo limits, and hook fire() to your
 *    feeder. Until fire() is wired, trigger the shot by hand and still record.
 *
 * 2. Take at least 8-12 shots spread over the WHOLE envelope:
 *       hood 55, 65, 75, 80  x  two or three RPMs each.
 *    Shots clustered at one hood or one speed cannot separate efficiency from
 *    Cd -- the fit will return something, and it will be meaningless.
 *
 * 3. Mark FIRST floor contact, not where the ball stops rolling.
 *
 * 4. Press Y. Read eff and Cd. Put eff into BioBuzzShooterRed/Blue as
 *    `efficiency`. Cd feeds the trajectory model that generated the tables --
 *    if it comes back far from 0.55, the tables want regenerating.
 *
 * 5. Sanity: rms should be a couple of inches. Much more means the
 *    measurements, the hood map, or the RPM reading disagree with the model.
 *
 * ---------------------------------------------------------------------------
 * THE MORE DIRECT ALTERNATIVE
 *
 * If all you want is to SCORE, you do not need either number. Park at a known
 * range, sweep RPM until the ball goes in, record (range, hood, RPM), repeat
 * every few inches, and use that table instead of the modelled one. That is
 * strictly more accurate than any fit, because it absorbs every effect the
 * model leaves out -- ball wear, wheel compression, spin, air. Fitting eff and
 * Cd is worth doing when you want the model to EXTRAPOLATE to ranges you have
 * not measured.
 * ---------------------------------------------------------------------------
 */