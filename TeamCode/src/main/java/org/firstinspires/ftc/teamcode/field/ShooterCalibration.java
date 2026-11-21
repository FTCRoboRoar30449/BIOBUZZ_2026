package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.HoodMap;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.TurretController;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "BioBuzz Shooter Calibration", group = "calibration")
public class ShooterCalibration extends LinearOpMode {

    // ---- CONFIG: set these to match your robot ---------------------------
    public static String POLLEN_MOTOR = "pollenShootingMot";
    public static String NECTAR_MOTOR = "nectarShootingMot";
    public static String POLLEN_HOOD  = "hoodPollen";
    public static String NECTAR_HOOD  = "hoodNectar";
    public static String CONVEYOR_MOTOR = "conveyorMot";
    public static String TURRET_SERVO   = "turretRot";
    public static String POLLEN_BED     = "conveyorPollen";
    public static String NECTAR_BED     = "conveyorNectar";

    // Let the servo reach the bed before the belt pushes a ball at it.
    public static long FEED_SETTLE_MS = 150;
    // A is hold-to-run: counted strokes do not clear the tunnel. Releasing A
    // stops the belt; this is only the backstop if A is held down.
    public static long FEED_MAX_MS = 3000;

    // Measure to the FRONT BUMPER FACE instead of plumbing down from the
    // hood. The flight model starts at the ball's exit point, so the gap
    // between the two is added back before fitting.
    //
    // Datum is RobotHardware.BUMPER_FWD_IN, measured 17 Nov. A wrong datum
    // biases every sample equally, which the fit absorbs as a wrong
    // efficiency rather than showing up as scatter.
    public static boolean MEASURE_FROM_BUMPER = true;
    public static double BUMPER_FWD_IN = RobotHardware.BUMPER_FWD_IN;

    // Exit sits ShooterRed.AXIS_FWD ahead of centre, less the hood-dependent
    // setback, so the correction shifts about half an inch across the range.
    private static double exitFwdOfCentreIn(boolean nectar, double hoodDeg) {
        return ShooterRed.AXIS_FWD - ShooterRed.setbackIn(
                nectar ? ShooterRed.Ball.NECTAR : ShooterRed.Ball.POLLEN, hoodDeg);
    }

    private static double bumperToExitIn(boolean nectar, double hoodDeg) {
        return BUMPER_FWD_IN - exitFwdOfCentreIn(nectar, hoodDeg);
    }

    private static double toFlightIn(double measured, boolean nectar, double hoodDeg) {
        return MEASURE_FROM_BUMPER ? measured + bumperToExitIn(nectar, hoodDeg) : measured;
    }

    // TAKEN FROM RobotHardware, not copied, so the rig cannot drift away
    public static double TICKS_PER_REV   = RobotHardware.SHOOTER_TICKS_PER_REV;
    public static double VELOCITY_PIDF_P = RobotHardware.SHOOTER_KP;
    public static double VELOCITY_PIDF_I = RobotHardware.SHOOTER_KI;
    public static double VELOCITY_PIDF_D = RobotHardware.SHOOTER_KD;
    public static double VELOCITY_PIDF_F = RobotHardware.SHOOTER_KF;

    // ---- physical constants (must match the shooter classes) -------------
    public static final double WHEEL_DIA_M = RobotHardware.SHOOTER_WHEEL_DIA_M;
    public static final double AIR_DENSITY = 1.225;
    public static final double G           = 9.81;
    public static final double DT          = 0.0015;
    public static final double IN_PER_M    = 39.3700787;

    private static double ballDiaM(boolean nectar)  { return (nectar ? 3.6 : 2.8) / IN_PER_M; }
    private static double ballMassKg(boolean nectar){ return nectar ? 0.042 : 0.026; }

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
    // The six shooting presets ask for pollen 3813 rpm at hood 70 and
    // nectar 3123 at hood 75. Across the whole table it is pollen 3684-3887
    // and nectar 3076-3215, always hood 70/75/80. Start on the pollen
    // working point; LEFT on the ball field switches the nectar default in.
    private double targetRpm = 3813.0;
    private double measuredIn = 0.0;
    private double cd = 0.55;   // pinned; see runCdDiagnostic()
    private boolean spinning = false;
    private boolean feeding = false;
    private long feedStartMs = 0;
    private String fitResult = "(not run)";

    private DcMotorEx pollenMotor, nectarMotor;
    private Servo pollenHood, nectarHood;
    private DcMotorEx conveyorMotor;
    private Servo pollenBed, nectarBed;
    private Servo turretServo;

    @Override
    public void runOpMode() {
        pollenMotor = tryMotor(POLLEN_MOTOR);
        nectarMotor = tryMotor(NECTAR_MOTOR);
        pollenHood  = tryServo(POLLEN_HOOD);
        nectarHood  = tryServo(NECTAR_HOOD);
        // This OpMode never aims. If the turret is left off zero the ball
        // flies sideways and the measured distance is meaningless, so park
        // it straight ahead and say so.
        turretServo = tryServo(TURRET_SERVO);
        if (turretServo != null) {
            turretServo.setPosition(TurretController.SERVO_AT_ZERO);
        }
        conveyorMotor = tryMotor(CONVEYOR_MOTOR);
        if (conveyorMotor != null) conveyorMotor.setDirection(RobotHardware.CONVEYOR_DIR);
        pollenBed   = tryServo(POLLEN_BED);
        nectarBed   = tryServo(NECTAR_BED);

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
        // Only a warning: the rig is still usable hand-fed without
        telemetry.addLine(conveyorMotor == null || pollenBed == null || nectarBed == null
                ? "WARNING: no conveyor/beds -- A will not feed, hand-feed instead"
                : "Feed ready: " + CONVEYOR_MOTOR);

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
            boolean coarse = gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5;
            if (left && !pLeft)   adjust(-1, coarse);
            if (right && !pRight) adjust(+1, coarse);

            if (rb && !pRB) { spinning = !spinning; applyFlywheel(); }
            if (a && !pA)   beginFire();
            if (feeding && (!a || System.currentTimeMillis() - feedStartMs > FEED_MAX_MS)) {
                endFire(a);
            }
            if (b && !pB && measuredIn > 0) {
                DcMotorEx selectedMotor = activeMotor();
                double measuredRpm = actualRpm();
                if (selectedMotor == null) {
                    fitResult = "Cannot record: selected shooter motor is missing";
                } else if (!Double.isFinite(measuredRpm) || measuredRpm < 0.0) {
                    fitResult = "Cannot record: encoder RPM is invalid";
                } else {
                    // Store FLIGHT distance. The fit compares against a
                    // trajectory that starts at the ball's exit point, so
                    // the bumper offset is applied once, here, rather than
                    // being carried through the model.
                    double flightIn = toFlightIn(measuredIn, nectar, hoodDeg);
                    samples.add(new Sample(nectar, hoodDeg, measuredRpm, flightIn));
                    fitResult = String.format(Locale.US,
                            "rec %.0f rpm   %.1f in from bumper -> %.1f flight",
                            measuredRpm, measuredIn, flightIn);
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

    // Hold either trigger for the coarse step. Without it a 90 in shot takes
    // 90 presses to enter.
    private void adjust(int dir, boolean coarse) {
        switch (field) {
            case 0:
                nectar = !nectar;
                // Jump to that ball's working point rather than carrying the
                // other ball's numbers across.
                targetRpm = nectar ? 3123.0 : 3813.0;
                hoodDeg   = nectar ? 75.0   : 70.0;
                break;
            case 1: hoodDeg = clamp(hoodDeg + 5.0 * dir, 55.0, 80.0); break;
            case 2: targetRpm = clamp(targetRpm + (coarse ? 100.0 : 25.0) * dir, 0.0, 6000.0);
                if (spinning) applyFlywheel(); break;
            case 3: measuredIn = clamp(measuredIn + (coarse ? 10.0 : 1.0) * dir, 0.0, 400.0); break;
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
        telemetry.addLine(sel(3) + "distance  : " + fmt(measuredIn) + " in"
                + (MEASURE_FROM_BUMPER
                   ? "  from BUMPER  (flight " + fmt(toFlightIn(measuredIn, nectar, hoodDeg)) + ")"
                   : "  from EXIT point"));
        telemetry.addLine(sel(4) + "Cd (pinned): " + String.format(Locale.US, "%.2f", cd));
        telemetry.addLine("");
        telemetry.addLine("flywheel " + (spinning ? "SPINNING" : "stopped")
                + (feeding ? "   BELT RUNNING" : "")
                + "   samples " + samples.size());
        telemetry.addLine("fit: " + fitResult);
        telemetry.addLine("");
        telemetry.addLine(turretServo == null
                ? "TURRET NOT FOUND -- check it points straight before firing"
                : "turret parked at zero: the ball goes straight ahead");
        telemetry.addLine("dpad up/dn pick  L/R change  trigger = coarse");
        telemetry.addLine("RB spin  A HOLD to feed  B record  X undo  Y fit  LB Cd-sweep  back CSV");
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

    // Mirrors RobotHardware.configureShooter().
    private void configureShooterMotor(DcMotorEx motor) {
        if (motor == null) return;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);   // both, measured 13 Nov
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Coast like the robot does.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setVelocityPIDFCoefficients(
                VELOCITY_PIDF_P, VELOCITY_PIDF_I,
                VELOCITY_PIDF_D, VELOCITY_PIDF_F);
    }

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
        // The belt too.
        stopFeed();
    }

    private void applyHood() {
        Servo s = activeHood();
        if (s == null) return;
        s.setPosition(nectar ? HoodMap.nectarPos(hoodDeg) : HoodMap.pollenPos(hoodDeg));
    }

    // Hold A: the belt runs until you let go. A counted stroke does not carry
    // the ball from the tunnel to the flywheel.
    private void beginFire() {
        if (conveyorMotor == null || pollenBed == null || nectarBed == null) {
            fitResult = "no conveyor/beds in config -- hand-feed the ball";
            return;
        }
        if (!spinning) {
            fitResult = "spin the flywheel first (RB) -- feeding a stopped wheel jams it";
            return;
        }

        // Only the selected tunnel opens.
        pollenBed.setPosition(MechController.bedPollenPos(!nectar));
        nectarBed.setPosition(MechController.bedNectarPos(nectar));
        sleep(FEED_SETTLE_MS);

        conveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyorMotor.setPower(MechController.CONVEYOR_POWER);
        feeding = true;
        feedStartMs = System.currentTimeMillis();
        fitResult = "feeding " + (nectar ? "NECTAR" : "POLLEN") + " -- release A to stop";
    }

    private void endFire(boolean hitLimit) {
        long ran = System.currentTimeMillis() - feedStartMs;
        feeding = false;
        stopFeed();
        restBeds();
        fitResult = (hitLimit ? "FEED STOPPED at the " + FEED_MAX_MS + " ms limit -- jammed?"
                              : "fed " + (nectar ? "NECTAR" : "POLLEN"))
                + String.format(Locale.US, " (%d ms)", ran);
    }

    private void stopFeed() {
        if (conveyorMotor == null) return;
        conveyorMotor.setPower(0);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Both beds back up, which is where MechController leaves them after
    private void restBeds() {
        if (pollenBed != null) pollenBed.setPosition(MechController.bedPollenPos(true));
        if (nectarBed != null) nectarBed.setPosition(MechController.bedNectarPos(true));
    }

    // ---- ballistics -------------------------------------------------------

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

    public static double exitSpeed(double rpm, double efficiency) {
        return Math.PI * WHEEL_DIA_M * rpm * efficiency / 60.0;
    }

    // ---- the fit ----------------------------------------------------------

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

    private String runFit() {
        if (samples.size() < 3) return "need at least 3 samples";
        double[] f = fitEfficiency(cd);
        double rms = Math.sqrt(f[1] / samples.size());
        return String.format(Locale.US, "eff %.3f at Cd %.2f   rms %.2f in  (n=%d)",
                f[0], cd, rms, samples.size());
    }

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
