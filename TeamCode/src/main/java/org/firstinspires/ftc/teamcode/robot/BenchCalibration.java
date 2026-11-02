package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Locale;

/**
 * Finds the five constants every other class is waiting on. Nothing here
 * guesses -- each mode makes the robot do something and you read the answer
 * off the driver station.
 *
 *   1. LIMIT POLARITY   does the switch read true or false at home?
 *   2. TURRET SIGN      does rising servo position turn the turret CW?
 *   3. ENCODER SIGN     does the encoder count up when the turret goes CW?
 *      and SCALE        confirms 8192 ticks/rev and the 1:1 ratio
 *   4. TURRET ZERO      the servo position with the shooters facing forward
 *   5. HOOD ZERO        the servo position with each hood at 55 deg
 *
 * Work through them in order -- later modes rely on earlier answers. Copy each
 * result into TurretController or HoodMap as you go.
 *
 * SAFETY: the turret moves in small steps and never near its mechanical stops.
 * Modes 2-4 move it a few degrees at a time; keep hands clear anyway.
 *
 * CONTROLS
 *   dpad left/right   change mode
 *   dpad up/down      jog the thing this mode is about (fine)
 *   bumpers           jog coarse
 *   A                 accept / record the current value
 *   Y                 run this mode's automatic test
 *   B                 reset this mode
 *   back              dump every result as copy-paste constants
 */
@TeleOp(name = "Bench Calibration", group = "calibration")
public class BenchCalibration extends LinearOpMode {

    private enum Mode {
        LIMIT_POLARITY, TURRET_SIGN, ENCODER_SIGN, TURRET_ZERO, HOOD_POLLEN, HOOD_NECTAR
    }

    private RobotHardware robot;
    private Mode mode = Mode.LIMIT_POLARITY;

    // live servo commands
    private double turretPos = 0.500;
    private double pollenHoodPos = 0.100;
    private double nectarHoodPos = 0.100;

    // results as discovered
    private Boolean limitAtHome = null;
    private Integer turretSign = null;
    private Integer encoderSign = null;
    private Double turretZero = null;
    private Double pollenAt55 = null;
    private Double nectarAt55 = null;
    private Double encoderTicksPerRev = null;

    private String note = "";
    private int encStart = 0;
    private double posStart = 0;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap, telemetry);
        widen(robot.turretRot);
        widen(robot.hoodPollen);
        widen(robot.hoodNectar);

        telemetry.addLine("Bench calibration ready.");
        telemetry.addLine("Work through the modes left to right.");
        telemetry.update();
        waitForStart();

        boolean pL=false,pR=false,pU=false,pD=false,pA=false,pY=false,pB=false,pBk=false,pLB=false,pRB=false;

        while (opModeIsActive()) {
            boolean l=gamepad1.dpad_left, r=gamepad1.dpad_right;
            boolean u=gamepad1.dpad_up,  d=gamepad1.dpad_down;
            boolean a=gamepad1.a, y=gamepad1.y, b=gamepad1.b, bk=gamepad1.back;
            boolean lb=gamepad1.left_bumper, rb=gamepad1.right_bumper;

            if (l && !pL) mode = prev(mode);
            if (r && !pR) mode = next(mode);
            if (u && !pU) jog(+1, 0.0005);
            if (d && !pD) jog(-1, 0.0005);
            if (rb && !pRB) jog(+1, 0.0100);
            if (lb && !pLB) jog(-1, 0.0100);
            if (a && !pA) accept();
            if (y && !pY) runTest();
            if (b && !pB) resetMode();
            if (bk && !pBk) { dump(); }

            pL=l;pR=r;pU=u;pD=d;pA=a;pY=y;pB=b;pBk=bk;pLB=lb;pRB=rb;

            push();
            draw();
        }
        // leave everything where it is; nothing to stop
    }

    private void widen(Servo s) {
        if (s instanceof ServoImplEx) {
            ((ServoImplEx) s).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    private void push() {
        robot.turretRot.setPosition(clamp01(turretPos));
        robot.hoodPollen.setPosition(clamp01(pollenHoodPos));
        robot.hoodNectar.setPosition(clamp01(nectarHoodPos));
    }

    private void jog(int dir, double step) {
        switch (mode) {
            case HOOD_POLLEN: pollenHoodPos = clamp01(pollenHoodPos + dir * step); break;
            case HOOD_NECTAR: nectarHoodPos = clamp01(nectarHoodPos + dir * step); break;
            default:          turretPos     = clamp01(turretPos     + dir * step); break;
        }
    }

    private void accept() {
        switch (mode) {
            case LIMIT_POLARITY:
                limitAtHome = robot.magneticLimitSwitch.getState();
                note = "recorded switch reads " + limitAtHome + " AT HOME";
                break;
            case TURRET_SIGN:
                note = "use Y to run the test, not A";
                break;
            case ENCODER_SIGN:
                note = "use Y to run the test, not A";
                break;
            case TURRET_ZERO:
                turretZero = turretPos;
                note = String.format(Locale.US, "SERVO_AT_ZERO = %.4f", turretZero);
                break;
            case HOOD_POLLEN:
                pollenAt55 = pollenHoodPos;
                note = String.format(Locale.US, "POLLEN_POS_AT_55 = %.4f", pollenAt55);
                break;
            case HOOD_NECTAR:
                nectarAt55 = nectarHoodPos;
                note = String.format(Locale.US, "NECTAR_POS_AT_55 = %.4f", nectarAt55);
                break;
        }
    }

    /**
     * Moves the turret a known amount and reads the encoder, which settles the
     * two signs and checks the ticks/rev at the same time.
     */
    private void runTest() {
        if (mode != Mode.TURRET_SIGN && mode != Mode.ENCODER_SIGN) {
            note = "no automatic test in this mode";
            return;
        }
        // +10 deg of turret = +10/900 of servo position
        final double moveDeg = 10.0;
        final double dPos = moveDeg / TurretController.TURRET_TRAVEL_DEG;

        posStart = turretPos;
        encStart = robot.intakeMot.getCurrentPosition();

        turretPos = clamp01(turretPos + dPos);
        robot.turretRot.setPosition(turretPos);
        sleep(900);

        int dTicks = robot.intakeMot.getCurrentPosition() - encStart;
        double dDegMeasured = dTicks * 360.0 / TurretController.ENCODER_TICKS_PER_REV;

        if (Math.abs(dTicks) < 20) {
            note = "encoder barely moved (" + dTicks + " ticks). Encoder wired? "
                 + "PWM range widened? Servo in POSITION mode?";
            return;
        }

        encoderSign = dTicks > 0 ? +1 : -1;
        encoderTicksPerRev = Math.abs(dTicks) * 360.0 / moveDeg;

        note = String.format(Locale.US,
                "servo +%.4f -> %d ticks (%.1f deg). ENCODER_SIGN=%+d, implied ticks/rev=%.0f. "
                + "WATCH THE TURRET: if it went CW from above, SERVO_SIGN=+1 else -1.",
                dPos, dTicks, dDegMeasured, encoderSign, encoderTicksPerRev);

        turretPos = posStart;   // put it back
    }

    private void resetMode() {
        switch (mode) {
            case LIMIT_POLARITY: limitAtHome = null; break;
            case TURRET_SIGN:    turretSign = null; break;
            case ENCODER_SIGN:   encoderSign = null; encoderTicksPerRev = null; break;
            case TURRET_ZERO:    turretZero = null; break;
            case HOOD_POLLEN:    pollenAt55 = null; break;
            case HOOD_NECTAR:    nectarAt55 = null; break;
        }
        note = "cleared";
    }

    private void draw() {
        telemetry.addLine("MODE  " + mode + "   (dpad L/R to change)");
        telemetry.addLine(instructions());
        telemetry.addLine("");

        switch (mode) {
            // The signs are MEASURED now, so where 80 deg lands is known the
            // moment you pick 55. Showing it stops you committing a reference
            // whose other end falls outside 0..1 and silently clamps.
            case HOOD_POLLEN: {
                double far = pollenHoodPos + HoodMap.POLLEN_SIGN * 25.0 * HoodMap.POLLEN_POS_PER_DEG;
                telemetry.addData("pollen hood pos", "%.4f", pollenHoodPos);
                telemetry.addData("if this is 55 deg, 80 deg is", "%.4f  %s",
                        far, (far >= 0 && far <= 1) ? "OK" : "OUT OF RANGE");
                telemetry.addData("sweep needs", "%.4f  (sign %+d)",
                        25.0 * HoodMap.POLLEN_POS_PER_DEG, HoodMap.POLLEN_SIGN);
                break;
            }
            case HOOD_NECTAR: {
                double far = nectarHoodPos + HoodMap.NECTAR_SIGN * 25.0 * HoodMap.NECTAR_POS_PER_DEG;
                telemetry.addData("nectar hood pos", "%.4f", nectarHoodPos);
                telemetry.addData("if this is 55 deg, 80 deg is", "%.4f  %s",
                        far, (far >= 0 && far <= 1) ? "OK" : "OUT OF RANGE");
                telemetry.addData("sweep needs", "%.4f  (sign %+d)",
                        25.0 * HoodMap.NECTAR_POS_PER_DEG, HoodMap.NECTAR_SIGN);
                break;
            }
            default:
                telemetry.addData("turret servo pos", "%.4f", turretPos);
                telemetry.addData("turret deg (if zero set)", "%.1f",
                        (turretPos - (turretZero == null ? 0.5 : turretZero))
                                * TurretController.TURRET_TRAVEL_DEG);
                break;
        }

        telemetry.addData("limit switch raw", robot.magneticLimitSwitch.getState());
        telemetry.addData("encoder ticks", robot.intakeMot.getCurrentPosition());
        telemetry.addLine("");
        telemetry.addLine("note: " + note);
        telemetry.addLine("");
        telemetry.addLine("results so far:");
        telemetry.addLine("  LIMIT_TRIGGERED_STATE = " + show(limitAtHome));
        telemetry.addLine("  SERVO_SIGN            = " + show(turretSign));
        telemetry.addLine("  ENCODER_SIGN          = " + show(encoderSign));
        telemetry.addLine("  ticks/rev measured    = " + show(encoderTicksPerRev));
        telemetry.addLine("  SERVO_AT_ZERO         = " + show(turretZero));
        telemetry.addLine("  POLLEN_POS_AT_55      = " + show(pollenAt55));
        telemetry.addLine("  NECTAR_POS_AT_55      = " + show(nectarAt55));
        telemetry.addLine("");
        telemetry.addLine("dpad U/D fine  bumpers coarse  A accept  Y test  B clear  back dump");
        telemetry.update();
    }

    private String instructions() {
        switch (mode) {
            case LIMIT_POLARITY:
                return "Push the turret to forward by hand until the switch trips,\n"
                     + "then press A. Records what the channel reads AT home.";
            case TURRET_SIGN:
                return "Press Y. The turret steps +10 deg of COMMAND.\n"
                     + "Watch it from above: CW means SERVO_SIGN=+1, CCW means -1.\n"
                     + "Set it by hand in TurretController -- only you can see which way it went.";
            case ENCODER_SIGN:
                return "Press Y again here. Reads ENCODER_SIGN and ticks/rev\n"
                     + "from the same +10 deg move. Expect about 8192 ticks/rev.";
            case TURRET_ZERO:
                return "Jog until the limit switch trips AND the shooters look\n"
                     + "square to robot-forward, then press A.";
            case HOOD_POLLEN:
                return "Jog the POLLEN hood to exactly 55 deg from HORIZONTAL\n"
                     + "(the shallow end of travel), then press A.";
            case HOOD_NECTAR:
                return "Jog the NECTAR hood to exactly 55 deg from HORIZONTAL,\n"
                     + "then press A.";
        }
        return "";
    }

    private void dump() {
        telemetry.clearAll();
        telemetry.addLine("// ---- paste into TurretController ----");
        telemetry.addLine("SERVO_AT_ZERO = " + show(turretZero) + ";");
        telemetry.addLine("SERVO_SIGN = " + show(turretSign) + ";");
        telemetry.addLine("ENCODER_SIGN = " + show(encoderSign) + ";");
        telemetry.addLine("LIMIT_TRIGGERED_STATE = " + show(limitAtHome) + ";");
        telemetry.addLine("ENCODER_TICKS_PER_REV = " + show(encoderTicksPerRev) + ";");
        telemetry.addLine("// ---- paste into HoodMap ----");
        telemetry.addLine("POLLEN_POS_AT_55 = " + show(pollenAt55) + ";");
        telemetry.addLine("NECTAR_POS_AT_55 = " + show(nectarAt55) + ";");
        telemetry.update();
        sleep(8000);
    }

    private static String show(Object o) {
        if (o == null) return "<not set>";
        if (o instanceof Double) return String.format(Locale.US, "%.4f", (Double) o);
        return o.toString();
    }
    private static double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

    private static Mode next(Mode m) {
        Mode[] v = Mode.values();
        return v[(m.ordinal() + 1) % v.length];
    }
    private static Mode prev(Mode m) {
        Mode[] v = Mode.values();
        return v[(m.ordinal() + v.length - 1) % v.length];
    }
}
