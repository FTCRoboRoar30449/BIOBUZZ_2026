package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.util.Locale;

/**
 * Electronics bench check. Everything here works with loose motors, loose
 * servos and no mechanism attached -- it answers the questions you can settle
 * before the turret, hoods and conveyor exist.
 *
 * BenchCalibration is the other half: it needs the mechanism built. Use this
 * one first.
 *
 * CONTROLS
 *   dpad left/right   change page
 *   dpad up/down      select item on this page
 *   A / B / X / Y     page-specific actions, listed on screen
 *   bumpers           adjust value coarse
 *   triggers          adjust value fine
 */
@TeleOp(name = "Hardware Check", group = "calibration")
public class HardwareCheck extends LinearOpMode {

    private enum Page { DEVICES, SERVO, FLOWER, BEDS, SHOOTER, CONVEYOR, DIRECTION, ENCODER, DIGITAL, COLOR, BALLS, LIMELIGHT, PINPOINT }

    private RobotHardware robot;
    private Page page = Page.DEVICES;
    private int sel = 0;
    private String note = "";

    // servo page
    private Servo[] servos;
    private String[] servoNames;
    private double servoPos = 0.5;

    // beds page -- which of the four feed patterns is commanded
    private int bedPattern = 0;

    // direction page
    private DcMotor[] dirMotors;
    private String[] dirNames;
    private double dirPower = 0.20;
    private int dirZero = 0;

    // flower page -- walks MechController's two endpoints in live, so the
    // servo is always sitting at the number on screen rather than at whatever
    // it was last commanded to.
    private boolean flowerAtScoop = false;

    // shooter page
    private double targetRpm = 2650;
    private boolean openLoop = false;
    // Seeded from RobotHardware so the page never shows stale coefficients.
    private double kP = RobotHardware.SHOOTER_KP, kI = RobotHardware.SHOOTER_KI,
                   kD = RobotHardware.SHOOTER_KD, kF = RobotHardware.SHOOTER_KF;
    private double vMin = 1e9, vMax = -1e9, observedMaxTps = 0;
    private long spreadResetMs = 0;

    // encoder page
    private int encZero = 0;

    // balls page
    private VisionController vision;
    private int ballTypeIdx = 0;

    // conveyor page
    private int convTicks = 300;
    private double convPower = 0.65;
    private long convStartMs = 0;
    private int convStartPos = 0;
    private boolean convRunning = false;
    private long convSettleMs = -1;

    @Override
    public void runOpMode() {
        try {
            robot = new RobotHardware(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addLine("RobotHardware FAILED to construct:");
            telemetry.addLine(e.getMessage());
            telemetry.addLine("");
            telemetry.addLine("A hardware config name does not match. Fix it before anything else.");
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) { sleep(100); }
            return;
        }

        // C270 pipeline, so the BALLS page has something to show.
        vision = new VisionController(robot, new ShooterAdapter(ShooterAdapter.Alliance.RED));
        vision.initAprilTag();

        /*
         * START THE LIMELIGHT TOO.
         *
         * This was missing, and it made the LIMELIGHT page permanently blank:
         * getLatestResult() returns null until pipelineSwitch + start have
         * been called, so the page reported "no result" no matter what the
         * camera was pointed at. The TeleOps do this at init; the bench tool
         * never did, so the one place you would go to ask "is the Limelight
         * seeing a tag" could not answer.
         */
        vision.startHiveAprilTagPipeline();

        servos = new Servo[] { robot.turretRot, robot.hoodPollen, robot.hoodNectar,
                               robot.flowerIntake, robot.conveyorPollen, robot.conveyorNectar };
        servoNames = new String[] { "turretRot", "hoodPollen", "hoodNectar",
                                    "flowerIntake", "conveyorPollen", "conveyorNectar" };

        dirMotors = new DcMotor[] { robot.intakeMot, robot.conveyorMot,
                                    robot.pollenShootingMot, robot.nectarShootingMot };
        dirNames = new String[] { "intakeMot", "conveyorMot",
                                  "pollenShootingMot", "nectarShootingMot" };

        telemetry.addLine("All 21 devices resolved.");
        telemetry.addLine("Press START.");
        telemetry.update();
        waitForStart();

        boolean pL=false,pR=false,pU=false,pD=false,pA=false,pB=false,pX=false,pY=false;
        boolean pLB=false,pRB=false;

        while (opModeIsActive()) {
            boolean l=gamepad1.dpad_left,  r=gamepad1.dpad_right;
            boolean u=gamepad1.dpad_up,    d=gamepad1.dpad_down;
            boolean a=gamepad1.a, b=gamepad1.b, x=gamepad1.x, y=gamepad1.y;
            boolean lb=gamepad1.left_bumper, rb=gamepad1.right_bumper;

            // Changing page or selection on the DIRECTION page would otherwise
            // leave the motor you were just driving still spinning, with the
            // controls now pointing at a different one. Stop first.
            boolean leavingSelection = (l && !pL) || (r && !pR) || (u && !pU) || (d && !pD);
            if (page == Page.DIRECTION && leavingSelection) stopDirectionMotors();

            if (l && !pL) { page = prev(page); sel = 0; note = ""; }
            if (r && !pR) { page = next(page); sel = 0; note = ""; }
            if (u && !pU) sel = Math.max(0, sel - 1);
            if (d && !pD) sel = sel + 1;
            if (page == Page.BEDS) bedPattern = sel;

            if (lb && !pLB) adjust(-1, false);
            if (rb && !pRB) adjust(+1, false);
            if (gamepad1.left_trigger  > 0.5) adjust(-1, true);
            if (gamepad1.right_trigger > 0.5) adjust(+1, true);

            if (a && !pA) actionA();
            if (b && !pB) actionB();
            if (x && !pX) actionX();
            if (y && !pY) actionY();

            pL=l;pR=r;pU=u;pD=d;pA=a;pB=b;pX=x;pY=y;pLB=lb;pRB=rb;

            draw();
        }
        stopAll();
    }

    // ---- actions ----------------------------------------------------------

    private void actionA() {
        switch (page) {
            case SERVO:
                servoPos = 0.0; servos[servoIdx()].setPosition(0.0);
                note = "commanded 0.000 -- count the turns to 1.000";
                break;
            case FLOWER:
                flowerAtScoop = false; driveFlower();
                note = "at REST";
                break;
            case DIRECTION:
                dirZero = dirMotor().getCurrentPosition();
                dirMotor().setPower(dirPower);
                note = "FORWARD at " + String.format(Locale.US, "%.2f", dirPower);
                break;
            case BEDS:
                driveBeds();
                note = "commanded " + BED_NAMES[bedPattern % 4];
                break;
            case LIMELIGHT:
                // Re-issue unconditionally. The normal path skips the switch
                // when it thinks it is already on 1, which is exactly the
                // belief under suspicion.
                vision.forcePipeline(sel % 10);
                note = "forced pipeline " + (sel % 10) + " and restarted";
                break;
            case SHOOTER:
                openLoop = false;
                robot.pollenShootingMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.pollenShootingMot.setVelocity(RobotHardware.rpmToTicksPerSec(targetRpm));
                note = "pollen closed-loop at " + (int) targetRpm + " rpm";
                break;
            case CONVEYOR:
                convStartPos = robot.conveyorMot.getCurrentPosition();
                robot.conveyorMot.setTargetPosition(convStartPos + convTicks);
                robot.conveyorMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.conveyorMot.setPower(convPower);
                convStartMs = System.currentTimeMillis();
                convRunning = true; convSettleMs = -1;
                note = "running " + convTicks + " ticks at " + convPower;
                break;
            case ENCODER:
                encZero = robot.intakeMot.getCurrentPosition();
                note = "zeroed -- now turn the encoder shaft exactly ONE revolution";
                break;
            default: break;
        }
    }

    private void actionB() {
        switch (page) {
            case SERVO:
                servoPos = 1.0; servos[servoIdx()].setPosition(1.0);
                note = "commanded 1.000 -- 5-turn servo should have moved 5 turns";
                break;
            case FLOWER:
                flowerAtScoop = true; driveFlower();
                note = "at SCOOP";
                break;
            case DIRECTION:
                dirZero = dirMotor().getCurrentPosition();
                dirMotor().setPower(-dirPower);
                note = "REVERSE at " + String.format(Locale.US, "%.2f", dirPower);
                break;
            case SHOOTER:
                openLoop = false;
                robot.nectarShootingMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.nectarShootingMot.setVelocity(RobotHardware.rpmToTicksPerSec(targetRpm));
                note = "nectar closed-loop at " + (int) targetRpm + " rpm";
                break;
            case CONVEYOR:
                robot.conveyorMot.setPower(0);
                robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                convRunning = false;
                note = "stopped";
                break;
            default: break;
        }
    }

    private void actionX() {
        switch (page) {
            case SERVO:
                servoPos = 0.5; servos[servoIdx()].setPosition(0.5);
                note = "centred";
                break;
            case SHOOTER:
                stopShooters();
                note = "stopped";
                break;
            case DIRECTION:
                dirMotor().setPower(0);
                note = "stopped";
                break;
            case BALLS:
                // Re-measure every gate after flipping: red and blue trade
                // places, so the old windows are not merely shifted, they are
                // pointing at the wrong colour.
                BallDetector.FRAME_IS_BGR = !BallDetector.FRAME_IS_BGR;
                note = "reading frames as "
                        + (BallDetector.FRAME_IS_BGR ? "BGR" : "RGB")
                        + " -- re-measure all three gates";
                break;
            default: break;
        }
    }

    /**
     * Full-power open loop. This is the only way to check ticks-per-rev without
     * a tachometer: closed-loop velocity always reports back whatever you asked
     * for, so it proves nothing. Free-running at full power the motor turns at
     * its rated speed, and the ticks/s you read tells you the real CPR.
     */
    private void actionY() {
        if (page == Page.DIRECTION) {
            // setDirection flips the encoder sign with the power sign, so a
            // motor in RUN_USING_ENCODER stays consistent and its velocity
            // loop does not invert. Safe to toggle live.
            DcMotor m = dirMotor();
            m.setPower(0);
            m.setDirection(m.getDirection() == DcMotor.Direction.FORWARD
                    ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            dirZero = m.getCurrentPosition();
            note = dirNames[dirIdx()] + " now " + m.getDirection()
                    + "  -- NOT saved, edit RobotHardware";
            return;
        }
        if (page != Page.SHOOTER) { note = "no Y action on this page"; return; }
        openLoop = true;
        robot.pollenShootingMot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.nectarShootingMot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.pollenShootingMot.setPower(1.0);
        robot.nectarShootingMot.setPower(1.0);
        note = "FULL POWER free-run -- compare ticks/s against the expected row";
    }

    private void adjust(int dir, boolean fine) {
        double step = fine ? 0.002 : 0.100;
        switch (page) {
            case SERVO:
                servoPos = clamp01(servoPos + dir * step);
                servos[servoIdx()].setPosition(servoPos);
                break;
            case DIRECTION:
                dirPower = clampR(dirPower + dir * (fine ? 0.02 : 0.10), 0.05, 1.0);
                break;
            case FLOWER: {
                // Move the endpoint you are LOOKING at, and go there at once.
                // Creeping up on a stop is the whole point -- stop the moment
                // the arm reaches its limit, do not push past it.
                double s = fine ? 0.005 : 0.025;
                if ((sel % 2) == 0) {
                    MechController.FLOWER_REST = clamp01(MechController.FLOWER_REST + dir * s);
                    flowerAtScoop = false;
                } else {
                    MechController.FLOWER_SCOOP = clamp01(MechController.FLOWER_SCOOP + dir * s);
                    flowerAtScoop = true;
                }
                driveFlower();
                break;
            }
            case SHOOTER:
                switch (sel % 5) {
                    case 0: targetRpm = clampR(targetRpm + dir * (fine ? 25 : 250), 0, 6500); break;
                    case 1: kP = clampR(kP + dir * (fine ? 0.1 : 1.0), 0, 200); applyPidf(); break;
                    case 2: kI = clampR(kI + dir * (fine ? 0.01 : 0.1), 0, 50); applyPidf(); break;
                    case 3: kD = clampR(kD + dir * (fine ? 0.1 : 1.0), 0, 200); applyPidf(); break;
                    case 4: kF = clampR(kF + dir * (fine ? 0.1 : 1.0), 0, 100); applyPidf(); break;
                }
                resetSpread();
                break;
            case CONVEYOR:
                if ((sel % 2) == 0) convTicks = (int) clampR(convTicks + dir * (fine ? 10 : 50), 0, 2000);
                else                convPower = clampR(convPower + dir * (fine ? 0.02 : 0.10), 0.05, 1.0);
                break;
            case BALLS: {
                BallDetector.BallType[] ts = BallDetector.BallType.values();
                if ((sel % 5) == 0) { ballTypeIdx = (ballTypeIdx + dir + ts.length) % ts.length; break; }
                BallDetector.BallType bt = ts[ballTypeIdx % ts.length];
                double[] gg = BallDetector.getGate(bt);
                double st = fine ? 1 : 5;
                switch (sel % 5) {
                    case 1: gg[0] = clampR(gg[0] + dir * st, 0, 179); break;
                    case 2: gg[1] = clampR(gg[1] + dir * st, 0, 179); break;
                    case 3: gg[2] = clampR(gg[2] + dir * (fine ? 5 : 20), 0, 255); break;
                    case 4: gg[3] = clampR(gg[3] + dir * (fine ? 5 : 20), 0, 255); break;
                }
                BallDetector.setGate(bt, gg[0], gg[1], gg[2], gg[3]);
                note = String.format(Locale.US, "%s  H %.0f-%.0f  S>%.0f  V>%.0f",
                        bt, gg[0], gg[1], gg[2], gg[3]);
                break;
            }
            default: break;
        }
    }

    // ---- drawing ----------------------------------------------------------

    private void draw() {
        telemetry.addLine("PAGE " + page + "   (dpad L/R)");
        telemetry.addLine("");
        switch (page) {
            case DEVICES:   drawDevices();   break;
            case SERVO:     drawServo();     break;
            case FLOWER:    drawFlower();    break;
            case BEDS:      drawBeds();      break;
            case DIRECTION: drawDirection(); break;
            case SHOOTER:   drawShooter();   break;
            case CONVEYOR:  drawConveyor();  break;
            case ENCODER:   drawEncoder();   break;
            case DIGITAL:   drawDigital();   break;
            case COLOR:     drawColor();     break;
            case BALLS:     drawBalls();     break;
            case LIMELIGHT: drawLimelight(); break;
            case PINPOINT:  drawPinpoint();  break;
        }
        if (!note.isEmpty()) { telemetry.addLine(""); telemetry.addLine("> " + note); }
        telemetry.update();
    }

    /*
     * ---------------------------------------------------------------------
     * SCREENS ARE FOR LIVE VALUES. The how-and-why lives in BENCH_TESTS.md.
     *
     * Every page used to print five to ten lines of prose explaining what to
     * do and how to read it -- redrawn at loop rate, on a phone screen that
     * shows about a dozen lines. The numbers you came to read were pushed
     * off the bottom by the instructions telling you to read them.
     *
     * So each page is now: one control hint, the selection, the live values,
     * and at most one line of interpretation where it is genuinely decisive.
     * Nothing was deleted -- the prose moved to BENCH_TESTS.md, which is
     * where you can actually read it while holding a gamepad.
     * ---------------------------------------------------------------------
     */
    private void drawDevices() {
        telemetry.addLine("All 21 resolved, or init would have failed.");
        telemetry.addLine("");
        telemetry.addLine("4 motors  6 servos  6 colour  1 digital");
        telemetry.addLine("Webcam 1  limeLight  pinpoint  imu");
    }

    private int servoIdx() { return sel % servos.length; }

    /** The four bed patterns the FSM ever commands. */
    private static final String[] BED_NAMES = {
            "BOTH DISENGAGED", "BOTH ENGAGED  (intake / feed BOTH)",
            "POLLEN feed  (pollen engaged)", "NECTAR feed  (nectar engaged)" };

    private void driveBeds() {
        boolean p, n;
        switch (bedPattern % 4) {
            case 1:  p = true;  n = true;  break;
            case 2:  p = true;  n = false; break;
            case 3:  p = false; n = true;  break;
            default: p = false; n = false; break;
        }
        robot.conveyorPollen.setPosition(MechController.bedPollenPos(p));
        robot.conveyorNectar.setPosition(MechController.bedNectarPos(n));
    }

    /**
     * Drive the two conveyor beds through the patterns the FSM uses.
     *
     * Positions come from MechController, not from copies -- a page with its
     * own numbers would pass while the robot ran something else.
     *
     * The beds are MIRRORED: pollen engages by raising its servo, nectar by
     * lowering its. Both must sweep the SAME angle, opposite ways. They did
     * not used to -- pollen swept three times as far as nectar.
     */
    private void drawBeds() {
        telemetry.addLine("dpad U/D pick pattern   A = command it");
        telemetry.addLine("");
        for (int i = 0; i < BED_NAMES.length; i++) {
            telemetry.addLine((i == (bedPattern % 4) ? "> " : "  ") + BED_NAMES[i]);
        }
        telemetry.addLine("");
        telemetry.addData("pollen engaged / disengaged", "%.3f / %.3f",
                MechController.bedPollenPos(true), MechController.bedPollenPos(false));
        telemetry.addData("nectar engaged / disengaged", "%.3f / %.3f",
                MechController.bedNectarPos(true), MechController.bedNectarPos(false));
        telemetry.addData("travel each", "%.3f / %.3f   (must match)",
                Math.abs(MechController.bedPollenPos(true) - MechController.bedPollenPos(false)),
                Math.abs(MechController.bedNectarPos(true) - MechController.bedNectarPos(false)));
    }

    private void stopDirectionMotors() {
        for (DcMotor m : dirMotors) m.setPower(0);
    }

    private int dirIdx() { return sel % dirMotors.length; }
    private DcMotor dirMotor() { return dirMotors[dirIdx()]; }

    /**
     * Which way each motor actually turns, and a live flip to fix it.
     *
     * WHAT THIS CAN AND CANNOT SETTLE NOW
     *   Encoder-vs-power agreement is already proven for the two motors that
     *   use closed loop: the shooters hit 6214 rpm under RUN_USING_ENCODER,
     *   and the conveyor's RUN_TO_POSITION drives its error to ~0. Neither is
     *   possible with an inverted encoder, so those are settled.
     *
     *   Which way the shaft PHYSICALLY turns cannot be judged against a
     *   mechanism that does not exist. What you CAN do now is write down what
     *   each motor does at positive power, so that when the mechanism arrives
     *   the fix is one Y press and one line edited, not a recompile per guess.
     */
    private void drawDirection() {
        telemetry.addLine("dpad U/D pick motor   A=fwd  B=rev  X=stop");
        telemetry.addLine("Y = flip direction    bumpers/triggers power");
        telemetry.addLine("");

        for (int i = 0; i < dirMotors.length; i++) {
            telemetry.addLine((i == dirIdx() ? "> " : "  ")
                    + String.format(Locale.US, "%-18s %s", dirNames[i],
                            dirMotors[i].getDirection()));
        }
        telemetry.addLine("");
        telemetry.addData("power", "%.2f", dirPower);

        DcMotor m = dirMotor();
        telemetry.addData("encoder delta", m.getCurrentPosition() - dirZero);
        if (dirIdx() == 0) telemetry.addLine("  (this port reads the TURRET encoder)");
        telemetry.addLine("");
        telemetry.addLine("Y is live only -- edit RobotHardware to keep it.");
    }

    /** Send the flower intake to whichever endpoint is currently selected. */
    private void driveFlower() {
        robot.flowerIntake.setPosition(
                flowerAtScoop ? MechController.FLOWER_SCOOP : MechController.FLOWER_REST);
    }

    /**
     * Walk in the two flower-intake endpoints. These start at 0.0 and 1.0,
     * which is full servo travel and almost certainly past the mechanism's
     * own stops -- a servo held against a stop stalls, heats and strips.
     */
    private void drawFlower() {
        telemetry.addLine("dpad U/D pick endpoint   A=go REST   B=go SCOOP");
        telemetry.addLine("bumpers +/-0.025   triggers +/-0.005");
        telemetry.addLine("");

        boolean onRest = (sel % 2) == 0;
        telemetry.addData((onRest ? "> " : "  ") + "FLOWER_REST",
                "%.3f", MechController.FLOWER_REST);
        telemetry.addData((!onRest ? "> " : "  ") + "FLOWER_SCOOP",
                "%.3f", MechController.FLOWER_SCOOP);
        telemetry.addLine("");
        telemetry.addData("servo now at", flowerAtScoop ? "SCOOP" : "REST");

        double span = Math.abs(MechController.FLOWER_SCOOP - MechController.FLOWER_REST);
        telemetry.addData("span", "%.3f%s", span,
                span < 0.05 ? "   !! barely moves"
                            : span > 0.80 ? "   !! near full travel - narrow before bolting on"
                                          : "");
        telemetry.addLine("");
        telemetry.addLine(String.format(Locale.US,
                "FLOWER_REST = %.3f, FLOWER_SCOOP = %.3f;",
                MechController.FLOWER_REST, MechController.FLOWER_SCOOP));
        telemetry.addLine("NOT saved -- paste into MechController.");
    }

    private void drawServo() {
        telemetry.addLine("dpad U/D pick servo   A=0.0  X=0.5  B=1.0");
        telemetry.addLine("bumpers +/-0.100   triggers +/-0.002");
        telemetry.addLine("");
        for (int i = 0; i < servos.length; i++) {
            telemetry.addLine((i == servoIdx() ? "> " : "  ") + servoNames[i]);
        }
        telemetry.addLine("");
        telemetry.addData("commanded", "%.3f", servoPos);
        telemetry.addLine("");
        telemetry.addLine("A then B: ~5 turns good, ~1.7 = no PWM range");
    }

    private void drawShooter() {
        double pTps = robot.pollenShootingMot.getVelocity();
        double nTps = robot.nectarShootingMot.getVelocity();
        double watch = openLoop ? Math.max(pTps, nTps) : pTps;
        if (watch > observedMaxTps) observedMaxTps = watch;
        if (System.currentTimeMillis() - spreadResetMs > 2500) resetSpread();
        if (watch < vMin) vMin = watch;
        if (watch > vMax) vMax = watch;

        String[] lbl = { "target rpm", "P", "I", "D", "F" };
        double[] val = { targetRpm, kP, kI, kD, kF };
        telemetry.addLine("A=pollen loop  B=nectar loop  Y=FULL POWER  X=stop");
        telemetry.addLine("dpad U/D pick field   bumpers coarse   triggers fine");
        telemetry.addLine("");
        for (int i = 0; i < lbl.length; i++) {
            telemetry.addLine(String.format(Locale.US, "%s%-10s %.2f",
                    (i == sel % 5 ? "> " : "  "), lbl[i], val[i]));
        }
        telemetry.addLine("");
        telemetry.addData("pollen", "%.0f ticks/s = %.0f rpm", pTps,
                RobotHardware.ticksPerSecToRpm(pTps));
        telemetry.addData("nectar", "%.0f ticks/s = %.0f rpm", nTps,
                RobotHardware.ticksPerSecToRpm(nTps));
        telemetry.addData("commanded", "%.0f ticks/s",
                RobotHardware.rpmToTicksPerSec(targetRpm));
        telemetry.addLine("");
        // A number beats eyeballing a jittery readout. Real hunting shows as a
        // wide spread AND an audible surge; a steady hum with a wide spread is
        // just encoder quantisation at 28 cpr.
        // This number is test B14 and sets DIP_ENTER. Under 30 is steady;
        // over 100 is hunting, so drop P.
        telemetry.addData("spread (2s)", "%.0f ticks/s   [%.0f .. %.0f]  %s",
                (vMax - vMin), vMin, vMax,
                (vMax - vMin) < 30 ? "steady" : (vMax - vMin) > 100 ? "HUNTING" : "");
        if (openLoop) {
            telemetry.addLine("");
            telemetry.addData("free-run max", "%.0f ticks/s   (~2800 = 28 cpr)",
                    observedMaxTps);
            if (observedMaxTps > 100) {
                telemetry.addData("suggested", "F %.2f   P %.2f   I 0.1xP   D 0",
                        32767.0 / observedMaxTps, 0.1 * 32767.0 / observedMaxTps);
            }
        }
    }

    private void applyPidf() {
        robot.pollenShootingMot.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        robot.nectarShootingMot.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        note = String.format(Locale.US, "PIDF %.2f / %.2f / %.2f / %.2f", kP, kI, kD, kF);
    }

    private void resetSpread() { vMin = 1e9; vMax = -1e9; spreadResetMs = System.currentTimeMillis(); }

    private static double clampR(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    private void drawConveyor() {
        int pos = robot.conveyorMot.getCurrentPosition();
        int target = convStartPos + convTicks;
        int err = convRunning ? target - pos : 0;
        boolean busy = robot.conveyorMot.isBusy();
        long elapsed = convRunning ? System.currentTimeMillis() - convStartMs : 0;
        // The feed phase ends when isBusy() goes false. If it never does, the
        // FSM hits SHOOTER_FEED_TIMEOUT_MS instead of completing the shot, so
        // this is the one behaviour the whole feed depends on.
        if (convRunning && !busy && convSettleMs < 0) convSettleMs = elapsed;

        telemetry.addLine("A = run   B = stop   dpad U/D pick field");
        telemetry.addLine("bumpers coarse   triggers fine");
        telemetry.addLine("");
        telemetry.addLine(((sel % 2) == 0 ? "> " : "  ") + "ticks  " + convTicks);
        telemetry.addLine(((sel % 2) == 1 ? "> " : "  ") + String.format(Locale.US, "power  %.2f", convPower));
        telemetry.addLine("");
        telemetry.addData("position", pos);
        telemetry.addData("target", convRunning ? String.valueOf(target) : "-");
        telemetry.addData("error", convRunning ? String.valueOf(err) : "-");
        telemetry.addData("isBusy", busy);
        telemetry.addData("elapsed", elapsed + " ms");
        telemetry.addData("settled at", convSettleMs < 0 ? "not yet" : convSettleMs + " ms");
        telemetry.addLine("");
        telemetry.addLine("Good: isBusy false inside ~1s, error near 0.");
        telemetry.addData("feed distances", "pollen %d  nectar %d  both %d",
                MechController.TICKS_TO_SHOOT_POLLEN,
                MechController.TICKS_TO_SHOOT_NECTAR,
                MechController.TICKS_TO_SHOOT_BOTH);
    }

    private void drawEncoder() {
        int raw = robot.intakeMot.getCurrentPosition();
        int delta = raw - encZero;
        telemetry.addLine("Through Bore V2 reads through the intakeMot port.");
        telemetry.addLine("A = zero here, then turn the shaft ONE revolution.");
        telemetry.addLine("");
        telemetry.addData("raw ticks", raw);
        telemetry.addData("since zero", delta);
        telemetry.addData("implied deg", "%.1f  (at 8192 cpr)", delta * 360.0 / 8192.0);
        telemetry.addLine("");
        telemetry.addLine("One turn ~8192, either sign. 0 = not wired.");
    }

    private void drawDigital() {
        telemetry.addLine("Wave a magnet at the switch.");
        telemetry.addLine("");
        telemetry.addData("magneticLimitSwitch", robot.magneticLimitSwitch.getState());
        telemetry.addLine("");
        telemetry.addLine("Reading WITH magnet = LIMIT_TRIGGERED_STATE.");
    }

    private void drawColor() {
        RevColorSensorV3[] s = { robot.sensorIntakeL, robot.sensorIntakeR,
                                 robot.sensorPollenL, robot.sensorPollenR,
                                 robot.sensorNectarL, robot.sensorNectarR };
        String[] n = { "intakeL", "intakeR", "pollenL", "pollenR", "nectarL", "nectarR" };
        int i = sel % s.length;
        telemetry.addLine("dpad U/D pick sensor. Hold a ball against it.");
        telemetry.addLine("");
        telemetry.addLine("> " + n[i]);
        double dist = s[i].getDistance(DistanceUnit.MM);
        int r = s[i].red(), g = s[i].green(), bl = s[i].blue();
        int max = Math.max(r, Math.max(g, bl));
        float[] hsv = new float[3];
        if (max > 0) {
            android.graphics.Color.RGBToHSV(r * 255 / max, g * 255 / max, bl * 255 / max, hsv);
        }
        telemetry.addData("distance", "%.1f mm  (ball if < 20)", dist);
        telemetry.addData("rgb", "%d / %d / %d", r, g, bl);
        telemetry.addData("hsv", "%.1f / %.2f / %.2f", hsv[0], hsv[1], hsv[2]);
        telemetry.addLine("");
        telemetry.addLine("expect H: yellow 80-110  red 15-35  blue 215-240");
    }

    private void drawBalls() {
        BallDetector bd = vision.getBallDetector();
        if (bd == null) { telemetry.addLine("camera not started"); return; }

        BallDetector.BallType[] types = BallDetector.BallType.values();
        BallDetector.BallType t = types[ballTypeIdx % types.length];
        double[] g = BallDetector.getGate(t);
        double[] c = bd.getCentreHsv();
        double[] raw = bd.getCentreRaw();

        telemetry.addLine("dpad U/D pick field   bumpers coarse   triggers fine");
        telemetry.addLine("");
        // Measuring the ball beats guessing the gate. Hold one in the middle
        // of the view and set the window around what this actually reads.
        // Hue is 0-179 in OpenCV, so halve any 0-360 value you look up.
        telemetry.addData("centre HSV", "H %.0f  S %.0f  V %.0f", c[0], c[1], c[2]);
        // Channel-order check. On a YELLOW ball ch0 and ch1 must both be high
        // and ch2 low; if ch0 is low and ch2 is high the frame is BGR and the
        // hue readout above is meaningless, not merely mis-gated.
        telemetry.addData("centre raw", "%.0f/%.0f/%.0f  as %s%s",
                raw[0], raw[1], raw[2],
                BallDetector.FRAME_IS_BGR ? "BGR" : "RGB",
                (raw[0] + raw[1] + raw[2] > 30 && raw[2] > raw[0] + 40)
                        ? "   !! looks SWAPPED, X to flip" : "");
        telemetry.addLine("");
        String[] lbl = { "colour", "H low", "H high", "S min", "V min" };
        double[] val = { ballTypeIdx, g[0], g[1], g[2], g[3] };
        for (int i = 0; i < lbl.length; i++) {
            String v = (i == 0) ? t.toString() : String.format(Locale.US, "%.0f", val[i]);
            telemetry.addLine((i == sel % 5 ? "> " : "  ") + String.format("%-7s %s", lbl[i], v));
        }
        telemetry.addLine("");
        java.util.List<BallDetector.Detection> ds = bd.getDetections();
        int p = 0, r = 0, b = 0;
        for (BallDetector.Detection d : ds) {
            if (d.type == BallDetector.BallType.POLLEN) p++;
            else if (d.type == BallDetector.BallType.NECTAR_RED) r++; else b++;
        }
        telemetry.addData("seen", "pollen %d   red %d   blue %d", p, r, b);
        if (!ds.isEmpty()) {
            BallDetector.Detection n = ds.get(0);
            telemetry.addData("nearest", "%s  %.1f in  %+.1f deg  %.0f px",
                    n.type, n.rangeIn, n.bearingDeg, n.areaPx);
        } else {
            telemetry.addLine("nearest  none");
        }
        telemetry.addData("fov", "H %.1f  V %.1f deg", bd.hfovDeg(), bd.vfovDeg());
    }

    private void drawLimelight() {
        LLResult res = robot.limeLight.getLatestResult();
        telemetry.addLine("dpad U/D pick a pipeline index   A = FORCE it");
        telemetry.addLine("");
        telemetry.addData("index to force", sel % 10);
        telemetry.addLine("");
        /*
         * SEPARATE THE THREE FAILURE MODES.
         *
         * "no tags" has three completely different causes and they need
         * different fixes:
         *
         *   no result at all   -> the Limelight is not talking to the hub.
         *                         Config name, cabling, power.
         *   result but stale   -> it is connected but not processing.
         *   result, 0 tags     -> it IS running, and pipeline 1 is almost
         *                         certainly not an APRILTAG pipeline. That is
         *                         configured on the Limelight's own web page,
         *                         not from this code -- nothing here can set
         *                         a pipeline's TYPE, only select its index.
         */
        /*
         * isConnected() answers the question none of the other fields can:
         * is the hub talking to the Limelight AT ALL?
         *
         * Everything else -- valid, tx, fiducials -- can read as "nothing
         * there" for a camera that is perfectly healthy but simply not being
         * polled. This separates a dead link from an empty frame.
         */
        telemetry.addData("connected", vision.isLimelightConnected());
        telemetry.addData("last update", "%.2f s ago", vision.limelightAgeSec());
        telemetry.addData("commanded pipeline", vision.getCurrentPipeline());
        if (res == null) {
            telemetry.addLine("NO RESULT -- the Limelight is not responding.");
            telemetry.addLine("  check the config name is exactly 'limeLight',");
            telemetry.addLine("  the ethernet cable, and that it has power.");
            return;
        }
        /*
         * WHICH PIPELINE IS ACTUALLY RUNNING.
         *
         * This is the decisive field when the web page shows tags and the hub
         * does not. `commanded` is what pipelineSwitch() asked for; `running`
         * is what the Limelight says it is doing. If they differ, the switch
         * did not take -- and a colour pipeline returns valid=true with real
         * tx/ty and an empty fiducial list forever, which looks exactly like
         * a broken AprilTag pipeline.
         *
         * Selecting pipeline 1 in the WEB UI does not change what the robot
         * commands at init.
         */
        telemetry.addData("running pipeline", res.getPipelineIndex());
        // valid / tx / ty are about a PRIMARY TARGET and mean nothing for tag
        // reading. An AprilTag pipeline happily reports IDs with valid=false
        // and tx=ty=0. Shown for completeness; ignore them here.
        telemetry.addData("valid / tx / ty", "%s  %.1f / %.1f   (not used)",
                res.isValid(), res.getTx(), res.getTy());
        List<LLResultTypes.FiducialResult> tags = res.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            // Alive and processing, but no tags: almost always a pipeline
            // that is not APRILTAG/36h11, or an ID filter excluding 30-45.
            telemetry.addLine("RESULT but NO FIDUCIALS -- check pipeline type");
            telemetry.addLine("is APRILTAG 36h11 and the ID filter is open.");
        } else {
            StringBuilder ids = new StringBuilder();
            for (LLResultTypes.FiducialResult f : tags) ids.append(f.getFiducialId()).append(" ");
            telemetry.addData("tag ids", ids.toString());
        }
        telemetry.addLine("");
        // The thing the TeleOp actually acts on. A tag seen here does not
        // move the turret until the 4-frame filter agrees on it.
        telemetry.addData("filtered target", "%s   (%d ours in view)",
                vision.getRaisedTarget(), vision.getVisibleTagCount());
        telemetry.addLine("RED 30-33 scoring / 34-37 audience");
        telemetry.addLine("BLUE 42-45 scoring / 38-41 audience");
    }

    private void drawPinpoint() {
        // The Pinpoint latches its values and only refreshes on update(). On
        // the robot Pedro owns that call, which is why RobotHardware does not
        // make it -- but nothing else does here, so without this the readings
        // sit at 0 forever and look like dead hardware.
        robot.pinpoint.update();

        // RAW COUNTS ARE THE DIAGNOSTIC. X/Y in inches depend on encoder
        // resolution and pod offsets, neither of which is configured yet, so
        // they can read 0 on perfectly good hardware. Raw ticks depend on
        // nothing but the pod being plugged in and turning.
        telemetry.addLine("Spin each pod by hand: raw counts must move.");
        telemetry.addLine("");
        telemetry.addData("device status", robot.pinpoint.getDeviceStatus());
        telemetry.addData("raw pod X / Y", "%d / %d",
                robot.pinpoint.getEncoderX(), robot.pinpoint.getEncoderY());
        telemetry.addData("heading  pp / imu", "%.1f / %.1f deg",
                robot.pinpoint.getHeading(AngleUnit.DEGREES),
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addLine("");
        robot.addPinpointTelemetry(telemetry);
        telemetry.addLine("");
        telemetry.addLine("X/Y inches still need resolution, offsets and");
        telemetry.addLine("directions -- none set until the pods are mounted.");
    }

    // ---- helpers ----------------------------------------------------------

    private void stopShooters() {
        robot.pollenShootingMot.setPower(0);
        robot.nectarShootingMot.setPower(0);
        robot.pollenShootingMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.nectarShootingMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pollenShootingMot.setVelocity(0);
        robot.nectarShootingMot.setVelocity(0);
        openLoop = false;
    }

    private void closeVision() { if (vision != null) vision.stop(); }

    private void stopAll() {
        closeVision();
        stopShooters();
        robot.intakeMot.setPower(0);
        robot.conveyorMot.setPower(0);
    }

    private static double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
    private static Page next(Page p) { Page[] v = Page.values(); return v[(p.ordinal()+1)%v.length]; }
    private static Page prev(Page p) { Page[] v = Page.values(); return v[(p.ordinal()+v.length-1)%v.length]; }
}






