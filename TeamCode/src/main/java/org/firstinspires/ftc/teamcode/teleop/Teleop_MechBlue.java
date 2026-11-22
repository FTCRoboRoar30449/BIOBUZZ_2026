package org.firstinspires.ftc.teamcode.teleop;

import java.util.Locale;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.MatchState;
import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.BallDetector;
import org.firstinspires.ftc.teamcode.robot.BallSeek;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;
import org.firstinspires.ftc.teamcode.robot.TurretController;
import org.firstinspires.ftc.teamcode.robot.VisionController;

@TeleOp(name = "TeleopMech BLUE", group = "Test")
public class Teleop_MechBlue extends OpMode {




    // Autonomous end pose, carried into TeleOp.
    private double seedX, seedY, seedH;
    private boolean seeded = false;

    private RobotHardware robot;
    private MechController mech;
    private VisionController vision;
    private ShooterAdapter shooter;
    private TurretController turret;

    private boolean p1a, p1b, p1y, p1du, p1dd, p1dl;
    private boolean p2a, p2x, p2b, p2y, p2du, p2dd, p2dl, p2dr;
    private boolean p2lb, p2rb, p2bk, p2st, p2lt, p2rt, p2lsb;

    // Drive feel. PRECISION_SCALE is held, not toggled -- a held modifier
    public static double PRECISION_SCALE = 0.35;
    // Above this on any stick, an automated drive hands back to the driver.
    public static double STICK_CANCEL = 0.15;

    // true rotates the sticks into the field frame; only as good as the heading.
    public static boolean FIELD_CENTRIC = true;
    public static int LATERAL_SIGN = +1;   // flip if strafe mirrors again

    // Where this alliance's driver stands, as a rotation of the stick frame
    // into the field frame. BLUE 0, BLUE 180.
    public static double DRIVER_FRAME_DEG = 180;

    // Spin should NOT need this: rotating your viewpoint by 180 deg does not
    // reverse which way clockwise looks. If BLUE really does spin backwards
    // after DRIVER_FRAME_DEG is right, the cause is a drivetrain motor
    // direction, not the driver frame -- flip this to get through a match,
    // then find the real one in RobotSetup step 2.
    public static int SPIN_SIGN = +1;

    // Name of the pose an automated drive is heading for, or null when
    private String autoTarget = null;
    private String driveNote = "";   // so a button with no motion still reports

    // Spin test: gamepad1 start marks a reference, telemetry then shows how
    // far pose has drifted and how many turns it took.

    @Override public void init() {
        robot   = new RobotHardware(hardwareMap, telemetry);
        shooter = new ShooterAdapter(ShooterAdapter.Alliance.BLUE);
        vision  = new VisionController(robot, shooter);
        turret  = new TurretController(robot);
        mech    = new MechController(robot, vision, shooter, turret);

        vision.initAprilTag();
        vision.startHiveAprilTagPipeline();
        mech.setState(MechState.START);
        // Beds to the resting pattern now, not when an intake state
        mech.initPositions(MatchState.isFresh(Blue.ALLIANCE) && MatchState.deployed);
        // Autonomous already drove off the wall, so the arm is safe to swing.
        mech.armDeploy();

        // Commands the turret to forward and confirms on the limit switch.
        turret.beginHoming();

        follower = Constants.create(hardwareMap);

        telemetry.addLine("BLUE alliance. Homing turret...");
        telemetry.update();
    }

    private boolean initRb = false;

    @Override public void init_loop() {
        turret.update();
        telemetry.addData("Turret", turret.getHomeMessage());
        telemetry.addData("Press", "gamepad2 RB to search if it did not find home");
        // Rising edge only: calling beginSearch() every loop would reset
        if (gamepad2.right_bumper && !initRb) turret.beginSearch();
        initRb = gamepad2.right_bumper;
        telemetry.update();
    }

    @Override public void start() {
        // Inherit autonomous's inventory when it ran in this process
        if (MatchState.isFresh(Blue.ALLIANCE)) {
            mech.countPollen = MatchState.countPollen;
            mech.countNectar = MatchState.countNectar;
            mech.cellPollenIn = MatchState.cellPollenIn;
            mech.cellNectarIn = MatchState.cellNectarIn;
            // Seeded ONCE.
            seedX = MatchState.poseX; seedY = MatchState.poseY;
            seedH = MatchState.headingDeg; seeded = true;
        } else {
            // WITHOUT THIS the three seed fields keep their Java defaults
            seedX = Blue.START.x();
            seedY = Blue.START.y();
            seedH = Math.toDegrees(Blue.START.heading());
            seeded = false;
        }
        // Hand the inherited pose to the follower.
        if (follower != null) {
            follower.setPose(new Pose(seedX, seedY, Math.toRadians(seedH)));
        }
        mech.setState(MechState.IDLE);
    }

    @Override public void loop() {
        updatePoseFromSource();

        // ================= GAMEPAD 1 -- DRIVER ===========================
        // A is the scoring button: go to the best shoot pose and stream.
        // Y parks -- worth 5 points, and the end of a match is exactly when
        // you do not want to be lining up by hand.
        // Stick cancels any of them, like any macro.
        boolean a1 = gamepad1.a && !p1a;
        boolean b1 = gamepad1.b && !p1b;
        boolean y1 = gamepad1.y && !p1y;
        if (a1) { driveToNearestShoot(); mech.setState(MechState.SHOOTING_HIVE); }
        if (y1) driveToPose(Blue.PARK, "PARK");
        // You only drive to the GARDEN to collect, so arm the intake too,
        // same as RB and as gamepad 2's flower button.
        if (b1) { driveToPose(Blue.GARDEN, "GARDEN"); mech.setState(MechState.INTAKE_FIELD); }
        p1a = gamepad1.a; p1b = gamepad1.b; p1y = gamepad1.y;

        // Partner cell scoring.
        boolean du1 = gamepad1.dpad_up && !p1du;
        boolean dd1 = gamepad1.dpad_down && !p1dd;
        boolean dl1 = gamepad1.dpad_left && !p1dl;
        if (du1) mech.addPartnerPollen();
        if (dd1) mech.addPartnerNectar();
        if (dl1) mech.resetCellCount();
        p1du = gamepad1.dpad_up; p1dd = gamepad1.dpad_down; p1dl = gamepad1.dpad_left;

        // ================= GAMEPAD 2 -- OPERATOR =========================
        boolean a2 = gamepad2.a && !p2a, b2 = gamepad2.b && !p2b;
        boolean x2 = gamepad2.x && !p2x, y2 = gamepad2.y && !p2y;
        // A and Y drive there as well as arming: the operator owns the whole
        // action, and gamepad 1 keeps only manual driving.
        // Fire where the robot already stands -- no drive. The driver owns
        // the positioning on gamepad 1 A; this is the one to use when the
        // robot is already lined up, or when a macro would be in the way.
        if (a2) mech.setState(MechState.SHOOTING_HIVE);
        else if (x2) mech.setState(MechState.INTAKE_FIELD);
        else if (y2) { driveToNearestFlower(); mech.setState(MechState.INTAKE_FLOWER); }
        else if (b2) mech.setState(MechState.IDLE);
        p2a = gamepad2.a; p2b = gamepad2.b; p2x = gamepad2.x; p2y = gamepad2.y;

        // Triggers are edge-detected like the buttons.
        boolean lt2 = gamepad2.left_trigger > 0.5;
        boolean rt2 = gamepad2.right_trigger > 0.5;
        if (lt2 && !p2lt) mech.setState(MechState.SHOOTING_FLOWER_POLLEN);
        if (rt2 && !p2rt) mech.setState(MechState.SHOOTING_FLOWER_NECTAR);
        p2lt = lt2; p2rt = rt2;

        // ---- index: what replaces the colour-sensor edge ----------------
        boolean lb2 = gamepad2.left_bumper && !p2lb;
        boolean rb2 = gamepad2.right_bumper && !p2rb;
        if (lb2) mech.indexPollen();
        if (rb2) mech.indexNectar();
        p2lb = gamepad2.left_bumper; p2rb = gamepad2.right_bumper;

        // ---- bed drop: jam clearing -------------------------------------
        boolean lsb2 = gamepad2.left_stick_button && !p2lsb;
        if (lsb2) mech.dropBedsMomentarily();
        p2lsb = gamepad2.left_stick_button;

        // ---- count corrections ------------------------------------------
        boolean du2 = gamepad2.dpad_up && !p2du, dr2 = gamepad2.dpad_right && !p2dr;
        boolean dd2 = gamepad2.dpad_down && !p2dd, dl2 = gamepad2.dpad_left && !p2dl;
        if (du2) mech.countPollen = clampCount(mech.countPollen + 1);
        if (dr2) mech.countPollen = clampCount(mech.countPollen - 1);
        if (dd2) mech.countNectar = clampCount(mech.countNectar + 1);
        if (dl2) mech.countNectar = clampCount(mech.countNectar - 1);
        p2du = gamepad2.dpad_up; p2dr = gamepad2.dpad_right;
        p2dd = gamepad2.dpad_down; p2dl = gamepad2.dpad_left;

        boolean bk2 = gamepad2.back && !p2bk, st2 = gamepad2.start && !p2st;
        if (bk2) { mech.countPollen = 0; mech.countNectar = 0; }
        if (st2) turret.beginSearch();
        p2bk = gamepad2.back; p2st = gamepad2.start;

        // Turret trim.
        double trim = gamepad2.right_stick_x;
        if (Math.abs(trim) > 0.2) {
            turret.setTurretDeg(turret.getCommandedDeg() + trim * 2.0);
        }

        serviceDrive();
        mech.update();
        // Before allTelemetry(), which calls telemetry.update() --
        telemetry.addData("Mode", "%s%s", FIELD_CENTRIC ? "FIELD centric" : "robot centric",
                driveNote.isEmpty() ? "" : "   " + driveNote);
        telemetry.addData("Drive", autoTarget != null
                ? "AUTO -> " + autoTarget + "   (stick to take over)"
                : seeking ? "BALL SEEK   (release RB to take over)"
                : gamepad1.left_trigger > 0.5 ? "manual, PRECISION" : "manual");
        if (mech.bedsDropped()) {
            telemetry.addData("BEDS", "DOWN  %.1f s left", mech.bedDropRemainingMs() / 1000.0);
        }
        if (vision.getBallDetector() != null) {
            java.util.List<BallDetector.Detection> ds = vision.getBallDetector().getDetections();
            int pol = 0, red = 0, blu = 0;
            for (BallDetector.Detection dd : ds) {
                if (dd.type == BallDetector.BallType.POLLEN) pol++;
                else if (dd.type == BallDetector.BallType.NECTAR_RED) red++;
                else blu++;
            }
            telemetry.addData("Detector", "cam %s   %d seen  P%d R%d B%d   size-chk %s",
                    vision.isBallDetectionEnabled() ? "ON" : "OFF",
                    ds.size(), pol, red, blu,
                    BallDetector.SIZE_CHECK ? "ON" : "off");
            // Pixels through the POLLEN gate BEFORE any shape filter. Large
            // here with 0 seen means the colour is fine and shape/size is
            // rejecting; near zero means the HSV gate itself is wrong.
            double[] hsvC = vision.getBallDetector().getCentreHsv();
            telemetry.addData("Gate", "pollen mask %d px (min area %.0f)   centre HSV %.0f %.0f %.0f",
                    vision.getBallDetector().pollenMaskPixels(), BallDetector.MIN_AREA_PX,
                    hsvC[0], hsvC[1], hsvC[2]);
            telemetry.addData("Pollen gate", "H %.0f-%.0f  S>=%.0f  V>=%.0f   frame %s",
                    BallDetector.YELLOW_LO.val[0], BallDetector.YELLOW_HI.val[0],
                    BallDetector.YELLOW_LO.val[1], BallDetector.YELLOW_LO.val[2],
                    BallDetector.FRAME_IS_BGR ? "BGR" : "RGB");
        }
        telemetry.addData("Seek", "%s   [%.0f s ago]", seekWhy,
                (System.currentTimeMillis() - seekWhyMs) / 1000.0);
        telemetry.addData("Seek state", "RB %s  seeking %s  busy %s  autoTarget %s  sticks %s",
                gamepad1.right_bumper, seeking,
                follower != null && follower.isBusy(),
                autoTarget == null ? "-" : autoTarget,
                lastSticks ? "MOVED" : "-");
        telemetry.addData("Sensors", "intake %s   tunnels %s",
                MechController.USE_INTAKE_COLOR ? "ON, auto count" : "OFF, count by hand",
                MechController.USE_TUNNEL_SENSORS ? "ON" : "OFF, index on LB/RB");
        if (MatchState.isFresh(Blue.ALLIANCE)) {
            telemetry.addData("From auto", MatchState.summary(Blue.ALLIANCE));
        }
        mech.allTelemetry();
    }

    private void updatePoseFromSource() {
        // A real follower is the only LIVE source; everything else here
        if (follower != null) {
            Pose p = follower.pose();
            double hd = Math.toDegrees(p.heading());
            mech.updatePose(p.x(), p.y(), hd);
            vision.setPose(p.x(), p.y(), hd);
            return;
        }
        mech.updatePose(seedX, seedY, seedH);
        // The ball detector needs it too, or detections come back with NaN
        vision.setPose(seedX, seedY, seedH);
    }

    @Override public void stop() {
        if (mech != null) mech.stopAll();
        if (vision != null) vision.stop();
    }

    // ---- drivetrain ------------------------------------------------------
    private Follower follower;
    private boolean p1rb = false;
    private boolean seeking = false;
    private String seekWhy = "idle";
    // Latched text goes stale the moment RB is released, and a stale line
    // reads exactly like a live one. Stamp it.
    private long seekWhyMs = System.currentTimeMillis();
    // A fresh follow() cannot be judged idle until the follower has had a
    // few cycles to act on it.
    public static long SEEK_REISSUE_MS = 300;
    private long lastSeekMs = 0;
    private int seekIssues = 0;
    private boolean lastSticks = false;
    // follow() is issued from the gamepad block, before update() has run on
    // it, so isBusy() means nothing for the first few loops.
    public static long MACRO_SETTLE_MS = 400;
    private long autoTargetMs = 0;
    // When the whole macro began, as opposed to the current leg.
    private long macroStartMs = 0;

    private void serviceDrive() {
        if (follower == null) return;

        double ly = -gamepad1.left_stick_y;
        // Negated 17 Nov: strafe was mirrored while forward was correct.
        double lx = -gamepad1.left_stick_x * LATERAL_SIGN;
        double rx = gamepad1.right_stick_x * SPIN_SIGN;

        // Field centric is computed in FIELD coordinates, where +y is one
        // fixed direction for everyone. The two alliances' drivers stand at
        // opposite ends, so the field's +y is "forward" to one of them and
        // "back" to the other -- which is why BLUE drove in reverse with the
        // same code. Rotate the sticks into THIS driver's frame first.
        if (DRIVER_FRAME_DEG != 0) {
            double a = Math.toRadians(DRIVER_FRAME_DEG);
            double ca = Math.cos(a), sa = Math.sin(a);
            double fx = ly * ca - lx * sa;
            double fy = ly * sa + lx * ca;
            ly = fx; lx = fy;
        }

        if (FIELD_CENTRIC) {
            double h = follower.pose().heading();
            double c = Math.cos(h), s = Math.sin(h);
            double f = ly * c + lx * s;
            double t = -ly * s + lx * c;
            ly = f; lx = t;
        }

        // A driver reaching for the stick means "I am taking over", and they
        boolean sticks = Math.abs(ly) > STICK_CANCEL
                      || Math.abs(lx) > STICK_CANCEL
                      || Math.abs(rx) > STICK_CANCEL;
        lastSticks = sticks;
        if (sticks && (autoTarget != null || seeking)) {
            autoTarget = null;
            macroFinal = null;
            seeking = false;
        }

        // Turn leg landed? Start the translation.
        serviceMacroStage();

        // A macro that has arrived releases itself, so the driver gets
        // control back.
        if (autoTarget != null && macroFinal == null
                && System.currentTimeMillis() - autoTargetMs > MACRO_SETTLE_MS
                && !follower.isBusy()) {
            autoTarget = null;
        }
        // Dead-man. A macro that cannot converge used to spin until the
        // driver fought it off the sticks; now it gives up and hands back.
        if (autoTarget != null
                && System.currentTimeMillis() - macroStartMs > MACRO_MAX_S * 1000) {
            autoTarget = null;
            macroFinal = null;
            follower.manual(0, 0, 0);
        }

        boolean rb = gamepad1.right_bumper;
        // Pressing RB means "go get that ball", so it also commands
        if (rb && autoTarget == null) {
            if (!p1rb) mech.setState(MechState.INTAKE_FIELD);
            // Re-issuing follow() every loop restarts the path before it can
            // move, so the robot sits still while the Seek line keeps saying
            // it is driving. Only re-issue once the follower is genuinely
            // idle AND the last attempt has had time to take effect.
            long now = System.currentTimeMillis();
            boolean stale = now - lastSeekMs > SEEK_REISSUE_MS;
            if (!seeking || (stale && !follower.isBusy())) {
                lastSeekMs = now;
                seekBall();
            }
        } else if (!rb) {
            seeking = false;
        }

        // Both flags, not one. Dropping autoTarget from this test is what
        // killed every macro: manual() then ran on the same loops follow()
        // was meant to own, and the last command wins.
        if (autoTarget == null && !seeking) {
            // Held, not toggled: a precision mode you can leave switched
            double scale = gamepad1.left_trigger > 0.5 ? PRECISION_SCALE : 1.0;
            follower.manual(ly * scale, lx * scale, rx * scale);
        }
        follower.update();
        p1rb = rb;
    }

    // ---- drive-to-pose, driven from GAMEPAD 2 ---------------------------

    // Cost weights for picking a shoot preset, and the macro's dead-man.
    public static double TURN_COST_IN_PER_DEG = 0.35;
    public static double CLOSE_BONUS_IN = 20.0;
    public static double MACRO_MAX_S = 5.0;
    public static double MACRO_TURN_FIRST_DEG = 45.0;
    public static double MACRO_TURN_LEG_IN = 8.0;

    private Pose macroFinal = null;

    static double wrapRad(double r) {
        while (r > Math.PI) r -= 2 * Math.PI;
        while (r <= -Math.PI) r += 2 * Math.PI;
        return r;
    }

    // linear(to, to) holds the target heading from the first instant. That is
    // right for a short hop, but from across the field it asks the robot to
    // spin and translate at once, which this drivetrain cannot do -- it just
    // spins. So a big turn gets its own short leg first, exactly like the
    // autos, and the long leg that follows is pure translation.
    private void driveToPose(Pose to, String name) {
        if (follower == null) return;
        Pose here = follower.pose();
        seeking = false;
        autoTarget = name;
        autoTargetMs = System.currentTimeMillis();
        macroStartMs = autoTargetMs;

        double dh = Math.abs(Math.toDegrees(wrapRad(to.heading() - here.heading())));
        double gap = Math.hypot(to.x() - here.x(), to.y() - here.y());
        if (dh > MACRO_TURN_FIRST_DEG && gap > MACRO_TURN_LEG_IN * 2) {
            double f = MACRO_TURN_LEG_IN / gap;
            Pose mid = new Pose(here.x() + (to.x() - here.x()) * f,
                                here.y() + (to.y() - here.y()) * f,
                                to.heading());
            macroFinal = to;
            follower.follow(Paths.line(here, mid).linear(to, to));
            return;
        }
        macroFinal = null;
        follower.follow(Paths.line(here, to).linear(to, to));
    }

    // Stage two, once the turn leg has landed.
    private void serviceMacroStage() {
        if (macroFinal == null || follower == null || follower.isBusy()) return;
        if (System.currentTimeMillis() - autoTargetMs <= MACRO_SETTLE_MS) return;
        Pose to = macroFinal;
        macroFinal = null;
        follower.follow(Paths.line(follower.pose(), to).linear(to, to));
        autoTargetMs = System.currentTimeMillis();
    }

    // Three per CELL; pick by which CELL is raised, then by distance.
    private void driveToNearestShoot() {
        if (follower == null) return;
        Pose here = follower.pose();
        ShooterAdapter.Target tgt = vision.getRaisedTarget();
        if (tgt == null) tgt = shooter.hiveTargetForPosition(here.x(), here.y());
        Pose[] opts;
        String[] names;
        // All four per cell: the close pose plus the three spread ones, so a
        // blocked spot still has somewhere to go.
        if (tgt == ShooterAdapter.Target.AUDIENCE_SIDE) {
            opts = new Pose[] { Blue.SHOOT_CLOSE_AUD, Blue.SHOOT_AUD_MID,
                                Blue.SHOOT_AUD_WEST, Blue.SHOOT_AUD_EAST };
            names = new String[] { "AUD_CLOSE", "AUD_MID", "AUD_W", "AUD_E" };
        } else if (tgt == ShooterAdapter.Target.SCORING_SIDE) {
            opts = new Pose[] { Blue.SHOOT_CLOSE_SCO, Blue.SHOOT_SCO_MID,
                                Blue.SHOOT_SCO_WEST, Blue.SHOOT_SCO_EAST };
            names = new String[] { "SCO_CLOSE", "SCO_MID", "SCO_W", "SCO_E" };
        } else {
            opts = new Pose[] { Blue.SHOOT_CLOSE_AUD, Blue.SHOOT_CLOSE_SCO };
            names = new String[] { "AUD_CLOSE", "SCO_CLOSE" };
        }
        // Cost, not raw distance. A preset 10 in nearer but 120 deg round is
        // a worse trip than one slightly further that the robot already
        // faces, because the turn is what this drivetrain is slow at.
        // CLOSE gets a bonus because it is the pose that actually scores.
        int best = 0;
        double bestC = Double.MAX_VALUE;
        for (int i = 0; i < opts.length; i++) {
            double d = Math.hypot(opts[i].x() - here.x(), opts[i].y() - here.y());
            double dh = Math.abs(Math.toDegrees(wrapRad(
                    opts[i].heading() - here.heading())));
            double c = d + dh * TURN_COST_IN_PER_DEG + (i == 0 ? -CLOSE_BONUS_IN : 0);
            if (c < bestC) { bestC = c; best = i; }
        }
        driveToPose(opts[best], names[best]);
    }

    private void driveToNearestFlower() {
        if (follower == null) return;
        Pose here = follower.pose();
        // All four, not this alliance's two. ShooterAdapter's flowerZones
        // already cover the whole field, and looking at only two is why this
        // kept driving to the same flower whatever the robot was near.
        Pose[] opts = { Blue.FLOWER_RED_AUD, Blue.FLOWER_RED_SCO,
                        Blue.FLOWER_BLU_AUD, Blue.FLOWER_BLU_SCO };
        String[] names = { "FLW_R_AUD", "FLW_R_SCO", "FLW_B_AUD", "FLW_B_SCO" };
        int best = 0;
        double bestC = Double.MAX_VALUE;
        for (int i = 0; i < opts.length; i++) {
            double d = Math.hypot(opts[i].x() - here.x(), opts[i].y() - here.y());
            double dh = Math.abs(Math.toDegrees(wrapRad(
                    opts[i].heading() - here.heading())));
            double c = d + dh * TURN_COST_IN_PER_DEG;
            if (c < bestC) { bestC = c; best = i; }
        }
        driveToPose(opts[best], names[best]);
    }

    private static int clampCount(int v) { return v < 0 ? 0 : (v > 4 ? 4 : v); }

    // Why a seek did nothing, in one line.
    private void seekBall() {
        Pose here = follower.pose();
        if (!vision.isBallDetectionEnabled()) {
            // serviceBallVision() only enables it from mech.update(), which
            // runs AFTER this. Turn it on now so the first press is not wasted.
            vision.setBallDetectionEnabled(true);
            seekWhyMs = System.currentTimeMillis();
            seekWhy = "camera off -- just enabled, hold RB";
            seeking = false;
            return;
        }
        BallDetector.Detection d = vision.nearestWantedBall(mech.allianceTeamNectar());
        if (d == null) {
            // "Nothing wanted" and "nothing detected at all" are different
            // faults: the first is the colour filter, the second is the HSV
            // gates or SIZE_CHECK, which assumes the ball is on the floor.
            BallDetector bd = vision.getBallDetector();
            int n = (bd == null) ? -1 : bd.getDetections().size();
            seekWhyMs = System.currentTimeMillis();
            seekWhy = (n <= 0)
                    ? String.format(Locale.US,
                        "detector sees NOTHING (%d) -- HSV gates, or SIZE_CHECK %s",
                        n, BallDetector.SIZE_CHECK ? "ON: hold it on the floor" : "off")
                    : String.format(Locale.US,
                        "%d detected but none wanted -- want POLLEN or nectar id %d",
                        n, mech.allianceTeamNectar());
            seeking = false;
            return;
        }

        // The same helper autonomous uses, so the intake offset
        Pose to = BallSeek.targetFor(here, d);
        if (to == null) {
            double gap = Math.hypot(d.fieldX - here.x(), d.fieldY - here.y());
            seekWhyMs = System.currentTimeMillis();
            seekWhy = String.format("ball %.0f in away, move < %.0f in -- already on it",
                    gap, BallSeek.MIN_TRAVEL_IN);
            seeking = false;
            return;
        }
        double move = Math.hypot(to.x() - here.x(), to.y() - here.y());
        seekIssues++;
        follower.follow(Paths.line(here, to).linear(here, to));
        seeking = true;
        // Latched after the follow, so it survives releasing RB and says
        // whether the follower ever accepted the path.
        seekWhyMs = System.currentTimeMillis();
        seekWhy = String.format(Locale.US,
                "drive %.0f in to %s  #%d  busyAfterFollow=%s",
                move, d.type, seekIssues, follower.isBusy());
    }}
