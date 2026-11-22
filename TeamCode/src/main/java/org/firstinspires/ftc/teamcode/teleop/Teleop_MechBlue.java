package org.firstinspires.ftc.teamcode.teleop;

import java.util.Locale;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.field.MatchState;
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

    private boolean p1a, p1b, p1du, p1dd, p1dl;
    private boolean p2a, p2x, p2b, p2y, p2du, p2dd, p2dl, p2dr;
    private boolean p2lb, p2rb, p2bk, p2st, p2lt, p2rt, p2lsb;

    // Drive feel. PRECISION_SCALE is held, not toggled -- a held modifier
    public static double PRECISION_SCALE = 0.35;
    // Above this on any stick, an automated drive hands back to the driver.
    public static double STICK_CANCEL = 0.15;

    // true rotates the sticks into the field frame; only as good as the heading.
    public static boolean FIELD_CENTRIC = true;
    public static int LATERAL_SIGN = +1;   // flip if strafe mirrors again

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

        telemetry.addLine("RED alliance. Homing turret...");
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
        // A parks. Worth 5 points, and the end of a match is exactly when you
        // do not want to be lining up by hand. Stick cancels it like any macro.
        boolean a1 = gamepad1.a && !p1a;
        boolean b1 = gamepad1.b && !p1b;
        if (a1) driveToPose(Blue.PARK, "PARK");
        // You only drive to the GARDEN to collect, so arm the intake too,
        // same as RB and as gamepad 2's flower button.
        if (b1) { driveToPose(Blue.GARDEN, "GARDEN"); mech.setState(MechState.INTAKE_FIELD); }
        p1a = gamepad1.a; p1b = gamepad1.b;

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
        if (a2) { driveToNearestShoot(); mech.setState(MechState.SHOOTING_HIVE); }
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
        telemetry.addData("Seek", seekWhy);
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

    private void serviceDrive() {
        if (follower == null) return;

        double ly = -gamepad1.left_stick_y;
        // Negated 17 Nov: strafe was mirrored while forward was correct.
        double lx = -gamepad1.left_stick_x * LATERAL_SIGN;
        double rx = gamepad1.right_stick_x;

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
            seeking = false;
        }

        // A macro that has arrived releases itself, so the driver gets
        // control back.
        if (autoTarget != null
                && System.currentTimeMillis() - autoTargetMs > MACRO_SETTLE_MS
                && !follower.isBusy()) {
            autoTarget = null;
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

    private void driveToPose(Pose to, String name) {
        if (follower == null) return;
        Pose here = follower.pose();
        // linear(to, to) holds the target heading for the whole move: this
        // drivetrain oscillates on a moving heading setpoint.
        follower.follow(Paths.line(here, to).linear(to, to));
        seeking = false;
        autoTarget = name;
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
        if (tgt == ShooterAdapter.Target.AUDIENCE_SIDE) {
            opts = new Pose[] { Blue.SHOOT_AUD_WEST, Blue.SHOOT_AUD_MID, Blue.SHOOT_AUD_EAST };
            names = new String[] { "AUD_W", "AUD_MID", "AUD_E" };
        } else if (tgt == ShooterAdapter.Target.SCORING_SIDE) {
            opts = new Pose[] { Blue.SHOOT_SCO_WEST, Blue.SHOOT_SCO_MID, Blue.SHOOT_SCO_EAST };
            names = new String[] { "SCO_W", "SCO_MID", "SCO_E" };
        } else {
            opts = new Pose[] { Blue.SHOOT_AUD_MID, Blue.SHOOT_SCO_MID };
            names = new String[] { "AUD_MID", "SCO_MID" };
        }
        int best = 0;
        double bestD = Double.MAX_VALUE;
        for (int i = 0; i < opts.length; i++) {
            double d = Math.hypot(opts[i].x() - here.x(), opts[i].y() - here.y());
            if (d < bestD) { bestD = d; best = i; }
        }
        driveToPose(opts[best], names[best]);
    }

    private void driveToNearestFlower() {
        if (follower == null) return;
        Pose here = follower.pose();
        Pose[] opts = { Blue.FLOWER_NEAR, Blue.FLOWER_FAR };
        String[] names = { "FLOWER_NEAR", "FLOWER_FAR" };
        int best = 0;
        double bestD = Double.MAX_VALUE;
        for (int i = 0; i < opts.length; i++) {
            double d = Math.hypot(opts[i].x() - here.x(), opts[i].y() - here.y());
            if (d < bestD) { bestD = d; best = i; }
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
        seekWhy = String.format(Locale.US,
                "drive %.0f in to %s  #%d  busyAfterFollow=%s",
                move, d.type, seekIssues, follower.isBusy());
    }}
