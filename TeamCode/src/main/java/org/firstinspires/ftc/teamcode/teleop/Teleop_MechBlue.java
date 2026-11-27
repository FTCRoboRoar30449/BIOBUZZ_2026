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

    public static double PRECISION_SCALE = 0.35;
    public static double STICK_CANCEL = 0.15;

    public static boolean FIELD_CENTRIC = true;
    public static int LATERAL_SIGN = +1;

    public static double DRIVER_FRAME_DEG = 180;

    public static int SPIN_SIGN = +1;

    private String autoTarget = null;
    private String driveNote = "";   // so a button with no motion still reports

    @Override public void init() {
        robot   = new RobotHardware(hardwareMap, telemetry);
        shooter = new ShooterAdapter(ShooterAdapter.Alliance.BLUE);
        vision  = new VisionController(robot, shooter);
        turret  = new TurretController(robot);
        mech    = new MechController(robot, vision, shooter, turret);

        vision.initAprilTag();
        vision.startHiveAprilTagPipeline();
        mech.setState(MechState.START);
        mech.initPositions(MatchState.isFresh(Blue.ALLIANCE) && MatchState.deployed);
        mech.armDeploy();

        turret.assumeHomed();

        follower = Constants.create(hardwareMap);

        telemetry.addLine("BLUE alliance. Turret at stored zero.");
        telemetry.update();
    }

    @Override public void init_loop() {
        turret.update();
        telemetry.addData("Turret", turret.getHomeMessage());
        telemetry.update();
    }

    @Override public void start() {
        if (MatchState.isFresh(Blue.ALLIANCE)) {
            mech.countPollen = MatchState.countPollen;
            mech.countNectar = MatchState.countNectar;
            mech.cellPollenIn = MatchState.cellPollenIn;
            mech.cellNectarIn = MatchState.cellNectarIn;
            seedX = MatchState.poseX; seedY = MatchState.poseY;
            seedH = MatchState.headingDeg; seeded = true;
        } else {
            seedX = Blue.START.x();
            seedY = Blue.START.y();
            seedH = Math.toDegrees(Blue.START.heading());
            seeded = false;
        }
        if (follower != null) {
            follower.setPose(new Pose(seedX, seedY, Math.toRadians(seedH)));
        }
        mech.setState(MechState.IDLE);
    }

    @Override public void loop() {
        updatePoseFromSource();

        // you do not want to be lining up by hand.
        boolean a1 = gamepad1.a && !p1a;
        boolean b1 = gamepad1.b && !p1b;
        boolean y1 = gamepad1.y && !p1y;
        if (a1) { driveToNearestShoot(); mech.setState(MechState.SHOOTING_HIVE); }
        if (y1) driveToPose(Blue.PARK, "PARK");
        if (b1) { driveToPose(Blue.GARDEN, "GARDEN"); mech.setState(MechState.INTAKE_FIELD); }
        p1a = gamepad1.a; p1b = gamepad1.b; p1y = gamepad1.y;

        boolean du1 = gamepad1.dpad_up && !p1du;
        boolean dd1 = gamepad1.dpad_down && !p1dd;
        boolean dl1 = gamepad1.dpad_left && !p1dl;
        if (du1) mech.addPartnerPollen();
        if (dd1) mech.addPartnerNectar();
        if (dl1) mech.resetCellCount();
        p1du = gamepad1.dpad_up; p1dd = gamepad1.dpad_down; p1dl = gamepad1.dpad_left;

        boolean a2 = gamepad2.a && !p2a, b2 = gamepad2.b && !p2b;
        boolean x2 = gamepad2.x && !p2x, y2 = gamepad2.y && !p2y;
        if (a2) mech.setState(MechState.SHOOTING_HIVE);
        else if (x2) mech.setState(MechState.INTAKE_FIELD);
        else if (y2) { driveToNearestFlower(); mech.setState(MechState.INTAKE_FLOWER); }
        else if (b2) mech.setState(MechState.IDLE);
        p2a = gamepad2.a; p2b = gamepad2.b; p2x = gamepad2.x; p2y = gamepad2.y;

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
        if (bk2) { mech.countPollen = 0; mech.countNectar = 0; mech.resetDipEvidence();
                   vision.resetIntakeMin(); }
        if (st2) turret.assumeHomed();
        p2bk = gamepad2.back; p2st = gamepad2.start;

        double trim = gamepad2.right_stick_x;
        if (Math.abs(trim) > 0.2) {
            turret.setTurretDeg(turret.getCommandedDeg() + trim * 2.0);
        }

        serviceDrive();
        mech.update();
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
        if (follower != null) {
            Pose p = follower.pose();
            double hd = Math.toDegrees(p.heading());
            mech.updatePose(p.x(), p.y(), hd);
            vision.setPose(p.x(), p.y(), hd);
            return;
        }
        mech.updatePose(seedX, seedY, seedH);
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
    private long seekWhyMs = System.currentTimeMillis();
    public static long SEEK_REISSUE_MS = 300;
    private long lastSeekMs = 0;
    private int seekIssues = 0;
    private boolean lastSticks = false;
    public static long MACRO_SETTLE_MS = 400;
    private long autoTargetMs = 0;
    private long macroStartMs = 0;

    private void serviceDrive() {
        if (follower == null) return;

        double ly = -gamepad1.left_stick_y;
        double lx = -gamepad1.left_stick_x * LATERAL_SIGN;
        double rx = gamepad1.right_stick_x * SPIN_SIGN;

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

        boolean sticks = Math.abs(ly) > STICK_CANCEL
                      || Math.abs(lx) > STICK_CANCEL
                      || Math.abs(rx) > STICK_CANCEL;
        lastSticks = sticks;
        if (sticks && (autoTarget != null || seeking)) {
            autoTarget = null;
            macroFinal = null;
            seeking = false;
        }

        serviceMacroStage();

        if (autoTarget != null && macroFinal == null
                && System.currentTimeMillis() - autoTargetMs > MACRO_SETTLE_MS
                && !follower.isBusy()) {
            autoTarget = null;
        }
        if (autoTarget != null
                && System.currentTimeMillis() - macroStartMs > MACRO_MAX_S * 1000) {
            autoTarget = null;
            macroFinal = null;
            follower.manual(0, 0, 0);
        }

        boolean rb = gamepad1.right_bumper;
        if (rb && autoTarget == null) {
            if (!p1rb) mech.setState(MechState.INTAKE_FIELD);
            long now = System.currentTimeMillis();
            boolean stale = now - lastSeekMs > SEEK_REISSUE_MS;
            if (!seeking || (stale && !follower.isBusy())) {
                lastSeekMs = now;
                seekBall();
            }
        } else if (!rb) {
            seeking = false;
        }

        if (autoTarget == null && !seeking) {
            double scale = gamepad1.left_trigger > 0.5 ? PRECISION_SCALE : 1.0;
            follower.manual(ly * scale, lx * scale, rx * scale);
        }
        follower.update();
        p1rb = rb;
    }

    // ---- drive-to-pose, driven from GAMEPAD 2 ---------------------------

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

    private void serviceMacroStage() {
        if (macroFinal == null || follower == null || follower.isBusy()) return;
        if (System.currentTimeMillis() - autoTargetMs <= MACRO_SETTLE_MS) return;
        Pose to = macroFinal;
        macroFinal = null;
        follower.follow(Paths.line(follower.pose(), to).linear(to, to));
        autoTargetMs = System.currentTimeMillis();
    }

    private void driveToNearestShoot() {
        if (follower == null) return;
        Pose here = follower.pose();
        ShooterAdapter.Target tgt = vision.getRaisedTarget();
        if (tgt == null) tgt = shooter.hiveTargetForPosition(here.x(), here.y());
        Pose[] opts;
        String[] names;
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

    private void seekBall() {
        Pose here = follower.pose();
        if (!vision.isBallDetectionEnabled()) {
            vision.setBallDetectionEnabled(true);
            seekWhyMs = System.currentTimeMillis();
            seekWhy = "camera off -- just enabled, hold RB";
            seeking = false;
            return;
        }
        BallDetector.Detection d = vision.nearestWantedBall(mech.allianceTeamNectar());
        if (d == null) {
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
        seekWhyMs = System.currentTimeMillis();
        seekWhy = String.format(Locale.US,
                "drive %.0f in to %s  #%d  busyAfterFollow=%s",
                move, d.type, seekIssues, follower.isBusy());
    }}
