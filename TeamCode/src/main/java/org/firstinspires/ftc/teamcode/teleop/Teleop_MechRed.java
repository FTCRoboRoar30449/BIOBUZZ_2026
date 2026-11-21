package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.MatchState;
import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.BallSeek;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;
import org.firstinspires.ftc.teamcode.robot.TurretController;
import org.firstinspires.ftc.teamcode.robot.VisionController;
import java.util.Locale;

@TeleOp(name = "TeleopMech RED", group = "Test")
public class Teleop_MechRed extends OpMode {

    public static double BENCH_X = 54.0, BENCH_Y = 114.0, BENCH_HEADING_DEG = 2.7;

    public static boolean USE_POSE_PRESETS = true;

    private static final double[][] POSES = {
        //    x,      y,  heading
        {  54.0, 114.0,    2.7 },   // P1  hive SCORING_SIDE
        {  54.0,  20.0,  180.0 },   // P2  hive AUDIENCE_SIDE
        {   4.0,  48.0,    0.0 },   // P5  flower red/audience, range 0
        {  48.0, 140.0,    0.0 },   // P6  flower red/scoring
        {  96.0,   4.0,    0.0 },   // P7  flower blue/audience
        { 140.0,  96.0,    0.0 },   // P8  flower blue/scoring
        {   4.0,  49.5,    0.0 },   // P9  1.5 in -- scoop AND intake
        {   4.0,  52.0,    0.0 },   // P10 4.0 in -- scoop, no intake
        {   4.0,  54.0,    0.0 },   // P11 6.0 in -- scoop folded
        {  48.0,  40.0,    0.0 },   // P12 overlap, nearest = 4,48
        {  96.0, 120.0,    0.0 },   // P13 overlap, nearest = 140,96
        {  72.0,  72.0,    0.0 },   // P14 no zone at all
        {  54.0, 100.0,    0.0 },   // P15 between the two hive zones
        {   4.0,  96.0,    0.0 }    // P16 on the zone edge, inclusive
    };
    private static final String[] POSE_NAMES = {
        "P1  hive SCORING_SIDE", "P2  hive AUDIENCE_SIDE",
        "P5  flower red/aud",    "P6  flower red/scoring",
        "P7  flower blue/aud",   "P8  flower blue/scoring",
        "P9  1.5in scoop+intake","P10 4.0in scoop only",
        "P11 6.0in folded",      "P12 overlap -> 4,48",
        "P13 overlap -> 140,96", "P14 nowhere",
        "P15 between zones",     "P16 zone edge"
    };
    private int poseIdx = 0;

    // Autonomous end pose, carried into TeleOp.
    private double seedX, seedY, seedH;
    private boolean seeded = false;

    private RobotHardware robot;
    private MechController mech;
    private VisionController vision;
    private ShooterAdapter shooter;
    private TurretController turret;

    private boolean p1a, p1x, p1b, p1y, p1du, p1dd, p1dl, p1bk, p1lb;
    private boolean p2a, p2x, p2b, p2y, p2du, p2dd, p2dl, p2dr;
    private boolean p2lb, p2rb, p2bk, p2st, p2lt, p2rt, p2lsb;
    private boolean p1st;

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
    private Double spinRefX = null, spinRefY = null;
    private double spinTurns = 0, spinLastH = 0;

    @Override public void init() {
        robot   = new RobotHardware(hardwareMap, telemetry);
        shooter = new ShooterAdapter(ShooterAdapter.Alliance.RED);
        vision  = new VisionController(robot, shooter);
        turret  = new TurretController(robot);
        mech    = new MechController(robot, vision, shooter, turret);

        vision.initAprilTag();
        vision.startHiveAprilTagPipeline();
        mech.setState(MechState.START);
        // Beds to the resting pattern now, not when an intake state
        mech.initPositions(MatchState.isFresh(Red.ALLIANCE) && MatchState.deployed);

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
        if (MatchState.isFresh(Red.ALLIANCE)) {
            mech.countPollen = MatchState.countPollen;
            mech.countNectar = MatchState.countNectar;
            mech.cellPollenIn = MatchState.cellPollenIn;
            mech.cellNectarIn = MatchState.cellNectarIn;
            // Seeded ONCE.
            seedX = MatchState.poseX; seedY = MatchState.poseY;
            seedH = MatchState.headingDeg; seeded = true;
        } else {
            // WITHOUT THIS the three seed fields keep their Java defaults
            seedX = Red.START.x();
            seedY = Red.START.y();
            seedH = Math.toDegrees(Red.START.heading());
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
        serviceSpinTest();

        // ================= GAMEPAD 1 -- DRIVER ===========================
        boolean a1 = gamepad1.a && !p1a, b1 = gamepad1.b && !p1b;
        boolean x1 = gamepad1.x && !p1x, y1 = gamepad1.y && !p1y;
        if (a1) driveToNearestShoot();
        else if (b1) driveToPose(Red.GARDEN, "GARDEN");
        else if (x1) {
            driveNote = autoTarget != null ? "cancelled " + autoTarget
                                           : "nothing to cancel";
            autoTarget = null;
        }
        else if (y1) reseedPose();
        p1a = gamepad1.a; p1b = gamepad1.b; p1x = gamepad1.x; p1y = gamepad1.y;

        // Partner cell scoring.
        boolean du1 = gamepad1.dpad_up && !p1du;
        boolean dd1 = gamepad1.dpad_down && !p1dd;
        boolean dl1 = gamepad1.dpad_left && !p1dl;
        if (du1) mech.addPartnerPollen();
        if (dd1) mech.addPartnerNectar();
        if (dl1) mech.resetCellCount();
        p1du = gamepad1.dpad_up; p1dd = gamepad1.dpad_down; p1dl = gamepad1.dpad_left;

        // ---- bench pose preset, also the re-seed source -----------------
        boolean bk1 = gamepad1.back && !p1bk;
        boolean lb1 = gamepad1.left_bumper && !p1lb;
        if (bk1) poseIdx = (poseIdx + 1) % POSES.length;
        if (lb1) poseIdx = (poseIdx + POSES.length - 1) % POSES.length;
        p1bk = gamepad1.back; p1lb = gamepad1.left_bumper;

        // ================= GAMEPAD 2 -- OPERATOR =========================
        boolean a2 = gamepad2.a && !p2a, b2 = gamepad2.b && !p2b;
        boolean x2 = gamepad2.x && !p2x, y2 = gamepad2.y && !p2y;
        if (a2) mech.setState(MechState.SHOOTING_HIVE);
        else if (x2) mech.setState(MechState.INTAKE_FIELD);
        else if (y2) mech.setState(MechState.INTAKE_FLOWER);
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
        if (spinRefX != null && follower != null) {
            Pose sp = follower.pose();
            telemetry.addData("SPIN", "turns %.2f   drift %+.1f, %+.1f in",
                    spinTurns, sp.x() - spinRefX, sp.y() - spinRefY);
        }
        telemetry.addData("Drive", autoTarget != null
                ? "AUTO -> " + autoTarget + "   (stick or X to take over)"
                : seeking ? "BALL SEEK   (release RB to take over)"
                : gamepad1.left_trigger > 0.5 ? "manual, PRECISION" : "manual");
        if (mech.bedsDropped()) {
            telemetry.addData("BEDS", "DOWN  %.1f s left", mech.bedDropRemainingMs() / 1000.0);
        }
        telemetry.addData("Sensors", "intake %s   tunnels %s",
                MechController.USE_INTAKE_COLOR ? "ON, auto count" : "OFF, count by hand",
                MechController.USE_TUNNEL_SENSORS ? "ON" : "OFF, index on LB/RB");
        telemetry.addData("Bench pose", USE_POSE_PRESETS
                ? String.format("%s   (%.1f, %.1f @ %.0f)   back/LB to change",
                        POSE_NAMES[poseIdx], POSES[poseIdx][0],
                        POSES[poseIdx][1], POSES[poseIdx][2])
                : "fixed BENCH_X/Y -- presets off");
        if (MatchState.isFresh(Red.ALLIANCE)) {
            telemetry.addData("From auto", MatchState.summary(Red.ALLIANCE));
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
        double x = BENCH_X, y = BENCH_Y, h = BENCH_HEADING_DEG;
        if (seeded) {
            x = seedX; y = seedY; h = seedH;
        } else if (USE_POSE_PRESETS) {
            x = POSES[poseIdx][0]; y = POSES[poseIdx][1]; h = POSES[poseIdx][2];
        }
        mech.updatePose(x, y, h);
        // The ball detector needs it too, or detections come back with NaN
        vision.setPose(x, y, h);
    }

    @Override public void stop() {
        if (mech != null) mech.stopAll();
        if (vision != null) vision.stop();
    }

    // ---- drivetrain ------------------------------------------------------
    private Follower follower;
    private boolean p1rb = false;
    private boolean seeking = false;

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
        if (sticks && (autoTarget != null || seeking)) {
            autoTarget = null;
            seeking = false;
        }

        // A macro that has arrived releases itself, so the driver gets
        if (autoTarget != null && !follower.isBusy()) autoTarget = null;

        boolean rb = gamepad1.right_bumper;
        // Pressing RB means "go get that ball", so it also commands
        if (rb && autoTarget == null) {
            if (!p1rb) mech.setState(MechState.INTAKE_FIELD);
            // KEEP TRYING while held, not just on the press.
            if (!seeking || !follower.isBusy()) seekBall();
        } else if (!rb) {
            seeking = false;
        }

        if (autoTarget == null && !seeking) {
            // Held, not toggled: a precision mode you can leave switched
            double scale = gamepad1.left_trigger > 0.5 ? PRECISION_SCALE : 1.0;
            follower.manual(ly * scale, lx * scale, rx * scale);
        }
        follower.update();
        p1rb = rb;
    }

    // ---- drive-to-pose macros -------------------------------------------

    private void driveToPose(Pose to, String name) {
        if (follower == null) return;
        Pose here = follower.pose();
        // linear(to, to) holds the target heading for the whole move. The live
        // heading is never exactly the preset's, so linear(here, to) gave a
        // moving setpoint, which this drivetrain oscillates on.
        follower.follow(Paths.line(here, to).linear(to, to));
        // Macros and ball seek are mutually exclusive: both own
        seeking = false;
        autoTarget = name;
    }

    // Nearest of the three shooting poses, because in TeleOp the robot can
    private void driveToNearestShoot() {
        if (follower == null) return;
        Pose here = follower.pose();
        Pose[] opts = { Red.SHOOT_NEAR, Red.SHOOT_MID, Red.SHOOT_FAR };
        String[] names = { "SHOOT_NEAR", "SHOOT_MID", "SHOOT_FAR" };
        int best = 0;
        double bestD = Double.MAX_VALUE;
        for (int i = 0; i < opts.length; i++) {
            double d = Math.hypot(opts[i].x() - here.x(), opts[i].y() - here.y());
            if (d < bestD) { bestD = d; best = i; }
        }
        driveToPose(opts[best], names[best]);
    }

    // Odometry recovery.
    private void serviceSpinTest() {
        if (follower == null) return;
        Pose p = follower.pose();
        double h = Math.toDegrees(p.heading());

        if (gamepad1.start && !p1st) {
            if (spinRefX == null) {
                spinRefX = p.x(); spinRefY = p.y();
                spinTurns = 0; spinLastH = h;
                driveNote = "spin ref set";
            } else {
                spinRefX = null; spinRefY = null;
                driveNote = "spin ref cleared";
            }
        }
        p1st = gamepad1.start;

        if (spinRefX == null) return;
        // Unwrap so turns accumulate past +/-180 instead of folding back.
        double d = h - spinLastH;
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        spinTurns += d / 360.0;
        spinLastH = h;
    }

    private void reseedPose() {
        if (follower == null) { driveNote = "no follower -- cannot re-seed"; return; }
        double x = POSES[poseIdx][0], y = POSES[poseIdx][1], h = POSES[poseIdx][2];
        follower.setPose(new Pose(x, y, Math.toRadians(h)));
        seedX = x; seedY = y; seedH = h;
        autoTarget = null;
        driveNote = String.format(Locale.US, "re-seeded to %s  %.1f, %.1f @ %.0f",
                POSE_NAMES[poseIdx], x, y, h);
    }

    private static int clampCount(int v) { return v < 0 ? 0 : (v > 4 ? 4 : v); }

    private void seekBall() {
        Pose here = follower.pose();
        // The same helper autonomous uses, so the intake offset
        Pose to = BallSeek.targetFor(here,
                vision.nearestWantedBall(mech.allianceTeamNectar()));
        if (to == null) {
            seeking = false;
            return;
        }
        follower.follow(Paths.line(here, to).linear(here, to));
        seeking = true;
    }}
