package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.field.MatchState;
import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;
import org.firstinspires.ftc.teamcode.robot.TurretController;
import org.firstinspires.ftc.teamcode.robot.VisionController;
import org.firstinspires.ftc.teamcode.robot.BallSeek;

public abstract class Auto_Base extends OpMode {

    // A step's path is built when the step LAUNCHES, not when the route
    protected interface PathSource { Path build(); }

    protected static final class Step {
        final PathSource source;
        final MechState onStart, onArrive;
        final boolean repeat;
        final boolean waitForMech;

        public Step(Path path, MechState onArrive) { this(path, null, onArrive); }

        public Step(Path path, MechState onStart, MechState onArrive) {
            this(() -> path, onStart, onArrive, false, false);
        }

        public Step(PathSource source, MechState onStart, MechState onArrive, boolean repeat) {
            this(source, onStart, onArrive, repeat, false);
        }

        public Step(PathSource source, MechState onStart, MechState onArrive,
                    boolean repeat, boolean waitForMech) {
            this.source = source;
            this.onStart = onStart;
            this.onArrive = onArrive;
            this.repeat = repeat;
            this.waitForMech = waitForMech;
        }
    }

    // Drive there AND finish the volley before moving on.
    protected Step shootLeg(Pose a, Pose b) {
        Path p = leg(a, b);
        return new Step(() -> p, MechState.SHOOTING_HIVE, null, false, true);
    }

    protected Step shootCurve(Pose a, Pose control, Pose b) {
        Path p = curve(a, control, b);
        return new Step(() -> p, MechState.SHOOTING_HIVE, null, false, true);
    }

    // START FROM WHEREVER THE ROBOT ACTUALLY IS.
    protected Step shootTo(Pose to) {
        return new Step(() -> legConst(follower.pose(), to),
                MechState.SHOOTING_HIVE, null, false, true);
    }

    protected Step driveTo(Pose to, MechState onStart, MechState onArrive) {
        return new Step(() -> legConst(follower.pose(), to), onStart, onArrive, false);
    }

    // Hold the TARGET heading for the whole leg instead of interpolating from
    // the live one. Measured 19 Nov: this drivetrain tracks a fixed heading
    // cleanly but oscillates on a moving one. leg() from two fixed poses with
    // equal headings is already constant; a leg from follower.pose() never is,
    // because the live heading is never exactly the target's. linear(b, b)
    // interpolates b to b, so the setpoint is a step at the start that the
    // controller nulls, not a ramp it chases the whole way.
    protected static Path legConst(Pose a, Pose b) {
        return Paths.line(a, b).linear(b, b);
    }

    // What is in the robot at start, on every route. Four pollen fills the
    // magazine -- the counts clamp at 4 total, so there is no room for nectar
    // alongside them. Override the two methods on a route that starts empty.
    public static int PRELOAD_POLLEN = 4;
    public static int PRELOAD_NECTAR = 0;

    protected int preloadPollen() { return PRELOAD_POLLEN; }
    protected int preloadNectar() { return PRELOAD_NECTAR; }

    // Give up waiting rather than hang the whole route.
    public static double MECH_WAIT_MAX_S = 5.0;
    private long mechWaitStart = 0;

    private boolean mechDone() {
        long now = System.currentTimeMillis();
        if (mechWaitStart == 0) mechWaitStart = now;
        if (mech.isOperationComplete() || (now - mechWaitStart) / 1000.0 > MECH_WAIT_MAX_S) {
            mechWaitStart = 0;
            return true;
        }
        return false;
    }

    protected RobotHardware robot;
    protected MechController mech;
    protected VisionController vision;
    protected ShooterAdapter shooter;
    protected TurretController turret;
    protected Follower follower;

    protected ElapsedTime pathTimer = new ElapsedTime();
    protected ElapsedTime opModeTimer = new ElapsedTime();

    private Step[] steps = new Step[0];
    private int index = 0;
    private boolean launched = false;
    private boolean routeReleased = false;
    private boolean entered = false;
    private long emptySince = 0;

    // MEASURED 21 Nov with the eight TestPure/TestBase runs: this drivetrain
    // holds a fixed heading cleanly and oscillates on a moving one, on
    // straight lines and curves alike. headingFeedback is a pure proportional
    // controller, so a ramp leaves a standing error it cannot null -- there
    // is no gain that fixes that, only feedforward or a piecewise gain.
    //
    // So every path holds its TARGET heading throughout. The turn happens as
    // a step at the start, which the controller does null, instead of being
    // dragged out along the path. Set false to go back to interpolation.
    public static boolean CONSTANT_HEADING = true;

    protected static Path leg(Pose a, Pose b) {
        if (CONSTANT_HEADING) return Paths.line(a, b).linear(b, b);
        return Paths.line(a, b).linear(a, b);
    }

    protected static Path curve(Pose a, Pose control, Pose b) {
        if (CONSTANT_HEADING) return Paths.curve(a, control, b).linear(b, b);
        return Paths.curve(a, control, b).linear(a, b);
    }

    // Tangent heading IS a ramp by definition, so it cannot be used at all
    // until the heading controller is fixed. Falls back to constant.
    protected static Path curveTangent(Pose a, Pose control, Pose b) {
        if (CONSTANT_HEADING) return Paths.curve(a, control, b).linear(b, b);
        return Paths.curve(a, control, b).tangent();
    }

    protected abstract ShooterAdapter.Alliance alliance();
    protected abstract Pose startPose();
    protected abstract Step[] buildSteps();

    private void saveMatchState(Pose e) {
        MatchState.save(
                alliance() == ShooterAdapter.Alliance.RED ? Red.ALLIANCE : Blue.ALLIANCE,
                e.x(), e.y(), Math.toDegrees(e.heading()),
                mech.countPollen, mech.countNectar, mech.isDeployed(),
                mech.cellPollenIn, mech.cellNectarIn);
    }


    @Override
    public void init() {
        robot   = new RobotHardware(hardwareMap, telemetry);
        shooter = new ShooterAdapter(alliance());
        vision  = new VisionController(robot, shooter);
        turret  = new TurretController(robot);
        mech    = new MechController(robot, vision, shooter, turret);

        vision.initAprilTag();
        vision.startHiveAprilTagPipeline();
        mech.setState(MechState.START);
        mech.initPositions();
        // Both counts default to 0, and a volley counts down from them, so a
        // route that shoots a hand-loaded preload fires nothing unless it
        // says what is aboard. Routes that start empty leave this at 0.
        mech.countPollen = preloadPollen();
        mech.countNectar = preloadNectar();
        turret.beginHoming();

        // create() builds a real Follower as of 14 Nov.
        follower = Constants.create(hardwareMap);
        if (follower != null) follower.setPose(startPose());

        steps = buildSteps();
        telemetry.addLine(follower == null
                ? "FOLLOWER IS NULL -- no drivetrain, mech only"
                : "ready, " + steps.length + " steps");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        turret.update();
        telemetry.addData("Turret", turret.getHomeMessage());
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.reset();
        pathTimer.reset();
    }

    @Override
    public void loop() {
        // No follower: skip DRIVING, not the mechanism.
        if (follower == null) {
            telemetry.addLine("FOLLOWER IS NULL -- mech only, cannot drive");
            mech.update();
            mech.allTelemetry();
            return;
        }
        follower.update();
        feedPose();
        advance();
        mech.update();

        // The route is over but update() keeps holding the last path's end
        // pose, which jitters until STOP. Hand the drive back once, and keep
        // calling update() so the pose stays live for MatchState.
        if (index >= steps.length && !routeReleased) {
            follower.manual(0, 0, 0);
            routeReleased = true;
        }

        telemetry.addData("Step", index + " / " + steps.length);
        telemetry.addData("Path s", "%.1f", pathTimer.seconds());
        telemetry.addData("Match s", "%.1f", opModeTimer.seconds());
        mech.allTelemetry();
    }

    private void feedPose() {
        Pose p = follower.pose();
        double headingDeg = Math.toDegrees(p.heading());
        mech.updatePose(p.x(), p.y(), headingDeg);
        vision.setPose(p.x(), p.y(), headingDeg);
    }

    private void advance() {
        if (index >= steps.length) return;
        Step s = steps[index];

        if (!launched) {
            // onStart BEFORE the source runs.
            if (!entered && s.onStart != null) mech.setState(s.onStart);
            entered = true;

            Path p = s.source.build();
            if (p == null) {
                // A repeating step gets a grace window before "nothing here"
                if (s.repeat) {
                    long now = System.currentTimeMillis();
                    if (emptySince == 0) emptySince = now;
                    if ((now - emptySince) / 1000.0 < SWEEP_ACQUIRE_S) return;
                }
                if (s.onArrive != null) mech.setState(s.onArrive);
                index++; entered = false;
                emptySince = 0; mechWaitStart = 0;
                return;
            }
            emptySince = 0;
            follower.follow(p);
            launched = true;
            pathTimer.reset();
            return;
        }

        if (follower.isBusy()) return;
        if (s.waitForMech && !mechDone()) return;
        launched = false;
        if (s.repeat) return;             // re-query the source next loop
        if (s.onArrive != null) mech.setState(s.onArrive);
        index++;
        // Off the wall now, so the intake arm may swing out.
        mech.armDeploy();
        entered = false;
        mechWaitStart = 0;
    }

    // ---- ball sweep --------------------------------------------------------

    // How long to keep looking before believing there is nothing here.
    public static double SWEEP_ACQUIRE_S = 1.0;
    public static int SWEEP_MAX_BALLS = 4;
    public static double SWEEP_BUDGET_S = 8.0;

    // Approach geometry and the degenerate-leg guard live in BallSeek, shared
    protected Step sweep() {
        final ElapsedTime budget = new ElapsedTime();
        final boolean[] fresh = { true };
        return new Step(() -> {
            if (fresh[0]) { budget.reset(); fresh[0] = false; }
            if (mech.countPollen + mech.countNectar >= SWEEP_MAX_BALLS) return null;
            if (budget.seconds() > SWEEP_BUDGET_S) return null;
            Pose here = follower.pose();
            Pose to = BallSeek.targetFor(here,
                    vision.nearestWantedBall(mech.allianceTeamNectar()));
            // Null covers no detection, a detection with no pose behind
            if (to == null) return null;
            return leg(here, to);
        }, MechState.INTAKE_FIELD, null, true);
    }

    @Override
    public void stop() {
        if (follower != null && mech != null) saveMatchState(follower.pose());
        if (mech != null) mech.stopAll();
        if (vision != null) vision.stop();
    }
}
