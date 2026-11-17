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

    // A step's path is built when the step LAUNCHES, not when the route is
    // declared. Fixed legs ignore that and hand back the same Path; a sweep
    // uses it to aim at a ball the camera can only see once the robot is
    // there. Returning null means "nothing left here" and the route moves on.
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
    //
    // Arriving is not scoring. A plain Step advances the moment the PATH is
    // done, so a shooting step commanded at launch was cut off the instant
    // the robot arrived -- the next step immediately set another state and
    // drove away. With the steadiness gate holding fire until the robot
    // stops, that left no window at all: autonomous would have driven the
    // whole route without scoring once.
    protected Step shootLeg(Pose a, Pose b) {
        Path p = leg(a, b);
        return new Step(() -> p, MechState.SHOOTING_HIVE, null, false, true);
    }

    protected Step shootCurve(Pose a, Pose control, Pose b) {
        Path p = curve(a, control, b);
        return new Step(() -> p, MechState.SHOOTING_HIVE, null, false, true);
    }

    // START FROM WHEREVER THE ROBOT ACTUALLY IS.
    //
    // The fixed helpers above bake their start pose in at buildSteps() time,
    // which is init(). That chains correctly only while every step ends where
    // the next one begins -- true for a fixed route, false the moment a
    // sweep() is in it, because a sweep ends at the last ball it found, not
    // at the pose that was declared before the match.
    //
    // Use these for ANY step that follows a dynamic one. The start is read at
    // launch, so the path always begins under the robot.
    protected Step shootTo(Pose to) {
        return new Step(() -> leg(follower.pose(), to),
                MechState.SHOOTING_HIVE, null, false, true);
    }

    protected Step driveTo(Pose to, MechState onStart, MechState onArrive) {
        return new Step(() -> leg(follower.pose(), to), onStart, onArrive, false);
    }

    // Give up waiting rather than hang the whole route. A volley that cannot
    // finish -- jam, miscount, no shot from here -- must not cost the match.
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
    private boolean entered = false;
    private long emptySince = 0;

    // Path helpers. Paths.line/curve is the public 3.x factory; routes should
    // not construct AtomicPath directly.
    protected static Path leg(Pose a, Pose b) {
        return Paths.line(a, b).linear(a, b);
    }

    protected static Path curve(Pose a, Pose control, Pose b) {
        return Paths.curve(a, control, b).linear(a, b);
    }

    protected static Path curveTangent(Pose a, Pose control, Pose b) {
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
        turret.beginHoming();

        // create() builds a real Follower as of 14 Nov. It no longer returns
        // null -- it THROWS if a drivetrain motor name is missing from the
        // config, because Mecanum resolves all four here. That is a change in
        // failure mode: a name typo used to be invisible and now stops init.
        // The null guards below stay, cheaply, so a future stub cannot hand
        // the field a robot that silently does nothing.
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
        // No follower: skip DRIVING, not the mechanism. The arm still needs
        // to deploy and the turret still needs to finish homing, and both
        // live in mech.update(). Returning here outright left the robot
        // completely inert for the whole autonomous period and handed a
        // stowed arm to TeleOp.
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
            // onStart BEFORE the source runs. A sweep asks the camera, and
            // the camera is only enabled in the collecting states -- asking
            // first and commanding second means the very first query reads a
            // detector that is still switched off.
            //
            // Fires once per STEP, not once per lap, or a repeating sweep
            // would re-command the same state after every ball.
            if (!entered && s.onStart != null) mech.setState(s.onStart);
            entered = true;

            Path p = s.source.build();
            if (p == null) {
                // A repeating step gets a grace window before "nothing here"
                // is believed. Enabling the detector only takes effect in
                // mech.update(), which runs AFTER this, and the camera needs
                // a frame after that -- so the first query always comes back
                // empty and the sweep would skip itself.
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
        entered = false;
        mechWaitStart = 0;
    }

    // ---- ball sweep --------------------------------------------------------
    // Drive to balls the camera can actually see, instead of to where balls
    // are usually found. Re-targets after each one until the robot is full,
    // the budget runs out, or nothing wanted is in view.

    // How long to keep looking before believing there is nothing here.
    public static double SWEEP_ACQUIRE_S = 1.0;
    public static int SWEEP_MAX_BALLS = 4;
    public static double SWEEP_BUDGET_S = 8.0;

    // Approach geometry and the degenerate-leg guard live in BallSeek, shared
    // with the TeleOp RB handler. Tune BallSeek.APPROACH_OFFSET_IN.
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
            // Null covers no detection, a detection with no pose behind it,
            // and a ball the robot is already standing on -- the last of
            // which would otherwise be a zero-length path re-issued every
            // loop. The sweep ends; the route moves on.
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
