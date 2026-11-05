package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shared, non-blocking mechanism FSM. Main state logic stays in handleMechState(). */
public class MechController {
    public enum OperationStatus {
        NOT_STARTED, RUNNING, COMPLETED, ROBOT_FULL, FLOWER_EMPTY,
        VISION_UNAVAILABLE, TARGET_UNKNOWN, NOT_CONFIGURED, NO_SHOT,
        TURRET_NOT_HOMED, CANCELLED, TIMED_OUT, FAILED
    }
    private enum ConveyorType { NONE, POLLEN, NECTAR }
    private enum ShotPhase { IDLE, AIM, SPIN_UP, FEEDING, RECOVER }
    /**
     * BOTH is a real option: with both conveyor beds UP, one feed stroke sends
     * a pollen AND a nectar to their own shooters at the same time. So
     * SIMULTANEOUS mode costs one feed cycle for two balls, not two.
     */
    private enum BallToShoot { NONE, POLLEN, NECTAR, BOTH }

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private final VisionController vision;
    private final ShooterAdapter shooter;
    private final TurretController turret;

    private MechState currentState = MechState.START;
    private MechState previousState = MechState.START;
    private ShotPhase shotPhase = ShotPhase.IDLE;
    private BallToShoot activeShotBall = BallToShoot.NONE;
    private long shotPhaseStartMs = 0;
    private int feedTargetPosition = 0;
    private int shotsCompleted = 0;
    private OperationStatus operationStatus = OperationStatus.NOT_STARTED;

    // ---- robot pose, pushed in by the OpMode ------------------------------
    // MechController never drives. PedroPathing owns the follower; the OpMode
    // hands the pose down each loop so shot planning has something to work with.
    private double poseX = 72.0, poseY = 72.0, poseHeadingDeg = 0.0;
    private boolean poseValid = false;

    // ---- conveyor / storage ----------------------------------------------
    private static final int TICKS_PER_POLLEN_STORAGE = 100;
    private static final int TICKS_PER_NECTAR_STORAGE = 150;
    /** Shooting feed distances. Separate from storage transfer. Tune on robot. */
    public static int TICKS_TO_SHOOT_POLLEN = 300;
    public static int TICKS_TO_SHOOT_NECTAR = 200;
    /**
     * Feed distance when both beds are up and both balls go together. The two
     * tunnels want different distances on their own (300 / 200), so this is
     * its own constant rather than either of them -- start at the longer one
     * so the pollen definitely clears, and tune down if the nectar overshoots.
     */
    public static int TICKS_TO_SHOOT_BOTH = 300;
    private static final int MAX_CONVEYOR_ROTATION = 300;
    /*
     * BOTH SWAPPED after measuring. These divide by MAX_CONVEYOR_ROTATION to
     * become servo positions, so the larger number is the higher position.
     *
     *   pollen bed   raising the servo ENGAGES  -> UP must be the larger
     *   nectar bed   raising the servo DISENGAGES -> UP must be the smaller
     *
     * The beds are mirrored on the robot, so they really do invert opposite
     * ways. Both were previously backwards: conveyorPollen(1) drove 0.15 when
     * engage is 0.30, and conveyorNectar(1) drove 0.20 when engage is 0.15.
     * Every feed and every index moved the wrong bed.
     */
    /**
     * Bed travel, in the same /MAX_CONVEYOR_ROTATION units. The two beds are
     * mirror images, so they must sweep the SAME angle in OPPOSITE directions.
     * Derived from one constant so they cannot drift apart again.
     */
    private static final int BED_DISENGAGED = 45;
    private static final int BED_TRAVEL     = 45;

    /*
     * MEASURED directions, EQUAL travel.
     *
     *   pollen  engaging RAISES the servo  ->  engage = disengaged + travel
     *   nectar  engaging LOWERS the servo  ->  engage = disengaged - travel
     *
     * so disengaged -> engaged turns pollen CW and nectar CCW, by the same
     * amount. Both beds live in 45..90 units = 0.150..0.300 of servo range.
     *
     * The old numbers swept 45 units of pollen against only 15 of nectar --
     * three times as far on one side. That was wrong before today's direction
     * swap and survived it, because swapping a pair does not equalise it.
     */
    private static final int CONVEYOR_UP_POLLEN   = BED_DISENGAGED + BED_TRAVEL; // 90
    private static final int CONVEYOR_DOWN_POLLEN = BED_DISENGAGED;              // 45
    private static final int CONVEYOR_UP_NECTAR   = BED_DISENGAGED;              // 45
    private static final int CONVEYOR_DOWN_NECTAR = BED_DISENGAGED + BED_TRAVEL; // 90
    private static final long REVERSE_INTAKE_TIME_MS = 700;
    private static final long SENSOR_CLEAR_TIMEOUT_MS = 1800;
    private static final double CONVEYOR_POWER = 0.65;
    /**
     * Full power. This was 0.75 for no recorded reason. The robot is built to
     * score aggressively and a faster intake is strictly better here -- the
     * ball is caught by the tunnel, not by the intake's gentleness.
     *
     * Back it off only if balls bounce back out, or if stall current trips the
     * hub when one jams.
     */
    public static double INTAKE_POWER = 1.0;

    // ---- START deployment --------------------------------------------------
    /**
     * The intake arm is folded for inspection and has to swing out once, at
     * the start of the match. It is driven open loop for a fixed time because
     * intakeMot has NO usable encoder -- that port carries the turret's
     * through-bore.
     *
     * Approximately one rotation is enough; this does not need to be exact.
     * TUNE `DEPLOY_MS` on the real arm, and remember a fresh battery swings it
     * further than a flat one.
     *
     * The scoop must not move until the arm is clear of it, hence the second
     * delay. Both are public so they can be adjusted without a rebuild.
     */
    public static long DEPLOY_MS = 600;
    public static double DEPLOY_POWER = 0.60;
    public static long DEPLOY_SCOOP_DELAY_MS = 250;

    private long deployStartMs = 0;
    private boolean deployDone = false;

    // ---- flower approach ---------------------------------------------------
    /**
     * The scoop swings out past the robot's footprint, so it needs clear air
     * before it deploys and clear air again before it folds back. Driving it
     * while the robot is hard against the flower jams it.
     *
     * Distances are robot centre to the flower, inches:
     *
     *   > DEPLOY_IN      scoop stays at REST
     *   <= DEPLOY_IN     scoop goes to SCOOP      (5 in of clearance)
     *   <= INTAKE_IN     intake motor runs        (2 in -- essentially there)
     *   > RETRACT_IN     scoop returns to REST    (5 in, on the way out)
     *
     * DEPLOY and RETRACT are separate numbers on purpose: making them equal
     * would chatter the scoop every time the robot wobbled across 5.0 in.
     */
    public static double FLOWER_DEPLOY_IN  = 5.0;
    public static double FLOWER_INTAKE_IN  = 2.0;
    public static double FLOWER_RETRACT_IN = 5.5;

    /** Robot centre to the nearest flower, inches. NaN when not in a zone. */
    public double flowerRangeIn() {
        if (!poseValid) return Double.NaN;
        double[] f = shooter.flowerTargetForPosition(poseX, poseY);
        if (f == null) return Double.NaN;
        return Math.hypot(f[0] - poseX, f[1] - poseY);
    }

    // ---- shooting timing / tolerances -------------------------------------
    /**
     * MEASURED BARE: the shooter took 3060 ms to reach speed with no flywheel
     * fitted. This used to be 3500, leaving 440 ms of margin -- 14% -- against
     * a figure that can only get WORSE once the flywheel's inertia is added.
     * Every shot would have timed out instead of firing.
     *
     * A generous ceiling costs one slow failure; a tight one costs every shot,
     * so it errs long deliberately. Re-check once real flywheels are on: if
     * spin-up lands near this, the problem is the motor, not the timeout.
     */
    private static final long SHOOTER_SPINUP_TIMEOUT_MS = 6000;
    private static final long SHOOTER_FEED_TIMEOUT_MS = 2500;
    private static final long SHOOTER_RECOVER_MS = 250;
    /** Minimum FEEDING dwell before isBusy() is believed. See runShotSequence. */
    private static final long FEED_MIN_MS = 150;
    /** Hard ceiling on the AIM phase; the real wait is TurretController.isSettled(). */
    private static final long TURRET_SETTLE_TIMEOUT_MS = 2500;
    /**
     * Fraction of target velocity that counts as ready.
     *
     * 0.95, up from the 0.90 that was only ever that slack because a bare
     * motor with no flywheel hunts around its setpoint.
     *
     * Justified by the flywheel energy budget. A 72 mm Rhino plus
     * the 60 mm steel flywheel stores about 5.4 J at 2653 rpm; a pollen takes
     * 3.1% of the wheel's speed and a nectar 5.0%. At 43% of free speed the
     * motor claws that back quickly, so the wait is short -- and range goes as
     * the SQUARE of exit speed, so 5% of speed is ~10% of range. Inches at
     * hive distance, bought for about 100 ms.
     */
    public static double SHOOTER_READY_TOLERANCE = 0.95;
    /**
     * BENCH ONLY. The turret homing guard blocks every shooting state, which
     * on an electronics bench with no turret and no magnet means the FSM never
     * reaches the feed. Set false to exercise the sequence anyway.
     * MUST be true on the robot -- without a homed turret every commanded
     * angle is measured from an unknown zero.
     */
    public static boolean REQUIRE_HOMED = true;
    /**
     * UNCALIBRATED. Flower-intake servo endpoints.
     *
     * The servo is still loose on the bench -- it drives nothing, so there is
     * no stop to find and these two numbers are NOT measurements. They are a
     * deliberately NARROW placeholder pair, chosen so that the first time the
     * servo is bolted to the winch pulley and scoop it cannot drive anything
     * into a stop. Widen from here; do not start wide and come back.
     *
     * RE-DERIVE once the winch pulley and scoop exist, on the HardwareCheck
     * FLOWER page. What matters then is the SPAN, not the endpoints: the
     * scoop travel you get is span x servo travel x pulley circumference, so
     * changing the pulley diameter changes the span you need even if the
     * geometry of the scoop itself is untouched.
     *
     * NOTE: flowerIntake is NOT in RobotHardware.configureServos(), so it runs
     * the SDK default PWM range. If the winch turns out to need a multi-turn
     * servo, widening its range to 500-2500 roughly triples the travel a given
     * span buys, and every number here changes with it.
     */
    /*
     * SWAPPED after measuring. Raising the servo position lifts the scoop to
     * REST; lowering it drops the scoop into the flower. So REST is the HIGHER
     * number. They were the wrong way round, which would have dropped the
     * scoop on entering IDLE and lifted it to collect.
     */
    public static double FLOWER_REST = 0.465, FLOWER_SCOOP = 0.050;

    /**
     * FLOWER setpoints. The 4.0 in opening leaves a 3.6 in nectar only 0.2 in
     * of radial margin -- about +/-0.5 deg at 20 in, finer than the turret
     * resolves. It is a BACKSTOP BANK SHOT, so these are empirical starting
     * points, not a solved trajectory. Expect the tuned aim to sit past the
     * flower centre, into the backstop.
     */
    public static double FLOWER_HOOD_DEG = 80.0;
    public static double FLOWER_POLLEN_RPM = 1800.0;
    public static double FLOWER_NECTAR_RPM = 1800.0;
    /** Added to the computed turret angle to walk the shot into the backstop. */
    public static double FLOWER_AIM_TRIM_DEG = 0.0;

    // ---- inventory ---------------------------------------------------------
    public int countNectar = 0, countPollen = 0;

    /**
     * Running estimate of what WE have put into the currently raised CELL.
     * Resets by itself when the raised target changes, because that change is
     * the CELL tipping. The co-driver can add the partner's contributions.
     * Nectar is worth 4 and pollen 3 against a threshold of 24 -- fitted from
     * "3 nectar + 4 pollen" and "8 pollen", and UNVERIFIED, so this is shown
     * as information and never gates the FSM.
     */
    public int cellNectarIn = 0, cellPollenIn = 0;
    public static int CELL_NECTAR_VALUE = 4;
    public static int CELL_POLLEN_VALUE = 3;
    public static int CELL_TIP_THRESHOLD = 24;
    private ShooterAdapter.Target lastKnownTarget = null;

    private int lastConveyorNectar = -1, lastConveyorPollen = -1;
    private ConveyorType activeConveyor = ConveyorType.NONE;
    private int conveyorTargetPosition = -1;
    private long reverseStartMs = 0, wrongBallClearStartMs = 0;
    private boolean reversingWrongBall = false, waitingForWrongBallClear = false;
    private boolean rejectingFlowerNectar = false;
    private int previousFieldColor = 0, previousFlowerColor = 0;
    /** Bed-sensor edge state, so one ball produces exactly one transfer. */
    private boolean pollenBedSeen = false, nectarBedSeen = false;
    /** Consecutive re-index attempts while a bed sensor stays blocked. */
    private int pollenRetries = 0, nectarRetries = 0;
    private static final int MAX_CLEAR_RETRIES = 3;
    private long lastCountMs = 0;
    private static final long COUNT_COOLDOWN_MS = 350;

    // ---- last computed plan, for telemetry --------------------------------
    private ShooterAdapter.FirePlan lastPlan = null;
    private String shotNote = "";

    // Loop timing. Six I2C colour sensors polled every cycle is the most
    // likely thing to throttle this code, and a slow loop quietly degrades
    // every timeout and settle estimate in the FSM.
    // Per-phase shot timing. On a bench with bare shafts and an unloaded
    // conveyor every phase finishes almost instantly, which flatters the
    // cycle time badly. These numbers are what the timeouts should be set
    // from once real flywheels and a loaded conveyor are fitted.
    private long cycleStartMs = 0;
    private long lastAimMs = 0, lastSpinMs = 0, lastFeedMs = 0, lastCycleMs = 0;

    private long lastLoopMs = 0;
    private double loopMsAvg = 0;
    private double loopMsWorst = 0;

    public MechController(RobotHardware robot, VisionController vision,
                          ShooterAdapter shooter, TurretController turret) {
        this.robot = robot;
        this.telemetry = robot.telemetry;
        this.vision = vision;
        this.shooter = shooter;
        this.turret = turret;
    }

    /** Alliance nectar colour id: 2 red, 3 blue. Taken from the adapter. */
    public int allianceTeamNectar() { return shooter.allianceNectarColorId(); }

    /** Called by the OpMode every loop with the current Pedro pose. */
    public void updatePose(double xIn, double yIn, double headingDeg) {
        /*
         * Track how fast we are moving, from the pose deltas themselves.
         *
         * Needed because planHere() solves for the pose it is given, and a
         * shot is not instantaneous. A robot translating at 30 in/s has moved
         * several inches before the ball leaves the shooter, so the solution
         * it was aimed with is already wrong. turret.isSettled() cannot catch
         * this -- the turret IS settled, on an angle that stopped being
         * correct while the ball was in the barrel.
         *
         * Parked at a flower this never mattered. Firing while driving the
         * field, it decides whether the shot is worth taking.
         */
        long now = System.currentTimeMillis();
        if (poseValid && lastPoseMs != 0) {
            double dt = (now - lastPoseMs) / 1000.0;
            if (dt > 0.001) {
                double d = Math.hypot(xIn - poseX, yIn - poseY);
                double inst = d / dt;
                // Lightly smoothed: odometry jitter at 150 Hz would otherwise
                // read as motion and gate every shot.
                speedInPerSec = speedInPerSec * 0.8 + inst * 0.2;
                double dh = Math.abs(ShooterAdapter.wrap180(headingDeg - poseHeadingDeg));
                turnDegPerSec = turnDegPerSec * 0.8 + (dh / dt) * 0.2;
            }
        }
        lastPoseMs = now;

        poseX = xIn; poseY = yIn; poseHeadingDeg = headingDeg; poseValid = true;
    }

    private long lastPoseMs = 0;
    private double speedInPerSec = 0, turnDegPerSec = 0;

    /**
     * Still enough that a firing solution will still be true when the ball
     * leaves. Generous by default -- tune against real shots on a field.
     */
    public static double FIRE_MAX_SPEED_IN_S = 4.0;
    public static double FIRE_MAX_TURN_DEG_S = 15.0;

    public boolean isSteadyEnoughToFire() {
        return speedInPerSec <= FIRE_MAX_SPEED_IN_S
                && turnDegPerSec <= FIRE_MAX_TURN_DEG_S;
    }

    /*
     * READ-ONLY STATE FOR AUTONOMOUS.
     *
     * These and the handful like them -- isPreSpinning, isFlowing,
     * getPollenExits, getNectarExits, isDeployed, isCellTipped -- have no
     * caller inside this class, and that is deliberate. They are the surface
     * a Pedro autonomous will need to decide when to drive on: has the volley
     * finished, did the balls actually leave, is the scoop clear, has the
     * CELL tipped.
     *
     * Do not delete them as dead code. The telemetry reads the private fields
     * directly, so nothing here is a duplicate accessor for debugging.
     */
    public double getSpeedInPerSec() { return speedInPerSec; }

    public void update() {
        long nowMs = System.currentTimeMillis();
        if (lastLoopMs != 0) {
            double dt = nowMs - lastLoopMs;
            loopMsAvg = loopMsAvg == 0 ? dt : (loopMsAvg * 0.9 + dt * 0.1);
            if (dt > loopMsWorst) loopMsWorst = dt;
        }
        lastLoopMs = nowMs;

        turret.update();

        /*
         * POLL THE LIMELIGHT EVERY LOOP, in every state.
         *
         * updateRaisedTarget() was called only from pre-spin, continuous fire
         * and SHOOTING_HIVE. So in IDLE -- which is where you sit while
         * watching telemetry -- nothing polled the camera at all, the cached
         * target stayed null, and the Target line reported "(by zone)" no
         * matter what the Limelight could see. That reads as a broken camera
         * and is really just nobody asking it.
         *
         * Polling here also keeps the 4-frame stability filter CONVERGED, so
         * entering a shooting state does not start by waiting for it. One
         * Limelight read per loop; the shooting paths now use the cached
         * value rather than each re-polling.
         */
        vision.updateRaisedTarget();

        handleMechState(this.currentState);
        // AFTER the state, not before: deployment owns the intake motor for
        // its window and must win. Run first and IDLE's stopIntake() would
        // cancel it every loop and the arm would never swing.
        serviceDeploy(nowMs);
        // Single owner for the scoop, by distance only. See serviceScoop().
        serviceScoop();
        // Last word on the intake motor. See serviceIntakeLimit().
        serviceIntakeLimit();

        // Watch both flywheels for the dip a departing ball causes. One loop
        // behind the state logic, which is harmless: a dip seen now is
        // consumed by the next serviceContinuousFire().
        serviceExitDetect();

        /*
         * Completion ALWAYS runs, in every state.
         *
         * This used to live inside updateConveyor(), behind the intake-state
         * gate, and that was a deadlock. An index started in
         * INTAKE_FLOWER_SHOOT_HIVE leaves activeConveyor set; the flower then
         * empties, the state becomes SHOOTING_HIVE, and the only code that
         * clears activeConveyor is no longer being called. SPIN_UP waits on
         * it and the robot never fires again for the rest of the match.
         *
         * A transfer in progress has to be allowed to finish no matter what
         * state the robot moved to while it was running.
         */
        serviceConveyorCompletion();
        /*
         * Called in EVERY state now. It gates itself internally, and its first
         * act is to read the bed sensors -- so the seen-flags and the
         * telemetry stay current even while shooting, when no indexing can
         * happen. Calling it only in intake states left that history frozen
         * for the whole of a volley.
         */
        updateConveyor();
    }

    /**
     * Finish a transfer that is already moving, in ANY state. Releases the
     * motor and restores the beds when it arrives.
     */
    private void serviceConveyorCompletion() {
        if (activeConveyor == ConveyorType.NONE) return;
        if (shotOwnsBelt()) return;
        if (robot.conveyorMot.isBusy()) return;

        robot.conveyorMot.setPower(0);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        activeConveyor = ConveyorType.NONE;
        conveyorTargetPosition = -1;
        // Restore the resting pattern only if nothing else owns the beds.
        if (!flowing) setBedsUp();
    }

    /**
     * Swing the intake arm out, once per match, then release the scoop.
     *
     * Runs from update() in EVERY state, deliberately. It used to live inside
     * the START case, which meant a driver who pressed intake within the first
     * 850 ms aborted it -- `deployDone` never latched, and scoopTo() then
     * silently refused every scoop command for the rest of the match. The
     * flower states would have looked broken for no visible reason.
     *
     * Running it here also means it overlaps whatever else the robot is doing:
     * homing, driving, spinning up. It owns only the intake motor, and only
     * until the arm is out.
     *
     * Open loop and timed, because intakeMot has no usable encoder -- that
     * port carries the turret's through-bore. Approximately one rotation is
     * enough. A fresh battery swings further than a flat one, so tune
     * DEPLOY_MS on a representative charge.
     */
    private void serviceDeploy(long now) {
        if (deployDone) return;
        if (deployStartMs == 0) deployStartMs = now;

        long since = now - deployStartMs;
        if (since < DEPLOY_MS) {
            robot.intakeMot.setPower(DEPLOY_POWER);
            return;
        }
        robot.intakeMot.setPower(0);
        if (since >= DEPLOY_MS + DEPLOY_SCOOP_DELAY_MS) {
            // Direct write: scoopTo() is gated on the very flag being set here.
            robot.flowerIntake.setPosition(FLOWER_REST);
            deployDone = true;
        }
    }

    /**
     * Put the beds in a known position at INIT, before play is pressed.
     *
     * The FSM cannot do this itself: update() is only called from loop(), and
     * loop() does not run until play. So the beds used to be uncommanded for
     * the whole init period and only snapped up when an intake state was
     * first entered -- which looks like the servos waking up late, and leaves
     * them unpowered and free to be knocked out of position beforehand.
     *
     * Safe to call at init: it moves only the two bed servos, which are
     * internal to the conveyor, and commands them to the resting pattern.
     * Call it from the OpMode's init(), after RobotHardware is built.
     */
    public void initPositions() {
        setBedsUp();
    }

    public MechState getCurrentState() { return currentState; }

    public void setState(MechState newState) {
        if (newState == null || newState == currentState) return;
        onStateExit(currentState, newState);
        previousState = currentState;
        currentState = newState;
        resetIntakeTracking();
        shotPhase = ShotPhase.IDLE;
        activeShotBall = BallToShoot.NONE;
        shotsCompleted = 0;
        shotNote = "";
        preSpinning = false;
        haltFlow();
        // A new command is the driver's answer to a tipped CELL, so clear the
        // latch -- otherwise one tip would mute shooting for the whole match.
        cellTipped = false;
        /*
         * Zero the dip counters with shotsCompleted, not just re-baseline
         * them.
         *
         * The telemetry shows the two witnesses side by side and the whole
         * value of that line is that they cover the SAME window. shotsCompleted
         * resets on every state change; pollenExits used to run for the whole
         * OpMode. Two counters over different windows look like a disagreement
         * on the very first state change, which is exactly the reading B16
         * asks you to act on.
         */
        pollenExits = 0; nectarExits = 0;
        pollenExitsAtArm = 0; nectarExitsAtArm = 0;
        pollenDipLow = false; nectarDipLow = false;
        if (newState == MechState.IDLE) {
            stopIntake(); stopShooterMotors(); stopFeedConveyor();
            // scoop: serviceScoop() folds it once clear of the flower
            operationStatus = OperationStatus.CANCELLED;
        } else if (newState == MechState.START) {
            operationStatus = OperationStatus.NOT_STARTED;
        } else {
            operationStatus = OperationStatus.RUNNING;
            if (newState == MechState.SHOOTING_HIVE) vision.startHiveAprilTagPipeline();
            if (newState == MechState.INTAKE_FLOWER || newState == MechState.INTAKE_FLOWER_SHOOT_HIVE
                    || newState == MechState.INTAKE_FLOWER_SHOOT_FLOWER) vision.startFlowerCountPipeline();
        }
    }

    private void onStateExit(MechState oldState, MechState newState) {
        if (oldState == MechState.INTAKE_FLOWER || oldState == MechState.INTAKE_FLOWER_SHOOT_HIVE
                || oldState == MechState.INTAKE_FLOWER_SHOOT_FLOWER) {
            stopIntake();
            // scoop: serviceScoop() folds it once clear of the flower
        }
        /*
         * Stop the flywheels when leaving a state that spins them FOR a state
         * that does not.
         *
         * Originally only setState(IDLE) did this, which left shooters running
         * when a shooting state handed straight over to an intake state --
         * live flywheels while balls are fed in. Then the combo flower states
         * learned to shoot, and re-opened the same hole by a different route:
         * INTAKE_FLOWER_SHOOT_HIVE -> INTAKE_FIELD spins up and never stops.
         *
         * The `!maySpin(newState)` half is what keeps the handoff fast. Going
         * combo -> SHOOTING_HIVE must NOT stop the wheels; re-spinning costs
         * over a second and the balls are already queued.
         */
        if (maySpin(oldState) && !maySpin(newState)) {
            stopFeedConveyor();
            stopShooterMotors();
        }
    }

    /** States that may have the flywheels running. */
    private static boolean maySpin(MechState s) {
        return s == MechState.SHOOTING_HIVE
                || s == MechState.SHOOTING_FLOWER_NECTAR
                || s == MechState.SHOOTING_FLOWER_POLLEN
                || s == MechState.INTAKE_FLOWER_SHOOT_HIVE
                || s == MechState.INTAKE_FLOWER_SHOOT_FLOWER;
    }

    /** All state decisions live here; helper methods only perform atomic actions. */
    public void handleMechState(MechState state) {
        long now = System.currentTimeMillis();
        switch (state) {
            case START:
                stopShooterMotors();
                // Deployment is NOT here -- see serviceDeploy(), which runs
                // from update() regardless of state. START simply reports it.
                if (!deployDone) operationStatus = OperationStatus.RUNNING;
                else operationStatus = OperationStatus.COMPLETED;
                break;

            case IDLE:
                stopIntake(); stopShooterMotors();
                // scoop: serviceScoop() folds it once clear of the flower
                if (activeConveyor == ConveyorType.NONE) robot.conveyorMot.setPower(0);
                break;

            case INTAKE_FIELD: {
                /*
                 * WATCH FOR THE TIP, FIRST.
                 *
                 * checkCellTipped() used to be called only from inside
                 * serviceContinuousFire() -- that is, from behind the fire
                 * gate. A tip is exactly what closes that gate: the raised
                 * target becomes the far CELL, this position is no longer in
                 * its zone, canFireFromHere() goes false and the tip check is
                 * never reached. So while sweeping the field the tip went
                 * undetected: no status, no note, nothing on the driver's
                 * screen to say why the balls had stopped leaving.
                 *
                 * Only on a target vision actually saw. hiveTargetForPosition()
                 * infers one from the zone, and an inference that flips as the
                 * robot drives across a boundary is not a tip.
                 *
                 * Collecting continues through a tip unless TIP_STOPS_INTAKE
                 * says otherwise -- the balls are still worth having when you
                 * arrive at the other side.
                 */
                ShooterAdapter.Target seenCell = vision.getRaisedTarget();
                if (seenCell != null && checkCellTipped(seenCell) && TIP_STOPS_INTAKE) {
                    haltFlow();
                    releaseWheels();
                    stopIntake();
                    resetFieldReject();
                    setBedsUp();
                    break;
                }
                /*
                 * FIRE WHILE SWEEPING THE FIELD, when the position allows it.
                 *
                 * Nothing used to shoot here -- INTAKE_FIELD only pre-spun and
                 * waited for the driver. But a robot standing in a hive zone
                 * holding balls should be putting them away, not idling with
                 * the wheels up.
                 *
                 * Two gates, and the second is the important one:
                 *
                 *   inHiveZone   -- a position known to produce valid shots
                 *   steady       -- see isSteadyEnoughToFire()
                 *
                 * The steadiness gate exists because planHere() solves for the
                 * pose it is handed, and the ball has not left yet. Driving at
                 * 30 in/s puts the robot several inches from where the
                 * solution assumed by the time the shot goes. The turret is
                 * settled; it is settled on an angle that expired. Parked at a
                 * flower this could not happen, which is why continuous flow
                 * did not need it until now.
                 *
                 * Falls back to pre-spin when not steady, so the wheels are
                 * already up the moment the robot stops -- and pre-spin now
                 * spins them back DOWN when its own gates fail, which is what
                 * a tip makes happen.
                 */
                if (FIRE_DURING_FIELD_INTAKE && canFireFromHere() && isSteadyEnoughToFire()) {
                    serviceContinuousFire(now, true);
                } else {
                    haltFlow();
                    // Before the full check: a robot that has just filled up
                    // is the one most likely to shoot next, and ROBOT_FULL
                    // breaks out below.
                    servicePreSpin();
                }
                /*
                 * Beds BEFORE the full check.
                 *
                 * ROBOT_FULL breaks out, and setBedsUp() used to sit below it
                 * -- so entering INTAKE_FIELD already holding 4 balls left the
                 * bed servos never commanded at all. They sat wherever they
                 * happened to be, which on the bench looks exactly like dead
                 * servos. A full robot still needs its beds in a known state.
                 *
                 * Still skipped while flowing, or this fights the pollen-only
                 * pattern serviceContinuousFire() sets and the nectar bed is
                 * driven up and down on alternate loops.
                 */
                if (!flowing) setBedsUp();

                if (totalBalls() >= 4) {
                    stopIntake(); resetFieldReject(); operationStatus = OperationStatus.ROBOT_FULL; break;
                }
                int color = intakeColor();
                if (reversingWrongBall) {
                    robot.intakeMot.setPower(-INTAKE_POWER);
                    if (now - reverseStartMs >= REVERSE_INTAKE_TIME_MS) {
                        reversingWrongBall = false; waitingForWrongBallClear = true; wrongBallClearStartMs = now;
                    }
                } else if (waitingForWrongBallClear) {
                    robot.intakeMot.setPower(0);
                    if (color == 0) { waitingForWrongBallClear = false; previousFieldColor = 0; }
                    else if (now - wrongBallClearStartMs > SENSOR_CLEAR_TIMEOUT_MS) {
                        operationStatus = OperationStatus.TIMED_OUT; stopIntake();
                    }
                } else if (isWrongNectar(color) || nectarTunnelFull(color)) {
                    // nectarTunnelFull: our own nectar, but the tunnel holds
                    // only 3 and a fourth would be pushed straight through
                    // into the shooter on the next index.
                    reversingWrongBall = true; reverseStartMs = now; robot.intakeMot.setPower(-INTAKE_POWER);
                } else {
                    robot.intakeMot.setPower(INTAKE_POWER);
                    if (color != 0 && previousFieldColor == 0 && now - lastCountMs >= COUNT_COOLDOWN_MS) {
                        if (color == 1) countPollen++;
                        else if (color == allianceTeamNectar()) countNectar++;
                        lastCountMs = now;
                    }
                }
                previousFieldColor = color;
                break;
            }

            case INTAKE_FLOWER:
            case INTAKE_FLOWER_SHOOT_HIVE:
            case INTAKE_FLOWER_SHOOT_FLOWER: {
                if (totalBalls() >= 4) {
                    stopFlowerIntake();
                    if (state == MechState.INTAKE_FLOWER) {
                        operationStatus = OperationStatus.ROBOT_FULL;
                    } else {
                        // No deferral needed. Continuous flow has no discrete
                        // stroke to interrupt, and setState() calls haltFlow()
                        // which stops the belt cleanly. The flywheels keep
                        // spinning across the handoff (maySpin covers both
                        // sides), so the first shot in SHOOTING_HIVE is not
                        // paying a spin-up.
                        setState(state == MechState.INTAKE_FLOWER_SHOOT_HIVE
                                ? MechState.SHOOTING_HIVE : MechState.SHOOTING_FLOWER_POLLEN);
                    }
                    break;
                }
                int flowerPollen = vision.getFlowerPollenCount();
                if (flowerPollen < 0) {
                    /*
                     * BLIND FALLBACK. This used to stop dead.
                     *
                     * A dead camera is not a reason to stop collecting: the
                     * robot is parked at a flower because something put it
                     * there, and scooping at nothing costs only time while
                     * refusing to scoop costs the whole trip. Keep going to
                     * the 4-ball limit and let the status say why.
                     *
                     * The one thing lost is the "flower is empty" transition,
                     * so a blind combo state fills up before it hands off
                     * rather than leaving early.
                     */
                    operationStatus = OperationStatus.VISION_UNAVAILABLE;
                    flowerPollen = 1;                 // assume there is more
                }
                if (flowerPollen == 0) {
                    stopFlowerIntake();
                    if (state == MechState.INTAKE_FLOWER) {
                        operationStatus = OperationStatus.FLOWER_EMPTY;
                    } else {
                        // Flower is dry but we may still be holding balls, so
                        // hand over to the shooting state rather than stopping.
                        // haltFlow() inside setState() parks the belt.
                        setState(state == MechState.INTAKE_FLOWER_SHOOT_HIVE
                                ? MechState.SHOOTING_HIVE : MechState.SHOOTING_FLOWER_POLLEN);
                    }
                    break;
                }
                /*
                 * SCOOP AND INTAKE BY DISTANCE, not just by state.
                 *
                 * The scoop swings outside the footprint, so it only deploys
                 * once there is room and folds back only after there is room
                 * again. The intake does not bother running until the robot is
                 * essentially at the flower.
                 *
                 * Range comes back NaN when the pose is unknown or we are not
                 * in a flower zone. Treat that as "just do it" -- a driver who
                 * has put the robot at a flower it cannot localise still wants
                 * the scoop down.
                 */
                /*
                 * Same NaN split as serviceScoop(). No pose -> honour the
                 * driver. Pose but no flower zone -> we know we are nowhere
                 * near a flower, so do NOT run the intake. Before the four
                 * zones went live this read "just do it" for both, which
                 * would now spin the intake in the middle of the field.
                 */
                boolean noPose = !poseValid;
                double fRange = flowerRangeIn();
                boolean knowRange = !Double.isNaN(fRange);
                // No clearToDeploy here any more -- serviceScoop() owns the
                // scoop and applies FLOWER_DEPLOY_IN itself. This case only
                // decides whether to run the INTAKE.
                boolean atFlower = noPose || (knowRange && fRange <= FLOWER_INTAKE_IN);

                if (!noPose && (!knowRange || fRange > FLOWER_RETRACT_IN)) {
                    /*
                     * Driven away from the flower. Fold the scoop -- but KEEP
                     * SHOOTING. The drive between a flower and a hive zone is
                     * free cycle time, and serviceContinuousFire() will decide
                     * for itself whether the new position supports a shot.
                     */
                    stopFlowerIntake();
                    if (state != MechState.INTAKE_FLOWER) {
                        serviceContinuousFire(now,
                                state == MechState.INTAKE_FLOWER_SHOOT_HIVE);
                    } else {
                        haltFlow();
                    }
                    break;
                }

                // NOT while a shot is feeding: beginFeed() has set a specific
                // bed pattern for the ball going out, and setBedsUp() would
                /*
                 * Beds are set by whoever owns the belt.
                 *
                 * In the combo states that is serviceContinuousFire(), which
                 * wants pollen engaged and nectar disengaged. Calling
                 * setBedsUp() first and letting flow override it meant the
                 * nectar bed was commanded UP then DOWN every single loop --
                 * servo thrash at 150 Hz, buzzing and never settling, because
                 * conveyorNectar()'s change-detection saw a genuine change
                 * each time.
                 *
                 * Only plain INTAKE_FLOWER, which never fires, sets them here.
                 */
                if (state == MechState.INTAKE_FLOWER) setBedsUp();
                // Scoop is serviceScoop()'s job now -- same distances, one owner.
                if (!atFlower) {
                    // Approaching, not there yet. Do not run the intake -- but
                    // DO keep shooting: the drive in is free cycle time, and a
                    // shot already in flight must not be abandoned.
                    stopIntake();
                    if (state != MechState.INTAKE_FLOWER) {
                        serviceContinuousFire(now,
                                state == MechState.INTAKE_FLOWER_SHOOT_HIVE);
                    }
                    break;
                }

                /*
                 * CONTINUOUS FLOW -- no index/feed alternation.
                 *
                 * A tunnel is a queue on one belt, so running the belt with
                 * the pollen bed engaged does BOTH jobs in one motion: a new
                 * pollen enters at the back while the front one leaves into
                 * the shooter. Splitting that into discrete 100-tick index
                 * strokes and 300-tick feed strokes, mutually excluded, was
                 * modelling a constraint the mechanism does not have. It
                 * served no purpose and cost a stroke boundary per ball.
                 *
                 * Just run the belt and let pollen stream through. The only
                 * real limit is the flywheel: a ball takes ~3% of wheel speed
                 * with it, so the belt pauses while the wheel recovers rather
                 * than firing into a wheel that is still down.
                 */
                if (state != MechState.INTAKE_FLOWER) {
                    serviceContinuousFire(now, state == MechState.INTAKE_FLOWER_SHOOT_HIVE);
                }

                int color = intakeColor();
                if (rejectingFlowerNectar) {
                    robot.intakeMot.setPower(-INTAKE_POWER);
                    if (color == 0) rejectingFlowerNectar = false;
                } else if (isWrongNectar(color) || nectarTunnelFull(color)) {
                    /*
                     * Reject only the OTHER alliance's nectar -- and our own
                     * once the nectar tunnel is full.
                     *
                     * This used to reject every nectar on the reasoning that
                     * flowers only yield pollen. But a stray nectar of our own
                     * colour near a flower is worth 4 points and there is no
                     * reason to spit it out. Same rule as INTAKE_FIELD now.
                     */
                    rejectingFlowerNectar = true; robot.intakeMot.setPower(-INTAKE_POWER);
                } else {
                    robot.intakeMot.setPower(INTAKE_POWER);
                    if (color != 0 && previousFlowerColor == 0
                            && now - lastCountMs >= COUNT_COOLDOWN_MS) {
                        if (color == 1) countPollen++;
                        else if (color == allianceTeamNectar()) countNectar++;
                        lastCountMs = now;
                    }
                }
                previousFlowerColor = color;
                break;
            }

            case SHOOTING_HIVE: {
                if (REQUIRE_HOMED && !turret.isHomed()) {
                    stopShooterMotors(); stopFeedConveyor();
                    operationStatus = OperationStatus.TURRET_NOT_HOMED;
                    shotNote = turret.getHomeMessage();
                    break;
                }
                if (!poseValid) {
                    stopShooterMotors(); operationStatus = OperationStatus.NOT_CONFIGURED;
                    shotNote = "no pose - OpMode must call updatePose()"; break;
                }

                // Which CELL is up: tags first, zone only as a fallback.
                ShooterAdapter.Target tgt = vision.getRaisedTarget();
                if (tgt == null) tgt = shooter.hiveTargetForPosition(poseX, poseY);
                if (tgt == null) {
                    /*
                     * WAIT, do not shut down.
                     *
                     * This used to stop the flywheels. But a CELL mid-tip is a
                     * normal, brief condition -- the cluster is simply out of
                     * view for a moment -- and spinning back up from stopped
                     * costs over a second. Hold the wheels at speed and keep
                     * looking; the belt is released so nothing can feed into a
                     * turret that has no target.
                     */
                    stopFeedConveyor();
                    shotPhase = ShotPhase.IDLE;
                    operationStatus = OperationStatus.TARGET_UNKNOWN;
                    shotNote = "waiting for a CELL -- flywheels held at speed";
                    break;
                }
                /*
                 * A tipped CELL ends the volley.
                 *
                 * This used to reset the deposit estimate and keep firing at
                 * whichever CELL came up. That is wrong: the newly raised CELL
                 * faces the other way and is shot from the opposite side of
                 * the field. Firing on from here throws balls at a target this
                 * position cannot reach.
                 *
                 * Stop and keep the inventory for after the reposition.
                 */
                if (checkCellTipped(tgt)) {
                    /*
                     * STOP THE FLYWHEELS TOO.
                     *
                     * This released the belt and set the phase idle but left
                     * both shooters spinning. A tipped CELL means the robot
                     * has to drive to the other side of the field before it
                     * can score again -- seconds, not milliseconds -- so
                     * holding them at speed burns battery and heats motors
                     * for a shot that cannot happen from here.
                     *
                     * This is the opposite of the TARGET_UNKNOWN case just
                     * below, where the cluster is briefly out of view and
                     * spinning down would cost more than it saves.
                     */
                    stopShooterMotors();
                    stopFeedConveyor();
                    shotPhase = ShotPhase.IDLE;
                    break;
                }

                /*
                 * Only finish once the shot in flight has finished too.
                 *
                 * This used to break out the moment the count hit zero, which
                 * happens inside finishFeed() -- i.e. partway through the LAST
                 * shot. runShotSequence() was then never called again, so
                 * RECOVER never completed: lastCycleMs stayed 0 and the
                 * flywheels were cut while the final ball was still leaving.
                 */
                if (totalBalls() <= 0 && shotPhase == ShotPhase.IDLE) {
                    stopShooterMotors(); operationStatus = OperationStatus.COMPLETED; break;
                }

                /*
                 * planHere(), NOT plan().
                 *
                 * plan() chooses a firing mode, works out the heading that
                 * mode wants, and solves THERE -- its own documentation says
                 * the yaw it returns "is generally NOT your current heading,
                 * so turn first, then re-solve". This FSM never drives: the
                 * driver owns the heading in TeleOp and Pedro has already
                 * finished its move in autonomous. Feeding plan()'s turretDeg
                 * to an unturned robot mis-aims by the entire difference
                 * between the two headings.
                 *
                 * planHere() solves at the yaw we actually have and picks the
                 * mode from the divergence that applies there.
                 */
                ShooterAdapter.FirePlan plan =
                        shooter.planHere(tgt, poseX, poseY, poseHeadingDeg);
                lastPlan = plan;
                if (!plan.valid) {
                    stopShooterMotors(); stopFeedConveyor();
                    operationStatus = OperationStatus.NO_SHOT;
                    // plan() answers the question planHere() cannot: IS there a
                    // heading that works from this spot? Turning it into a
                    // number the driver can act on beats "reposition".
                    /*
                     * Name the TARGET, the distance and WHICH ball failed.
                     *
                     * "drive closer" alone is useless: a nectar 0.3 in past
                     * the table edge and a robot 60 in away read identically,
                     * and so does aiming at the wrong CELL entirely. All
                     * three have different fixes.
                     */
                    ShooterAdapter.Shot ps = shooter.solve(
                            ShooterAdapter.Ball.POLLEN, tgt, poseX, poseY, poseHeadingDeg);
                    ShooterAdapter.Shot ns = shooter.solve(
                            ShooterAdapter.Ball.NECTAR, tgt, poseX, poseY, poseHeadingDeg);
                    double gd = Math.hypot(shooter.goalX(tgt) - poseX,
                                           shooter.goalY(tgt) - poseY);
                    ShooterAdapter.FirePlan would =
                            shooter.plan(tgt, poseX, poseY, poseHeadingDeg);
                    shotNote = String.format(
                            "no shot: %s @ %.0f in  [P %s  N %s]  %s",
                            tgt, gd,
                            ps.valid ? "ok" : "FAIL",
                            ns.valid ? "ok" : "FAIL",
                            would.valid
                                ? String.format("turn to %.0f deg", would.yawDeg)
                                : "table is 15-39 in");
                    break;
                }
                operationStatus = OperationStatus.RUNNING;

                // Arming is shared with the concurrent path in the combo
                // intake states, so both choose BOTH vs nectar-first by the
                // same rule. Nectar goes first when single: 4 points to 3.
                if (shotPhase == ShotPhase.IDLE) armFromPlan(plan, now);
                runShotSequence(now);
                break;
            }

            case SHOOTING_FLOWER_NECTAR:
            case SHOOTING_FLOWER_POLLEN: {
                BallToShoot requested = state == MechState.SHOOTING_FLOWER_NECTAR
                        ? BallToShoot.NECTAR : BallToShoot.POLLEN;
                int available = requested == BallToShoot.NECTAR ? countNectar : countPollen;
                if (available <= 0) {
                    stopShooterMotors(); operationStatus = OperationStatus.COMPLETED; break;
                }
                if (REQUIRE_HOMED && !turret.isHomed()) {
                    stopShooterMotors(); operationStatus = OperationStatus.TURRET_NOT_HOMED; break;
                }
                if (!poseValid) {
                    stopShooterMotors(); operationStatus = OperationStatus.NOT_CONFIGURED; break;
                }
                double[] flower = shooter.flowerTargetForPosition(poseX, poseY);
                if (flower == null) {
                    stopShooterMotors(); operationStatus = OperationStatus.NO_SHOT;
                    shotNote = "not in a flower zone"; break;
                }
                operationStatus = OperationStatus.RUNNING;

                // ONE ball per command: only arm if nothing has gone out yet.
                if (shotPhase == ShotPhase.IDLE && shotsCompleted == 0) {
                    activeShotBall = requested;
                    double turretDeg = ShooterAdapter.turretDegToPoint(
                            poseX, poseY, poseHeadingDeg, flower[0], flower[1])
                            + FLOWER_AIM_TRIM_DEG;
                    double rpm = requested == BallToShoot.NECTAR
                            ? FLOWER_NECTAR_RPM : FLOWER_POLLEN_RPM;
                    aimAndPrepare(requested, turretDeg, FLOWER_HOOD_DEG, rpm);
                    // cycleStartMs too, or lastCycleMs reports the time since
                    // the last HIVE shot -- which can be minutes.
                    shotPhase = ShotPhase.AIM; shotPhaseStartMs = now; cycleStartMs = now;
                }
                // Flower shooting is ONE ball per command, by driver choice.
                runShotSequence(now);
                if (shotPhase == ShotPhase.IDLE && shotsCompleted > 0) {
                    stopShooterMotors();
                    operationStatus = OperationStatus.COMPLETED;
                }
                break;
            }
        }
    }

    /**
     * Spin up and point the turret BEFORE the shoot command arrives.
     *
     * Spin-up from rest measured 1680 ms. If the robot is already standing in
     * a hive zone holding balls, that second and a half is dead time that can
     * be spent while still collecting -- so the first shot after the driver
     * commits is a feed stroke, not a spin-up.
     *
     * GATED HARD, because flywheels spinning for a whole match is current draw
     * and wear for nothing:
     *   - something to shoot (inventory > 0)
     *   - standing in a hive zone for a known target
     *   - not already running a shot of our own
     *
     * Only ever called from INTAKE_FIELD. The combo flower states already run
     * the full shot sequence, which spins up as part of arming.
     */
    /** Turn off if shooting mid-sweep proves more trouble than it is worth. */
    public static boolean FIRE_DURING_FIELD_INTAKE = true;

    /**
     * Are we standing somewhere a hive shot is known to work from?
     *
     * Zone membership only -- whether a solution exists is planHere()'s
     * problem, and serviceContinuousFire() asks it. This is the cheap test
     * that avoids solving from the far side of the field every loop.
     */
    /** Pure test -- no side effects, safe to call from telemetry. */
    private boolean inFiringPosition() {
        if (!poseValid || totalBalls() <= 0) return false;
        if (REQUIRE_HOMED && !turret.isHomed()) return false;
        ShooterAdapter.Target tgt = vision.getRaisedTarget();
        if (tgt == null) tgt = shooter.hiveTargetForPosition(poseX, poseY);
        return tgt != null && shooter.inHiveZone(tgt, poseX, poseY);
    }

    private boolean canFireFromHere() {
        if (!inFiringPosition()) return false;

        /*
         * Standing in the right zone for the current target RESOLVES a tip.
         *
         * The clear has to happen HERE, not only in checkCellTipped(). An
         * earlier version returned false on cellTipped before reaching the
         * zone test -- so serviceContinuousFire() never ran, checkCellTipped()
         * never ran, and the only thing that could clear the latch was behind
         * the latch. Firing stayed muted for the rest of the state no matter
         * where the driver drove.
         *
         * The two hive zones do not overlap, so being in the one that matches
         * the newly raised CELL is unambiguous: the reposition happened.
         */
        if (cellTipped) {
            cellTipped = false;
            shotNote = "";
        }
        return true;
    }

    private void servicePreSpin() {
        if (!PRE_SPIN_ENABLED) { releaseWheels(); return; }
        /*
         * A real shot owns the wheels. This is the ONE gate that must not
         * release them -- everything below is "pre-spin has no reason to be
         * holding speed", which is a reason to spin DOWN.
         */
        if (shotPhase != ShotPhase.IDLE) { preSpinHoldUntilMs = 0; return; }
        if (totalBalls() <= 0 || !poseValid) { releaseWheels(); return; }
        if (REQUIRE_HOMED && !turret.isHomed()) { releaseWheels(); return; }

        // The cache is refreshed every loop by update(), so it is always
        // current and the stability filter is already converged.
        ShooterAdapter.Target tgt = vision.getRaisedTarget();
        if (tgt == null) tgt = shooter.hiveTargetForPosition(poseX, poseY);
        if (tgt == null || !shooter.inHiveZone(tgt, poseX, poseY)) { releaseWheels(); return; }

        ShooterAdapter.FirePlan plan = shooter.planHere(tgt, poseX, poseY, poseHeadingDeg);
        if (!plan.valid) { releaseWheels(); return; }

        // Aim and spin, but do NOT advance shotPhase -- nothing is armed and
        // no ball is committed. The driver still decides when to fire.
        preSpinHoldUntilMs = 0;
        preSpinning = true;
        ShooterAdapter.Shot s = countNectar > 0 ? plan.nectar : plan.pollen;
        double turretDeg = (plan.mode == ShooterAdapter.FireMode.SIMULTANEOUS)
                ? plan.sharedTurretDeg : s.turretDeg;
        turret.setTurretDeg(turretDeg);
        if (countPollen > 0) {
            robot.hoodPollen.setPosition(HoodMap.pollenPos(plan.pollen.hoodDeg));
            targetPollenTps = RobotHardware.rpmToTicksPerSec(plan.pollen.rpm);
            robot.pollenShootingMot.setVelocity(targetPollenTps);
        }
        if (countNectar > 0) {
            robot.hoodNectar.setPosition(HoodMap.nectarPos(plan.nectar.hoodDeg));
            targetNectarTps = RobotHardware.rpmToTicksPerSec(plan.nectar.rpm);
            robot.nectarShootingMot.setVelocity(targetNectarTps);
        }
    }

    /**
     * LET THE FLYWHEELS GO when nothing in INTAKE_FIELD wants them up.
     *
     * Every gate in servicePreSpin() used to be a bare `return`, which left
     * whatever velocity was last commanded standing. In INTAKE_FIELD that is
     * the only governor the wheels have, so once they were up nothing could
     * bring them down again short of leaving the state -- they ran for the
     * rest of the match.
     *
     * The case that exposed it: the CELL tips. The raised target becomes the
     * far one, canFireFromHere() goes false (we are in the wrong zone for it),
     * so the state falls to this branch, and pre-spin's own zone gate also
     * fails -- it returned in silence with the wheels at full speed. That is
     * "the shooting motor is not stopping even after the hive tips".
     *
     * It also catches the untuned-dip failure the same way. If DIP_ENTER never
     * trips, countPollen never decrements, totalBalls() stays positive and
     * pre-spin re-commands speed every loop on balls that are long gone. The
     * inventory is still wrong, but the wheels now come down when the robot
     * leaves the zone instead of running on a phantom.
     *
     * The grace window is there because the raised-target filter and the pose
     * can both flicker for a loop or two, and spinning down and back up costs
     * 1.7 s. Hold speed briefly, then let go.
     */
    public static long PRE_SPIN_RELEASE_MS = 750;
    private long preSpinHoldUntilMs = 0;

    private void releaseWheels() {
        /*
         * "Commanded", not "pre-spinning". serviceContinuousFire() clears
         * preSpinning when it takes the wheels over, so keying off that flag
         * would miss the exact handover this is for: firing while sweeping the
         * field, the CELL tips, the fire gate closes, and the wheels are left
         * running by the code that is no longer being called.
         */
        if (targetPollenTps <= 0 && targetNectarTps <= 0) { preSpinHoldUntilMs = 0; return; }
        long now = System.currentTimeMillis();
        if (preSpinHoldUntilMs == 0) { preSpinHoldUntilMs = now + PRE_SPIN_RELEASE_MS; return; }
        if (now < preSpinHoldUntilMs) return;
        preSpinHoldUntilMs = 0;
        stopShooterMotors();        // clears preSpinning
    }

    /** Turn pre-spin off if battery or motor temperature becomes a problem. */
    public static boolean PRE_SPIN_ENABLED = true;
    private boolean preSpinning = false;

    // ---- ball-exit detection by flywheel RPM dip ---------------------------
    /*
     * A ball leaving takes energy out of the wheel and the velocity dips. That
     * dip is the only DIRECT evidence a ball actually left -- belt travel only
     * proves the conveyor moved, which is equally true of a jam.
     *
     * It is nonetheless the SECOND witness, not the first. See
     * countFlowByBeltTravel(): better evidence that rests on an unmeasured
     * threshold is worse than weaker evidence that rests on a measured one.
     * This runs every loop and counts into pollenExits / nectarExits so the
     * two can be compared on the telemetry; it does not touch the inventory
     * unless COUNT_FLOW_BY_DIP is set.
     *
     * Predicted from the flywheel energy budget (72 mm Rhino + 60 mm steel,
     * ~5.4 J at 2653 rpm):
     *     pollen   3.1% of wheel speed   ~38 ticks/s of 1238
     *     nectar   5.0%                  ~62 ticks/s
     *
     * UNTUNED. The thresholds below are the predictions, not measurements.
     * Measure the steady-state noise floor on the HardwareCheck SHOOTER page
     * (`spread (2s)`) and set DIP_ENTER just outside it.
     *
     * Hysteresis matters: one ball must produce exactly one count, so the
     * latch only re-arms once velocity has climbed back above DIP_EXIT.
     */
    public static double DIP_ENTER = 0.975;   // below this fraction of target = a ball went
    public static double DIP_EXIT  = 0.990;   // above this, re-arm for the next
    /*
     * A DIP_COUNTS_BALLS flag used to sit here and was deleted, because it had
     * stopped gating anything: continuous flow was dip-only, so the dip was
     * authoritative whatever the flag said, and the telemetry went on
     * reporting "advisory" while the dip drove the inventory. A switch that
     * lies about what the code is doing is worse than no switch.
     *
     * COUNT_FLOW_BY_DIP is not that flag returning. It selects between two
     * counters that both exist and both run, and the telemetry label changes
     * with it. Belt travel is primary on both sides of it today:
     *
     *     discrete shooting states   belt travel, always (finishFeed)
     *     continuous flow            belt travel, unless COUNT_FLOW_BY_DIP
     *
     * The dip counts alongside either way, and the two numbers sit next to
     * each other on screen. That comparison is the measurement -- see B16.
     */

    private boolean pollenDipLow = false, nectarDipLow = false;
    private int pollenExits = 0, nectarExits = 0;
    private int pollenExitsAtArm = 0, nectarExitsAtArm = 0;

    /**
     * Watch both flywheels for the dip that means a ball passed through.
     * Called every loop from update(); costs two velocity reads that the
     * shot sequence is taking anyway.
     */
    private void serviceExitDetect() {
        if (targetPollenTps > 0) {
            double r = robot.pollenShootingMot.getVelocity() / targetPollenTps;
            if (!pollenDipLow && r < DIP_ENTER) { pollenDipLow = true; pollenExits++; }
            else if (pollenDipLow && r > DIP_EXIT) pollenDipLow = false;
        } else {
            pollenDipLow = false;
        }
        if (targetNectarTps > 0) {
            double r = robot.nectarShootingMot.getVelocity() / targetNectarTps;
            if (!nectarDipLow && r < DIP_ENTER) { nectarDipLow = true; nectarExits++; }
            else if (nectarDipLow && r > DIP_EXIT) nectarDipLow = false;
        } else {
            nectarDipLow = false;
        }
    }

    /**
     * Balls the flywheels say have left **since the current state began** --
     * the same window `shotsCompleted` covers, so the two can be compared
     * directly. Both are zeroed by setState().
     */
    public int getPollenExits() { return pollenExits; }
    public int getNectarExits() { return nectarExits; }

    /** True while the flywheels are being held ready ahead of a command. */
    public boolean isPreSpinning() { return preSpinning; }

    /**
     * Stream pollen through the shooter while the intake keeps collecting.
     *
     * Flowers yield pollen, so only the pollen tunnel runs. The nectar bed
     * stays disengaged and any nectar we picked up sits still in its tunnel
     * until a dedicated shooting state asks for it.
     *
     * No phases, no strokes. The belt runs whenever all of these hold, and
     * stops the moment one does not:
     *
     *   - we have pollen
     *   - turret settled on a valid target
     *   - the pollen flywheel is at speed
     *
     * The flywheel condition IS the rate limit. A ball costs ~3% of wheel
     * speed; pause until it is back, then carry on. That self-regulates to
     * exactly the fire rate the shooter can sustain, with no tuning.
     *
     * COUNTING: belt travel means nothing here -- there are no discrete
     * strokes to count. The RPM dip is the only evidence a ball left, so
     * this mode depends on DIP_ENTER being tuned to the real flywheel.
     * Until it is, the inventory in this mode is a guess.
     */
    private void serviceContinuousFire(long now, boolean hive) {
        // Each refusal names itself. Collecting continues either way -- these
        // set only shotNote, never operationStatus.
        if (REQUIRE_HOMED && !turret.isHomed()) {
            shotNote = "holding fire: turret not homed"; haltFlow(); return;
        }
        if (!poseValid) {
            shotNote = "holding fire: no pose"; haltFlow(); return;
        }
        // A flower shot is pollen by definition. A hive volley will take
        // whatever we are holding, so it only needs SOMETHING.
        if (hive ? totalBalls() <= 0 : countPollen <= 0) {
            shotNote = "holding fire: nothing to shoot"; haltFlow(); return;
        }
        // We own the wheels now, not pre-spin. Clearing the flag keeps the
        // telemetry honest -- a label that says PRE-SPUN while something else
        // is driving is the same class of lie DIP_COUNTS_BALLS was.
        preSpinning = false;
        streamNectar = false;

        // Where are we shooting?
        double turretDeg;
        double hoodDeg;
        double rpm;
        if (hive) {
            ShooterAdapter.Target tgt = vision.getRaisedTarget();
            if (tgt == null) tgt = shooter.hiveTargetForPosition(poseX, poseY);
            if (tgt == null) {
                shotNote = "holding fire: no CELL in view, not in a hive zone";
                haltFlow(); return;
            }
            // A tipped CELL ends the volley -- see checkCellTipped().
            if (checkCellTipped(tgt)) {
                // Same as the dedicated state: a tip means no shot is
                // possible from here until the robot repositions, so the
                // wheels come down rather than idling at speed.
                haltFlow();
                stopShooterMotors();
                if (TIP_STOPS_INTAKE) stopIntake();
                return;
            }
            ShooterAdapter.FirePlan plan =
                    shooter.planHere(tgt, poseX, poseY, poseHeadingDeg);
            if (!plan.valid) {
                /*
                 * Say WHY, without touching operationStatus.
                 *
                 * This used to return in silence so that collecting carried
                 * on uninterrupted. It does -- but the driver then sees an
                 * intake state running happily and no balls leaving, with
                 * nothing on screen to explain it.
                 *
                 * The usual cause is simply range. No flower on the field is
                 * within hive shooting distance: the closest approach is
                 * 45.4 in against a table that stops at 39. So a combo state
                 * standing AT a flower can never fire at the hive, and only
                 * starts once the robot has driven back toward the zone.
                 */
                double d = Math.hypot(shooter.goalX(tgt) - poseX,
                                      shooter.goalY(tgt) - poseY);
                shotNote = String.format(
                        "holding fire: %.0f in to %s (table 15-39) - drive closer",
                        d, tgt);
                haltFlow();
                return;
            }
            lastPlan = plan;
            turretDeg = (plan.mode == ShooterAdapter.FireMode.SIMULTANEOUS)
                    ? plan.sharedTurretDeg : plan.pollen.turretDeg;
            hoodDeg = plan.pollen.hoodDeg;
            rpm = plan.pollen.rpm;

            /*
             * Stream NECTAR alongside, when the geometry allows it.
             *
             * SIMULTANEOUS means one turret angle serves both shooters, so
             * both beds can engage and both tunnels empty at once. Nectar is
             * 4 points to pollen's 3, so leaving it in the tunnel while the
             * angle happens to suit it is throwing away the better ball.
             *
             * SEQUENTIAL needs a different angle per shooter, so only pollen
             * streams and the nectar waits for a dedicated volley.
             */
            streamNectar = plan.mode == ShooterAdapter.FireMode.SIMULTANEOUS
                    && countNectar > 0;
            if (streamNectar) {
                robot.hoodNectar.setPosition(HoodMap.nectarPos(plan.nectar.hoodDeg));
                targetNectarTps = RobotHardware.rpmToTicksPerSec(plan.nectar.rpm);
                robot.nectarShootingMot.setVelocity(targetNectarTps);
            }
        } else {
            double[] f = shooter.flowerTargetForPosition(poseX, poseY);
            if (f == null) { haltFlow(); return; }
            turretDeg = ShooterAdapter.turretDegToPoint(
                    poseX, poseY, poseHeadingDeg, f[0], f[1]) + FLOWER_AIM_TRIM_DEG;
            hoodDeg = FLOWER_HOOD_DEG;
            rpm = FLOWER_POLLEN_RPM;
        }

        // Hold the aim and the speed. Cheap to re-issue; the servo and the
        // velocity controller both ignore an unchanged command.
        turret.setTurretDeg(turretDeg);
        robot.hoodPollen.setPosition(HoodMap.pollenPos(hoodDeg));
        targetPollenTps = RobotHardware.rpmToTicksPerSec(rpm);
        robot.pollenShootingMot.setVelocity(targetPollenTps);
        operationStatus = OperationStatus.RUNNING;

        /*
         * NOTHING THIS MODE CAN ACTUALLY FIRE.
         *
         * The gate at the top only asks for SOMETHING in a hive volley, and
         * that is not the same question. Holding nectar alone under a
         * SEQUENTIAL plan leaves streamNectar false and the pollen tunnel
         * empty: the belt would run, the pollen bed would be up over nothing,
         * the nectar bed would be down over the balls we have, and the robot
         * would sit there flowing for ever having fired nothing.
         *
         * Say so and hold. The dedicated SHOOTING_* states take nectar on its
         * own; continuous flow is a pollen stream that nectar may ride along
         * with when one turret angle happens to serve both.
         */
        if (countPollen <= 0 && !streamNectar) {
            shotNote = "holding fire: nectar only, and this angle needs a "
                     + "separate nectar volley";
            haltFlow();
            return;
        }

        conveyorPollen(1);
        conveyorNectar(streamNectar ? 1 : 0);

        // BOTH wheels must be up when both are streaming -- feeding a nectar
        // into a wheel that is still recovering wastes the better ball.
        boolean ready = turret.isSettled()
                && shooterAtSpeed(BallToShoot.POLLEN)
                && (!streamNectar || shooterAtSpeed(BallToShoot.NECTAR));
        if (ready) {
            /*
             * Claim the belt outright.
             *
             * An index may have been part way through a RUN_TO_POSITION move.
             * Simply overwriting the mode left activeConveyor set, and
             * serviceConveyorCompletion() then saw a non-busy motor, declared
             * the transfer finished and called setBedsUp() -- which engages
             * BOTH beds and silently undid the pollen-only pattern two lines
             * above. Clearing it here means completion has nothing to find.
             *
             * No ball is lost: the queue does not care that the motion it was
             * riding changed owner mid-stroke.
             */
            // An index was mid-stroke and is being taken over. Its travel is
            // not ours to count, so restart the measurement from here.
            if (activeConveyor != ConveyorType.NONE) flowAnchorValid = false;
            activeConveyor = ConveyorType.NONE;
            conveyorTargetPosition = -1;
            /*
             * Anchor once per flow SESSION, not once per loop -- a per-loop
             * anchor measures one loop of belt and never reaches a ball's
             * worth.
             *
             * And not once per `flowing` either. `flowing` drops on every
             * recovery pause, which happens immediately AFTER a ball leaves
             * and the wheel sags. Re-anchoring there would throw away travel
             * that was a hair short of a full ball and the count would run
             * systematically low. The encoder holds its reading through the
             * pause, so the partial carries over intact.
             */
            if (!flowAnchorValid) {
                flowAnchorTicks = robot.conveyorMot.getCurrentPosition();
                flowAnchorValid = true;
            }
            flowing = true;
            robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.conveyorMot.setPower(CONVEYOR_POWER);
        } else {
            // Wheel still recovering, or turret still swinging. Hold the belt
            // but keep everything else live. Re-baseline the exit counters:
            // nothing can leave with the belt stopped, so any dip from here
            // until it restarts is noise.
            flowing = false;
            robot.conveyorMot.setPower(0);
            syncExitBaseline();
            return;
        }

        countFlowByBeltTravel(hive);
    }

    /*
     * ------------------------------------------------------------------------
     * COUNTING BALLS OUT OF CONTINUOUS FLOW
     *
     * Two independent witnesses, and deliberately so:
     *
     *   BELT TRAVEL  (primary)   the conveyor has moved one ball's worth of
     *                            ticks, so one ball has been pushed through
     *   RPM DIP      (secondary) the flywheel lost speed, so something took
     *                            energy out of it
     *
     * The dip used to be primary and the only witness. It is the better
     * evidence in principle -- belt travel only proves the conveyor turned,
     * which is equally true of a jam -- but it rests on DIP_ENTER being tuned
     * to the real flywheel, and that measurement has not been taken. An
     * untuned threshold does not fail safe: too loose and noise decrements the
     * inventory for balls that never left, so the robot believes it is
     * emptying while it fills, and the intake never reaches its 4-ball stop.
     *
     * Belt travel has no such failure mode on the bench. It is geometry and a
     * tuned tick count, both of which can be measured without firing.
     *
     * So: belt travel drives the inventory. The dip runs alongside, counting
     * into pollenExits / nectarExits, and the two are shown side by side on
     * the telemetry. Once DIP_ENTER has been measured on the robot (B14) and
     * the two agree over a volley, swap COUNT_FLOW_BY_DIP to true and the dip
     * becomes primary again.
     * ------------------------------------------------------------------------
     */
    /** false = belt travel counts (default). true = RPM dip counts. */
    public static boolean COUNT_FLOW_BY_DIP = false;

    private int flowAnchorTicks = 0;
    private boolean flowAnchorValid = false;

    private void countFlowByBeltTravel(boolean hive) {
        if (COUNT_FLOW_BY_DIP) { countFlowByDip(hive); return; }

        /*
         * One ball's worth of belt is the same distance the discrete feed
         * uses, so the two modes share a tuning constant rather than drifting
         * apart. BOTH when the nectar bed is also engaged: one stroke pushes
         * one ball out of each tunnel, not two out of the pollen tunnel.
         */
        int perBall = streamNectar ? TICKS_TO_SHOOT_BOTH : TICKS_TO_SHOOT_POLLEN;
        if (perBall <= 0) return;

        // Signed, and only forward travel counts. The belt is commanded one
        // way in flow; a negative delta means an encoder glitch or a reversal
        // from somewhere else, and neither is a ball.
        int travelled = robot.conveyorMot.getCurrentPosition() - flowAnchorTicks;
        while (travelled >= perBall) {
            flowAnchorTicks += perBall;
            travelled -= perBall;
            if (countPollen > 0) {
                countPollen--;
                shotsCompleted++;
                if (hive) cellPollenIn++;
            }
            // Nectar only while its bed is up -- a disengaged bed cannot have
            // released anything, however far the belt ran.
            if (streamNectar && countNectar > 0) {
                countNectar--;
                shotsCompleted++;
                if (hive) cellNectarIn++;
            }
        }
        // The dip counters keep running as the second opinion, but they must
        // not bank a backlog while belt travel owns the inventory.
        syncExitBaseline();
    }

    /** The old primary. Kept whole so the swap back is one flag. */
    private void countFlowByDip(boolean hive) {
        // Count what actually left, not what the belt did. Each wheel reports
        // its own tunnel, so pollen and nectar are counted independently.
        int goneP = pollenExits - pollenExitsAtArm;
        if (goneP > 0) {
            pollenExitsAtArm = pollenExits;
            for (int i = 0; i < goneP && countPollen > 0; i++) {
                countPollen--;
                shotsCompleted++;
                if (hive) cellPollenIn++;
            }
        }
        /*
         * Nectar counts ONLY while its bed is engaged.
         *
         * targetNectarTps stays set after streamNectar goes false -- the wheel
         * is still spinning, just not being fed. serviceExitDetect() keeps
         * watching it, and a noise dip would then decrement countNectar for a
         * ball that could not possibly have left, because the bed holding it
         * is disengaged.
         *
         * The baseline is advanced either way, so dips recorded while the bed
         * was down are discarded rather than banked for later.
         */
        int goneN = nectarExits - nectarExitsAtArm;
        if (goneN > 0) {
            nectarExitsAtArm = nectarExits;
            if (streamNectar) {
                for (int i = 0; i < goneN && countNectar > 0; i++) {
                    countNectar--;
                    shotsCompleted++;
                    if (hive) cellNectarIn++;
                }
            }
        }
    }

    private boolean streamNectar = false;

    /** Stop the flow but leave the flywheels spinning. */
    private void haltFlow() {
        syncExitBaseline();
        // End of the session, so the partial travel goes with it. Unlike the
        // recovery pause inside serviceContinuousFire(), this means something
        // refused the shot -- a tip, a lost pose, out of range -- and the belt
        // will not simply resume where it left off.
        flowAnchorValid = false;
        if (!flowing) return;
        flowing = false;
        robot.conveyorMot.setPower(0);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Forget dips that happened while the belt was stopped.
     *
     * serviceExitDetect() watches the wheels whenever a target velocity is
     * set, which includes pre-spin and the pauses while a wheel recovers. No
     * ball can leave then, so any dip is noise -- but it still incremented the
     * counter, and the next flowing loop would read the backlog as several
     * balls at once and decrement the inventory for each.
     *
     * Only dips recorded while the belt is actually running mean anything.
     */
    private void syncExitBaseline() {
        pollenExitsAtArm = pollenExits;
        nectarExitsAtArm = nectarExits;
    }

    /** True while pollen is streaming through the shooter. */
    public boolean isFlowing() { return flowing; }
    private boolean flowing = false;

    /**
     * Has the CELL we were shooting at tipped?
     *
     * When it does, its tag cluster rotates out of view and the OTHER cell's
     * comes in -- so the target simply changes. That is not a reason to
     * re-aim and carry on: the newly raised CELL faces the opposite way and
     * is shot from the other side of the field, which this position cannot
     * reach. Stop, keep the balls, and let the driver reposition.
     *
     * Nothing here depends on who tipped it. The geometry changed; that is
     * the whole of it.
     */
    private boolean checkCellTipped(ShooterAdapter.Target tgt) {
        if (lastKnownTarget != null && tgt != lastKnownTarget) {
            resetCellCount();
            lastKnownTarget = tgt;
            cellTipped = true;
            operationStatus = OperationStatus.TARGET_UNKNOWN;
            shotNote = "CELL tipped -- holding " + totalBalls()
                     + " balls, reposition to the other side";
            return true;
        }
        lastKnownTarget = tgt;

        /*
         * Clear the latch once we are standing where the NEW target can be
         * shot from.
         *
         * The latch used to be cleared only by setState(). Repositioning does
         * not change state, so a driver who did exactly what the message asked
         * -- drive to the other side -- found firing still muted, with nothing
         * to do about it but cycle states. The two hive zones do not overlap,
         * so arriving in the right one is unambiguous evidence the reposition
         * happened.
         */
        if (cellTipped && shooter.inHiveZone(tgt, poseX, poseY)) {
            cellTipped = false;
            shotNote = "";
        }
        return cellTipped;
    }

    /**
     * Should a tipped CELL also stop COLLECTING, not just shooting?
     *
     * A tip means no shot is possible until the robot repositions. It does
     * NOT mean the flower in front of you has stopped being useful, so the
     * default is to keep collecting -- the balls are still worth having when
     * you arrive at the other side.
     *
     * Set true if you would rather the robot go quiet on a tip and wait for
     * the driver. Only affects the combo flower states; the dedicated
     * shooting states never run the intake anyway.
     */
    public static boolean TIP_STOPS_INTAKE = false;

    private boolean cellTipped = false;

    /** Clear the tipped latch. Bind to the driver's re-arm button. */
    public void clearCellTipped() { cellTipped = false; }
    public boolean isCellTipped() { return cellTipped; }

    /*
     * serviceConcurrentShot() lived here. It arbitrated discrete index and
     * feed strokes so they never overlapped -- a constraint the mechanism
     * does not actually have, since a tunnel is a queue and one belt motion
     * carries a ball in and another out at the same time.
     *
     * serviceContinuousFire() replaces it. Deleted rather than left behind:
     * two firing paths that drift apart are worse than one.
     */

    /** Arm a hive shot from a solved plan. Shared by concurrent and dedicated. */
    private void armFromPlan(ShooterAdapter.FirePlan plan, long now) {
        boolean pairAvailable = countNectar > 0 && countPollen > 0;
        if (plan.mode == ShooterAdapter.FireMode.SIMULTANEOUS && pairAvailable) {
            activeShotBall = BallToShoot.BOTH;
            aimAndPrepareBoth(plan.sharedTurretDeg,
                    plan.pollen.hoodDeg, plan.pollen.rpm,
                    plan.nectar.hoodDeg, plan.nectar.rpm);
        } else {
            activeShotBall = countNectar > 0 ? BallToShoot.NECTAR : BallToShoot.POLLEN;
            ShooterAdapter.Shot s = (activeShotBall == BallToShoot.NECTAR)
                    ? plan.nectar : plan.pollen;
            double turretDeg = (plan.mode == ShooterAdapter.FireMode.SIMULTANEOUS)
                    ? plan.sharedTurretDeg : s.turretDeg;
            aimAndPrepare(activeShotBall, turretDeg, s.hoodDeg, s.rpm);
        }
        shotPhase = ShotPhase.AIM; shotPhaseStartMs = now; cycleStartMs = now;
    }

    /** Shared AIM -> SPIN_UP -> FEEDING -> RECOVER progression. */
    private void runShotSequence(long now) {
        switch (shotPhase) {
            case AIM:
                // Wait on the turret actually arriving, not a fixed time. A
                // large re-aim takes several times longer than a small trim,
                // and firing mid-swing throws the ball wide.
                if (turret.isSettled()) {
                    lastAimMs = now - shotPhaseStartMs;
                    shotPhase = ShotPhase.SPIN_UP; shotPhaseStartMs = now;
                } else if (now - shotPhaseStartMs > TURRET_SETTLE_TIMEOUT_MS) {
                    stopShooterMotors(); operationStatus = OperationStatus.TIMED_OUT;
                    shotNote = "turret never arrived - check slip and SERVO_MS_PER_TURRET_DEG";
                    shotPhase = ShotPhase.IDLE;
                }
                break;
            case SPIN_UP:
                /*
                 * An index may have started while we were spinning up. Let it
                 * finish -- beginFeed() would otherwise overwrite its target
                 * mid-move. The flywheel just holds speed meanwhile.
                 *
                 * The TIMEOUT STILL APPLIES. An earlier version returned
                 * early here, which skipped the timeout check too: anything
                 * that wedged activeConveyor hung SPIN_UP forever with no
                 * error and no recovery. Never let a wait escape its own
                 * deadline.
                 */
                boolean beltFree = activeConveyor == ConveyorType.NONE;
                if (beltFree && shooterAtSpeed(activeShotBall)) {
                    lastSpinMs = now - shotPhaseStartMs;
                    beginFeed(activeShotBall); shotPhase = ShotPhase.FEEDING; shotPhaseStartMs = now;
                } else if (now - shotPhaseStartMs > SHOOTER_SPINUP_TIMEOUT_MS) {
                    stopShooterMotors(); operationStatus = OperationStatus.TIMED_OUT;
                    shotPhase = ShotPhase.IDLE;
                }
                break;
            case FEEDING:
                /*
                 * The minimum dwell is NOT cosmetic.
                 *
                 * isBusy() reads false for a loop or two after
                 * setTargetPosition() -- the controller has not picked up the
                 * new target yet. At 150+ Hz that is caught immediately, so
                 * FEEDING exited with lastFeedMs = 0 and finishFeed()
                 * decremented the count for a ball the conveyor never moved.
                 * The counts dropped and the telemetry looked right.
                 *
                 * A real feed is 300 ticks at 0.65 power -- hundreds of ms --
                 * so nothing legitimate is delayed by waiting this out.
                 */
                if (now - shotPhaseStartMs >= FEED_MIN_MS && !robot.conveyorMot.isBusy()) {
                    lastFeedMs = now - shotPhaseStartMs;
                    finishFeed(activeShotBall); shotPhase = ShotPhase.RECOVER; shotPhaseStartMs = now;
                } else if (now - shotPhaseStartMs > SHOOTER_FEED_TIMEOUT_MS) {
                    stopFeedConveyor(); stopShooterMotors();
                    operationStatus = OperationStatus.TIMED_OUT; shotPhase = ShotPhase.IDLE;
                }
                break;
            case RECOVER:
                if (now - shotPhaseStartMs >= SHOOTER_RECOVER_MS) {
                    lastCycleMs = now - cycleStartMs;
                    activeShotBall = BallToShoot.NONE; shotPhase = ShotPhase.IDLE;
                }
                break;
            default:
                break;
        }
    }

    private void aimAndPrepare(BallToShoot ball, double turretDeg, double hoodDeg, double rpm) {
        turret.setTurretDeg(turretDeg);
        double tps = RobotHardware.rpmToTicksPerSec(rpm);
        if (ball == BallToShoot.POLLEN) {
            robot.hoodPollen.setPosition(HoodMap.pollenPos(hoodDeg));
            robot.pollenShootingMot.setVelocity(tps);
            targetPollenTps = tps;
        } else if (ball == BallToShoot.NECTAR) {
            robot.hoodNectar.setPosition(HoodMap.nectarPos(hoodDeg));
            robot.nectarShootingMot.setVelocity(tps);
            targetNectarTps = tps;
        }
        shotNote = String.format("%s  turret %.1f  hood %.1f  rpm %.0f",
                ball, turretDeg, hoodDeg, rpm);
    }

    /** Aim once and spin up BOTH shooters for a paired shot. */
    private void aimAndPrepareBoth(double turretDeg,
                                   double pollenHoodDeg, double pollenRpm,
                                   double nectarHoodDeg, double nectarRpm) {
        turret.setTurretDeg(turretDeg);
        targetPollenTps = RobotHardware.rpmToTicksPerSec(pollenRpm);
        targetNectarTps = RobotHardware.rpmToTicksPerSec(nectarRpm);
        robot.hoodPollen.setPosition(HoodMap.pollenPos(pollenHoodDeg));
        robot.hoodNectar.setPosition(HoodMap.nectarPos(nectarHoodDeg));
        robot.pollenShootingMot.setVelocity(targetPollenTps);
        robot.nectarShootingMot.setVelocity(targetNectarTps);
        shotNote = String.format("BOTH  turret %.1f  P hood %.0f/%.0f rpm  N hood %.0f/%.0f rpm",
                turretDeg, pollenHoodDeg, pollenRpm, nectarHoodDeg, nectarRpm);
    }

    private double targetPollenTps = 0, targetNectarTps = 0;

    private boolean shooterAtSpeed(BallToShoot ball) {
        if (ball == BallToShoot.POLLEN) {
            return targetPollenTps > 0 && robot.pollenShootingMot.getVelocity()
                    >= targetPollenTps * SHOOTER_READY_TOLERANCE;
        }
        if (ball == BallToShoot.NECTAR) {
            return targetNectarTps > 0 && robot.nectarShootingMot.getVelocity()
                    >= targetNectarTps * SHOOTER_READY_TOLERANCE;
        }
        if (ball == BallToShoot.BOTH) {
            // Both must be up to speed -- feeding on one would waste the other.
            return shooterAtSpeed(BallToShoot.POLLEN) && shooterAtSpeed(BallToShoot.NECTAR);
        }
        return false;
    }

    /**
     * True while a shot has the conveyor. Indexing must stand clear: both are
     * the same belt, and an index started mid-feed would overwrite the feed's
     * target position and strand the ball halfway into the shooter.
     */
    private boolean shotOwnsBelt() { return shotPhase == ShotPhase.FEEDING; }

    private void updateConveyor() {
        /*
         * READ THE SENSORS FIRST, before any early return.
         *
         * The seen-flags carry the rising-edge history, and they used to be
         * updated only at the bottom -- after five conditions that can each
         * skip the whole method. Any pause (a transfer running, continuous
         * flow, a full robot) froze that history at whatever it was, so a
         * ball that arrived and left during the pause was invisible, and the
         * telemetry showed a state several seconds stale.
         *
         * Reading is two I2C distance checks. Cheap, and always honest.
         */
        boolean pollen = vision.ballCheck(robot.sensorPollenL, robot.sensorPollenR);
        boolean nectar = vision.ballCheck(robot.sensorNectarL, robot.sensorNectarR);

        // Continuous flow already runs the belt and carries balls in as it
        // carries them out. Indexing on top of that would fight it.
        if (flowing || shotOwnsBelt()
                || activeConveyor != ConveyorType.NONE
                || !isIntakeState(currentState)
                || totalBalls() >= 4) {
            // Keep the history current so the NEXT eligible loop sees a true
            // edge rather than one invented by the gap.
            pollenBedSeen = pollen;
            nectarBedSeen = nectar;
            return;
        }

        /*
         * ADVANCE UNTIL CLEAR, not once per rising edge.
         *
         * A ball sitting over a bed sensor blocks the intake behind it, so it
         * has to keep moving until the sensor reads empty. Pure edge detection
         * could not do that: if one index did not carry the ball clear, there
         * was never a second edge and it sat there for the rest of the match.
         *
         * The edge still matters for the FIRST index -- it is what stops a
         * ball being double-counted -- so both are used: fire on a rising
         * edge, OR when the sensor is still blocked after a completed move.
         * `retries` bounds it, so a failed or dirty sensor cannot run the
         * conveyor forever.
         */
        boolean pollenEdge = pollen && !pollenBedSeen;
        boolean nectarEdge = nectar && !nectarBedSeen;
        boolean pollenStuck = pollen && pollenBedSeen && pollenRetries < MAX_CLEAR_RETRIES;
        boolean nectarStuck = nectar && nectarBedSeen && nectarRetries < MAX_CLEAR_RETRIES;

        if (!pollen) pollenRetries = 0;
        if (!nectar) nectarRetries = 0;
        pollenBedSeen = pollen;
        nectarBedSeen = nectar;

        if (pollenEdge || pollenStuck) {
            if (pollenStuck) pollenRetries++;
            startConveyor(ConveyorType.POLLEN);
        } else if ((nectarEdge || nectarStuck) && currentState == MechState.INTAKE_FIELD) {
            if (nectarStuck) nectarRetries++;
            startConveyor(ConveyorType.NECTAR);
        }
    }

    private void startConveyor(ConveyorType type) {
        int ticks;
        if (type == ConveyorType.POLLEN) {
            conveyorNectar(0); conveyorPollen(1); ticks = TICKS_PER_POLLEN_STORAGE;
        } else if (type == ConveyorType.NECTAR) {
            conveyorNectar(1); conveyorPollen(0); ticks = TICKS_PER_NECTAR_STORAGE;
        } else return;
        conveyorTargetPosition = robot.conveyorMot.getCurrentPosition() + ticks;
        // This stroke is an index, not a feed. Continuous flow must not read
        // it as balls leaving the shooter.
        flowAnchorValid = false;
        activeConveyor = type;
        robot.conveyorMot.setTargetPosition(conveyorTargetPosition);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.conveyorMot.setPower(CONVEYOR_POWER);
    }

    private void beginFeed(BallToShoot ball) {
        setBedsUp();
        int ticks;
        /*
         * Bed convention, confirmed against the mechanism:
         *     1 = UP   = ENGAGED,    that tunnel's ball moves with the belt
         *     0 = DOWN = DISENGAGED, that tunnel's ball stays put
         *
         * setBedsUp() leaving both engaged at rest is harmless because the
         * conveyor motor is only powered here and in startConveyor() -- an
         * engaged bed with a stopped belt moves nothing.
         *
         * Engage the tunnel you are FIRING. The two single-ball branches had
         * this inverted -- firing POLLEN engaged the NECTAR tunnel and
         * disengaged the pollen one, so the wrong ball moved. Compare
         * startConveyor(), which has had it right all along: a pollen ball
         * indexes with conveyorPollen(1).
         */
        if (ball == BallToShoot.BOTH) {
            // Both engaged: a pollen and a nectar leave on the same stroke.
            // If a tunnel is empty only one goes, which is harmless -- the
            // counts in finishFeed() are guarded.
            conveyorPollen(1); conveyorNectar(1); ticks = TICKS_TO_SHOOT_BOTH;
        } else if (ball == BallToShoot.POLLEN) {
            conveyorPollen(1); conveyorNectar(0); ticks = TICKS_TO_SHOOT_POLLEN;
        } else {
            conveyorNectar(1); conveyorPollen(0); ticks = TICKS_TO_SHOOT_NECTAR;
        }
        feedTargetPosition = robot.conveyorMot.getCurrentPosition() + ticks;
        robot.conveyorMot.setTargetPosition(feedTargetPosition);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.conveyorMot.setPower(CONVEYOR_POWER);
    }

    private void finishFeed(BallToShoot ball) {
        stopFeedConveyor();
        // No exit sensor: this is an ESTIMATE that the ball left, inferred
        // from the feed travelling its full distance.
        boolean hive = currentState == MechState.SHOOTING_HIVE;
        boolean firedPollen = (ball == BallToShoot.POLLEN || ball == BallToShoot.BOTH);
        boolean firedNectar = (ball == BallToShoot.NECTAR || ball == BallToShoot.BOTH);
        if (firedPollen && countPollen > 0) {
            countPollen--;
            if (hive) cellPollenIn++;
            shotsCompleted++;
        }
        if (firedNectar && countNectar > 0) {
            countNectar--;
            if (hive) cellNectarIn++;
            shotsCompleted++;
        }
        setBedsUp();
    }

    private void stopFeedConveyor() {
        robot.conveyorMot.setPower(0);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feedTargetPosition = 0;
    }

    // ---- CELL deposit estimate --------------------------------------------

    public void addPartnerNectar() { cellNectarIn++; }
    public void addPartnerPollen() { cellPollenIn++; }
    public void resetCellCount() { cellNectarIn = 0; cellPollenIn = 0; }
    public int cellPoints() {
        return cellNectarIn * CELL_NECTAR_VALUE + cellPollenIn * CELL_POLLEN_VALUE;
    }

    // ---- atomic helpers ----------------------------------------------------

    /**
     * Servo positions for the two bed states. Public and static so the bench
     * OpMode drives the SAME numbers the FSM does -- a test page with its own
     * copy of these would happily pass while the robot used something else.
     */
    public static double bedPollenPos(boolean engaged) {
        return (double) (engaged ? CONVEYOR_UP_POLLEN : CONVEYOR_DOWN_POLLEN)
                / MAX_CONVEYOR_ROTATION;
    }
    public static double bedNectarPos(boolean engaged) {
        return (double) (engaged ? CONVEYOR_UP_NECTAR : CONVEYOR_DOWN_NECTAR)
                / MAX_CONVEYOR_ROTATION;
    }

    public void conveyorNectar(int up1Down0) {
        int p = up1Down0 == 1 ? 1 : 0;
        if (p == lastConveyorNectar) return;
        robot.conveyorNectar.setPosition(bedNectarPos(p == 1));
        lastConveyorNectar = p;
    }
    public void conveyorPollen(int up1Down0) {
        int p = up1Down0 == 1 ? 1 : 0;
        if (p == lastConveyorPollen) return;
        robot.conveyorPollen.setPosition(bedPollenPos(p == 1));
        lastConveyorPollen = p;
    }

    private void setBedsUp() { conveyorNectar(1); conveyorPollen(1); }
    private int intakeColor() { return vision.ballColor(robot.sensorIntakeR, robot.sensorIntakeL); }
    /**
     * The NECTAR tunnel physically holds 3; the POLLEN tunnel holds 4. The
     * game caps us at 4 balls TOTAL, so pollen can never overfill -- but four
     * nectar would, and an index move past capacity pushes the front ball out
     * of the tunnel and into the shooter.
     *
     * So our own nectar has to be refused at 3 even while total is under 4.
     * Nothing enforced this before.
     */
    public static final int NECTAR_TUNNEL_CAPACITY = 3;
    public static final int POLLEN_TUNNEL_CAPACITY = 4;

    private boolean nectarTunnelFull(int color) {
        return color == allianceTeamNectar() && countNectar >= NECTAR_TUNNEL_CAPACITY;
    }

    private boolean isWrongNectar(int color) {
        return (color == 2 || color == 3) && color != allianceTeamNectar();
    }
    private int totalBalls() { return countNectar + countPollen; }

    /** Every state in which balls can arrive and need indexing into storage. */
    private static boolean isIntakeState(MechState s) {
        return s == MechState.INTAKE_FIELD || s == MechState.INTAKE_FLOWER
                || s == MechState.INTAKE_FLOWER_SHOOT_HIVE
                || s == MechState.INTAKE_FLOWER_SHOOT_FLOWER;
    }
    private void stopIntake() { robot.intakeMot.setPower(0); }

    /**
     * A FULL ROBOT DOES NOT INTAKE. Last word, after every state.
     *
     * Each intake state already breaks out at `totalBalls() >= 4`, and that is
     * still where the status and the reject tracking are handled. This is the
     * backstop underneath it, and it exists because the per-state check is
     * only as good as the inventory it reads: continuous flow decrements the
     * count as it fires, so a miscount there -- a loose DIP_ENTER crediting
     * balls that never left, a belt slipping past its tick budget -- makes the
     * robot believe it has room it does not have. The intake keeps pulling,
     * and the tunnel jams against balls already in it.
     *
     * The game cap and the pollen tunnel are both 4, so this is one test said
     * two ways on purpose -- if either constant moves, the stricter one still
     * holds. The NECTAR tunnel is deliberately NOT here: it holds 3, and a
     * robot carrying 3 nectar still has room for a 4th ball as long as it is
     * pollen. Turning a nectar away is `nectarTunnelFull()`'s job in
     * INTAKE_FIELD, which reverses that one ball out rather than shutting the
     * intake down.
     *
     * Runs AFTER serviceDeploy() so it is genuinely last, and skips while the
     * arm is still swinging -- deployment drives the same motor, and a robot
     * carrying four balls out of autonomous would otherwise never deploy.
     */
    private void serviceIntakeLimit() {
        if (!deployDone) return;
        if (totalBalls() >= 4 || countPollen >= POLLEN_TUNNEL_CAPACITY) {
            stopIntake();
        }
    }
    /**
     * The ONLY way the scoop is commanded. Everything routes through here so
     * the "arm first, scoop second" rule cannot be bypassed by some other
     * path writing the servo directly.
     *
     * Before the intake arm has swung out, the scoop has nowhere safe to go:
     * commanding it would drive it into a stowed arm. START latches
     * `deployDone` once the arm is clear, and until then this does nothing.
     */
    private void scoopTo(double pos) {
        if (!deployDone) return;
        robot.flowerIntake.setPosition(pos);
    }

    /**
     * ONE owner for the scoop, called every loop from update().
     *
     * Scoop position is a function of DISTANCE and nothing else. It used to be
     * written from five places -- IDLE, onStateExit, stopFlowerIntake, the
     * flower case, stopAll -- and three of those fired on INVENTORY. Filling
     * up folded the scoop instantly, while the robot was still hard against
     * the flower, which is the exact thing the clearance distances exist to
     * prevent. Same scattered-ownership mistake the beds had.
     *
     *   collecting AND inside DEPLOY      -> SCOOP
     *   beyond RETRACT, or nowhere near   -> REST
     *   in between, or close but idle     -> leave it alone
     *
     * That last case matters: a robot that stops collecting while still at
     * the flower keeps the scoop out until it has driven clear. Folding early
     * is what jams it.
     */
    private void serviceScoop() {
        if (!deployDone) return;

        /*
         * NaN means two different things, and they want opposite answers.
         *
         *   no pose at all        -> we have no idea where we are. A driver
         *                            who commanded a flower state presumably
         *                            parked at a flower, so honour it.
         *   pose, but no zone     -> we know EXACTLY where we are, and it is
         *                            not near any flower. Stay folded.
         *
         * Treating both as "just do it" was fine when the zones were sparse.
         * With all four live they cover most of the field, so the only poses
         * that fall outside are genuinely nowhere near a flower -- the middle
         * of the field, for instance -- and deploying the scoop there is
         * exactly wrong.
         */
        boolean noPose = !poseValid;
        double r = flowerRangeIn();
        boolean know = !Double.isNaN(r);
        boolean collecting = currentState == MechState.INTAKE_FLOWER
                || currentState == MechState.INTAKE_FLOWER_SHOOT_HIVE
                || currentState == MechState.INTAKE_FLOWER_SHOOT_FLOWER;

        boolean nearEnough = noPose || (know && r <= FLOWER_DEPLOY_IN);
        boolean wellClear  = noPose ? false : (!know || r > FLOWER_RETRACT_IN);

        if (collecting && nearEnough) {
            robot.flowerIntake.setPosition(FLOWER_SCOOP);
        } else if (wellClear) {
            robot.flowerIntake.setPosition(FLOWER_REST);
        }
        // else: close but not collecting -- hold position until clear.
    }

    /** True once the start-of-match arm deployment has finished. */
    public boolean isDeployed() { return deployDone; }

    /**
     * Stop collecting. Does NOT fold the scoop -- serviceScoop() owns that,
     * and folds only once the robot is clear of the flower. Folding here is
     * what made a full robot retract the scoop while still against the flower.
     */
    private void stopFlowerIntake() { stopIntake(); }
    private void stopShooterMotors() {
        preSpinning = false;
        robot.nectarShootingMot.setVelocity(0);
        robot.pollenShootingMot.setVelocity(0);
        targetPollenTps = 0; targetNectarTps = 0;
    }
    private void resetFieldReject() {
        reversingWrongBall = false; waitingForWrongBallClear = false; previousFieldColor = 0;
    }
    private void resetIntakeTracking() {
        resetFieldReject(); previousFlowerColor = 0; rejectingFlowerNectar = false;
        pollenBedSeen = false; nectarBedSeen = false;
        // Retries too, or a state change part way through clearing a stuck
        // ball leaves the budget spent and the next one gets fewer attempts.
        pollenRetries = 0; nectarRetries = 0;
    }

    public MechState getState() { return currentState; }
    public MechState getPreviousState() { return previousState; }
    public OperationStatus getOperationStatus() { return operationStatus; }
    public boolean isOperationComplete() {
        return operationStatus != OperationStatus.RUNNING
                && operationStatus != OperationStatus.NOT_STARTED;
    }

    public void stopAll() {
        // flowing must be cleared too, not just the motor stopped -- leaving
        // it set means the bed-ownership guards still believe something is
        // streaming, and setBedsUp() stays suppressed.
        flowing = false;
        stopIntake(); stopShooterMotors(); robot.conveyorMot.setPower(0);
        scoopTo(FLOWER_REST);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        activeConveyor = ConveyorType.NONE; conveyorTargetPosition = -1;
        operationStatus = OperationStatus.CANCELLED;
    }

    /** Read-only. Never runs FSM logic. */
    /**
     * Trim the display to what fits on one Driver Station screen.
     *
     * The full page runs past 30 lines, so everything below the fire plan --
     * phase timing included -- scrolls off and looks like missing telemetry.
     * Compact keeps the lines you actually watch while driving.
     */
    public static boolean COMPACT_TELEMETRY = true;

    public void allTelemetry() {
        // The numbers you are reading off the screen go FIRST, so they cannot
        // be pushed out of view by anything added below them later.
        telemetry.addData("State", currentState + "  /  " + operationStatus);
        telemetry.addData("Inventory", "pollen %d   nectar %d   total %d",
                countPollen, countNectar, totalBalls());
        telemetry.addData("Shot phase", shotPhase + "  " + activeShotBall
                + (flowing ? "   [FLOWING]" : "")
                + (preSpinning ? "   [PRE-SPUN]" : "")
                + (cellTipped ? "   [CELL TIPPED]" : "")
                + (deployDone ? "" : "   [ARM STOWED]"));
        /*
         * THE TWO WITNESSES, SIDE BY SIDE. This line is the whole B16 test.
         *
         * `shots` is the inventory's own count and comes from BELT TRAVEL --
         * primary everywhere now, continuous flow included. The dip columns
         * are the second opinion, counted but not acted on.
         *
         * Watch them track each other over a volley. Dip running AHEAD means
         * DIP_ENTER is too loose and noise is being read as balls; dip running
         * BEHIND means it is too tight to see a real one. They agree when
         * DIP_ENTER is right, and that is the moment COUNT_FLOW_BY_DIP is
         * worth flipping.
         */
        telemetry.addData(
                COUNT_FLOW_BY_DIP ? "Exits  dip=PRIMARY  belt=2nd"
                                  : "Exits  belt=PRIMARY  dip=2nd",
                "dip pollen %d   dip nectar %d   |   belt shots %d",
                pollenExits, nectarExits, shotsCompleted);
        // Why a shot is or is not going out right now, in one line.
        /*
         * Pose and target belong on the COMPACT screen.
         *
         * Nearly every "why is it not shooting" question comes down to one of
         * three things: the robot is not where you think, the target is not
         * the CELL you think, or the range is outside the table. All three are
         * invisible unless you switch to full telemetry, which is exactly when
         * you are least likely to think of it.
         */
        telemetry.addData("Pose", poseValid
                ? String.format("%.0f, %.0f @ %.0f deg", poseX, poseY, poseHeadingDeg)
                : "NOT SET");
        ShooterAdapter.Target seen = vision.getRaisedTarget();
        telemetry.addData("Target", "%s%s",
                seen != null ? seen + " (tag)"
                             : String.valueOf(shooter.hiveTargetForPosition(poseX, poseY)),
                seen == null ? " (by zone)" : "");
        /*
         * RAW tag IDs, unfiltered. This is what separates "the camera sees
         * nothing" from "the camera sees a tag that is not ours".
         *
         * targetForTagId() accepts only 30-37 for RED and 38-45 for BLUE.
         * Any other ID is counted by the Limelight, listed by HardwareCheck,
         * and then dropped here -- producing "(by zone)" with a tag plainly
         * in view, which looks exactly like a dead camera.
         */
        telemetry.addData("Tags seen", "%d raw [%s]   %d ours",
                vision.getRawTagCount(),
                vision.getLastRawIds().isEmpty() ? "none" : vision.getLastRawIds(),
                vision.getVisibleTagCount());
        // Filter progress. Tags arriving but this stuck near 0 means the
        // detection is intermittent and confidence is being lost as fast as
        // it is gained -- which is what the old reset-to-zero guaranteed.
        telemetry.addData("Tag filter", "%d/%d agreed   %d lost",
                vision.getCandidateFrames(), VisionController.REQUIRED_STABLE_FRAMES,
                vision.getLostFrames());
        /*
         * Bed sensors and conveyor state, on the COMPACT screen.
         *
         * The beds only split when a BED sensor sees a ball and
         * updateConveyor() starts a transfer. Four separate conditions can
         * silently stop that -- full inventory, a transfer already running,
         * continuous flow owning the belt, or simply no ball at the sensor --
         * and none of them were visible. "The opposite servo did not move"
         * could be any of them.
         */
        telemetry.addData("Bed sees", "pollen %s   nectar %s",
                pollenBedSeen ? "BALL" : "-", nectarBedSeen ? "BALL" : "-");
        telemetry.addData("Index", "%s%s", activeConveyor,
                activeConveyor != ConveyorType.NONE
                        ? "  busy " + robot.conveyorMot.isBusy()
                        : (totalBalls() >= 4 ? "   (BLOCKED: 4 balls)"
                           : flowing ? "   (BLOCKED: flowing)" : ""));
        telemetry.addData("Fire gate", "%s   %.1f in/s %s",
                inFiringPosition() ? "in zone" : "no zone/target",
                speedInPerSec,
                isSteadyEnoughToFire() ? "steady" : "MOVING - holding fire");
        telemetry.addData("Phase ms", "aim %d  spin %d  feed %d  -> cycle %d",
                lastAimMs, lastSpinMs, lastFeedMs, lastCycleMs);
        telemetry.addData("Loop ms", "%.1f avg   %.0f worst   (%.0f Hz)",
                loopMsAvg, loopMsWorst, loopMsAvg > 0 ? 1000.0 / loopMsAvg : 0);
        telemetry.addData("Shots completed", shotsCompleted);
        // ALWAYS shown, not just in the flower states. It was gated to them at
        // first, which made it impossible to tell a missing build from a line
        // that simply was not drawn yet. -1 means the camera has not started.
        // Pipeline is here so it can be watched while the FSM changes state:
        // it must stay 1 through a flower collection, not drop back to 4.
        telemetry.addData("Pollen seen (cam)", "%d   (LL pipeline %d)",
                vision.getFlowerPollenCount(), vision.getCurrentPipeline());
        if (!shotNote.isEmpty()) telemetry.addData("Note", shotNote);

        // A hood calibration whose 55-80 sweep runs off the end of the servo's
        // 0..1 range clamps SILENTLY -- every commanded hood angle comes out
        // wrong and nothing says so. HoodMap has always been able to detect
        // this; nothing ever asked it. Costs one comparison when correct.
        if (!HoodMap.pollenRangeFits() || !HoodMap.nectarRangeFits()) {
            telemetry.addData("!! HOOD RANGE", "%s%s -- angles are being clamped",
                    HoodMap.pollenRangeFits() ? "" : "POLLEN ",
                    HoodMap.nectarRangeFits() ? "" : "NECTAR ");
        }

        if (COMPACT_TELEMETRY) {
            telemetry.update();
            return;
        }

        telemetry.addLine();
        telemetry.addData("Alliance", shooter.getAlliance());
        telemetry.addData("Operation Complete", isOperationComplete());
        telemetry.addLine();
        telemetry.addData("Pose", poseValid
                ? String.format("%.1f, %.1f @ %.1f deg", poseX, poseY, poseHeadingDeg)
                : "NOT SET - OpMode must call updatePose()");
        telemetry.addData("Raised CELL", vision.getRaisedTarget() == null
                ? "unknown" : vision.getRaisedTarget()
                  + "  (tag " + vision.getLastSeenTagId()
                  + ", " + vision.getVisibleTagCount() + " in view)");
        if (lastPlan != null && lastPlan.valid) {
            telemetry.addData("Fire mode", lastPlan.mode);
            telemetry.addData("Plan yaw", "%.1f deg", lastPlan.yawDeg);
            telemetry.addData("Pollen shot", "%.1f in  hood %.0f  %.0f rpm  %.0f%%",
                    lastPlan.pollen.rangeIn, lastPlan.pollen.hoodDeg,
                    lastPlan.pollen.rpm, lastPlan.pollen.confidencePct);
            telemetry.addData("Nectar shot", "%.1f in  hood %.0f  %.0f rpm  %.0f%%",
                    lastPlan.nectar.rangeIn, lastPlan.nectar.hoodDeg,
                    lastPlan.nectar.rpm, lastPlan.nectar.confidencePct);
        } else {
            telemetry.addData("Fire plan", "none");
        }
        telemetry.addData("Timeout headroom", "spin %d/%d   feed %d/%d",
                lastSpinMs, SHOOTER_SPINUP_TIMEOUT_MS, lastFeedMs, SHOOTER_FEED_TIMEOUT_MS);
        telemetry.addLine();
        telemetry.addData("CELL estimate", "%d nectar + %d pollen = %d/%d pts (PROVISIONAL)",
                cellNectarIn, cellPollenIn, cellPoints(), CELL_TIP_THRESHOLD);
        telemetry.addLine();
        telemetry.addData("Field rejection active", reversingWrongBall);
        telemetry.addData("Waiting for rejected ball", waitingForWrongBallClear);
        telemetry.addData("Flower nectar rejection", rejectingFlowerNectar);
        // Cached from handleBedSensors(), NOT re-read. Four extra I2C distance
        // reads per loop for a display line is loop time spent on nothing --
        // and the colour sensor bus is already the slowest thing in here.
        telemetry.addData("Bed sensors", "pollen %s   nectar %s",
                pollenBedSeen ? "BALL" : "-", nectarBedSeen ? "BALL" : "-");
        telemetry.addData("Conveyor transfer", activeConveyor);
        telemetry.addData("Conveyor target ticks", conveyorTargetPosition);
        telemetry.addData("Feed target ticks", feedTargetPosition);
        turret.addTelemetry(telemetry);
        // Do not call pinpoint.update() here; PedroPathing/localizer owns it.
        robot.addPinpointTelemetry(telemetry);
        telemetry.update();
    }
}






