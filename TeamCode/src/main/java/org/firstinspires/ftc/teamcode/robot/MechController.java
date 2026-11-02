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
    private static final double INTAKE_POWER = 0.75;

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
     * NEEDS TIGHTENING ONCE FLYWHEELS ARE ON. Fraction of target velocity that
     * counts as ready.
     *
     * 0.90 is loose for a shooter. Range goes roughly as the square of exit
     * speed, so firing at 90% of the commanded wheel speed lands about 20%
     * short -- several inches at hive range, far more than the aperture
     * forgives. It is only this slack because a bare motor with no flywheel
     * hunts around its setpoint and a tight gate would never open.
     *
     * With real flywheel inertia the velocity loop settles much harder. Aim
     * for 0.97-0.98 and let spin-up take the extra time; a late shot beats a
     * short one.
     */
    public static double SHOOTER_READY_TOLERANCE = 0.90;
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
        poseX = xIn; poseY = yIn; poseHeadingDeg = headingDeg; poseValid = true;
    }

    public void update() {
        long nowMs = System.currentTimeMillis();
        if (lastLoopMs != 0) {
            double dt = nowMs - lastLoopMs;
            loopMsAvg = loopMsAvg == 0 ? dt : (loopMsAvg * 0.9 + dt * 0.1);
            if (dt > loopMsWorst) loopMsWorst = dt;
        }
        lastLoopMs = nowMs;

        turret.update();
        handleMechState(this.currentState);
        // Bed sensors must index balls into storage during EVERY intake
        // state. The two combined states were missing, so a ball entering a
        // tunnel during INTAKE_FLOWER_SHOOT_* was never advanced.
        if (isIntakeState(currentState)) {
            updateConveyor();
        }
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
        if (newState == MechState.IDLE) {
            stopIntake(); stopShooterMotors(); stopFeedConveyor();
            robot.flowerIntake.setPosition(FLOWER_REST);
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
            robot.flowerIntake.setPosition(FLOWER_REST);
        }
        if (oldState == MechState.SHOOTING_HIVE || oldState == MechState.SHOOTING_FLOWER_NECTAR
                || oldState == MechState.SHOOTING_FLOWER_POLLEN) {
            stopFeedConveyor();
            /*
             * Stop the flywheels too. Only setState(IDLE) used to do this, so
             * going straight from a shooting state to an intake state left
             * both shooters spinning indefinitely -- draining the battery and,
             * worse, leaving live flywheels running while balls are being fed
             * into the robot. Leaving a shooting state always means the shot
             * is over, so this is the right place for it.
             */
            stopShooterMotors();
        }
    }

    /** All state decisions live here; helper methods only perform atomic actions. */
    public void handleMechState(MechState state) {
        long now = System.currentTimeMillis();
        switch (state) {
            case START:
                stopIntake(); stopShooterMotors();
                break;

            case IDLE:
                stopIntake(); stopShooterMotors();
                robot.flowerIntake.setPosition(FLOWER_REST);
                if (activeConveyor == ConveyorType.NONE) robot.conveyorMot.setPower(0);
                break;

            case INTAKE_FIELD: {
                if (totalBalls() >= 4) {
                    stopIntake(); resetFieldReject(); operationStatus = OperationStatus.ROBOT_FULL; break;
                }
                setBedsUp();
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
                } else if (isWrongNectar(color)) {
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
                    if (state == MechState.INTAKE_FLOWER) operationStatus = OperationStatus.ROBOT_FULL;
                    else setState(state == MechState.INTAKE_FLOWER_SHOOT_HIVE
                            ? MechState.SHOOTING_HIVE : MechState.SHOOTING_FLOWER_POLLEN);
                    break;
                }
                int flowerPollen = vision.getFlowerPollenCount();
                if (flowerPollen < 0) {
                    stopFlowerIntake(); operationStatus = OperationStatus.VISION_UNAVAILABLE; break;
                }
                if (flowerPollen == 0) {
                    stopFlowerIntake();
                    if (state == MechState.INTAKE_FLOWER) operationStatus = OperationStatus.FLOWER_EMPTY;
                    else setState(state == MechState.INTAKE_FLOWER_SHOOT_HIVE
                            ? MechState.SHOOTING_HIVE : MechState.SHOOTING_FLOWER_POLLEN);
                    break;
                }
                setBedsUp(); robot.flowerIntake.setPosition(FLOWER_SCOOP);
                int color = intakeColor();
                if (rejectingFlowerNectar) {
                    robot.intakeMot.setPower(-INTAKE_POWER);
                    if (color == 0) rejectingFlowerNectar = false;
                } else if (color == 2 || color == 3) {
                    rejectingFlowerNectar = true; robot.intakeMot.setPower(-INTAKE_POWER);
                } else {
                    robot.intakeMot.setPower(INTAKE_POWER);
                    if (color == 1 && previousFlowerColor == 0 && now - lastCountMs >= COUNT_COOLDOWN_MS) {
                        countPollen++; lastCountMs = now;
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
                ShooterAdapter.Target tgt = vision.updateRaisedTarget();
                if (tgt == null) tgt = shooter.hiveTargetForPosition(poseX, poseY);
                if (tgt == null) {
                    stopShooterMotors(); stopFeedConveyor();
                    operationStatus = OperationStatus.TARGET_UNKNOWN;
                    shotNote = "no cluster in view and not in a hive zone";
                    break;
                }
                // Target change means the CELL tipped: our deposit count is stale.
                if (lastKnownTarget != null && tgt != lastKnownTarget) resetCellCount();
                lastKnownTarget = tgt;

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
                    ShooterAdapter.FirePlan would =
                            shooter.plan(tgt, poseX, poseY, poseHeadingDeg);
                    shotNote = would.valid
                            ? String.format("no shot at this heading - turn to %.0f deg",
                                            would.yawDeg)
                            : "no shot from here - drive closer";
                    break;
                }
                operationStatus = OperationStatus.RUNNING;

                if (shotPhase == ShotPhase.IDLE) {
                    boolean pairAvailable = countNectar > 0 && countPollen > 0;
                    if (plan.mode == ShooterAdapter.FireMode.SIMULTANEOUS && pairAvailable) {
                        // One turret angle serves both AND both beds can feed
                        // together, so a pollen and a nectar leave on the same
                        // conveyor stroke. Half the cycles.
                        activeShotBall = BallToShoot.BOTH;
                        aimAndPrepareBoth(plan.sharedTurretDeg,
                                plan.pollen.hoodDeg, plan.pollen.rpm,
                                plan.nectar.hoodDeg, plan.nectar.rpm);
                    } else {
                        // Nectar first: 4 points of progress per ball vs pollen's 3.
                        activeShotBall = countNectar > 0 ? BallToShoot.NECTAR : BallToShoot.POLLEN;
                        ShooterAdapter.Shot s = (activeShotBall == BallToShoot.NECTAR)
                                ? plan.nectar : plan.pollen;
                        double turretDeg = (plan.mode == ShooterAdapter.FireMode.SIMULTANEOUS)
                                ? plan.sharedTurretDeg : s.turretDeg;
                        aimAndPrepare(activeShotBall, turretDeg, s.hoodDeg, s.rpm);
                    }
                    shotPhase = ShotPhase.AIM; shotPhaseStartMs = now; cycleStartMs = now;
                }
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
                if (shooterAtSpeed(activeShotBall)) {
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

    private void updateConveyor() {
        if (activeConveyor != ConveyorType.NONE) {
            if (!robot.conveyorMot.isBusy()) {
                robot.conveyorMot.setPower(0);
                robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                activeConveyor = ConveyorType.NONE;
                conveyorTargetPosition = -1;
                setBedsUp();
            }
            return;
        }
        if (!isIntakeState(currentState)) return;
        if (totalBalls() >= 4) return;
        boolean pollen = vision.ballCheck(robot.sensorPollenL, robot.sensorPollenR);
        boolean nectar = vision.ballCheck(robot.sensorNectarL, robot.sensorNectarR);

        // Index on the RISING edge only. Without this, a ball still sitting
        // over the bed sensor after a transfer triggers another one, and the
        // tunnel double-advances for a single ball.
        boolean pollenEdge = pollen && !pollenBedSeen;
        boolean nectarEdge = nectar && !nectarBedSeen;
        pollenBedSeen = pollen;
        nectarBedSeen = nectar;

        if (pollenEdge) startConveyor(ConveyorType.POLLEN);
        else if (nectarEdge && currentState == MechState.INTAKE_FIELD) startConveyor(ConveyorType.NECTAR);
    }

    private void startConveyor(ConveyorType type) {
        int ticks;
        if (type == ConveyorType.POLLEN) {
            conveyorNectar(0); conveyorPollen(1); ticks = TICKS_PER_POLLEN_STORAGE;
        } else if (type == ConveyorType.NECTAR) {
            conveyorNectar(1); conveyorPollen(0); ticks = TICKS_PER_NECTAR_STORAGE;
        } else return;
        conveyorTargetPosition = robot.conveyorMot.getCurrentPosition() + ticks;
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
    private void stopFlowerIntake() { stopIntake(); robot.flowerIntake.setPosition(FLOWER_REST); }
    private void stopShooterMotors() {
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
    }

    public MechState getState() { return currentState; }
    public MechState getPreviousState() { return previousState; }
    public OperationStatus getOperationStatus() { return operationStatus; }
    public boolean isOperationComplete() {
        return operationStatus != OperationStatus.RUNNING
                && operationStatus != OperationStatus.NOT_STARTED;
    }

    public void stopAll() {
        stopIntake(); stopShooterMotors(); robot.conveyorMot.setPower(0);
        robot.flowerIntake.setPosition(FLOWER_REST);
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
        telemetry.addData("Shot phase", shotPhase + "  " + activeShotBall);
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






