package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MechController {
    public enum OperationStatus {
        NOT_STARTED, RUNNING, COMPLETED, ROBOT_FULL, FLOWER_EMPTY,
        VISION_UNAVAILABLE, TARGET_UNKNOWN, NOT_CONFIGURED, NO_SHOT,
        TURRET_NOT_HOMED, CANCELLED, TIMED_OUT, FAILED
    }
    private enum ConveyorType { NONE, POLLEN, NECTAR }
    private enum ShotPhase { IDLE, AIM, SPIN_UP, FEEDING, RECOVER }
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
    private double poseX = 72.0, poseY = 72.0, poseHeadingDeg = 0.0;
    private boolean poseValid = false;

    // ---- conveyor / storage ----------------------------------------------
    public static int TICKS_PER_POLLEN_STORAGE = 200;   // measured 14 Nov (was 100)
    public static int TICKS_PER_NECTAR_STORAGE = 250;   // measured 14 Nov (was 150)
    public static int TICKS_TO_SHOOT_POLLEN = 400;   // measured 14 Nov (was 300)
    public static int TICKS_TO_SHOOT_NECTAR = 500;   // measured 14 Nov (was 200)
    public static int TICKS_TO_SHOOT_BOTH = 500;   // measured 14 Nov (was 300)
    public static final int MAX_CONVEYOR_ROTATION = 300;   // bed servo span, deg
    // Four positions, not a shared pair.
    public static double BED_POLLEN_ENGAGED    = 0.360;
    public static double BED_POLLEN_DISENGAGED = 0.170;
    public static double BED_NECTAR_ENGAGED    = 0.230;
    public static double BED_NECTAR_DISENGAGED = 0.500;

    public static long REVERSE_INTAKE_TIME_MS = 700;
    private static final long SENSOR_CLEAR_TIMEOUT_MS = 1800;
        // 1.0, measured 14 Nov.
    public static double CONVEYOR_POWER = 1.0;
    public static double INTAKE_POWER = 1.0;

    // ---- START deployment --------------------------------------------------
    public static long DEPLOY_MS = 600;
    public static double DEPLOY_POWER = 0.60;
    public static long DEPLOY_SCOOP_DELAY_MS = 250;

    private long deployStartMs = 0;
    private boolean deployDone = false;

    // ---- flower approach ---------------------------------------------------
    public static double FLOWER_DEPLOY_IN  = 19.5;
    public static double FLOWER_INTAKE_IN  = 16.5;
    public static double FLOWER_RETRACT_IN = 20.0;

    public double flowerRangeIn() {
        if (!poseValid) return Double.NaN;
        double[] f = shooter.flowerTargetForPosition(poseX, poseY);
        if (f == null) return Double.NaN;
        return Math.hypot(f[0] - poseX, f[1] - poseY);
    }

    // ---- shooting timing / tolerances -------------------------------------
    private static final long SHOOTER_SPINUP_TIMEOUT_MS = 6000;
    private static final long SHOOTER_FEED_TIMEOUT_MS = 2500;
    // MEASURED 14 Nov, timed from the lowest point back to within 2%
    public static long SHOOTER_RECOVER_MS = 1750;
    private static final long FEED_MIN_MS = 150;
    private static final long TURRET_SETTLE_TIMEOUT_MS = 2500;
    public static double SHOOTER_READY_TOLERANCE = 0.95;
    public static boolean REQUIRE_HOMED = true;
    // flowerIntake measured stops, 13 Nov: 0.445 ..
    public static double FLOWER_REST = 0.540, FLOWER_SCOOP = 0.445;

    // Where the scoop sits BEFORE the arm deploys, measured 14 Nov.
    public static double FLOWER_PREDEPLOY = 0.820;

    public static double FLOWER_HOOD_DEG = 80.0;
    public static double FLOWER_POLLEN_RPM = 1800.0;
    public static double FLOWER_NECTAR_RPM = 1800.0;
    public static double FLOWER_AIM_TRIM_DEG = 0.0;

    // ---- inventory ---------------------------------------------------------
    public int countNectar = 0, countPollen = 0;

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
    private boolean pollenBedSeen = false, nectarBedSeen = false;
    private int pollenRetries = 0, nectarRetries = 0;
    private static final int MAX_CLEAR_RETRIES = 3;
    private long lastCountMs = 0;
    public static long COUNT_COOLDOWN_MS = 350;

    // ---- last computed plan, for telemetry --------------------------------
    private ShooterAdapter.FirePlan lastPlan = null;
    private String shotNote = "";

    // Loop timing.
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

    public int allianceTeamNectar() { return shooter.allianceNectarColorId(); }

    public void updatePose(double xIn, double yIn, double headingDeg) {
        long now = System.currentTimeMillis();
        if (poseValid && lastPoseMs != 0) {
            double dt = (now - lastPoseMs) / 1000.0;
            if (dt > 0.001) {
                double d = Math.hypot(xIn - poseX, yIn - poseY);
                double inst = d / dt;
                // Lightly smoothed: odometry jitter at 150 Hz would otherwise
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

    public static double FIRE_MAX_SPEED_IN_S = 4.0;
    public static double FIRE_MAX_TURN_DEG_S = 15.0;

    // One meaning for the flag everywhere.
    private boolean steadyOrDisabled() {
        return !FIRE_ONLY_WHEN_STEADY || isSteadyEnoughToFire();
    }

    public boolean isSteadyEnoughToFire() {
        return speedInPerSec <= FIRE_MAX_SPEED_IN_S
                && turnDegPerSec <= FIRE_MAX_TURN_DEG_S;
    }

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

        vision.updateRaisedTarget();
        serviceBallVision();

        handleMechState(this.currentState);
        // AFTER the state, not before: deployment owns the intake motor
        serviceDeploy(nowMs);
        // AFTER the state for the same reason deployment is: while the drop
        serviceBedDrop(nowMs);
        // Single owner for the scoop, by distance only.
        serviceScoop();
        // Last word on the intake motor.
        serviceIntakeLimit();

        // Watch both flywheels for the dip a departing ball causes.
        serviceExitDetect();

        serviceConveyorCompletion();
        updateConveyor();
    }

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

    // Everything commanded to a known place at INIT, before play.
    public void initPositions() { initPositions(false); }

    // armAlreadyDeployed: autonomous starts stowed and deploys, so it passes
    public void initPositions(boolean armAlreadyDeployed) {
        // setBedsUp() is the ENGAGED pattern -- conveyorX(1) maps
        setBedsUp();
        robot.turretRot.setPosition(TurretController.SERVO_AT_ZERO);
        // Hoods start at their BOTTOM limit, 80 deg.
        robot.hoodPollen.setPosition(HoodMap.POLLEN_INIT_POS);
        robot.hoodNectar.setPosition(HoodMap.NECTAR_INIT_POS);

        if (armAlreadyDeployed) {
            deployDone = true;              // do not re-run the sequence
            robot.flowerIntake.setPosition(FLOWER_REST);
        } else {
            robot.flowerIntake.setPosition(FLOWER_PREDEPLOY);
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
        preSpinning = false;
        haltFlow();
        // A new command is the driver's answer to a tipped CELL, so clear
        cellTipped = false;
        pollenExits = 0; nectarExits = 0;
        pollenExitsAtArm = 0; nectarExitsAtArm = 0;
        // DISARMED, not armed -- a state change is not evidence either wheel
        pollenDipLow = true; nectarDipLow = true;
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
        if (maySpin(oldState) && !maySpin(newState)) {
            stopFeedConveyor();
            stopShooterMotors();
        }
    }

    private static boolean maySpin(MechState s) {
        return s == MechState.SHOOTING_HIVE
                || s == MechState.SHOOTING_FLOWER_NECTAR
                || s == MechState.SHOOTING_FLOWER_POLLEN
                || s == MechState.INTAKE_FLOWER_SHOOT_HIVE
                || s == MechState.INTAKE_FLOWER_SHOOT_FLOWER;
    }

    public void handleMechState(MechState state) {
        long now = System.currentTimeMillis();
        switch (state) {
            case START:
                stopShooterMotors();
                // Deployment is NOT here -- see serviceDeploy(), which runs
                if (!deployDone) operationStatus = OperationStatus.RUNNING;
                else operationStatus = OperationStatus.COMPLETED;
                break;

            case IDLE:
                stopIntake(); stopShooterMotors();
                // scoop: serviceScoop() folds it once clear of the flower
                if (activeConveyor == ConveyorType.NONE) robot.conveyorMot.setPower(0);
                break;

            case INTAKE_FIELD: {
                ShooterAdapter.Target seenCell = vision.getRaisedTarget();
                if (seenCell != null && checkCellTipped(seenCell) && TIP_STOPS_INTAKE) {
                    haltFlow();
                    releaseWheels();
                    stopIntake();
                    resetFieldReject();
                    setBedsUp();
                    break;
                }
                if (FIRE_DURING_FIELD_INTAKE && canFireFromHere() && steadyOrDisabled()) {
                    serviceContinuousFire(now, true);
                } else {
                    haltFlow();
                    // Before the full check: a robot that has just filled up
                    servicePreSpin();
                }
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
                        // No deferral needed.
                        setState(state == MechState.INTAKE_FLOWER_SHOOT_HIVE
                                ? MechState.SHOOTING_HIVE : MechState.SHOOTING_FLOWER_POLLEN);
                    }
                    break;
                }
                int flowerPollen = vision.getFlowerPollenCount();
                if (flowerPollen < 0) {
                    operationStatus = OperationStatus.VISION_UNAVAILABLE;
                    flowerPollen = 1;                 // assume there is more
                }
                if (flowerPollen == 0) {
                    stopFlowerIntake();
                    if (state == MechState.INTAKE_FLOWER) {
                        operationStatus = OperationStatus.FLOWER_EMPTY;
                    } else {
                        // Flower is dry but we may still be holding balls
                        setState(state == MechState.INTAKE_FLOWER_SHOOT_HIVE
                                ? MechState.SHOOTING_HIVE : MechState.SHOOTING_FLOWER_POLLEN);
                    }
                    break;
                }
                boolean noPose = !poseValid;
                double fRange = flowerRangeIn();
                boolean knowRange = !Double.isNaN(fRange);
                // No clearToDeploy here any more -- serviceScoop() owns
                boolean atFlower = noPose || (knowRange && fRange <= FLOWER_INTAKE_IN);

                if (!noPose && (!knowRange || fRange > FLOWER_RETRACT_IN)) {
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
                if (!atFlower) {
                    // Approaching, not there yet.
                    stopIntake();
                    if (state != MechState.INTAKE_FLOWER) {
                        serviceContinuousFire(now,
                                state == MechState.INTAKE_FLOWER_SHOOT_HIVE);
                    }
                    if (!flowing) setBedsUp();
                    break;
                }

                if (state != MechState.INTAKE_FLOWER) {
                    serviceContinuousFire(now, state == MechState.INTAKE_FLOWER_SHOOT_HIVE);
                }
                if (!flowing) setBedsUp();

                int color = intakeColor();
                if (rejectingFlowerNectar) {
                    robot.intakeMot.setPower(-INTAKE_POWER);
                    if (color == 0) rejectingFlowerNectar = false;
                } else if (isWrongNectar(color) || nectarTunnelFull(color)) {
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
                    stopFeedConveyor();
                    shotPhase = ShotPhase.IDLE;
                    operationStatus = OperationStatus.TARGET_UNKNOWN;
                    shotNote = "waiting for a CELL -- flywheels held at speed";
                    break;
                }
                if (checkCellTipped(tgt)) {
                    stopShooterMotors();
                    stopFeedConveyor();
                    shotPhase = ShotPhase.IDLE;
                    break;
                }

                if (totalBalls() <= 0 && shotPhase == ShotPhase.IDLE) {
                    stopShooterMotors(); operationStatus = OperationStatus.COMPLETED; break;
                }

                ShooterAdapter.FirePlan plan =
                        shooter.planHere(tgt, poseX, poseY, poseHeadingDeg);
                lastPlan = plan;
                if (!plan.valid) {
                    stopShooterMotors(); stopFeedConveyor();
                    operationStatus = OperationStatus.NO_SHOT;
                    // plan() answers the question planHere() cannot: IS there
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

    public static boolean FIRE_DURING_FIELD_INTAKE = true;

    private boolean inFiringPosition() {
        if (!poseValid || totalBalls() <= 0) return false;
        if (REQUIRE_HOMED && !turret.isHomed()) return false;
        ShooterAdapter.Target tgt = vision.getRaisedTarget();
        if (tgt == null) tgt = shooter.hiveTargetForPosition(poseX, poseY);
        return tgt != null && shooter.inHiveZone(tgt, poseX, poseY);
    }

    private boolean canFireFromHere() {
        if (!inFiringPosition()) return false;

        if (cellTipped) {
            cellTipped = false;
            shotNote = "";
        }
        return true;
    }

    private void servicePreSpin() {
        if (!PRE_SPIN_ENABLED) { releaseWheels(); return; }
        if (shotPhase != ShotPhase.IDLE) { preSpinHoldUntilMs = 0; return; }
        if (totalBalls() <= 0 || !poseValid) { releaseWheels(); return; }
        if (REQUIRE_HOMED && !turret.isHomed()) { releaseWheels(); return; }

        // The cache is refreshed every loop by update(), so it is always
        ShooterAdapter.Target tgt = vision.getRaisedTarget();
        if (tgt == null) tgt = shooter.hiveTargetForPosition(poseX, poseY);
        if (tgt == null || !shooter.inHiveZone(tgt, poseX, poseY)) { releaseWheels(); return; }

        ShooterAdapter.FirePlan plan = shooter.planHere(tgt, poseX, poseY, poseHeadingDeg);
        if (!plan.valid) { releaseWheels(); return; }

        // Aim and spin, but do NOT advance shotPhase -- nothing is armed
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

    public static long PRE_SPIN_RELEASE_MS = 750;
    private long preSpinHoldUntilMs = 0;

    private void releaseWheels() {
        if (targetPollenTps <= 0 && targetNectarTps <= 0) { preSpinHoldUntilMs = 0; return; }
        long now = System.currentTimeMillis();
        if (preSpinHoldUntilMs == 0) { preSpinHoldUntilMs = now + PRE_SPIN_RELEASE_MS; return; }
        if (now < preSpinHoldUntilMs) return;
        preSpinHoldUntilMs = 0;
        stopShooterMotors();        // clears preSpinning
    }

    public static boolean PRE_SPIN_ENABLED = true;
    private boolean preSpinning = false;

    // ---- ball-exit detection by flywheel RPM dip ---------------------------
    public static double DIP_ENTER = 0.960;   // below this fraction of target = a ball went
    // 0.965, squeezed between DIP_ENTER 0.960 and the noise low point 0.968.
    public static double DIP_EXIT  = 0.965;   // above this, re-arm for the next

    private boolean pollenDipLow = true, nectarDipLow = true;
    private int pollenExits = 0, nectarExits = 0;
    private int pollenExitsAtArm = 0, nectarExitsAtArm = 0;

    private void serviceExitDetect() {
        if (targetPollenTps > 0) {
            double r = robot.pollenShootingMot.getVelocity() / targetPollenTps;
            if (!pollenDipLow && r < DIP_ENTER) { pollenDipLow = true; pollenExits++; }
            else if (pollenDipLow && r > DIP_EXIT) pollenDipLow = false;
        } else {
            // Wheel not commanded: disarm, so the next spin-up has to reach
            pollenDipLow = true;
        }
        if (targetNectarTps > 0) {
            double r = robot.nectarShootingMot.getVelocity() / targetNectarTps;
            if (!nectarDipLow && r < DIP_ENTER) { nectarDipLow = true; nectarExits++; }
            else if (nectarDipLow && r > DIP_EXIT) nectarDipLow = false;
        } else {
            nectarDipLow = true;      // disarmed -- see pollen above
        }
    }

    public int getPollenExits() { return pollenExits; }
    public int getNectarExits() { return nectarExits; }

    public boolean isPreSpinning() { return preSpinning; }

    private void serviceContinuousFire(long now, boolean hive) {
        // Each refusal names itself.
        if (REQUIRE_HOMED && !turret.isHomed()) {
            shotNote = "holding fire: turret not homed"; haltFlow(); return;
        }
        if (!poseValid) {
            shotNote = "holding fire: no pose"; haltFlow(); return;
        }
        // A flower shot is pollen by definition.
        if (hive ? totalBalls() <= 0 : countPollen <= 0) {
            shotNote = "holding fire: nothing to shoot"; haltFlow(); return;
        }
        // We own the wheels now, not pre-spin.
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
                // Same as the dedicated state: a tip means no shot
                haltFlow();
                stopShooterMotors();
                if (TIP_STOPS_INTAKE) stopIntake();
                return;
            }
            ShooterAdapter.FirePlan plan =
                    shooter.planHere(tgt, poseX, poseY, poseHeadingDeg);
            if (!plan.valid) {
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

        // Hold the aim and the speed.
        turret.setTurretDeg(turretDeg);
        robot.hoodPollen.setPosition(HoodMap.pollenPos(hoodDeg));
        targetPollenTps = RobotHardware.rpmToTicksPerSec(rpm);
        robot.pollenShootingMot.setVelocity(targetPollenTps);
        operationStatus = OperationStatus.RUNNING;

        if (countPollen <= 0 && !streamNectar) {
            shotNote = "holding fire: nectar only, and this angle needs a "
                     + "separate nectar volley";
            haltFlow();
            return;
        }

        conveyorPollen(1);
        conveyorNectar(streamNectar ? 1 : 0);

        // BOTH wheels must be up when both are streaming -- feeding a nectar
        boolean ready = turret.isSettled()
                && shooterAtSpeed(BallToShoot.POLLEN)
                && (!streamNectar || shooterAtSpeed(BallToShoot.NECTAR));
        if (ready) {
            // An index was mid-stroke and is being taken over.
            if (activeConveyor != ConveyorType.NONE) flowAnchorValid = false;
            activeConveyor = ConveyorType.NONE;
            conveyorTargetPosition = -1;
            if (!flowAnchorValid) {
                flowAnchorTicks = robot.conveyorMot.getCurrentPosition();
                flowAnchorValid = true;
            }
            flowing = true;
            robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.conveyorMot.setPower(CONVEYOR_POWER);
        } else {
            // Wheel still recovering, or turret still swinging.
            flowing = false;
            robot.conveyorMot.setPower(0);
            syncExitBaseline();
            return;
        }

        countFlowByBeltTravel(hive);
    }

    public static boolean COUNT_FLOW_BY_DIP = false;

    private int flowAnchorTicks = 0;
    private boolean flowAnchorValid = false;

    private void countFlowByBeltTravel(boolean hive) {
        if (COUNT_FLOW_BY_DIP) { countFlowByDip(hive); return; }

        int perBall = streamNectar ? TICKS_TO_SHOOT_BOTH : TICKS_TO_SHOOT_POLLEN;
        if (perBall <= 0) return;

        // Signed, and only forward travel counts.
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
            if (streamNectar && countNectar > 0) {
                countNectar--;
                shotsCompleted++;
                if (hive) cellNectarIn++;
            }
        }
        // The dip counters keep running as the second opinion, but they must
        syncExitBaseline();
    }

    private void countFlowByDip(boolean hive) {
        // Count what actually left, not what the belt did.
        int goneP = pollenExits - pollenExitsAtArm;
        if (goneP > 0) {
            pollenExitsAtArm = pollenExits;
            for (int i = 0; i < goneP && countPollen > 0; i++) {
                countPollen--;
                shotsCompleted++;
                if (hive) cellPollenIn++;
            }
        }
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

    private void haltFlow() {
        syncExitBaseline();
        // End of the session, so the partial travel goes with it.
        flowAnchorValid = false;
        if (!flowing) return;
        flowing = false;
        robot.conveyorMot.setPower(0);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void syncExitBaseline() {
        pollenExitsAtArm = pollenExits;
        nectarExitsAtArm = nectarExits;
    }

    public boolean isFlowing() { return flowing; }
    private boolean flowing = false;

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

        if (cellTipped && shooter.inHiveZone(tgt, poseX, poseY)) {
            cellTipped = false;
            shotNote = "";
        }
        return cellTipped;
    }

    public static boolean TIP_STOPS_INTAKE = false;

    private boolean cellTipped = false;

    public void clearCellTipped() { cellTipped = false; }
    public boolean isCellTipped() { return cellTipped; }

    private void issueAim(ShooterAdapter.FirePlan plan) {
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
    }

    private void armFromPlan(ShooterAdapter.FirePlan plan, long now) {
        issueAim(plan);
        shotPhase = ShotPhase.AIM; shotPhaseStartMs = now; cycleStartMs = now;
    }

    // Re-solve from where the robot is NOW, without touching the shot phase.
    private void reAim() {
        if (!poseValid) return;
        ShooterAdapter.Target tgt = vision.getRaisedTarget();
        if (tgt == null) tgt = shooter.hiveTargetForPosition(poseX, poseY);
        if (tgt == null) return;
        ShooterAdapter.FirePlan plan = shooter.planHere(tgt, poseX, poseY, poseHeadingDeg);
        if (!plan.valid) return;
        lastPlan = plan;
        issueAim(plan);
    }

    // Hold the ball until the robot is steady.
    public static boolean FIRE_ONLY_WHEN_STEADY = true;

    private void runShotSequence(long now) {
        switch (shotPhase) {
            case AIM:
                // Still moving: track the target and hold speed, but commit
                if (FIRE_ONLY_WHEN_STEADY && !isSteadyEnoughToFire()) {
                    reAim();
                    shotPhaseStartMs = now;
                    shotNote = String.format("holding: moving %.1f in/s", speedInPerSec);
                    break;
                }
                // Wait on the turret actually arriving, not a fixed time.
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

    private boolean shotOwnsBelt() { return shotPhase == ShotPhase.FEEDING; }

    private void updateConveyor() {
        // Both false while the sensors are off, which is the quiet state
        boolean pollen = USE_COLOR_SENSORS
                && vision.ballCheck(robot.sensorPollenL, robot.sensorPollenR);
        boolean nectar = USE_COLOR_SENSORS
                && vision.ballCheck(robot.sensorNectarL, robot.sensorNectarR);

        // Continuous flow already runs the belt and carries balls
        if (flowing || shotOwnsBelt()
                || activeConveyor != ConveyorType.NONE
                || !isIntakeState(currentState)
                || totalBalls() >= 4) {
            // Keep the history current so the NEXT eligible loop sees a true
            pollenBedSeen = pollen;
            nectarBedSeen = nectar;
            return;
        }

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

    // ---- driver-commanded bed drop ---------------------------------------
    public static long BED_DROP_MS = 2000;
    private long bedDropUntilMs = 0;
    private boolean bedDropActive = false;

    public void dropBedsMomentarily() {
        bedDropUntilMs = System.currentTimeMillis() + BED_DROP_MS;
        bedDropActive = true;
        conveyorNectar(0);
        conveyorPollen(0);
    }

    public boolean bedsDropped() { return bedDropActive; }

    public long bedDropRemainingMs() {
        return bedDropActive
                ? Math.max(0, bedDropUntilMs - System.currentTimeMillis()) : 0;
    }

    private void serviceBedDrop(long nowMs) {
        if (!bedDropActive) return;
        if (nowMs < bedDropUntilMs) {
            // Re-commanded every loop on purpose: the state handlers call
            conveyorNectar(0);
            conveyorPollen(0);
            return;
        }
        bedDropActive = false;
        // Put them back here rather than waiting for a state handler to do
        setBedsUp();
    }

    // ---- driver-commanded index -----------------------------------------
    public boolean indexPollen() { return manualIndex(ConveyorType.POLLEN); }
    public boolean indexNectar() { return manualIndex(ConveyorType.NECTAR); }

    private boolean manualIndex(ConveyorType type) {
        if (flowing || shotOwnsBelt() || activeConveyor != ConveyorType.NONE) {
            return false;
        }
        startConveyor(type);
        return true;
    }

    private void startConveyor(ConveyorType type) {
        int ticks;
        if (type == ConveyorType.POLLEN) {
            conveyorNectar(0); conveyorPollen(1); ticks = TICKS_PER_POLLEN_STORAGE;
        } else if (type == ConveyorType.NECTAR) {
            conveyorNectar(1); conveyorPollen(0); ticks = TICKS_PER_NECTAR_STORAGE;
        } else return;
        conveyorTargetPosition = robot.conveyorMot.getCurrentPosition() + ticks;
        // This stroke is an index, not a feed.
        flowAnchorValid = false;
        activeConveyor = type;
        robot.conveyorMot.setTargetPosition(conveyorTargetPosition);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.conveyorMot.setPower(CONVEYOR_POWER);
    }

    private void beginFeed(BallToShoot ball) {
        setBedsUp();
        int ticks;
        if (ball == BallToShoot.BOTH) {
            // Both engaged: a pollen and a nectar leave on the same stroke.
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

    public static double bedPollenPos(boolean engaged) {
        // pollen ENGAGES by raising, so engaged is the larger position
        return engaged ? BED_POLLEN_ENGAGED : BED_POLLEN_DISENGAGED;
    }
    public static double bedNectarPos(boolean engaged) {
        // nectar ENGAGES by lowering: the mirror of the above
        return engaged ? BED_NECTAR_ENGAGED : BED_NECTAR_DISENGAGED;
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

    // Colour sensors are mechanically blind: all six sit recessed in holes
    public static boolean USE_COLOR_SENSORS = false;

    // 0 is already this method's "nothing in front of the sensor" value
    private int intakeColor() {
        if (!USE_COLOR_SENSORS) return 0;
        return vision.ballColor(robot.sensorIntakeR, robot.sensorIntakeL);
    }
    public static final int NECTAR_TUNNEL_CAPACITY = 3;
    public static final int POLLEN_TUNNEL_CAPACITY = 4;

    private boolean nectarTunnelFull(int color) {
        return color == allianceTeamNectar() && countNectar >= NECTAR_TUNNEL_CAPACITY;
    }

    private boolean isWrongNectar(int color) {
        return (color == 2 || color == 3) && color != allianceTeamNectar();
    }
    private int totalBalls() { return countNectar + countPollen; }

    private static boolean isIntakeState(MechState s) {
        return s == MechState.INTAKE_FIELD || s == MechState.INTAKE_FLOWER
                || s == MechState.INTAKE_FLOWER_SHOOT_HIVE
                || s == MechState.INTAKE_FLOWER_SHOOT_FLOWER;
    }
    // The C270 is only read in four of the nine states: the three flower
    public static boolean BALL_VISION_BY_STATE = true;

    private void serviceBallVision() {
        if (!BALL_VISION_BY_STATE) { vision.setBallDetectionEnabled(true); return; }
        vision.setBallDetectionEnabled(
                currentState == MechState.INTAKE_FIELD
                || currentState == MechState.INTAKE_FLOWER
                || currentState == MechState.INTAKE_FLOWER_SHOOT_HIVE
                || currentState == MechState.INTAKE_FLOWER_SHOOT_FLOWER);
    }

    private void stopIntake() { robot.intakeMot.setPower(0); }

    private void serviceIntakeLimit() {
        if (!deployDone) return;
        if (totalBalls() >= 4 || countPollen >= POLLEN_TUNNEL_CAPACITY) {
            stopIntake();
        }
    }
    private void scoopTo(double pos) {
        if (!deployDone) return;
        robot.flowerIntake.setPosition(pos);
    }

    private void serviceScoop() {
        if (!deployDone) return;

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

    public boolean isDeployed() { return deployDone; }

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
        flowing = false;
        stopIntake(); stopShooterMotors(); robot.conveyorMot.setPower(0);
        scoopTo(FLOWER_REST);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        activeConveyor = ConveyorType.NONE; conveyorTargetPosition = -1;
        operationStatus = OperationStatus.CANCELLED;
    }

    public static boolean COMPACT_TELEMETRY = true;

    private String whyLine() {
        if (!turret.isHomed())      return "turret NOT HOMED";
        if (!deployDone)            return "arm still stowed";
        if (flowing)                return "FIRING";
        if (shotPhase != ShotPhase.IDLE) return shotPhase + " " + activeShotBall;
        if (cellTipped)             return "CELL TIPPED - reposition";
        if (!poseValid)             return "no pose";
        if (totalBalls() <= 0)      return "empty";
        if (!shotNote.isEmpty())    return shotNote;
        if (!inFiringPosition())    return "not in a hive zone for this target";
        if (!steadyOrDisabled())
            return String.format("moving %.1f in/s - holding", speedInPerSec);
        if (preSpinning)            return "pre-spun, ready";
        return "ready";
    }

    public void allTelemetry() {
        telemetry.addData("STATE ", "%s / %s", currentState, operationStatus);
        telemetry.addData("BALLS ", "P%d N%d = %d/4    shots %d  dip %d%s",
                countPollen, countNectar, totalBalls(),
                shotsCompleted, pollenExits + nectarExits,
                COUNT_FLOW_BY_DIP ? "  (DIP IS PRIMARY)" : "");
        telemetry.addData("WHY   ", whyLine());
        ShooterAdapter.Target seen = vision.getRaisedTarget();
        ShooterAdapter.Target guess = seen != null
                ? null : shooter.hiveTargetForPosition(poseX, poseY);
        telemetry.addData("WHERE ", "%s    %s",
                poseValid ? String.format("%.0f,%.0f @%.0f",
                                          poseX, poseY, poseHeadingDeg)
                          : "POSE NOT SET",
                seen != null ? seen + " (tag)"
                             // "none", not String.valueOf(null) -- a line
                             : (guess == null ? "none (no hive zone)"
                                              : guess + " (by zone)"));
        telemetry.addData("VISION", "%d tags [%s] %d ours   lock %d/%d   flower %d  pipe %d",
                vision.getRawTagCount(),
                vision.getLastRawIds().isEmpty() ? "-" : vision.getLastRawIds(),
                vision.getVisibleTagCount(),
                vision.getCandidateFrames(), VisionController.REQUIRED_STABLE_FRAMES,
                vision.getFlowerPollenCount(), vision.getCurrentPipeline());
        telemetry.addData("BELT  ", "beds P:%s N:%s   idx %s%s",
                pollenBedSeen ? "BALL" : "-", nectarBedSeen ? "BALL" : "-",
                activeConveyor,
                activeConveyor != ConveyorType.NONE
                        ? " busy " + robot.conveyorMot.isBusy()
                        : (totalBalls() >= 4 ? " BLOCKED:full"
                           : flowing ? " BLOCKED:flowing" : ""));
        telemetry.addData("LOOP  ", "%.1fms  %.0fHz   worst %.0f",
                loopMsAvg, loopMsAvg > 0 ? 1000.0 / loopMsAvg : 0, loopMsWorst);

        // Nearest ball worth having, only while sweeping the field.
        if (currentState == MechState.INTAKE_FIELD) {
            BallDetector.Detection d = vision.nearestWantedBall(allianceTeamNectar());
            telemetry.addData("BALL", d == null
                    ? "none in view"
                    : String.format("%s  %.0f in  %+.0f deg", d.type, d.rangeIn, d.bearingDeg));
        }
        if (currentState == MechState.INTAKE_FLOWER
                || currentState == MechState.INTAKE_FLOWER_SHOOT_HIVE
                || currentState == MechState.INTAKE_FLOWER_SHOOT_FLOWER) {
            double[] f = shooter.flowerTargetForPosition(poseX, poseY);
            double fr = flowerRangeIn();
            telemetry.addData("FLOWER", f == null
                    ? "not in any flower zone"
                    : String.format("%.0f,%.0f   %.1f in   %s", f[0], f[1], fr,
                            fr <= FLOWER_INTAKE_IN ? "scoop+intake"
                          : fr <= FLOWER_DEPLOY_IN ? "scoop only"
                          : fr >  FLOWER_RETRACT_IN ? "folded" : "holding"));
        }

        String alerts = "";
        if (cellTipped)                 alerts += "[CELL TIPPED] ";
        if (!turret.isHomed())          alerts += "[TURRET NOT HOMED] ";
        if (!deployDone)                alerts += "[ARM STOWED] ";
        if (!HoodMap.pollenRangeFits()) alerts += "[POLLEN HOOD CLAMPED] ";
        if (!HoodMap.nectarRangeFits()) alerts += "[NECTAR HOOD CLAMPED] ";
        if (!alerts.isEmpty()) telemetry.addData("!!    ", alerts.trim());

        if (COMPACT_TELEMETRY) {
            telemetry.update();
            return;
        }

        // ---- full diagnostic page ------------------------------------------
        telemetry.addLine();
        telemetry.addData("Shot phase", "%s %s%s%s", shotPhase, activeShotBall,
                flowing ? "  [FLOWING]" : "", preSpinning ? "  [PRE-SPUN]" : "");
        telemetry.addData("Exits", "%s  dip pollen %d  dip nectar %d  belt shots %d",
                COUNT_FLOW_BY_DIP ? "dip=PRIMARY" : "belt=PRIMARY",
                pollenExits, nectarExits, shotsCompleted);
        telemetry.addData("Fire gate", "%s   %.1f in/s  %.1f deg/s  %s",
                inFiringPosition() ? "in zone" : "no zone/target",
                speedInPerSec, turnDegPerSec,
                isSteadyEnoughToFire() ? "steady" : "MOVING");
        telemetry.addData("Tag filter", "%d/%d agreed   %d lost",
                vision.getCandidateFrames(), VisionController.REQUIRED_STABLE_FRAMES,
                vision.getLostFrames());
        telemetry.addData("Phase ms", "aim %d  spin %d  feed %d  -> cycle %d",
                lastAimMs, lastSpinMs, lastFeedMs, lastCycleMs);
        if (!shotNote.isEmpty()) telemetry.addData("Note", shotNote);

        telemetry.addLine();
        telemetry.addData("Alliance", shooter.getAlliance());
        telemetry.addData("Operation Complete", isOperationComplete());
        telemetry.addLine();
        // Pose is on WHERE already; this is the same number to one more
        telemetry.addData("Pose exact", poseValid
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
        // Bed sensors are on the BELT line; the cached flags are not re-read
        telemetry.addData("Conveyor target ticks", conveyorTargetPosition);
        telemetry.addData("Feed target ticks", feedTargetPosition);
        turret.addTelemetry(telemetry);
        // Do not call pinpoint.update() here; PedroPathing/localizer owns it.
        robot.addPinpointTelemetry(telemetry);
        telemetry.update();
    }
}
