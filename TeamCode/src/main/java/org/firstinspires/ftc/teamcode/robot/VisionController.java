package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Camera and sensor access for RoboRoar.
 * Color IDs: 0=unknown/clear, 1=yellow pollen, 2=red nectar, 3=blue nectar.
 *
 * CAMERA ROLES
 *   Limelight 3A  turret-mounted, 45 deg UP. Sees the HIVE. AprilTags only.
 *   Logitech C270 rear, fixed, 40 deg DOWN. Sees the tiles and the FLOWER.
 *
 * RAISED-CELL DETECTION
 *   Each CELL carries its own 4-tag cluster on its underside, facing the
 *   tiles. When the seesaw drops a CELL it inverts, so that cluster ends up
 *   facing the ceiling and the robot cannot see it. Seeing a tag at all is
 *   therefore what tells you which CELL is raised -- no ty threshold, no
 *   calibration, no ambiguity:
 *
 *       tags 30-33 visible -> RED far side is up
 *       tags 34-37 visible -> RED audience side is up
 *       tags 42-45 visible -> BLUE far side is up
 *       tags 38-41 visible -> BLUE audience side is up
 *
 *   Still filtered over several frames, because a single bad detection
 *   should not re-point the turret.
 */
public class VisionController {

    private final RobotHardware robot;
    private final ShooterAdapter shooter;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private BallDetector ballDetector;

    private static final double DISTANCE_BALL_MM = 20.0;

    /** Consecutive agreeing frames before a target is trusted. */
    public static int REQUIRED_STABLE_FRAMES = 4;
    /** Frames without any of our tags before the target is dropped. */
    public static int LOST_FRAMES_TO_CLEAR = 10;

    private ShooterAdapter.Target raisedTarget = null;
    private ShooterAdapter.Target candidate = null;
    private int candidateFrames = 0;
    private int lostFrames = 0;
    private int lastSeenTagId = -1;
    private int lastTagCount = 0;

    public VisionController(RobotHardware robot, ShooterAdapter shooter) {
        this.robot = robot;
        this.shooter = shooter;
    }

    // ---- C270 / VisionPortal ---------------------------------------------

    /**
     * The C270 does NOT see the HIVE, so it is not used for scoring tags.
     * The processor is built anyway so the portal has something to run and so
     * any wall tags in view can assist localisation. Ball and flower detection
     * replace this in BallDetector -- see Batch 3.
     */
    /**
     * Run the AprilTag processor on the C270 as well as the ball detector.
     *
     * OFF by default, and it costs real loop time to turn on. The C270 looks
     * DOWN at the tiles and cannot see the HIVE, so it contributes nothing to
     * scoring; the only tags it might catch are wall tags, and localisation
     * comes from the Pinpoint anyway. Measured 55.7 ms average loop with both
     * processors running, against a 30 ms budget.
     *
     * The drawing options were the worst of it -- axes, cube projection and
     * outlines are rendered per tag per frame purely for a preview nobody
     * watches during a match.
     */
    public static boolean ENABLE_C270_APRILTAG = false;

    public void initAprilTag() {
        ballDetector = new BallDetector();
        VisionPortal.Builder b = new VisionPortal.Builder()
                .setCamera(robot.camera)
                // 4:3 native. The C270's 55 deg diagonal gives 45.2 x 34.7
                // here, against 48.8 x 28.6 at 720p -- the taller frame sees
                // a 21 in deep ground strip instead of 16 in, and processes a
                // quarter of the pixels. BallDetector derives its focal length
                // from the live frame size, so it follows this automatically.
                .setCameraResolution(new android.util.Size(640, 480));

        if (ENABLE_C270_APRILTAG) {
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagOutline(false)
                    .setCameraPose(robot.cameraPosition, robot.cameraOrientation)
                    .build();
            b.addProcessor(aprilTag);
        }
        b.addProcessor(ballDetector);
        visionPortal = b.build();
    }

    public BallDetector getBallDetector() { return ballDetector; }

    /**
     * Feed the Pedro pose down so ball detections can be reported in field
     * coordinates. Without it ranges and bearings still work, but fieldX/fieldY
     * come back NaN and nothing can path to a ball.
     */
    public void setPose(double xIn, double yIn, double yawDeg) {
        if (ballDetector != null) ballDetector.setPose(xIn, yIn, yawDeg);
    }

    /** Nearest ball worth collecting, or null. */
    public BallDetector.Detection nearestWantedBall(int allianceNectarColorId) {
        return ballDetector == null ? null : ballDetector.nearestWanted(allianceNectarColorId);
    }

    public AprilTagProcessor getAprilTag() { return aprilTag; }
    public VisionPortal getVisionPortal() { return visionPortal; }

    // ---- Limelight pipelines ---------------------------------------------

    /** Last pipeline commanded, so redundant switches can be skipped. */
    private int currentPipeline = -1;

    /** Pipeline last commanded, or -1 if the Limelight has not been started. */
    public int getCurrentPipeline() { return currentPipeline; }

    /**
     * Pipeline 1: AprilTag pipeline aimed at the HIVE.
     *
     * Idempotent. Re-commanding the pipeline costs a switch AND wipes the
     * raised-CELL filter, which then needs REQUIRED_STABLE_FRAMES good frames
     * before it knows anything again. Since the FSM calls this on every entry
     * to SHOOTING_HIVE, re-entering a state the Limelight was already serving
     * used to throw away a perfectly good target for no reason.
     */
    public void startHiveAprilTagPipeline() {
        if (currentPipeline == 1) return;
        robot.limeLight.pipelineSwitch(1);
        robot.limeLight.start();
        currentPipeline = 1;
        resetRaisedCellFilter();
    }

    /**
     * Pipeline 4 used to be the flower pollen count. That job belongs to the
     * C270 now -- the Limelight looks UP and cannot see a flower. Kept so
     * existing calls still compile; it simply parks the Limelight.
     */
    public void startFlowerCountPipeline() {
        /*
         * DELIBERATE NO-OP.
         *
         * This used to switch the Limelight to pipeline 4 and reset the
         * raised-CELL filter. The FSM calls it on entry to every flower
         * intake state, so collecting from a flower parked the Limelight and
         * threw away the CELL target -- and INTAKE_FLOWER_SHOOT_HIVE then
         * handed straight over to SHOOTING_HIVE, which had to switch back and
         * re-acquire four stable frames before it could aim.
         *
         * The C270 counts flower pollen now. The Limelight has no job during
         * a flower collection, so the useful thing for it to do is keep
         * watching the HIVE: by the time the handoff fires, the target is
         * already known and stable.
         */
    }

    // ---- raised-cell detection -------------------------------------------

    /**
     * Call every loop while the Limelight is on pipeline 1. Returns the CELL
     * believed to be raised, or null while unknown.
     */
    public ShooterAdapter.Target updateRaisedTarget() {
        LLResult result = robot.limeLight.getLatestResult();
        if (result == null || !result.isValid()) {
            noteLost();
            return raisedTarget;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            noteLost();
            return raisedTarget;
        }

        // Majority vote across whichever of our tags are in frame. A cluster
        // is four tags, so a single misread never decides anything.
        int farVotes = 0, audVotes = 0, seen = 0, firstId = -1;
        for (LLResultTypes.FiducialResult f : tags) {
            ShooterAdapter.Target t = shooter.targetForTagId(f.getFiducialId());
            if (t == null) continue;          // other alliance, or a wall tag
            seen++;
            if (firstId < 0) firstId = f.getFiducialId();
            if (t == ShooterAdapter.Target.FAR_SIDE) farVotes++; else audVotes++;
        }
        lastTagCount = seen;

        if (seen == 0) { noteLost(); return raisedTarget; }
        lastSeenTagId = firstId;
        lostFrames = 0;

        ShooterAdapter.Target vote = farVotes >= audVotes
                ? ShooterAdapter.Target.FAR_SIDE : ShooterAdapter.Target.AUDIENCE_SIDE;

        if (vote == candidate) {
            candidateFrames++;
        } else {
            candidate = vote;
            candidateFrames = 1;
        }
        if (candidateFrames >= REQUIRED_STABLE_FRAMES) raisedTarget = candidate;
        return raisedTarget;
    }

    private void noteLost() {
        candidate = null;
        candidateFrames = 0;
        lastTagCount = 0;
        if (++lostFrames >= LOST_FRAMES_TO_CLEAR) raisedTarget = null;
    }

    public ShooterAdapter.Target getRaisedTarget() { return raisedTarget; }
    public boolean isTargetKnown() { return raisedTarget != null; }
    public int getLastSeenTagId() { return lastSeenTagId; }
    public int getVisibleTagCount() { return lastTagCount; }

    private void resetRaisedCellFilter() {
        candidate = null;
        candidateFrames = 0;
        lostFrames = 0;
        raisedTarget = null;
        lastTagCount = 0;
    }

    // ---- flower pollen count ----------------------------------------------

    /**
     * POLLEN the C270 can see. While the robot is at a flower this IS the
     * flower count -- balls sit on the tiles there too, so being at the
     * flower is what distinguishes flower pollen from field pollen.
     *
     * Returns -1 when the camera has not started -- the FSM treats that as
     * VISION_UNAVAILABLE and stops rather than intaking blindly.
     */
    public int getFlowerPollenCount() {
        if (ballDetector == null) return -1;      // camera not started
        return ballDetector.pollenInView();
    }

    // ---- lifecycle --------------------------------------------------------

    public void stopLimelight() {
        robot.limeLight.stop();
        // Forget the pipeline, or the idempotence guard in
        // startHiveAprilTagPipeline() would skip the restart and leave the
        // Limelight stopped while the FSM believed it was running.
        currentPipeline = -1;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        stopLimelight();
    }

    // ---- colour sensors ---------------------------------------------------

    public boolean ballCheck(RevColorSensorV3 sensorL, RevColorSensorV3 sensorR) {
        return sensorL.getDistance(DistanceUnit.MM) < DISTANCE_BALL_MM
                || sensorR.getDistance(DistanceUnit.MM) < DISTANCE_BALL_MM;
    }

    public int ballColor(RevColorSensorV3 sensorL, RevColorSensorV3 sensorR) {
        return combineColors(readColor(sensorL), readColor(sensorR),
                             sensorL.getDistance(DistanceUnit.MM),
                             sensorR.getDistance(DistanceUnit.MM));
    }

    /**
     * Two sensors per station exist because the balls are PERFORATED. One can
     * be looking through a hole and read nothing while the other sees the ball
     * perfectly well, so a zero from one side is not evidence of anything --
     * the other side wins.
     *
     * When both see something and disagree on the colour, trust the one that
     * is CLOSER. A nearer reading fills more of the sensor's field and is less
     * likely to be catching an edge, a neighbouring ball, or the tunnel wall.
     * This used to return 0 and throw the reading away.
     */
    private int combineColors(int left, int right, double distL, double distR) {
        if (left == 0) return right;
        if (right == 0) return left;
        if (left == right) return left;
        return distL <= distR ? left : right;
    }

    private int readColor(RevColorSensorV3 sensor) {
        if (sensor.getDistance(DistanceUnit.MM) >= DISTANCE_BALL_MM) return 0;
        int r = sensor.red(), g = sensor.green(), b = sensor.blue();
        int max = Math.max(r, Math.max(g, b));
        if (max == 0) return 0;
        float[] hsv = new float[3];
        Color.RGBToHSV(r * 255 / max, g * 255 / max, b * 255 / max, hsv);
        float h = hsv[0], s = hsv[1], v = hsv[2];
        if (h >= 80 && h <= 110 && s >= 0.50 && v >= 0.50) return 1;   // yellow pollen
        if (h >= 15 && h <= 35 && s >= 0.45 && v >= 0.50) return 2;    // red nectar
        if (h >= 215 && h <= 240 && s >= 0.60 && v >= 0.50) return 3;  // blue nectar
        return 0;
    }
}



