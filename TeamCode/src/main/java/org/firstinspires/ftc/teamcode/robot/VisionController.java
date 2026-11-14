package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionController {

    private final RobotHardware robot;
    private final ShooterAdapter shooter;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private BallDetector ballDetector;

    private static final double DISTANCE_BALL_MM = 20.0;

    public static int REQUIRED_STABLE_FRAMES = 4;
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

    public void setPose(double xIn, double yIn, double yawDeg) {
        if (ballDetector != null) ballDetector.setPose(xIn, yIn, yawDeg);
    }

    public BallDetector.Detection nearestWantedBall(int allianceNectarColorId) {
        return ballDetector == null ? null : ballDetector.nearestWanted(allianceNectarColorId);
    }

    public AprilTagProcessor getAprilTag() { return aprilTag; }
    public VisionPortal getVisionPortal() { return visionPortal; }

    // Switch the ball detector's per-frame work on and off without touching
    // the camera. setProcessorEnabled only skips the callback -- the stream
    // keeps running, so re-enabling is immediate. Stopping the stream would
    // cost seconds of camera re-init.
    //
    // Idempotent: the SDK call is skipped when nothing changed, because this
    // is driven from the FSM and would otherwise run every loop.
    public void setBallDetectionEnabled(boolean on) {
        if (visionPortal == null || ballDetector == null || on == ballDetectionOn) return;
        // Order matters when switching OFF: stop accepting results first, so
        // a frame already inside processFrame() is discarded instead of
        // landing after the processor is disabled.
        if (!on) ballDetector.setAccepting(false);
        visionPortal.setProcessorEnabled(ballDetector, on);
        if (on) ballDetector.setAccepting(true);
        ballDetectionOn = on;
    }

    public boolean isBallDetectionEnabled() { return ballDetectionOn; }

    private boolean ballDetectionOn = true;

    // ---- Limelight pipelines ---------------------------------------------

    private int currentPipeline = -1;

    public int getCurrentPipeline() { return currentPipeline; }

    public void forcePipeline(int index) {
        robot.limeLight.setPollRateHz(POLL_RATE_HZ);
        robot.limeLight.pipelineSwitch(index);
        robot.limeLight.start();
        currentPipeline = index;
        resetRaisedCellFilter();
    }

    public boolean isLimelightConnected() { return robot.limeLight.isConnected(); }

    public double limelightAgeSec() { return robot.limeLight.getTimeSinceLastUpdate(); }

    public static int POLL_RATE_HZ = 100;

    public static int HIVE_PIPELINE = 1;

    public void startHiveAprilTagPipeline() {
        if (currentPipeline == HIVE_PIPELINE) return;
        robot.limeLight.setPollRateHz(POLL_RATE_HZ);
        robot.limeLight.pipelineSwitch(HIVE_PIPELINE);
        robot.limeLight.start();
        currentPipeline = HIVE_PIPELINE;
        resetRaisedCellFilter();
    }

    public void startFlowerCountPipeline() {
    }

    // ---- raised-cell detection -------------------------------------------

    public ShooterAdapter.Target updateRaisedTarget() {
        LLResult result = robot.limeLight.getLatestResult();
        if (result == null) {
            noteLost();
            return raisedTarget;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        lastRawIds = "";
        lastRawCount = 0;
        if (tags != null) {
            lastRawCount = tags.size();
            StringBuilder sb = new StringBuilder();
            for (LLResultTypes.FiducialResult f : tags) {
                sb.append(f.getFiducialId()).append(' ');
            }
            lastRawIds = sb.toString().trim();
        }

        if (tags == null || tags.isEmpty()) {
            noteLost();
            return raisedTarget;
        }

        // Majority vote across whichever of our tags are in frame. A cluster
        // is four tags, so a single misread never decides anything.
        int scoringVotes = 0, audVotes = 0, seen = 0, firstId = -1;
        for (LLResultTypes.FiducialResult f : tags) {
            ShooterAdapter.Target t = shooter.targetForTagId(f.getFiducialId());
            if (t == null) continue;          // other alliance, or a wall tag
            seen++;
            if (firstId < 0) firstId = f.getFiducialId();
            if (t == ShooterAdapter.Target.SCORING_SIDE) scoringVotes++; else audVotes++;
        }
        lastTagCount = seen;

        if (seen == 0) { noteLost(); return raisedTarget; }
        lastSeenTagId = firstId;
        lostFrames = 0;

        ShooterAdapter.Target vote = scoringVotes >= audVotes
                ? ShooterAdapter.Target.SCORING_SIDE : ShooterAdapter.Target.AUDIENCE_SIDE;

        if (vote == candidate) {
            if (candidateFrames < REQUIRED_STABLE_FRAMES) candidateFrames++;
        } else {
            candidate = vote;
            candidateFrames = 1;
        }
        if (candidateFrames >= REQUIRED_STABLE_FRAMES) raisedTarget = candidate;
        return raisedTarget;
    }

    private void noteLost() {
        lastTagCount = 0;
        lostFrames++;
        if (candidateFrames > 0) candidateFrames--;
        if (lostFrames >= LOST_FRAMES_TO_CLEAR) {
            candidate = null;
            candidateFrames = 0;
            raisedTarget = null;
        }
    }

    public int getCandidateFrames() { return candidateFrames; }
    public int getLostFrames() { return lostFrames; }

    private String lastRawIds = "";
    private int lastRawCount = 0;
    public String getLastRawIds() { return lastRawIds; }
    public int getRawTagCount() { return lastRawCount; }

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
