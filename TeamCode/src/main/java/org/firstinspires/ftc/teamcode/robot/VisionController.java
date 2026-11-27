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

    // Seen when CLOSER than this. Must clear the empty-tunnel reading.

    public static double DISTANCE_BALL_MM = 31.0;

    public static int REQUIRED_STABLE_FRAMES = 4;
    public static int LOST_FRAMES_TO_CLEAR = 10;

    public static float POLLEN_H_LO = 80,  POLLEN_H_HI = 110;
    public static float NECTAR_R_H_LO = 15,  NECTAR_R_H_HI = 35;
    public static float NECTAR_B_H_LO = 215, NECTAR_B_H_HI = 240;
    public static float POLLEN_S_MIN = 0.50f, POLLEN_V_MIN = 0.50f;
    public static float NECTAR_R_S_MIN = 0.45f, NECTAR_R_V_MIN = 0.50f;
    public static float NECTAR_B_S_MIN = 0.60f, NECTAR_B_V_MIN = 0.50f;

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

    public void setBallDetectionEnabled(boolean on) {
        if (visionPortal == null || ballDetector == null || on == ballDetectionOn) return;
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

        int scoringVotes = 0, audVotes = 0, seen = 0, firstId = -1;
        for (LLResultTypes.FiducialResult f : tags) {
            ShooterAdapter.Target t = shooter.targetForTagId(f.getFiducialId());
            if (t == null) continue;
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
        if (ballDetector == null) return -1;
        return ballDetector.pollenInView();
    }

    // ---- lifecycle --------------------------------------------------------

    public void stopLimelight() {
        robot.limeLight.stop();
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

    // I2C, so not bulk cached: one distance and one RGB per sensor.

    // Open ramp, nothing near it, so the gate is long on purpose: a wider
    // window is more loops to catch the ball in. MIN_LEVEL is off until the
    // empty ramp is seen to false-trigger; SENSE shows the level to set it.
    public static double DISTANCE_INTAKE_MM = 75.0;
    public static int MIN_LEVEL = 0;

    private double distL = 999, distR = 999;
    private int colL = 0, colR = 0, lvlL = 0, lvlR = 0;
    private double minSeenL = 999, minSeenR = 999;
    private final float[] hsvL = new float[3], hsvR = new float[3];

    public int ballColor(RevColorSensorV3 sensorL, RevColorSensorV3 sensorR) {
        distL = sensorL.getDistance(DistanceUnit.MM);
        distR = sensorR.getDistance(DistanceUnit.MM);
        colL = readColorAt(sensorL, distL, hsvL, true);
        colR = readColorAt(sensorR, distR, hsvR, false);
        if (distL < minSeenL) minSeenL = distL;
        if (distR < minSeenR) minSeenR = distR;
        return combineColors(colL, colR, distL, distR);
    }

    public void resetIntakeMin() { minSeenL = 999; minSeenR = 999; }

    public String intakeRaw() {
        return String.format(java.util.Locale.US, "L %s | R %s  near %.0f/%.0f  gate %.0fmm lvl %.0f",
                one(distL, hsvL, colL, lvlL), one(distR, hsvR, colR, lvlR),
                minSeenL, minSeenR, DISTANCE_INTAKE_MM, (double) MIN_LEVEL);
    }

    private String one(double d, float[] hsv, int c, int lvl) {
        if (d >= DISTANCE_INTAKE_MM) {
            return String.format(java.util.Locale.US, "%.0fmm FAR", d);
        }
        return String.format(java.util.Locale.US, "%.0fmm L%d h%.0f s%.2f %s",
                d, lvl, hsv[0], hsv[1], c == 0 ? "NO MATCH" : ("c" + c));
    }

    private int combineColors(int left, int right, double distL, double distR) {
        if (left == 0) return right;
        if (right == 0) return left;
        if (left == right) return left;
        return distL <= distR ? left : right;
    }

    private int readColorAt(RevColorSensorV3 sensor, double distMm, float[] hsv, boolean isLeft) {
        hsv[0] = -1; hsv[1] = 0; hsv[2] = 0;
        if (isLeft) lvlL = 0; else lvlR = 0;
        if (distMm >= DISTANCE_INTAKE_MM) return 0;
        int r = sensor.red(), g = sensor.green(), b = sensor.blue();
        int level = r + g + b;
        if (isLeft) lvlL = level; else lvlR = level;
        int max = Math.max(r, Math.max(g, b));
        if (max == 0) return 0;
        Color.RGBToHSV(r * 255 / max, g * 255 / max, b * 255 / max, hsv);
        int c = classify(hsv[0], hsv[1], hsv[2]);
        return (c != 0 && level >= MIN_LEVEL) ? c : 0;
    }

    public static int classify(float h, float s, float v) {
        if (h >= POLLEN_H_LO && h <= POLLEN_H_HI
                && s >= POLLEN_S_MIN && v >= POLLEN_V_MIN) return 1;
        if (h >= NECTAR_R_H_LO && h <= NECTAR_R_H_HI
                && s >= NECTAR_R_S_MIN && v >= NECTAR_R_V_MIN) return 2;
        if (h >= NECTAR_B_H_LO && h <= NECTAR_B_H_HI
                && s >= NECTAR_B_S_MIN && v >= NECTAR_B_V_MIN) return 3;
        return 0;
    }

    public static String colorName(int c) {
        switch (c) {
            case 1:  return "POLLEN";
            case 2:  return "NECTAR_RED";
            case 3:  return "NECTAR_BLUE";
            default: return "none";
        }
    }
}
