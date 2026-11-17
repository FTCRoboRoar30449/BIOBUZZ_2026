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

    // Presence gate. A ball is "there" when either sensor reads closer than
    // this. UNMEASURED -- 20 mm is the shipped guess, and it is the number
    // RobotSetup step 13 exists to check: read the distance with a ball held
    // against the sensor, then with the throat empty, and put this between
    // them. Too large and an empty throat reads as full, which stalls the
    // conveyor with nothing in it; too small and balls pass uncounted.
    public static double DISTANCE_BALL_MM = 20.0;

    public static int REQUIRED_STABLE_FRAMES = 4;
    public static int LOST_FRAMES_TO_CLEAR = 10;

    // Colour gates, hue in DEGREES 0-360 (not OpenCV's 0-179 -- these come
    // from android.graphics.Color.RGBToHSV, which uses the full circle; the
    // BallDetector camera gates are the halved kind, do not copy between).
    //
    // ALL UNMEASURED. These are the shipped guesses and every one of them is
    // what step 13 is for. The hue is taken AFTER normalising rgb by its own
    // max, so it is insensitive to how bright the throat is -- but saturation
    // is not, and S is the gate most likely to be wrong under field light.
    //
    // Red nectar sits at 15-35, which is ORANGE on the colour circle. That is
    // deliberate: a red ball under a white LED rarely reads below 15, and a
    // gate that starts at 0 would also catch the dark plastic of the throat.
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

    // This logic is ORIGINAL and is VALIDATED. On 14 Nov it was run unchanged
    // on a spare Control Hub and a spare sensor and detected pollen and
    // nectar correctly, with the distance reading swinging as a ball passed.
    //
    // A brightness-based presence path was added here earlier that day, when
    // every sensor on the competition robot reported a fixed distance. That
    // turned out to be a WIRING fault on the robot, not a sensor, driver or
    // SDK problem -- so the workaround addressed something that did not
    // exist, and it is removed rather than left switched off. Do not
    // reintroduce it: if presence stops working, the reading has stopped
    // varying, and no threshold placed on a constant can help.
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
        return classify(h, s, v);
    }

    // Shared so RobotSetup step 13 can show the SAME verdict the robot will
    // reach, instead of raw numbers the driver has to gate in their head.
    // 0 none, 1 pollen, 2 red nectar, 3 blue nectar.
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
