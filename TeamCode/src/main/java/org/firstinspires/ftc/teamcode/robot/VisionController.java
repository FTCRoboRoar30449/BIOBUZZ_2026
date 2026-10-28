package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.limelightvision.LLResult;

public class VisionController {

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private int distanceBall = 20; // placement color sensor

    // Constructor
    public VisionController(RobotHardware RoboRoar) {
        this.robot = RoboRoar;
        this.telemetry = RoboRoar.telemetry;
    }

    // Vision setup
    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setCameraPose(robot.cameraPosition, robot.cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(robot.camera)
                .addProcessor(aprilTag)
                .build();
    }

    public AprilTagProcessor getAprilTag() {
        return aprilTag;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // Start Limelight
    public void cellCount() {
        robot.limeLight.pipelineSwitch(4);
        robot.limeLight.start();
    }

    public int getFlowerPollenCount() {
        LLResult result = robot.limeLight.getLatestResult();

        if (result != null && result.isValid()) {
            double[] output = result.getPythonOutput();

            if (output != null && output.length > 0) {
                return (int) output[0];
            }
        }

        return -1;
    }
    // Stop Limelight
    public void stopLimelight() {
        robot.limeLight.stop();
    }

    public boolean ballCheck(RevColorSensorV3 sensorL, RevColorSensorV3 sensorR) {
        return sensorL.getDistance(DistanceUnit.MM) < distanceBall || sensorR.getDistance(DistanceUnit.MM) < distanceBall;
    }
    public int ballColor(RevColorSensorV3 sensorL, RevColorSensorV3 sensorR) {
        return isFinalColor(
                isColor(sensorL),
                isColor(sensorR)
        );
    }

    private int isFinalColor(int ls, int rs) {
        int finalColor = 0;

        if (ls == 1 || rs == 1) {
            finalColor = 1; // Pollen
        } else if (ls == 2 || rs == 2) {
            finalColor = 2; // Red
        } else if (ls == 3 || rs == 3) {
            finalColor = 3; // Blue
        }

        return finalColor;
    }

    private int isColor(RevColorSensorV3 colorSensor) {

        // Get distance in millimeters
        double distance = colorSensor.getDistance(DistanceUnit.MM);

        if (distance < distanceBall) {

            float[] hsvValues = new float[3];

            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();

            int maxRGB = Math.max(r, Math.max(g, b));

            if (maxRGB == 0) {
                return 0;
            }

            int red = r * 255 / maxRGB;
            int green = g * 255 / maxRGB;
            int blue = b * 255 / maxRGB;
            Color.RGBToHSV(red, green, blue, hsvValues);

            float hue = hsvValues[0];
            float sat = hsvValues[1];
            float val = hsvValues[2];

            // Color classification
            if (isYellow(hue, sat, val)) {
                return 1; // Yellow
            } else if  (isRed(hue, sat, val)) {
                return 2; // Red
            } else if (isBlue(hue, sat, val)) {
                return 3; // Blue
            }
        }

        // Nothing detected / unknown color
        return 0;
    }

    private boolean isBlue(float hue, float sat, float val) {
        return hue >= 215 && hue <= 240
                && sat >= 0.60
                && val >= 0.50;
    }

    private boolean isRed(float hue, float sat, float val) {
        return hue >= 15 && hue <= 35
                && sat >= 0.45
                && val >= 0.50;
    }

    private boolean isYellow(float hue, float sat, float val) {
        return hue >= 80 && hue <= 110
                && sat >= 0.50
                && val >= 0.50;
    }
}
