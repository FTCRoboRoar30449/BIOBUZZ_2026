package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionController {

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private int distanceNectar = 10; // placement color sensor
    private int distancePollen = distanceNectar + 10;

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

    public int ballColor() {
        return isFinalColor(
                isColor(robot.sensorIntakeU),
                isColor(robot.sensorIntakeD)
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

        // Pollen
        if (distance > distanceNectar && distance < distancePollen) {
            return 1;
        }

        // Nectar
        else if (distance < distanceNectar) {

            float[] hsvValues = new float[3];

            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();

            // Scale RGB to 0–255 and convert to HSV
            Color.RGBToHSV(
                    r * 255 / 800,
                    g * 255 / 800,
                    b * 255 / 800,
                    hsvValues
            );

            float hue = hsvValues[0];
            float sat = hsvValues[1];
            float val = hsvValues[2];

            // Color classification
            if (isRed(hue, sat, val)) {
                return 2; // Red
            } else if (isBlue(hue, sat, val)) {
                return 3; // Blue
            }
        }

        // Nothing detected / unknown color
        return 0;
    }

    private boolean isRed(float hue, float sat, float val) {
        return (hue <= 40 || hue >= 320)
                && sat > 0.25
                && val > 0.15;
    }

    private boolean isBlue(float hue, float sat, float val) {
        return hue >= 190
                && hue <= 270
                && sat > 0.25
                && val > 0.15;
    }
}
