package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionController {

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

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

}
