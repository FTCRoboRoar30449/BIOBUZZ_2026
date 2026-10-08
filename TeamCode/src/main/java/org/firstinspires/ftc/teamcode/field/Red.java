package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

public class Red {
    // Auto
    // Start & Park Pose of our robot.
    public static final Pose AUTO_START_POSE = new Pose(135, 60, Math.toRadians(90));
    public static final Pose AUTO_PARK_POSE = new Pose(0, 0, Math.toRadians(0));

    // Teleop
    // teleopStart will be updated when Auto stops
    public static Pose teleopStart = new Pose(0, 0,Math.toRadians(0));
    public static final Pose TELEOP_PARK_POSE = new Pose(0, 0, Math.toRadians(0));

    // Prevent instantiation
    private Red() {}
}