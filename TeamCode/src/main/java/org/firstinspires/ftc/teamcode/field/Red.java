package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

public class Red {
    // Auto
    // Start & Park Pose of our robot.
    public static final Pose AUTO_START_POSE = new Pose(56, 8, Math.toRadians(90));
    public static final Pose AUTO_PARK_POSE = new Pose(12, 97, Math.toRadians(99));

    public static final Pose AUTO_GARDEN = new Pose(20, 8, Math.toRadians(180));

    public static final Pose AUTO_NEAR_FLOWER = new Pose(12, 48, Math.toRadians(180));

    public static final Pose AUTO_FAR_FLOWER = new Pose(48, 132, Math.toRadians(90));

    // Teleop
    // teleopStart will be updated when Auto stops
    public static Pose teleopStart = new Pose(0, 0,Math.toRadians(0));
    public static final Pose TELEOP_PARK_POSE = new Pose(0, 0, Math.toRadians(0));

    // Prevent instantiation
    private Red() {}
}