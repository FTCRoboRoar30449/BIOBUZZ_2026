package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;

public final class BallSeek {

    private BallSeek() {}

    public static double APPROACH_OFFSET_IN = 0.0;

    public static double MIN_TRAVEL_IN = 3.0;

    public static Pose approach(Pose here, BallDetector.Detection d) {
        double h = here.heading();
        double back = RobotHardware.INTAKE_BEHIND_IN + APPROACH_OFFSET_IN;
        return new Pose(d.fieldX + back * Math.cos(h),
                        d.fieldY + back * Math.sin(h), h);
    }

    public static Pose targetFor(Pose here, BallDetector.Detection d) {
        if (d == null || Double.isNaN(d.fieldX) || Double.isNaN(d.fieldY)) return null;
        Pose to = approach(here, d);
        if (Math.hypot(to.x() - here.x(), to.y() - here.y()) < MIN_TRAVEL_IN) return null;
        return to;
    }
}
