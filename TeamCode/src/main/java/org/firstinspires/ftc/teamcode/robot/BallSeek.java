package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;

// Where to drive so the INTAKE lands on a detected ball.
//
// This existed twice -- once in Auto_Base.sweep() and once in the TeleOp RB
// handler -- and the two had already diverged: autonomous applied the intake
// offset and TeleOp did not, so the same button drove the intake a foot past
// the ball. One copy now, used by both.
public final class BallSeek {

    private BallSeek() {}

    // Trim on top of the measured intake offset. 0 puts the intake wheel on
    // the ball; NEGATIVE drives further so the roller sweeps over it, which
    // is probably what you want. Positive stops short.
    public static double APPROACH_OFFSET_IN = 0.0;

    // Shorter than this and there is nothing worth driving. Re-targeting a
    // ball the robot is already on produces a zero-length path: the follower
    // reports not-busy immediately, so the caller re-targets every loop and
    // never progresses. A just-collected ball is still in view for a moment,
    // so this case is normal, not exceptional.
    public static double MIN_TRAVEL_IN = 3.0;

    // Pose to drive the robot CENTRE to. The intake sits behind centre, so
    //     intake = centre - BEHIND * forward   =>   centre = ball + BEHIND * forward
    // Offset along the HEADING, not the travel line: the leg holds heading,
    // so a ball off to one side is reached by strafing while the intake stays
    // on the robot's forward axis.
    public static Pose approach(Pose here, BallDetector.Detection d) {
        double h = here.heading();
        double back = RobotHardware.INTAKE_BEHIND_IN + APPROACH_OFFSET_IN;
        return new Pose(d.fieldX + back * Math.cos(h),
                        d.fieldY + back * Math.sin(h), h);
    }

    // Null when there is nothing worth driving to: no detection, a detection
    // made without a pose (NaN field coords), or a target the robot is
    // already standing on.
    public static Pose targetFor(Pose here, BallDetector.Detection d) {
        if (d == null || Double.isNaN(d.fieldX) || Double.isNaN(d.fieldY)) return null;
        Pose to = approach(here, d);
        if (Math.hypot(to.x() - here.x(), to.y() - here.y()) < MIN_TRAVEL_IN) return null;
        return to;
    }
}
