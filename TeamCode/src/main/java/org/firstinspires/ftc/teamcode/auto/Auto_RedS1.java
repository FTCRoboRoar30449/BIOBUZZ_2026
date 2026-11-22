package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "Auto RED S1 garden run", group = "Red")
public class Auto_RedS1 extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return Red.START; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Red.START, Red.SHOOT_NEAR),
            new Step(leg(Red.SHOOT_NEAR, Red.GARDEN_VIA), null),
            new Step(leg(Red.GARDEN_VIA, Red.GARDEN_APPROACH), null),
            new Step(leg(Red.GARDEN_APPROACH, Red.GARDEN), MechState.INTAKE_FIELD, null),
            shootCurve(Red.GARDEN, Red.C_GARDEN_SHOOT, Red.SHOOT_MID),
            new Step(leg(Red.SHOOT_MID, Red.PARK), MechState.IDLE),
        };
    }
}
