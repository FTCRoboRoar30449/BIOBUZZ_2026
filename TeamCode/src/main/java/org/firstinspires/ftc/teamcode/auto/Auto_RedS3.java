package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "Auto RED S3 full cycle", group = "Red")
public class Auto_RedS3 extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return Red.START; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Red.START, Red.SHOOT_NEAR),
            turnLeg(Red.SHOOT_NEAR, Red.GARDEN_VIA),
            new Step(leg(turnPoint(Red.SHOOT_NEAR, Red.GARDEN_VIA), Red.GARDEN_VIA), MechState.INTAKE_FIELD, null),
            new Step(leg(Red.GARDEN_VIA, Red.GARDEN), null, null),
            sweep(),
            driveTo(Red.GARDEN_VIA, null, null),
            turnTowards(Red.SHOOT_NEAR),
            shootTo(Red.SHOOT_NEAR),
            new Step(leg(Red.SHOOT_NEAR, Red.FLOWER_NEAR_TURN), MechState.INTAKE_FLOWER, null),
            new Step(leg(Red.FLOWER_NEAR_TURN, Red.FLOWER_NEAR), null, null),
            new Step(leg(Red.FLOWER_NEAR, Red.FLOWER_NEAR_TURN), null, null),
            shootLeg(Red.FLOWER_NEAR_TURN, Red.SHOOT_NEAR),
            turnLeg(Red.SHOOT_NEAR, Red.PARK),
            new Step(curve(turnPoint(Red.SHOOT_NEAR, Red.PARK), Red.C_SHOOT_PARK, Red.PARK), MechState.IDLE),
        };
    }
}
