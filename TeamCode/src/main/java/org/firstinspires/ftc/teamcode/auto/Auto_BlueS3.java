package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "Auto BLUE S3 full cycle", group = "Blue")
public class Auto_BlueS3 extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.BLUE; }
    @Override protected Pose startPose() { return Blue.START; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Blue.START, Blue.SHOOT_NEAR),
            turnLeg(Blue.SHOOT_NEAR, Blue.GARDEN_VIA),
            new Step(leg(turnPoint(Blue.SHOOT_NEAR, Blue.GARDEN_VIA), Blue.GARDEN_VIA), MechState.INTAKE_FIELD, null),
            new Step(leg(Blue.GARDEN_VIA, Blue.GARDEN), null, null),
            sweep(),
            driveTo(Blue.GARDEN_VIA, null, null),
            turnTowards(Blue.SHOOT_NEAR),
            shootTo(Blue.SHOOT_NEAR),
            new Step(leg(Blue.SHOOT_NEAR, Blue.FLOWER_NEAR_TURN), MechState.INTAKE_FLOWER, null),
            new Step(leg(Blue.FLOWER_NEAR_TURN, Blue.FLOWER_NEAR), null, null),
            new Step(leg(Blue.FLOWER_NEAR, Blue.FLOWER_NEAR_TURN), null, null),
            shootLeg(Blue.FLOWER_NEAR_TURN, Blue.SHOOT_NEAR),
            turnLeg(Blue.SHOOT_NEAR, Blue.PARK),
            new Step(curve(turnPoint(Blue.SHOOT_NEAR, Blue.PARK), Blue.C_SHOOT_PARK, Blue.PARK), MechState.IDLE),
        };
    }
}
