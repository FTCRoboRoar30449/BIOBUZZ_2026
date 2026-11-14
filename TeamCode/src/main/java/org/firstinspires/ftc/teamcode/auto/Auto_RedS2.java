package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "Auto RED S2 full cycle", group = "Red")
public class Auto_RedS2 extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return Red.START; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Red.START, Red.SHOOT_NEAR),
            new Step(leg(Red.SHOOT_NEAR, Red.GARDEN), MechState.INTAKE_FIELD, null),
            sweep(),
            // shootTo, not shootLeg: the sweep ended at the last ball it
            // found, not at Red.GARDEN, so the start must be read live.
            shootTo(Red.SHOOT_NEAR),
            new Step(leg(Red.SHOOT_NEAR, Red.FLOWER_NEAR), MechState.INTAKE_FLOWER, null),
            shootLeg(Red.FLOWER_NEAR, Red.SHOOT_NEAR),
            new Step(curve(Red.SHOOT_NEAR, Red.C_SHOOT_PARK, Red.PARK), MechState.IDLE),
        };
    }
}