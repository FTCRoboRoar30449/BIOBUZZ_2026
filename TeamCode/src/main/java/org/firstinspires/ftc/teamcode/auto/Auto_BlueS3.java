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
            new Step(leg(Blue.SHOOT_NEAR, Blue.GARDEN), MechState.INTAKE_FIELD, null),
            sweep(),
            // shootTo, not shootLeg: the sweep ended at the last ball
            shootTo(Blue.SHOOT_NEAR),
            new Step(leg(Blue.SHOOT_NEAR, Blue.FLOWER_NEAR), MechState.INTAKE_FLOWER, null),
            shootLeg(Blue.FLOWER_NEAR, Blue.SHOOT_NEAR),
            new Step(curve(Blue.SHOOT_NEAR, Blue.C_SHOOT_PARK, Blue.PARK), MechState.IDLE),
        };
    }
}
