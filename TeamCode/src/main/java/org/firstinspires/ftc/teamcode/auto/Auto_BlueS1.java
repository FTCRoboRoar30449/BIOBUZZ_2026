package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "Auto BLUE S1 garden run", group = "Blue")
public class Auto_BlueS1 extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.BLUE; }
    @Override protected Pose startPose() { return Blue.START; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Blue.START, Blue.SHOOT_NEAR),
            new Step(leg(Blue.SHOOT_NEAR, Blue.GARDEN_VIA), null),
            new Step(leg(Blue.GARDEN_VIA, Blue.GARDEN_APPROACH), null),
            new Step(leg(Blue.GARDEN_APPROACH, Blue.GARDEN), MechState.INTAKE_FIELD, null),
            shootCurve(Blue.GARDEN, Blue.C_GARDEN_SHOOT, Blue.SHOOT_MID),
            new Step(leg(Blue.SHOOT_MID, Blue.PARK), MechState.IDLE),
        };
    }
}
