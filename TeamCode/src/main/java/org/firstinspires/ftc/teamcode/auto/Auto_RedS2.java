package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

// RED far-side cycle: start far, shoot, sweep the far garden, shoot, flower
@Autonomous(name = "Auto RED S2 far cycle", group = "Red")
public class Auto_RedS2 extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return Red.START_FAR; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Red.START_FAR, Red.SHOOT_MID),
            new Step(leg(Red.SHOOT_MID, Red.GARDEN_VIA_FAR), MechState.INTAKE_FIELD, null),
            new Step(leg(Red.GARDEN_VIA_FAR, Red.GARDEN_FAR), null, null),
            sweep(),
            // The sweep ended at the last ball, not at GARDEN_FAR.
            driveTo(Red.GARDEN_VIA_FAR, null, null),
            shootTo(Red.SHOOT_MID),
            new Step(leg(Red.SHOOT_MID, Red.FLOWER_FAR), MechState.INTAKE_FLOWER, null),
            shootLeg(Red.FLOWER_FAR, Red.SHOOT_MID),
            new Step(curve(Red.SHOOT_MID, Red.C_SHOOT_SOUTH, Red.PARK_SOUTH), MechState.IDLE),
        };
    }
}
