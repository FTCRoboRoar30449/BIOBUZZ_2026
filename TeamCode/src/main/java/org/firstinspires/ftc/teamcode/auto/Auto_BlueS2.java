package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

// BLUE far-side cycle: start far, shoot, sweep the far garden, shoot, flower
@Autonomous(name = "Auto BLUE S2 far cycle", group = "Blue")
public class Auto_BlueS2 extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.BLUE; }
    @Override protected Pose startPose() { return Blue.START_FAR; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Blue.START_FAR, Blue.SHOOT_MID),
            new Step(leg(Blue.SHOOT_MID, Blue.GARDEN_FAR), MechState.INTAKE_FIELD, null),
            sweep(),
            // The sweep ended at the last ball, not at GARDEN_FAR.
            shootTo(Blue.SHOOT_MID),
            new Step(leg(Blue.SHOOT_MID, Blue.FLOWER_FAR), MechState.INTAKE_FLOWER, null),
            shootLeg(Blue.FLOWER_FAR, Blue.SHOOT_MID),
            new Step(curve(Blue.SHOOT_MID, Blue.C_SHOOT_SOUTH, Blue.PARK_SOUTH), MechState.IDLE),
        };
    }
}
