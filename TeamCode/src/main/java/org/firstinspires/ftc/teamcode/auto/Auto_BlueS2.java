package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "Auto BLUE S2 far cycle", group = "Blue")
public class Auto_BlueS2 extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.BLUE; }
    @Override protected Pose startPose() { return Blue.START_FAR; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Blue.START_FAR, Blue.SHOOT_MID),
            turnLeg(Blue.SHOOT_MID, Blue.GARDEN_VIA_FAR),
            new Step(leg(turnPoint(Blue.SHOOT_MID, Blue.GARDEN_VIA_FAR), Blue.GARDEN_VIA_FAR), MechState.INTAKE_FIELD, null),
            new Step(leg(Blue.GARDEN_VIA_FAR, Blue.GARDEN_FAR), null, null),
            sweep(),
            driveTo(Blue.GARDEN_VIA_FAR, null, null),
            turnTowards(Blue.SHOOT_MID),
            shootTo(Blue.SHOOT_MID),
            new Step(leg(Blue.SHOOT_MID, Blue.FLOWER_FAR), MechState.INTAKE_FLOWER, null),
            shootLeg(Blue.FLOWER_FAR, Blue.SHOOT_MID),
            new Step(curve(Blue.SHOOT_MID, Blue.C_SHOOT_SOUTH, Blue.PARK_SOUTH), MechState.IDLE),
        };
    }
}
