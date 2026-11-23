package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "Auto RED garden park", group = "Red")
public class Auto_R1_S_G_P extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return Red.START; }

    @Override
    protected Step[] buildSteps() {
        Path toTurn   = leg(Red.SHOOT_CLOSE_AUD, Red.GARDEN_TURN);
        Path toVia    = leg(Red.GARDEN_TURN, Red.GARDEN_VIA);
        Path toGarden = leg(Red.GARDEN_VIA, Red.GARDEN);
        Path toPark = curveTangent(Red.GARDEN, Red.C_GARDEN_PARK, Red.PARK);
        return new Step[] {
            shootLeg(Red.START, Red.SHOOT_CLOSE_AUD),
            new Step(toTurn, MechState.INTAKE_FIELD, null),
            new Step(toVia, null, null),
            new Step(toGarden, null, null),
            new Step(toPark, MechState.IDLE),
        };
    }
}
