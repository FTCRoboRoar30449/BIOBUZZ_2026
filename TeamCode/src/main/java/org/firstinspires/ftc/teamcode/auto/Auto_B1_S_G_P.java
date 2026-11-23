package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "Auto BLUE garden park", group = "Blue")
public class Auto_B1_S_G_P extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.BLUE; }
    @Override protected Pose startPose() { return Blue.START; }

    @Override
    protected Step[] buildSteps() {
        Path toTurn   = leg(turnPoint(Blue.SHOOT_CLOSE_AUD, Blue.GARDEN_TURN),
                            Blue.GARDEN_TURN);
        Path toVia    = leg(Blue.GARDEN_TURN, Blue.GARDEN_VIA);
        Path toGarden = leg(Blue.GARDEN_VIA, Blue.GARDEN);
        Path toPark = curveTangent(Blue.GARDEN, Blue.C_GARDEN_PARK, Blue.PARK);
        return new Step[] {
            shootLeg(Blue.START, Blue.SHOOT_CLOSE_AUD),
            turnLeg(Blue.SHOOT_CLOSE_AUD, Blue.GARDEN_TURN),
            new Step(toTurn, MechState.INTAKE_FIELD, null),
            new Step(toVia, null, null),
            new Step(toGarden, null, null),
            new Step(toPark, MechState.IDLE),
        };
    }
}
