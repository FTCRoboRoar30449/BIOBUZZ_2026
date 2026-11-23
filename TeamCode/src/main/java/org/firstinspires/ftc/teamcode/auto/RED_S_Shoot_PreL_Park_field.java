package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "RED S preload shoot park", group = "Red")
public class RED_S_Shoot_PreL_Park_field extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return Red.START_FAR; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Red.START_FAR, Red.SHOOT_CLOSE_SCO),
            new Step(leg(Red.SHOOT_CLOSE_SCO, Red.PRE_SCO_MID), MechState.IDLE),
            new Step(leg(Red.PRE_SCO_MID, Red.PRE_SCO_PARK), MechState.IDLE),
        };
    }
}
