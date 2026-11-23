package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "BLUE A preload shoot park", group = "Blue")
public class BLUE_A_Shoot_PreL_Park_field extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.BLUE; }
    @Override protected Pose startPose() { return Blue.START; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Blue.START, Blue.SHOOT_CLOSE_AUD),
            new Step(leg(Blue.SHOOT_CLOSE_AUD, Blue.PRE_AUD_PARK), MechState.IDLE),
        };
    }
}
