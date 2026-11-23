package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "BLUE S preload shoot park", group = "Blue")
public class BLUE_S_Shoot_PreL_Park_field extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.BLUE; }
    @Override protected Pose startPose() { return Blue.START_FAR; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Blue.START_FAR, Blue.SHOOT_CLOSE_SCO),
            new Step(leg(Blue.SHOOT_CLOSE_SCO, Blue.PRE_SCO_MID), MechState.IDLE),
            new Step(leg(Blue.PRE_SCO_MID, Blue.PRE_SCO_PARK), MechState.IDLE),
        };
    }
}
