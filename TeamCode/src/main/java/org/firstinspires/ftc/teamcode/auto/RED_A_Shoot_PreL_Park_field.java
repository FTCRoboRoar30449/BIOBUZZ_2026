package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

// Mirror of the BLUE audience run.
// Shoots from the measured best spot: 24.5 in, hood 75, turret +9.5.
@Autonomous(name = "RED A preload shoot park", group = "Red")
public class RED_A_Shoot_PreL_Park_field extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return Red.START; }

    @Override
    protected Step[] buildSteps() {
        return new Step[] {
            shootLeg(Red.START, Red.SHOOT_CLOSE_AUD),
            new Step(leg(Red.SHOOT_CLOSE_AUD, Red.PRE_AUD_PARK), MechState.IDLE),
        };
    }
}
