package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "Auto RED far flower", group = "Red")
public class Auto_R2_S_F_S_P extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return Red.START_FAR; }

    @Override
    protected Step[] buildSteps() {
        Path toFlower = curve(Red.START_FAR, Red.C_START_FLOWER, Red.FLOWER_FAR);
        Path toPark = curveTangent(Red.SHOOT_FAR, Red.C_SHOOT_PARK, Red.PARK_FAR);
        return new Step[] {
            new Step(toFlower, MechState.INTAKE_FLOWER, null),
            shootCurve(Red.FLOWER_FAR, Red.C_FLOWER_SHOOT, Red.SHOOT_FAR),
            new Step(toPark, MechState.IDLE),
        };
    }
}
