package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

@Autonomous(name = "Auto BLUE far flower", group = "Blue")
public class Auto_B2_S_F_S_P extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.BLUE; }
    @Override protected Pose startPose() { return Blue.START_FAR; }

    @Override
    protected Step[] buildSteps() {
        Path toFlower = curve(Blue.START_FAR, Blue.C_START_FLOWER, Blue.FLOWER_FAR);
        Path toPark = curveTangent(Blue.SHOOT_FAR, Blue.C_SHOOT_PARK, Blue.PARK_FAR);
        return new Step[] {
            new Step(toFlower, MechState.INTAKE_FLOWER, null),
            shootCurve(Blue.FLOWER_FAR, Blue.C_FLOWER_SHOOT, Blue.SHOOT_FAR),
            new Step(toPark, MechState.IDLE),
        };
    }
}
