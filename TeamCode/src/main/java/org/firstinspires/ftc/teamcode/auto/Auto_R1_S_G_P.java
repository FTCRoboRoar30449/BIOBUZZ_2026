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
        Path toGarden = curve(Red.START, Red.C_START_GARDEN, Red.GARDEN);
        Path toPark = curveTangent(Red.GARDEN, Red.C_GARDEN_PARK, Red.PARK);
        return new Step[] {
            new Step(toGarden, MechState.INTAKE_FIELD, null),
            new Step(toPark, MechState.IDLE),
        };
    }
}
