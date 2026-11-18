package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

// Garden run, drawn in the Pedro visualizer 13 Nov.
@Autonomous(name = "Auto RED garden sweep park", group = "Red")
public class Auto_Red extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.RED; }
    @Override protected Pose startPose() { return Red.START; }

    @Override
    protected Step[] buildSteps() {
        Path toGarden = curve(Red.START, Red.C_START_GARDEN, Red.GARDEN_SWEEP);
        return new Step[] {
            new Step(toGarden, MechState.INTAKE_FIELD, null),
            // Drive to balls that are actually there, not to where balls
            sweep(),
            // driveTo, not a baked curve: the sweep ended wherever the last
            driveTo(Red.PARK, null, MechState.IDLE),
        };
    }
}
