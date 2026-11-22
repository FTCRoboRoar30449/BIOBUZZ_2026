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
        Path toTurn   = leg(Red.START, Red.GARDEN_TURN);
        Path toVia    = leg(Red.GARDEN_TURN, Red.GARDEN_VIA);
        Path toGarden = leg(Red.GARDEN_VIA, Red.GARDEN_SWEEP);
        return new Step[] {
            new Step(toTurn, MechState.INTAKE_FIELD, null),
            new Step(toVia, null, null),
            new Step(toGarden, null, null),
            // Drive to balls that are actually there, not to where balls
            sweep(),
            // driveTo, not a baked curve: the sweep ended wherever the last
            driveTo(Red.PARK, null, MechState.IDLE),
        };
    }
}
