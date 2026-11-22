package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

// Garden run, drawn in the Pedro visualizer 13 Nov.
@Autonomous(name = "Auto BLUE garden sweep park", group = "Blue")
public class Auto_Blue extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.BLUE; }
    @Override protected Pose startPose() { return Blue.START; }

    @Override
    protected Step[] buildSteps() {
        Path toTurn   = leg(Blue.START, Blue.GARDEN_TURN);
        Path toVia    = leg(Blue.GARDEN_TURN, Blue.GARDEN_VIA);
        Path toGarden = leg(Blue.GARDEN_VIA, Blue.GARDEN_SWEEP);
        return new Step[] {
            new Step(toTurn, MechState.INTAKE_FIELD, null),
            new Step(toVia, null, null),
            new Step(toGarden, null, null),
            // Drive to balls that are actually there, not to where balls
            sweep(),
            // driveTo, not a baked curve: the sweep ended wherever the last
            driveTo(Blue.PARK, null, MechState.IDLE),
        };
    }
}
