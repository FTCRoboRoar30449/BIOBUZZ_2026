package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.Blue;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;

// Garden run, drawn in the Pedro visualizer 13 Nov. Mirror of Auto__Red.
//
// Collects and parks; it does NOT shoot. Add shootTo(Blue.SHOOT_NEAR) before
// the park leg if the balls should be scored rather than carried.
@Autonomous(name = "Auto BLUE garden sweep park", group = "Blue")
public class Auto_Blue extends Auto_Base {

    @Override protected ShooterAdapter.Alliance alliance() { return ShooterAdapter.Alliance.BLUE; }
    @Override protected Pose startPose() { return Blue.START; }

    @Override
    protected Step[] buildSteps() {
        Path toGarden = curve(Blue.START, Blue.C_START_GARDEN, Blue.GARDEN_SWEEP);
        return new Step[] {
            new Step(toGarden, MechState.INTAKE_FIELD, null),
            // Drive to balls that are actually there, not to where balls are
            // usually found. Ends at the last one it reached.
            sweep(),
            // driveTo, not a baked curve: the sweep ended wherever the last
            // ball was, so the start has to be read at launch.
            driveTo(Blue.PARK, null, MechState.IDLE),
        };
    }
}
