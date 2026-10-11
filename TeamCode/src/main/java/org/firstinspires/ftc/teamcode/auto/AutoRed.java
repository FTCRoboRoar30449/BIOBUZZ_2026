
package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.field.Red;

@Autonomous(name = "AutoRed", group = "Red")
public class AutoRed extends OpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose RED_START = poseFactory.of(56, 8, 90);
    private final Pose RED_START_TO_GARDEN = poseFactory.of(20, 8, 180);
    private final Pose RED_START_TO_GARDEN_Control1 = poseFactory.of(56, 20, 0);
    private final Pose RED_GARDEN_TO_PARK = poseFactory.of(12, 97, 99.3096);
    private final Pose RED_GARDEN_TO_PARK_Control1 = poseFactory.of(24, 24, 0);

    public Path red_start_to_garden() {
        return Paths.curve(RED_START, RED_START_TO_GARDEN_Control1, RED_START_TO_GARDEN).linear(RED_START, RED_START_TO_GARDEN);
    }

    public Path red_garden_to_park() {
        return Paths.curve(RED_START_TO_GARDEN, RED_GARDEN_TO_PARK_Control1, RED_GARDEN_TO_PARK).tangent();
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        //Red.teleopStart = new Pose(follower.getPose());
    }
}