package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.field.Blue;

@Autonomous(name = "Auto Blue", group = "Blue")
public class AutoBlue extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

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
        //Blue.teleopStart = new Pose(follower.getPose());
    }
}