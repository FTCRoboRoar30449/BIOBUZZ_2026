package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.follower.Follower;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.VisionController;

@Autonomous(name = "AutoR1_S_G_P", group = "Autonomous")
public class AutoR1_S_G_P extends OpMode {

    RobotHardware robot;
    MechController mechController;
    VisionController visionController;
    private Follower follower;
    private ElapsedTime pathTimer;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose RED_START = poseFactory.of(56, 8, 90);
    private final Pose RED_START_TO_GARDEN = poseFactory.of(20, 8, 180);
    private final Pose RED_START_TO_GARDEN_Control1 = poseFactory.of(56, 20, 0);
    private final Pose RED_GARDEN_TO_PARK = poseFactory.of(12, 97, 99.3096);
    private final Pose RED_GARDEN_TO_PARK_Control1 = poseFactory.of(24, 24, 0);

    public enum PathState {
        SHOOTING_START,
        RED_START_TO_GARDEN,
        INTAKE,
        RED_GARDEN_TO_PARK,
        STOP
    }
    private PathState pathState = PathState.SHOOTING_START;

    // Single continuous curves returned by Paths.curve
    private Path red_start_to_garden;
    private Path red_garden_to_park;
    public void buildPaths() {
        red_start_to_garden = Paths.curve(RED_START, RED_START_TO_GARDEN_Control1, RED_START_TO_GARDEN).linear(RED_START, RED_START_TO_GARDEN);
        red_garden_to_park = Paths.curve(RED_START_TO_GARDEN, RED_GARDEN_TO_PARK_Control1, RED_GARDEN_TO_PARK).tangent();
    }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();

        // (a) Hardware and subsystem initializations
        robot = new RobotHardware(hardwareMap, telemetry);

        visionController = new VisionController(robot);
        visionController.initAprilTag();

        mechController = new MechController(robot, visionController);
        mechController.handleMechState(MechState.START);

        // Pedro Pathing follower instantiation via Constants.follower
        follower = Constants.create(hardwareMap);
        buildPaths();
        follower.setPose(RED_START);
    }

    @Override
    public void start() {
        pathTimer.reset();
        setPathState(PathState.SHOOTING_START);
    }

    @Override
    public void loop() {
        follower.update();
        mechController.update();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        //telemetry.addData("Mech State", mechController.getState());
        telemetry.addData("Timer", "%.2f s", pathTimer.seconds());
        //follower.telemetryDebug(telemetry);
        telemetry.update();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset();

        switch (pathState) {
            case SHOOTING_START:
                // (b) Trigger shooting via SHOOTING_HIVE state
                mechController.handleMechState(MechState.SHOOTING_HIVE);
                break;

            case RED_START_TO_GARDEN:
                // (c) Follow path to garden; hold position at endpoint
                mechController.handleMechState(MechState.IDLE);
                follower.follow(red_start_to_garden);
                break;

            case INTAKE:
                // (d) Run field intake for 3 seconds
                mechController.handleMechState(MechState.INTAKE_FIELD);
                break;

            case RED_GARDEN_TO_PARK:
                // (e) Follow path to park; hold position at endpoint
                mechController.handleMechState(MechState.IDLE);
                follower.follow(red_garden_to_park);
                break;

            case STOP:
                mechController.handleMechState(MechState.IDLE);
                break;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case SHOOTING_START:
                // Wait 3.0 seconds for initial shooting to complete
                if (pathTimer.seconds() >= 3.0) {
                    setPathState(PathState.RED_START_TO_GARDEN);
                }
                break;

            case RED_START_TO_GARDEN:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE);
                }
                break;

            case INTAKE:
                // Wait 3.0 seconds for intaking to complete
                if (pathTimer.seconds() >= 3.0) {
                    setPathState(PathState.RED_GARDEN_TO_PARK);
                }
                break;

            case RED_GARDEN_TO_PARK:
                if (!follower.isBusy()) {
                    setPathState(PathState.STOP);
                }
                break;

            case STOP:
                break;
        }
    }
}