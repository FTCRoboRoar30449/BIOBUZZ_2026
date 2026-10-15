package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.VisionController;

//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.paths.PathChain;

@Autonomous(name = "AutoR2_S_F_S_P", group = "Autonomous")
public class AutoR2_S_F_S_P extends OpMode {

    RobotHardware robot;
    MechController mechController;
    VisionController visionController;
    private Follower follower;
    private ElapsedTime pathTimer;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose RED_START = poseFactory.of(60, 134, 270);
    private final Pose RED_START_TO_FLOWER = poseFactory.of(48, 130, 90);
    private final Pose RED_START_TO_FLOWER_Control1 = poseFactory.of(60, 120, 270);
    private final Pose RED_FLOWER_TO_SHOOT = poseFactory.of(60, 134, 270);
    private final Pose RED_FLOWER_TO_SHOOT_Control1 = poseFactory.of(48, 120, 270);
    private final Pose RED_SHOOT_TO_PARK = poseFactory.of(12, 115, 220);
    private final Pose RED_SHOOT_TO_PARK_Control1 = poseFactory.of(60, 100, 270);

    public enum PathState {
        SHOOTING_START,
        RED_START_TO_FLOWER,
        INTAKE,
        RED_FLOWER_TO_SHOOT,
        RED_SHOOT_TO_PARK,
        STOP
    }
    private PathState pathState = PathState.SHOOTING_START;

    // Single continuous curves returned by Paths.curve
    private Path red_start_to_flower;
    private Path red_flower_to_shoot;
    private Path red_shoot_to_park;
    public void buildPaths() {
        red_start_to_flower = Paths.curve(RED_START, RED_START_TO_FLOWER_Control1, RED_START_TO_FLOWER).linear(RED_START, RED_START_TO_FLOWER);
        red_flower_to_shoot = Paths.curve(RED_START_TO_FLOWER, RED_FLOWER_TO_SHOOT_Control1, RED_FLOWER_TO_SHOOT).linear(RED_START_TO_FLOWER, RED_FLOWER_TO_SHOOT);
        red_shoot_to_park = Paths.curve(RED_FLOWER_TO_SHOOT, RED_SHOOT_TO_PARK_Control1, RED_SHOOT_TO_PARK).linear(RED_FLOWER_TO_SHOOT, RED_SHOOT_TO_PARK);
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
                // Trigger shooting via SHOOTING_HIVE state
                mechController.handleMechState(MechState.SHOOTING_HIVE);
                break;

            case RED_START_TO_FLOWER:
                // Follow path to flower; hold position at endpoint
                mechController.handleMechState(MechState.IDLE);
                follower.follow(red_start_to_flower);
                break;

            case INTAKE:
                //  Run field intake for 3 seconds
                mechController.handleMechState(MechState.INTAKE_FIELD);
                break;

            case RED_FLOWER_TO_SHOOT:
                // Follow path to shoot; hold position at endpoint
                mechController.handleMechState(MechState.IDLE);
                follower.follow(red_flower_to_shoot);
                break;

            case RED_SHOOT_TO_PARK:
                // Follow path to park; hold position at endpoint
                mechController.handleMechState(MechState.SHOOTING_HIVE);
                follower.follow(red_shoot_to_park);
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
                    setPathState(PathState.RED_START_TO_FLOWER);
                }
                break;

            case RED_START_TO_FLOWER:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE);
                }
                break;

            case INTAKE:
                // Wait 3.0 seconds for intaking to complete
                if (pathTimer.seconds() >= 3.0) {
                    setPathState(PathState.RED_FLOWER_TO_SHOOT);
                }
                break;

            case RED_FLOWER_TO_SHOOT:
                // Wait 3.0 seconds for second shooting to complete
                if (pathTimer.seconds() >= 3.0) {
                    setPathState(PathState.RED_SHOOT_TO_PARK);
                }
                break;

            case RED_SHOOT_TO_PARK:
                if (!follower.isBusy()) {
                    setPathState(PathState.STOP);
                }
                break;

            case STOP:
                break;
        }
    }
}