package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.curves.bezier.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.Red;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.VisionController;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous (name = "AutoBlueBasic", group = "Blue")
public class Auto_Blue_SpEp extends OpMode {
    RobotHardware robot;
    MechController mechController;
    VisionController visionController;
    private VisionPortal visionPortal;
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;
    public enum PathState {
        //sp = starting position
        //sh = shoot position
        //ep = end position
        DRIVE_SP_SH,
        DRIVE_SH_EP,
        IDLE,
    }

    private PathState pathState;

    private final Pose startPose = new Pose(88,12,Math.toRadians(90));
    private final Pose shootHive = new Pose(83.68,103.34,Math.toRadians(90));
    private final Pose endPoint = new Pose(132,97, Math.toRadians(80.7));


    private Path driveSpSh, driveShEp;

    public void buildPaths() {
        driveSpSh = new AtomicPath(new Line(startPose, shootHive))
                .linear(startPose.heading(), shootHive.heading());
        driveShEp = new AtomicPath(new Line(shootHive, endPoint))
                .linear(shootHive.heading(), endPoint.heading());
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_SP_SH:
                follower.follow(driveSpSh);
                mechController.handleMechState(MechState.SHOOTING_HIVE);
                setPathState(PathState.DRIVE_SH_EP);
                break;
            case DRIVE_SH_EP:
                if (!follower.isBusy()) {
                    follower.follow(driveShEp);
                    setPathState(PathState.IDLE);
                    break;
                }
            case IDLE:
                if (!follower.isBusy()) {
                    telemetry.addLine("DONE!");
                    setPathState(PathState.IDLE);
                    break;
                }
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_SP_SH;
        pathTimer = new ElapsedTime();
        opModeTimer = new ElapsedTime();
        robot = new RobotHardware(hardwareMap, telemetry);
        follower = Constants.create(hardwareMap);
        //visionController = new VisionController(robot);
        visionController.initAprilTag();
        visionPortal = visionController.getVisionPortal();
        //mechController = new MechController(robot, visionController);
        mechController.handleMechState(MechState.START);

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        mechController.update();
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData("Path time", pathTimer.seconds());
    }

    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(pathState);
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.reset();
    }
}