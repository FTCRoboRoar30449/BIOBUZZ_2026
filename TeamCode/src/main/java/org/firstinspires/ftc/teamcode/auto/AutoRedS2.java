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

@Autonomous (name = "AutoRedS2", group = "Red")
public class AutoRedS2 extends OpMode {
    RobotHardware robot;
    MechController mechController;
    VisionController visionController;
    private VisionPortal visionPortal;
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;
    public enum PathState {
        //sp = starting position
        //sh = shoot position
        //in = intake
        //fl = flower
        //ep = end position
        DRIVE_SP_SH,
        DRIVE_SH_IN,
        DRIVE_IN_SH,
        DRIVE_SH_FL,
        DRIVE_FL_SH,
        DRIVE_SH_EP,
        IDLE,
    }

    private PathState pathState;

    private final Pose startPose = new Pose(56.34766584766585,8,Math.toRadians(90));
    private final Pose shootHive = new Pose(56.34766584766585,36,Math.toRadians(90));
    private final Pose intakeGarden = new Pose(8.434889434889442,11.760442260442261,Math.toRadians(90));
    private final Pose intakeFlower = new Pose(9.291154791154794,44.95331695331695,Math.toRadians(0));
    private final Pose endPoint = new Pose(10.28009828009828,100.55896805896806, Math.toRadians(0));
    private final Pose controlPoint = new Pose(12.85012285012285,37.15479115479116);

    private Path driveSpSh, driveShIn, driveInSh, driveShFl, driveFlSh, driveShEp;

    public void buildPaths() {
        driveSpSh = new AtomicPath(new Line(startPose, shootHive))
                .linear(startPose.heading(), shootHive.heading());
        driveShIn = new AtomicPath(new Line(shootHive, intakeGarden))
                .linear(shootHive.heading(), intakeGarden.heading());
        driveInSh = new AtomicPath(new Line(intakeGarden, shootHive ))
                .linear(intakeGarden.heading(),shootHive.heading());
        driveShFl = new AtomicPath(new Line(shootHive, intakeFlower))
                .linear(shootHive.heading(), intakeFlower.heading());
        driveFlSh = new AtomicPath(new Line(intakeFlower, shootHive))
                .linear(intakeFlower.heading(),shootHive.heading());
        driveShEp = new AtomicPath(new BezierCurve(shootHive, controlPoint, endPoint))
                .linear(shootHive.heading(), endPoint.heading());
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_SP_SH:
                follower.follow(driveSpSh);
                mechController.handleMechState(MechState.SHOOTING_HIVE);
                setPathState(PathState.DRIVE_SH_IN);
                break;
            case DRIVE_SH_IN:
                if (!follower.isBusy()) {
                    follower.follow(driveShIn);
                    mechController.handleMechState(MechState.INTAKE_FIELD);
                    setPathState(PathState.DRIVE_IN_SH);
                    break;
                }
            case DRIVE_IN_SH:
                if (!follower.isBusy()) {
                    follower.follow(driveInSh);
                    mechController.handleMechState(MechState.SHOOTING_HIVE);
                    setPathState(PathState.DRIVE_SH_FL);
                    break;
                }
            case DRIVE_SH_FL:
                if (!follower.isBusy()) {
                    follower.follow(driveShFl);
                    mechController.handleMechState(MechState.INTAKE_FLOWER);
                    setPathState(PathState.DRIVE_FL_SH);
                    break;
                }
            case DRIVE_FL_SH:
                if (!follower.isBusy()) {
                    follower.follow(driveFlSh);
                    mechController.handleMechState(MechState.SHOOTING_HIVE);
                    setPathState(PathState.DRIVE_SH_EP);
                    break;
                }
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
        visionController = new VisionController(robot);
        visionController.initAprilTag();
        visionPortal = visionController.getVisionPortal();
        mechController = new MechController(robot, visionController);
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