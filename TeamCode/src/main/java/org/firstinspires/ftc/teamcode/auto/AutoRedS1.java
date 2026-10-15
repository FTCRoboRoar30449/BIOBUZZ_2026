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

@Autonomous (name = "AutoRedS1", group = "Red")
public class AutoRedS1 extends OpMode {
    RobotHardware robot;
    MechController mechController;
    VisionController visionController;
    private VisionPortal visionPortal;
    private Follower follower;
    private ElapsedTime pathTimer, opModeTimer;
    public enum PathState {
        //sp = starting position
        //sh = shoot position
        //inp = intake position
        //in = intake
        //sh2 = shoot position two
        //ep = end position
        DRIVE_SP_SH,
        DRIVE_SH_INP,
        DRIVE_INP_IN,
        DRIVE_IN_SH2,
        DRIVE_SH2_EP,
        IDLE,
    }

    private PathState pathState;

    private final Pose startPose = new Pose(56.69533169533169,8,Math.toRadians(90));
    private final Pose shootOne = new Pose(56.69533169533169,33.91400491400491,Math.toRadians(90));
    private final Pose intakePose = new Pose(11.461156662309486,24.286185201710143,Math.toRadians(90));
    private final Pose intakeAction = new Pose(7.377861009568327,3.3115179481033046,Math.toRadians(90));
    private final Pose shootTwo = new Pose(59.49140049140049,104.53685503685503, Math.toRadians(270));
    private final Pose endPoint = new Pose(5.111793611793616,104.68058968058968, Math.toRadians(0));
    private final Pose controlPoint = new Pose(40.57371007371006,140.84889434889436);

    private Path driveSpSh, driveShInp, driveInpIn, driveInSh2, driveSh2Ep;

    public void buildPaths() {
        driveSpSh = new AtomicPath(new Line(startPose, shootOne))
                .linear(startPose.heading(), shootOne.heading());
        driveShInp = new AtomicPath(new Line(shootOne, intakePose))
                .linear(shootOne.heading(), intakePose.heading());
        driveInpIn = new AtomicPath(new Line(intakePose, intakeAction ))
                .linear(intakePose.heading(),intakeAction.heading());
        driveInSh2 = new AtomicPath(new BezierCurve(intakeAction, shootTwo))
                .linear(intakeAction.heading(), shootTwo.heading());
        driveSh2Ep = new AtomicPath(new Line(shootTwo, endPoint))
                .linear(shootTwo.heading(),endPoint.heading());
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_SP_SH:
                follower.follow(driveSpSh);
                mechController.handleMechState(MechState.SHOOTING_HIVE);
                setPathState(PathState.DRIVE_SH_INP);
                break;
            case DRIVE_SH_INP:
                if (!follower.isBusy()) {
                    follower.follow(driveShInp);
                    setPathState(PathState.DRIVE_INP_IN);
                    break;
                }
            case DRIVE_INP_IN:
                if (!follower.isBusy()) {
                    follower.follow(driveInpIn);
                    mechController.handleMechState(MechState.INTAKE_FIELD);
                    setPathState(PathState.DRIVE_IN_SH2);
                    break;
                }
            case DRIVE_IN_SH2:
                if (!follower.isBusy()) {
                    follower.follow(driveInSh2);
                    mechController.handleMechState(MechState.SHOOTING_HIVE);
                    setPathState(PathState.DRIVE_SH2_EP);
                    break;
                }
            case DRIVE_SH2_EP:
                if (!follower.isBusy()) {
                    follower.follow(driveSh2Ep);
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

