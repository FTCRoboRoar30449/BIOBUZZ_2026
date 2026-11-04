package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MechController;
import org.firstinspires.ftc.teamcode.robot.MechState;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.ShooterAdapter;
import org.firstinspires.ftc.teamcode.robot.TurretController;
import org.firstinspires.ftc.teamcode.robot.VisionController;

/**
 * BLUE alliance mechanism TeleOp. Bench version: no drivetrain, no Pedro.
 *
 * The pose normally comes from follower.getPose(). Here it comes from the
 * Pinpoint if it is reporting, otherwise from BENCH_X/Y/HEADING so the
 * shooting FSMs can be exercised on a table. Swap updatePoseFromSource() for
 * the follower call when this moves onto the robot.
 *
 * CONTROLS
 *   gamepad1  A intake field   X intake flower   B idle   Y vision test
 *             dpad up/down  bench: +1 pollen / +1 nectar
 *             dpad left     bench: clear inventory to 0
 *             dpad right    bench: toggle INTAKE_FLOWER_SHOOT_HIVE /
 *                           INTAKE_FLOWER_SHOOT_FLOWER
 *             RB drive to nearest ball (Batch 3 - not wired yet)
 *   gamepad2  A shoot hive     X shoot flower pollen   B idle
 *             Y shoot flower nectar (one ball)
 *             dpad up/down  partner scored nectar / pollen
 *             dpad left     reset CELL estimate
 *             RB            re-home turret
 *             LB (hold)     manual turret on left stick X
 *             RT            intake flower then shoot hive
 *             LT            intake flower then shoot flower
 */
@TeleOp(name = "TeleopMech BLUE", group = "Test")
public class TeleopMechBlue extends OpMode {

    /** Bench pose used when the Pinpoint is not reporting. PEDRO inches. */
    public static double BENCH_X = 90.0, BENCH_Y = 30.0, BENCH_HEADING_DEG = 182.7;

    private RobotHardware robot;
    private MechController mech;
    private VisionController vision;
    private ShooterAdapter shooter;
    private TurretController turret;

    private boolean p1a, p1x, p1b, p1y, p1du, p1dd, p1dl, p1dr;
    private boolean p2a, p2x, p2b, p2y, p2du, p2dd, p2dl, p2rb;
    private boolean hivePipelineSelected = false;

    @Override public void init() {
        robot   = new RobotHardware(hardwareMap, telemetry);
        shooter = new ShooterAdapter(ShooterAdapter.Alliance.BLUE);
        vision  = new VisionController(robot, shooter);
        turret  = new TurretController(robot);
        mech    = new MechController(robot, vision, shooter, turret);

        vision.initAprilTag();
        vision.startHiveAprilTagPipeline();
        mech.setState(MechState.START);
        // Beds to the resting pattern now, not when an intake state is
        // first entered. update() does not run until play is pressed.
        mech.initPositions();

        // Commands the turret to forward and confirms on the limit switch.
        // Does not sweep; see TurretController for why.
        turret.beginHoming();

        telemetry.addLine("BLUE alliance. Homing turret...");
        telemetry.update();
    }

    private boolean initRb = false;

    @Override public void init_loop() {
        turret.update();
        telemetry.addData("Turret", turret.getHomeMessage());
        telemetry.addData("Press", "gamepad2 RB to search if it did not find home");
        // Rising edge only: calling beginSearch() every loop would reset the
        // sweep each time and it would never advance.
        if (gamepad2.right_bumper && !initRb) turret.beginSearch();
        initRb = gamepad2.right_bumper;
        telemetry.update();
    }

    @Override public void start() { mech.setState(MechState.IDLE); }

    @Override public void loop() {
        updatePoseFromSource();

        // ---- gamepad1: intake -------------------------------------------
        boolean a1 = gamepad1.a && !p1a, x1 = gamepad1.x && !p1x;
        boolean b1 = gamepad1.b && !p1b, y1 = gamepad1.y && !p1y;
        if (a1) mech.setState(MechState.INTAKE_FIELD);
        else if (x1) mech.setState(MechState.INTAKE_FLOWER);
        else if (b1) mech.setState(MechState.IDLE);
        else if (y1) {
            hivePipelineSelected = !hivePipelineSelected;
            if (hivePipelineSelected) vision.startHiveAprilTagPipeline();
            else vision.startFlowerCountPipeline();
        }
        p1a = gamepad1.a; p1x = gamepad1.x; p1b = gamepad1.b; p1y = gamepad1.y;

        // ---- bench inventory -------------------------------------------
        // INTAKE_FIELD stops dead at 4 balls and polls no sensors, and the
        // shooting states finish instantly at 0. Being able to set the count
        // live is the difference between testing a path and watching it skip.
        boolean du1 = gamepad1.dpad_up && !p1du;
        boolean dd1 = gamepad1.dpad_down && !p1dd;
        boolean dl1 = gamepad1.dpad_left && !p1dl;
        if (du1) mech.countPollen = Math.min(4, mech.countPollen + 1);
        if (dd1) mech.countNectar = Math.min(4, mech.countNectar + 1);
        if (dl1) { mech.countPollen = 0; mech.countNectar = 0; }

        // The two combo flower states have no button of their own -- X only
        // reaches plain INTAKE_FLOWER. They share its case block, so the
        // scooping is identical; what differs is where they go when the
        // flower runs dry, and that handoff is the part worth watching.
        boolean dr1 = gamepad1.dpad_right && !p1dr;
        if (dr1) {
            if (mech.getState() == MechState.INTAKE_FLOWER_SHOOT_HIVE) {
                mech.setState(MechState.INTAKE_FLOWER_SHOOT_FLOWER);
            } else {
                mech.setState(MechState.INTAKE_FLOWER_SHOOT_HIVE);
            }
        }
        p1du = gamepad1.dpad_up; p1dd = gamepad1.dpad_down;
        p1dl = gamepad1.dpad_left; p1dr = gamepad1.dpad_right;

        // ---- gamepad2: shooting -----------------------------------------
        boolean a2 = gamepad2.a && !p2a, x2 = gamepad2.x && !p2x;
        boolean b2 = gamepad2.b && !p2b, y2 = gamepad2.y && !p2y;
        boolean du = gamepad2.dpad_up && !p2du, dd = gamepad2.dpad_down && !p2dd;
        boolean dl = gamepad2.dpad_left && !p2dl, rb2 = gamepad2.right_bumper && !p2rb;

        if (a2) mech.setState(MechState.SHOOTING_HIVE);
        else if (x2) mech.setState(MechState.SHOOTING_FLOWER_POLLEN);
        else if (y2) mech.setState(MechState.SHOOTING_FLOWER_NECTAR);
        else if (b2) mech.setState(MechState.IDLE);

        if (gamepad2.right_trigger > 0.5) mech.setState(MechState.INTAKE_FLOWER_SHOOT_HIVE);
        if (gamepad2.left_trigger  > 0.5) mech.setState(MechState.INTAKE_FLOWER_SHOOT_FLOWER);

        if (du) mech.addPartnerNectar();
        if (dd) mech.addPartnerPollen();
        if (dl) mech.resetCellCount();
        if (rb2) turret.beginSearch();

        // Manual turret override while LB is held, for bench checks.
        if (gamepad2.left_bumper) {
            turret.setTurretDeg(turret.getCommandedDeg() + gamepad2.left_stick_x * 2.0);
        }

        p2a = gamepad2.a; p2x = gamepad2.x; p2b = gamepad2.b; p2y = gamepad2.y;
        p2du = gamepad2.dpad_up; p2dd = gamepad2.dpad_down;
        p2dl = gamepad2.dpad_left; p2rb = gamepad2.right_bumper;

        mech.update();
        mech.allTelemetry();
    }

    /**
     * On the robot this becomes:
     *     Pose p = follower.getPose();
     *     mech.updatePose(p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
     * Pedro already reports in the frame the shooter classes expect, so there
     * is no conversion here.
     */
    private void updatePoseFromSource() {
        mech.updatePose(BENCH_X, BENCH_Y, BENCH_HEADING_DEG);
        // The ball detector needs it too, or detections come back with NaN
        // field coordinates and nothing can path to them.
        vision.setPose(BENCH_X, BENCH_Y, BENCH_HEADING_DEG);
    }

    @Override public void stop() {
        if (mech != null) mech.stopAll();
        if (vision != null) vision.stop();
    }
}




