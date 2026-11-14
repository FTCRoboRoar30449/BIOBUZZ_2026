package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

// THE tuning tool. Replaces HardwareCheck + BenchCalibration + RobotTuning.
//
// Those three were split by WHICH FILE a value lived in, which is of no
// interest to anyone holding a gamepad. The order you must actually work in
// cut across all three -- DIRECTION here, two modes there, eight pages back in
// the third -- and the only record of that order was a text page you could
// walk straight past. Worse, the three had different conventions: separate
// dumps, so half your session could be lost by stopping the wrong OpMode.
//
// One ordered walk now, step 1 to step 27, with one dump at the end.
//
//   dpad L/R   previous / next STEP
//   dpad U/D   pick a row within the step
//   bumpers    coarse      triggers   fine
//   A B X Y    per-step actions, always named on screen
//   back       jump to DUMP (also writes it to the log)
//
// Nothing is saved. DUMP prints paste-ready Java -- a tuner that writes to
// source makes the source a lie: the file says 45, the robot runs 62, and the
// next clean install gets a different robot.
@TeleOp(name = "Robot Setup", group = "calibration")
public class RobotSetup extends LinearOpMode {

    // Order matters: each step needs the one above it. This enum IS the
    // procedure -- there is no separate list to fall out of step with.
    private enum Step {
        // ---- CHECK: is it wired, does it turn the right way ----
        DEVICES,        //  1  everything resolves
        DIRECTION,      //  2  motor directions
        SERVO_LIMITS,   //  3  every servo's mechanical stops

        // ---- MECH: the half of the robot that is not the turret ----
        // Deliberately BEFORE the turret block. On 13 Nov the turret ate a
        // whole session and none of this got touched, because it sat behind
        // nine turret steps that were blocked on hardware. Nothing here needs
        // the turret, so nothing here should wait for it.
        DEPLOY,         //  4  arm swing, then scoop
        SCOOP,          //  5  flower endpoints
        BEDS,           //  6  bed endpoints
        INDEX,          //  7  ticks into the tunnels
        FEED,           //  8  ticks out, belt power
        HOOD_POLLEN,    //  9  POLLEN_POS_AT_55
        HOOD_NECTAR,    // 10  NECTAR_POS_AT_55
        SHOOTER,        // 11  PIDF -- flywheels mounted
        DIP,            // 12  thresholds vs spread

        // ---- SENSE: nothing here moves anything ----
        COLOR,          // 13  colour sensors
        BALLS,          // 14  C270 HSV gates
        LIMELIGHT,      // 15  pipeline and tags
        PINPOINT,       // 16  odometry pods

        // ---- TURRET: one block, and it stays together ----
        // Each of these needs the one above it, and all of them need a turret
        // that actually follows commands. If it does not, skip the whole
        // block rather than fighting it step by step.
        DIGITAL,        // 17  limit switch reads
        ENCODER,        // 18  through-bore is wired
        PWM_CHECK,      // 19  travel, and the breakaway test
        LIMIT_POLARITY, // 20  record LIMIT_TRIGGERED_STATE
        TURRET_SIGN,    // 21  SERVO_SIGN   (eyes only)
        ENCODER_SIGN,   // 22  ENCODER_SIGN + ticks/rev
        TURRET_ZERO,    // 23  SERVO_AT_ZERO, HOME_OFFSET_DEG
        HOME,           // 24  the real homing sequence
        TURRET,         // 25  closed loop, settle times

        // ---- GATES: needs the turret and a drivetrain ----
        RANGE,          // 26  flower approach bands
        FIRE,           // 27  fire gates
        FLOWERSHOT,     // 28  flower setpoints

        DUMP            // 29  paste-ready output
    }

    // Which block a step belongs to, shown in the header so you always know
    // whether what you are on can be done today.
    private static String section(Step s) {
        switch (s) {
            case DEVICES: case DIRECTION: case SERVO_LIMITS:
                return "CHECK";
            case DEPLOY: case SCOOP: case BEDS: case INDEX: case FEED:
            case HOOD_POLLEN: case HOOD_NECTAR: case SHOOTER: case DIP:
                return "MECH";
            case COLOR: case BALLS: case LIMELIGHT: case PINPOINT:
                return "SENSE";
            case RANGE: case FIRE: case FLOWERSHOT:
                return "GATES";
            case DUMP:
                return "DUMP";
            default:
                return "TURRET";
        }
    }

    private RobotHardware robot;
    private VisionController vision;
    private Step step = Step.DEVICES;
    private int sel = 0;
    private String note = "";

    // ---- discovered results (the BenchCalibration half) --------------------
    private Boolean limitAtHome = null;
    private Integer turretSign = null;
    private Integer encoderSign = null;
    private Double turretZero = null;
    private Double pollenAt55 = null;
    private Double nectarAt55 = null;
    private Double encoderTicksPerRev = null;
    // Measured, not guessed. The turret encoder sits on the turret axis at
    // 1:1, so the angle between the home switch and true forward is a
    // subtraction, not an estimate.
    private Integer homeMarkTicks = null;
    private Double homeOffsetDeg = null;

    // Mark-and-jog for the servos that have no encoder. Set a reference at one
    // mechanical stop, jog to the other, and read the gap -- in servo units
    // always, and in real degrees wherever the gear ratio is known. Cleared on
    // every step change so a stale mark from another servo cannot be read as
    // this one's.
    private Double servoMark = null;

    // ---- SERVO_LIMITS: one place to find every servo's mechanical stops ----
    // The old SERVO page did this with A=0.0 and B=1.0, which commands the
    // full ELECTRICAL range. Five of the six servos work in a band far
    // narrower than that -- the beds use 0.150-0.300 of it -- so those two
    // buttons drove them hard into their stops. Here A and B RECORD the stop
    // you have crept up to instead of jumping to one.
    private Servo[] servos;
    private String[] servoNames;
    private double[] servoPos;
    private Double[] servoLo, servoHi;
    // For the turret only: the ENCODER reading at each recorded end.
    //
    // servoLo/servoHi hold what was COMMANDED, and for a servo with real
    // breakaway friction that is not where the output is -- it lags, by a
    // varying amount that depends on which way you approached and how big the
    // last step was. Recording the command therefore gives a different answer
    // every attempt, which is exactly what 13 Nov showed. The turret has an
    // encoder on its output; use it and the span stops wandering.
    private Integer turretLoTicks = null, turretHiTicks = null;

    // The real homing sequence, run from here so the whole chain can be
    // checked before TeleOp: switch polarity, the search sweep, SERVO_AT_ZERO
    // being adopted, and HOME_OFFSET_DEG being backed out. Every one of those
    // is a thing that silently aims the robot somewhere else if it is wrong,
    // and none of them is visible from a shot that misses.
    private TurretController turret;
    private boolean homing = false;

    // ---- live servo commands -----------------------------------------------
    // push() commands these every loop while a turret/hood step is selected,
    // so they are where the robot sits the moment that step is reached. They
    // start at the CURRENT constants, never at an arbitrary number: 0.100 used
    // to drive both hoods ~79 deg past their low stop on the first loop.
    private double turretPos     = TurretController.SERVO_AT_ZERO;
    private double pollenHoodPos = HoodMap.POLLEN_POS_AT_55;
    private double nectarHoodPos = HoodMap.NECTAR_POS_AT_55;

    // ---- per-step state ----------------------------------------------------
    private DcMotor[] dirMotors;
    private String[] dirNames;
    private double dirPower = 0.20;
    private int dirZero = 0;
    // Per-motor loop mode on the DIRECTION step. CLOSED is the default and
    // the right one: in RUN_USING_ENCODER setPower(0.2) asks for 20% of max
    // SPEED and the controller supplies whatever duty that takes, so friction
    // is absorbed. In RUN_WITHOUT_ENCODER the same 0.2 is 20% DUTY, which a
    // built conveyor will not turn on -- it just hums. Open loop is only for
    // a motor whose encoder is not reading, and service() switches it there
    // on evidence rather than on assumption.
    private boolean[] dirOpen;
    private long dirCmdMs = 0;
    private double dirCmdPower = 0;
    private boolean dirCmdActive = false;
    public static double DIR_OPEN_LOOP_MIN_POWER = 0.50;
    public static long DIR_FALLBACK_MS = 400;

    private int encZero = 0;

    private double targetRpm = 2650;
    private boolean openLoop = false;
    private double kP = RobotHardware.SHOOTER_KP, kI = RobotHardware.SHOOTER_KI,
                   kD = RobotHardware.SHOOTER_KD, kF = RobotHardware.SHOOTER_KF;
    private double vMin = 1e9, vMax = -1e9, observedMaxTps = 0;
    private long spreadResetMs = 0;

    private long deployStartMs = 0;
    private boolean deployRunning = false;
    private boolean scoopAtPickup = false;
    private int bedPattern = 0;
    private boolean feedRunning = false;
    private long feedStartMs = 0;
    private int feedStartPos = 0, feedTargetTicks = 0;
    private boolean dipSpinning = false;
    private double dipRpm = 2650;

    private int ballTypeIdx = 0;
    private int encStart = 0;
    private double posStart = 0;

    public static double SIGN_TEST_DEG = 30.0;

    // PWM_CHECK ramps instead of jumping. setPosition() to the far end sends
    // the turret there as fast as the servo can go, with nothing you can do
    // about it once pressed -- and the turret carries cables. Ramping means
    // any button halts it where it is, and a 380 deg sweep takes about three
    // seconds rather than being over before you can react.
    public static double PWM_RAMP_PER_LOOP = 0.0030;   // ~2.7 deg of turret
    private Double pwmTarget = null;

    // The turret's REAL band. clamp01 is not a limit: it happily commands 0.0
    // or 1.0, which is +/-450 deg against a +/-190 deg turret.
    // Methods, not constants. These derive from SERVO_AT_ZERO, TURRET_LIMIT_DEG
    // and TURRET_TRAVEL_DEG -- all three of which are MEASURED during this
    // walk and change as you measure them. Computed once at class-init they
    // would freeze at the shipped guesses and silently clamp every later step
    // to a band that no longer matches the robot.
    private static double turretPosMin() {
        return TurretController.SERVO_AT_ZERO
                - TurretController.TURRET_LIMIT_DEG / TurretController.TURRET_TRAVEL_DEG;
    }
    private static double turretPosMax() {
        return TurretController.SERVO_AT_ZERO
                + TurretController.TURRET_LIMIT_DEG / TurretController.TURRET_TRAVEL_DEG;
    }

    // ---- knob abstraction, for the steps that are just numbers -------------
    private interface Getter { double get(); }
    private interface Setter { void set(double v); }

    private static final class Knob {
        final String name, fmt, owner;
        final Getter get; final Setter set;
        final double coarse, fine, lo, hi;
        Knob(String owner, String name, String fmt, double coarse, double fine,
             double lo, double hi, Getter get, Setter set) {
            this.owner = owner; this.name = name; this.fmt = fmt;
            this.coarse = coarse; this.fine = fine; this.lo = lo; this.hi = hi;
            this.get = get; this.set = set;
        }
        void nudge(int dir, boolean fineStep) {
            double v = get.get() + dir * (fineStep ? fine : coarse);
            set.set(v < lo ? lo : (v > hi ? hi : v));
        }
        String value() { return String.format(Locale.US, fmt, get.get()); }
    }

    private Knob[] knobs(Step s) {
        switch (s) {
            case DEPLOY: return new Knob[] {
                new Knob("MechController", "DEPLOY_MS", "%.0f ms", 50, 10, 0, 3000,
                        () -> MechController.DEPLOY_MS,
                        v -> MechController.DEPLOY_MS = (long) v),
                new Knob("MechController", "DEPLOY_POWER", "%.2f", 0.05, 0.01, 0, 1,
                        () -> MechController.DEPLOY_POWER,
                        v -> MechController.DEPLOY_POWER = v),
                new Knob("MechController", "DEPLOY_SCOOP_DELAY_MS", "%.0f ms", 50, 10, 0, 2000,
                        () -> MechController.DEPLOY_SCOOP_DELAY_MS,
                        v -> MechController.DEPLOY_SCOOP_DELAY_MS = (long) v),
            };
            case SCOOP: return new Knob[] {
                new Knob("MechController", "FLOWER_REST", "%.3f", 0.025, 0.005, 0, 1,
                        () -> MechController.FLOWER_REST,
                        v -> MechController.FLOWER_REST = v),
                new Knob("MechController", "FLOWER_SCOOP", "%.3f", 0.025, 0.005, 0, 1,
                        () -> MechController.FLOWER_SCOOP,
                        v -> MechController.FLOWER_SCOOP = v),
            };
            case BEDS: return new Knob[] {
                new Knob("MechController", "BED_POLLEN_ENGAGED", "%.3f", 0.010, 0.002, 0, 1,
                        () -> MechController.BED_POLLEN_ENGAGED,
                        v -> MechController.BED_POLLEN_ENGAGED = v),
                new Knob("MechController", "BED_POLLEN_DISENGAGED", "%.3f", 0.010, 0.002, 0, 1,
                        () -> MechController.BED_POLLEN_DISENGAGED,
                        v -> MechController.BED_POLLEN_DISENGAGED = v),
                new Knob("MechController", "BED_NECTAR_ENGAGED", "%.3f", 0.010, 0.002, 0, 1,
                        () -> MechController.BED_NECTAR_ENGAGED,
                        v -> MechController.BED_NECTAR_ENGAGED = v),
                new Knob("MechController", "BED_NECTAR_DISENGAGED", "%.3f", 0.010, 0.002, 0, 1,
                        () -> MechController.BED_NECTAR_DISENGAGED,
                        v -> MechController.BED_NECTAR_DISENGAGED = v),
            };
            case INDEX: return new Knob[] {
                new Knob("MechController", "TICKS_PER_POLLEN_STORAGE", "%.0f", 10, 2, 0, 1000,
                        () -> MechController.TICKS_PER_POLLEN_STORAGE,
                        v -> MechController.TICKS_PER_POLLEN_STORAGE = (int) v),
                new Knob("MechController", "TICKS_PER_NECTAR_STORAGE", "%.0f", 10, 2, 0, 1000,
                        () -> MechController.TICKS_PER_NECTAR_STORAGE,
                        v -> MechController.TICKS_PER_NECTAR_STORAGE = (int) v),
                new Knob("MechController", "COUNT_COOLDOWN_MS", "%.0f ms", 25, 5, 0, 2000,
                        () -> MechController.COUNT_COOLDOWN_MS,
                        v -> MechController.COUNT_COOLDOWN_MS = (long) v),
                new Knob("MechController", "REVERSE_INTAKE_TIME_MS", "%.0f ms", 50, 10, 0, 3000,
                        () -> MechController.REVERSE_INTAKE_TIME_MS,
                        v -> MechController.REVERSE_INTAKE_TIME_MS = (long) v),
            };
            case FEED: return new Knob[] {
                new Knob("MechController", "TICKS_TO_SHOOT_POLLEN", "%.0f", 25, 5, 0, 2000,
                        () -> MechController.TICKS_TO_SHOOT_POLLEN,
                        v -> MechController.TICKS_TO_SHOOT_POLLEN = (int) v),
                new Knob("MechController", "TICKS_TO_SHOOT_NECTAR", "%.0f", 25, 5, 0, 2000,
                        () -> MechController.TICKS_TO_SHOOT_NECTAR,
                        v -> MechController.TICKS_TO_SHOOT_NECTAR = (int) v),
                new Knob("MechController", "TICKS_TO_SHOOT_BOTH", "%.0f", 25, 5, 0, 2000,
                        () -> MechController.TICKS_TO_SHOOT_BOTH,
                        v -> MechController.TICKS_TO_SHOOT_BOTH = (int) v),
                new Knob("MechController", "CONVEYOR_POWER", "%.2f", 0.05, 0.01, 0.05, 1.0,
                        () -> MechController.CONVEYOR_POWER,
                        v -> MechController.CONVEYOR_POWER = v),
            };
            case DIP: return new Knob[] {
                new Knob("-", "dip test rpm", "%.0f rpm", 250, 25, 0, 6500,
                        () -> dipRpm, v -> dipRpm = v),
                new Knob("MechController", "DIP_ENTER", "%.4f", 0.005, 0.001, 0.80, 1.0,
                        () -> MechController.DIP_ENTER,
                        v -> MechController.DIP_ENTER = v),
                new Knob("MechController", "DIP_EXIT", "%.4f", 0.005, 0.001, 0.80, 1.0,
                        () -> MechController.DIP_EXIT,
                        v -> MechController.DIP_EXIT = v),
            };
            case TURRET: return new Knob[] {
                new Knob("TurretController", "HOME_OFFSET_DEG", "%.1f deg", 5.0, 1.0, -190, 190,
                        () -> TurretController.HOME_OFFSET_DEG,
                        v -> TurretController.HOME_OFFSET_DEG = v),
                new Knob("TurretController", "ENCODER_COUPLED", "%.0f  (0=no 1=yes)", 1, 1, 0, 1,
                        () -> TurretController.ENCODER_COUPLED ? 1 : 0,
                        v -> TurretController.ENCODER_COUPLED = v >= 0.5),
                new Knob("TurretController", "CLOSED_LOOP", "%.0f  (0=no 1=yes)", 1, 1, 0, 1,
                        () -> TurretController.CLOSED_LOOP ? 1 : 0,
                        v -> TurretController.CLOSED_LOOP = v >= 0.5),
                new Knob("TurretController", "CL_KP", "%.2f", 0.05, 0.01, 0, 2,
                        () -> TurretController.CL_KP,
                        v -> TurretController.CL_KP = v),
                new Knob("TurretController", "CL_TOL_DEG", "%.2f deg", 0.25, 0.05, 0.1, 10,
                        () -> TurretController.CL_TOL_DEG,
                        v -> TurretController.CL_TOL_DEG = v),
                new Knob("TurretController", "CL_MAX_TRIM", "%.3f", 0.010, 0.002, 0, 0.3,
                        () -> TurretController.CL_MAX_TRIM,
                        v -> TurretController.CL_MAX_TRIM = v),
                new Knob("TurretController", "SERVO_MS_PER_TURRET_DEG", "%.2f", 0.10, 0.02, 0, 20,
                        () -> TurretController.SERVO_MS_PER_TURRET_DEG,
                        v -> TurretController.SERVO_MS_PER_TURRET_DEG = v),
                new Knob("TurretController", "SETTLE_BASE_MS", "%.0f ms", 10, 5, 0, 1000,
                        () -> TurretController.SETTLE_BASE_MS,
                        v -> TurretController.SETTLE_BASE_MS = (long) v),
                new Knob("TurretController", "ARRIVED_TOLERANCE_DEG", "%.1f deg", 0.5, 0.1, 0.1, 20,
                        () -> TurretController.ARRIVED_TOLERANCE_DEG,
                        v -> TurretController.ARRIVED_TOLERANCE_DEG = v),
                new Knob("TurretController", "SLIP_TOLERANCE_DEG", "%.1f deg", 1.0, 0.5, 0.5, 45,
                        () -> TurretController.SLIP_TOLERANCE_DEG,
                        v -> TurretController.SLIP_TOLERANCE_DEG = v),
            };
            case RANGE: return new Knob[] {
                new Knob("MechController", "FLOWER_DEPLOY_IN", "%.1f in", 0.5, 0.1, 0, 36,
                        () -> MechController.FLOWER_DEPLOY_IN,
                        v -> MechController.FLOWER_DEPLOY_IN = v),
                new Knob("MechController", "FLOWER_INTAKE_IN", "%.1f in", 0.5, 0.1, 0, 36,
                        () -> MechController.FLOWER_INTAKE_IN,
                        v -> MechController.FLOWER_INTAKE_IN = v),
                new Knob("MechController", "FLOWER_RETRACT_IN", "%.1f in", 0.5, 0.1, 0, 36,
                        () -> MechController.FLOWER_RETRACT_IN,
                        v -> MechController.FLOWER_RETRACT_IN = v),
            };
            case FIRE: return new Knob[] {
                new Knob("MechController", "FIRE_ONLY_WHEN_STEADY", "%.0f  (0=no 1=yes)", 1, 1, 0, 1,
                        () -> MechController.FIRE_ONLY_WHEN_STEADY ? 1 : 0,
                        v -> MechController.FIRE_ONLY_WHEN_STEADY = v >= 0.5),
                new Knob("MechController", "SHOOTER_READY_TOLERANCE", "%.3f", 0.01, 0.002, 0.5, 1.0,
                        () -> MechController.SHOOTER_READY_TOLERANCE,
                        v -> MechController.SHOOTER_READY_TOLERANCE = v),
                new Knob("MechController", "FIRE_MAX_SPEED_IN_S", "%.1f in/s", 0.5, 0.1, 0, 60,
                        () -> MechController.FIRE_MAX_SPEED_IN_S,
                        v -> MechController.FIRE_MAX_SPEED_IN_S = v),
                new Knob("MechController", "FIRE_MAX_TURN_DEG_S", "%.1f deg/s", 2.5, 0.5, 0, 180,
                        () -> MechController.FIRE_MAX_TURN_DEG_S,
                        v -> MechController.FIRE_MAX_TURN_DEG_S = v),
                new Knob("MechController", "PRE_SPIN_RELEASE_MS", "%.0f ms", 50, 10, 0, 5000,
                        () -> MechController.PRE_SPIN_RELEASE_MS,
                        v -> MechController.PRE_SPIN_RELEASE_MS = (long) v),
                new Knob("MechController", "SHOOTER_RECOVER_MS", "%.0f ms", 25, 5, 0, 2000,
                        () -> MechController.SHOOTER_RECOVER_MS,
                        v -> MechController.SHOOTER_RECOVER_MS = (long) v),
            };
            case FLOWERSHOT: return new Knob[] {
                new Knob("MechController", "FLOWER_POLLEN_RPM", "%.0f rpm", 50, 10, 0, 6500,
                        () -> MechController.FLOWER_POLLEN_RPM,
                        v -> MechController.FLOWER_POLLEN_RPM = v),
                new Knob("MechController", "FLOWER_NECTAR_RPM", "%.0f rpm", 50, 10, 0, 6500,
                        () -> MechController.FLOWER_NECTAR_RPM,
                        v -> MechController.FLOWER_NECTAR_RPM = v),
                new Knob("MechController", "FLOWER_HOOD_DEG", "%.1f deg", 1.0, 0.5, 55, 80,
                        () -> MechController.FLOWER_HOOD_DEG,
                        v -> MechController.FLOWER_HOOD_DEG = v),
                new Knob("MechController", "FLOWER_AIM_TRIM_DEG", "%.1f deg", 1.0, 0.2, -45, 45,
                        () -> MechController.FLOWER_AIM_TRIM_DEG,
                        v -> MechController.FLOWER_AIM_TRIM_DEG = v),
            };
            default: return new Knob[0];
        }
    }

    // ---- lifecycle ---------------------------------------------------------

    @Override
    public void runOpMode() {
        try {
            robot = new RobotHardware(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addLine("RobotHardware FAILED to construct:");
            telemetry.addLine(e.getMessage());
            telemetry.addLine("");
            telemetry.addLine("A config name does not match. Fix that before anything else.");
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) { sleep(100); }
            return;
        }

        widen(robot.turretRot);
        widen(robot.hoodPollen);
        widen(robot.hoodNectar);

        turret = new TurretController(robot);
        vision = new VisionController(robot, new ShooterAdapter(ShooterAdapter.Alliance.RED));
        vision.initAprilTag();
        vision.startHiveAprilTagPipeline();

        dirMotors = new DcMotor[] { robot.intakeMot, robot.conveyorMot,
                                    robot.pollenShootingMot, robot.nectarShootingMot };
        dirNames = new String[] { "intakeMot", "conveyorMot",
                                  "pollenShootingMot", "nectarShootingMot" };
        servos = new Servo[] { robot.turretRot, robot.hoodPollen, robot.hoodNectar,
                               robot.flowerIntake, robot.conveyorPollen, robot.conveyorNectar };
        servoNames = new String[] { "turretRot", "hoodPollen", "hoodNectar",
                                    "flowerIntake", "conveyorPollen", "conveyorNectar" };
        // Start each one where the robot already believes it should be. Never
        // at 0.0, 0.5 or any other round number: the first push() would drive
        // it there, and for the beds and hoods that is well past a stop.
        servoPos = new double[] {
                TurretController.SERVO_AT_ZERO,
                HoodMap.POLLEN_POS_AT_55,
                HoodMap.NECTAR_POS_AT_55,
                MechController.FLOWER_REST,
                MechController.bedPollenPos(false),
                MechController.bedNectarPos(false) };
        servoLo = new Double[servos.length];
        servoHi = new Double[servos.length];

        dirOpen = new boolean[dirMotors.length];
        // intakeMot's encoder port carries the TURRET through-bore, so its own
        // shaft never shows up there. Closed loop would be asking a velocity
        // controller to chase a number that cannot move.
        dirOpen[0] = true;

        telemetry.addLine("ROBOT SETUP -- 29 steps, dpad L/R to walk them.");
        telemetry.addLine("");
        telemetry.addLine("Robot ON BLOCKS. gamepad1 only.");
        telemetry.addLine("Nothing is saved: read DUMP (step 29) before you stop.");
        telemetry.addLine("");
        telemetry.addLine("Press START.");
        telemetry.update();
        waitForStart();

        boolean pL=false,pR=false,pU=false,pD=false;
        boolean pA=false,pB=false,pX=false,pY=false,pLB=false,pRB=false,pBk=false;

        while (opModeIsActive()) {
            boolean l=gamepad1.dpad_left, r=gamepad1.dpad_right;
            boolean u=gamepad1.dpad_up,   d=gamepad1.dpad_down;
            boolean a=gamepad1.a, b=gamepad1.b, x=gamepad1.x, y=gamepad1.y;
            boolean lb=gamepad1.left_bumper, rb=gamepad1.right_bumper, bk=gamepad1.back;

            // Leaving a step stops whatever it was driving. Without this you
            // can walk away from FEED with the belt still running and adjust a
            // flywheel threshold while it does.
            boolean leaving = (l && !pL) || (r && !pR);
            if (leaving) { stopEverything(); pwmTarget = null; }
            // Changing the selected row on DIRECTION leaves the motor you were
            // driving spinning while the controls point at a different one.
            if (step == Step.DIRECTION && ((u && !pU) || (d && !pD))) stopDirectionMotors();

            if (l && !pL) { step = prev(step); sel = 0; note = ""; servoMark = null; resetEnc(); }
            if (r && !pR) { step = next(step); sel = 0; note = ""; servoMark = null; resetEnc(); }

            int n = Math.max(1, rows(step));
            if (u && !pU) sel = (sel + n - 1) % n;
            if (d && !pD) sel = (sel + 1) % n;
            if (step == Step.BEDS) bedPattern = sel % 4;

            // Any deliberate input halts a ramp in progress. "Press anything
            // to stop" has to be literally true when the thing moving carries
            // cables -- not "press the one button I documented".
            if (pwmTarget != null && (u || d || l || r || lb || rb || y
                    || gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5)) {
                pwmTarget = null;
                note = String.format(Locale.US, "HALTED at %.4f", turretPos);
            }

            if (lb && !pLB) adjust(-1, false);
            if (rb && !pRB) adjust(+1, false);
            if (gamepad1.left_trigger  > 0.5) adjust(-1, true);
            if (gamepad1.right_trigger > 0.5) adjust(+1, true);

            if (a && !pA) actionA();
            if (b && !pB) actionB();
            if (x && !pX) actionX();
            if (y && !pY) actionY();
            if (bk && !pBk) { step = Step.DUMP; dumpToLog(); note = "dump written to the log"; }

            pL=l;pR=r;pU=u;pD=d;pA=a;pB=b;pX=x;pY=y;pLB=lb;pRB=rb;pBk=bk;

            service();
            push();
            draw();
        }
        stopEverything();
        if (vision != null) vision.stop();
    }

    // How many selectable rows a step has, so dpad U/D wraps correctly.
    private int rows(Step s) {
        switch (s) {
            case HOME:         return 2;
            case PWM_CHECK:    return 2;
            case SERVO_LIMITS: return servos.length;
            case DIRECTION: return dirMotors.length;
            case SHOOTER:   return 5;
            case COLOR:     return 6;
            case BALLS:     return 5;
            case LIMELIGHT: return 10;
            case BEDS:      return 4;
            default:        return Math.max(1, knobs(s).length);
        }
    }

    private void widen(Servo s) {
        if (s instanceof ServoImplEx) {
            ((ServoImplEx) s).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    // Only command a servo while its own step is selected. The old
    // BenchCalibration pushed all three every loop regardless, so simply
    // reaching the OpMode moved hardware you were not working on.
    private int servoIdx() { return sel % servos.length; }

    private void push() {
        // While homing, TurretController owns turretRot. Two writers at loop
        // rate would fight: the search steps outward, we would drag it back.
        if (homing) return;
        switch (step) {
            case SERVO_LIMITS:
                // ONLY the selected one. Commanding all six every loop is how
                // the old page moved hardware you were not working on.
                servos[servoIdx()].setPosition(clamp01(servoPos[servoIdx()]));
                break;
            case PWM_CHECK:
            case LIMIT_POLARITY:
            case TURRET_SIGN:
            case ENCODER_SIGN:
            case TURRET_ZERO:
                robot.turretRot.setPosition(clamp01(turretPos));
                break;
            case HOOD_POLLEN:
                robot.hoodPollen.setPosition(clamp01(pollenHoodPos));
                break;
            case HOOD_NECTAR:
                robot.hoodNectar.setPosition(clamp01(nectarHoodPos));
                break;
            default: break;
        }
    }

    // ---- adjust ------------------------------------------------------------

    private void adjust(int dir, boolean fine) {
        switch (step) {
            case HOME:
                if ((sel % 2) == 0) {
                    // The search only sweeps +/-SEARCH_SPAN_DEG looking for
                    // the magnet. Ship it at 30 and a switch mounted 45 deg
                    // out can never be found -- homing reports FAILED and the
                    // cause is nowhere on screen.
                    TurretController.SEARCH_SPAN_DEG = clampR(
                            TurretController.SEARCH_SPAN_DEG + dir * (fine ? 1.0 : 10.0),
                            5.0, 190.0);
                } else {
                    // Where the search STARTS. beginHoming() commands turret
                    // 0, which is SERVO_AT_ZERO -- so if that position has the
                    // servo straining against something, every home begins by
                    // sitting there buzzing. Move the start somewhere the
                    // turret is free. Homing overwrites this anyway once it
                    // finds the magnet, so nothing downstream depends on it.
                    TurretController.SERVO_AT_ZERO = clampR(
                            TurretController.SERVO_AT_ZERO + dir * (fine ? 0.0020 : 0.0100),
                            0.0, 1.0);
                    robot.turretRot.setPosition(TurretController.SERVO_AT_ZERO);
                }
                break;
            case SERVO_LIMITS:
                // Per servo, because one step size cannot suit all six.
                //
                // 0.100 -- what the old page used -- is 30 deg on a bed servo
                // whose entire working span is 45, so you cannot creep up on
                // a stop with it. But 0.0100 is only 9 deg on the TURRET, and
                // on 13 Nov that proved to be below its breakaway friction:
                // the command changed and nothing moved, which reads exactly
                // like a dead servo. The turret therefore gets a coarse step
                // big enough to break loose.
                //
                // That is a WORKAROUND, not a fix. Measure the real figure
                // with the breakaway test on PWM_CHECK; if it is over a few
                // degrees the turret cannot aim and the gear train needs
                // attention, whatever this page does.
                servoPos[servoIdx()] = clamp01(servoPos[servoIdx()]
                        + dir * (fine ? fineStep(servoIdx()) : coarseStep(servoIdx())));
                break;
            case PWM_CHECK:
                // The two numbers that DEFINE the band, tuned on the page
                // that sweeps it. They have to be set together: the band is
                // SERVO_AT_ZERO +/- LIMIT/TRAVEL, so fixing one alone moves
                // the ends in the wrong direction.
                if ((sel % 2) == 0) {
                    TurretController.TURRET_LIMIT_DEG = clampR(
                            TurretController.TURRET_LIMIT_DEG + dir * (fine ? 1.0 : 5.0),
                            5.0, 190.0);
                } else {
                    TurretController.TURRET_TRAVEL_DEG = clampR(
                            TurretController.TURRET_TRAVEL_DEG + dir * (fine ? 5.0 : 25.0),
                            90.0, 1800.0);
                }
                break;
            case LIMIT_POLARITY:
            case TURRET_SIGN:
            case ENCODER_SIGN:
            case TURRET_ZERO:
                turretPos = clampR(turretPos + dir * (fine ? 0.0005 : 0.0100),
                                   turretPosMin(), turretPosMax());
                break;
            case HOOD_POLLEN:
                pollenHoodPos = clamp01(pollenHoodPos + dir * (fine ? 0.0005 : 0.0100));
                break;
            case HOOD_NECTAR:
                nectarHoodPos = clamp01(nectarHoodPos + dir * (fine ? 0.0005 : 0.0100));
                break;
            case DIRECTION:
                dirPower = clampR(dirPower + dir * (fine ? 0.02 : 0.10), 0.05, 1.0);
                break;
            case SHOOTER:
                switch (sel % 5) {
                    case 0: targetRpm = clampR(targetRpm + dir * (fine ? 25 : 250), 0, 6500); break;
                    case 1: kP = clampR(kP + dir * (fine ? 0.1 : 1.0), 0, 200); applyPidf(); break;
                    case 2: kI = clampR(kI + dir * (fine ? 0.01 : 0.1), 0, 50); applyPidf(); break;
                    case 3: kD = clampR(kD + dir * (fine ? 0.1 : 1.0), 0, 200); applyPidf(); break;
                    case 4: kF = clampR(kF + dir * (fine ? 0.1 : 1.0), 0, 100); applyPidf(); break;
                }
                resetSpread();
                break;
            case BALLS: {
                BallDetector.BallType[] ts = BallDetector.BallType.values();
                if ((sel % 5) == 0) { ballTypeIdx = (ballTypeIdx + dir + ts.length) % ts.length; break; }
                BallDetector.BallType bt = ts[ballTypeIdx % ts.length];
                double[] gg = BallDetector.getGate(bt);
                double st = fine ? 1 : 5;
                switch (sel % 5) {
                    case 1: gg[0] = clampR(gg[0] + dir * st, 0, 179); break;
                    case 2: gg[1] = clampR(gg[1] + dir * st, 0, 179); break;
                    case 3: gg[2] = clampR(gg[2] + dir * (fine ? 5 : 20), 0, 255); break;
                    case 4: gg[3] = clampR(gg[3] + dir * (fine ? 5 : 20), 0, 255); break;
                }
                BallDetector.setGate(bt, gg[0], gg[1], gg[2], gg[3]);
                note = String.format(Locale.US, "%s  H %.0f-%.0f  S>%.0f  V>%.0f",
                        bt, gg[0], gg[1], gg[2], gg[3]);
                break;
            }
            default: {
                Knob[] k = knobs(step);
                if (k.length == 0) return;
                k[sel % k.length].nudge(dir, fine);
                if (step == Step.BEDS) driveBeds();
                if (step == Step.SCOOP) driveScoop();
                break;
            }
        }
    }

    // ---- actions -----------------------------------------------------------

    private void actionA() {
        switch (step) {
            case DIRECTION:
                startDirection(+1);
                break;
            case SERVO_LIMITS:
                servoLo[servoIdx()] = servoPos[servoIdx()];
                if (servoIdx() == 0) turretLoTicks = robot.intakeMot.getCurrentPosition();
                note = String.format(Locale.US, "%s LOW stop = %.4f%s",
                        servoNames[servoIdx()], servoPos[servoIdx()],
                        servoIdx() == 0 ? "  (encoder " + turretLoTicks + ")" : "");
                break;
            case HOME:
                startHoming(false);
                break;
            case PWM_CHECK:
                pwmTarget = turretPosMin();
                // (placeholder-safe: PWM_CHECK keeps its own handler below)
                note = String.format(Locale.US,
                        "ramping to %.3f (-190 deg). ANY BUTTON HALTS IT.",
                        turretPosMin());
                break;
            case ENCODER:
                encZero = robot.intakeMot.getCurrentPosition();
                note = "zeroed -- now turn the turret through a known angle";
                break;
            case LIMIT_POLARITY:
                limitAtHome = robot.magneticLimitSwitch.getState();
                // The last of the four: applied live so homing on step 11 uses
                // what you just measured, not what shipped.
                TurretController.LIMIT_TRIGGERED_STATE = limitAtHome;
                note = "recorded: switch reads " + limitAtHome + " AT HOME";
                break;
            case TURRET_SIGN:
                // A RECORDS what you saw. Y runs the move; only a human can see
                // which way the turret went, and before this existed turretSign
                // was assigned null and nothing else -- SERVO_SIGN printed as
                // unset however carefully you watched.
                turretSign = (turretSign != null && turretSign > 0) ? -1 : +1;
                // Applied live for the same reason as ENCODER_SIGN: every
                // commanded angle from here on goes through it.
                TurretController.SERVO_SIGN = turretSign;
                note = "SERVO_SIGN = " + String.format(Locale.US, "%+d", turretSign)
                     + "   (A toggles, Y re-runs the move)";
                break;
            case ENCODER_SIGN:
                note = "use Y to run the test, not A";
                break;
            case TURRET_ZERO:
                turretZero = turretPos;
                if (homeMarkTicks == null) {
                    note = String.format(Locale.US,
                            "SERVO_AT_ZERO = %.4f. For HOME_OFFSET_DEG: jog to the"
                            + " switch, press Y to mark, jog back here, press A.",
                            turretZero);
                } else {
                    // Angle OF THE SWITCH, measured from here (true forward).
                    homeOffsetDeg = TurretController.ENCODER_SIGN
                            * (homeMarkTicks - robot.intakeMot.getCurrentPosition())
                            * 360.0 / TurretController.ENCODER_TICKS_PER_REV;
                    TurretController.HOME_OFFSET_DEG = homeOffsetDeg;
                    note = String.format(Locale.US,
                            "SERVO_AT_ZERO = %.4f, HOME_OFFSET_DEG = %+.1f (measured)",
                            turretZero, homeOffsetDeg);
                }
                break;
            case HOOD_POLLEN:
                pollenAt55 = pollenHoodPos;
                note = String.format(Locale.US, "POLLEN_POS_AT_55 = %.4f", pollenAt55);
                break;
            case HOOD_NECTAR:
                nectarAt55 = nectarHoodPos;
                note = String.format(Locale.US, "NECTAR_POS_AT_55 = %.4f", nectarAt55);
                break;
            case DEPLOY:
                deployStartMs = System.currentTimeMillis();
                deployRunning = true;
                note = "running deploy -- watch the arm, then the scoop";
                break;
            case SCOOP:
                scoopAtPickup = false; driveScoop(); note = "at REST";
                break;
            case BEDS:
                driveBeds(); note = "commanded " + BED_NAMES[bedPattern % 4];
                break;
            case INDEX:
                startIndex();
                break;
            case FEED:
                startFeed();
                break;
            case SHOOTER:
                openLoop = false;
                robot.pollenShootingMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.pollenShootingMot.setVelocity(RobotHardware.rpmToTicksPerSec(targetRpm));
                note = "pollen closed-loop at " + (int) targetRpm + " rpm";
                break;
            case DIP:
                dipSpinning = true;
                robot.pollenShootingMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.pollenShootingMot.setVelocity(RobotHardware.rpmToTicksPerSec(dipRpm));
                resetSpread();
                note = "pollen flywheel at " + (int) dipRpm + " rpm";
                break;
            case LIMELIGHT:
                vision.forcePipeline(sel % 10);
                note = "forced pipeline " + (sel % 10) + " and restarted";
                break;
            default: break;
        }
    }

    private void actionB() {
        switch (step) {
            case DIRECTION:
                startDirection(-1);
                break;
            case SERVO_LIMITS:
                servoHi[servoIdx()] = servoPos[servoIdx()];
                if (servoIdx() == 0) turretHiTicks = robot.intakeMot.getCurrentPosition();
                note = String.format(Locale.US, "%s HIGH stop = %.4f%s",
                        servoNames[servoIdx()], servoPos[servoIdx()],
                        servoIdx() == 0 ? "  (encoder " + turretHiTicks + ")" : "");
                break;
            case HOME:
                startHoming(true);          // skip VERIFY, sweep for the magnet
                break;
            case PWM_CHECK:
                pwmTarget = turretPosMax();
                note = String.format(Locale.US,
                        "ramping to %.3f (+190 deg). ANY BUTTON HALTS IT.",
                        turretPosMax());
                break;
            case SCOOP:
                scoopAtPickup = true; driveScoop(); note = "at SCOOP";
                break;
            case BEDS:
                bedPattern = (bedPattern + 1) % 4; sel = bedPattern; driveBeds();
                note = BED_NAMES[bedPattern];
                break;
            case INDEX:
            case FEED:
                stopFeed(); note = "stopped";
                break;
            case SHOOTER:
                openLoop = false;
                robot.nectarShootingMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.nectarShootingMot.setVelocity(RobotHardware.rpmToTicksPerSec(targetRpm));
                note = "nectar closed-loop at " + (int) targetRpm + " rpm";
                break;
            case DIP:
                stopDip(); note = "stopped";
                break;
            case LIMIT_POLARITY: limitAtHome = null; note = "cleared"; break;
            case TURRET_SIGN:    turretSign = null;  note = "cleared"; break;
            case ENCODER_SIGN:   encoderSign = null; encoderTicksPerRev = null; note = "cleared"; break;
            case TURRET_ZERO:    turretZero = null; homeMarkTicks = null;
                                 homeOffsetDeg = null; note = "cleared"; break;
            case HOOD_POLLEN:    pollenAt55 = null;  note = "cleared"; break;
            case HOOD_NECTAR:    nectarAt55 = null;  note = "cleared"; break;
            default: break;
        }
    }

    private void actionX() {
        switch (step) {
            case DIRECTION:
                // X also RE-ARMS closed loop. The fallback is sticky once it
                // fires, which is right while you are working on one motor and
                // wrong forever after: a motor demoted to open loop by a single
                // slow start would stay there for the rest of the session, at a
                // duty cycle that may be well below what it needs.
                dirMotor().setPower(0);
                dirCmdActive = false;
                dirOpen[dirIdx()] = (dirIdx() == 0);   // intakeMot stays open
                note = "stopped, " + dirNames[dirIdx()]
                     + " re-armed " + (dirIdx() == 0 ? "open" : "closed") + " loop";
                break;
            case HOME:
                homing = false;
                note = "homing abandoned, turret held where it is";
                break;
            case PWM_CHECK:
                // First press HALTS -- if cables are snagging, more travel is
                // the last thing you want. A second press, once already
                // stopped, ramps back to centre so the breakaway test has
                // room either side.
                if (pwmTarget != null) {
                    pwmTarget = null;
                    note = String.format(Locale.US, "HALTED at %.4f", turretPos);
                } else {
                    pwmTarget = TurretController.SERVO_AT_ZERO;
                    note = "ramping back to centre";
                }
                break;
            case LIMIT_POLARITY:
            case TURRET_SIGN:
            case ENCODER_SIGN:
            case TURRET_ZERO:
                turretPos = TurretController.SERVO_AT_ZERO;
                robot.turretRot.setPosition(turretPos);
                note = "turret back to SERVO_AT_ZERO";
                break;
            case SHOOTER:
                stopShooters(); note = "stopped";
                break;
            case DIP:
                resetSpread(); note = "spread reset";
                break;
            case BALLS:
                // Re-measure every gate after flipping: red and blue trade
                // places, so the old windows are not shifted, they point at
                // the wrong colour.
                BallDetector.FRAME_IS_BGR = !BallDetector.FRAME_IS_BGR;
                note = "reading frames as " + (BallDetector.FRAME_IS_BGR ? "BGR" : "RGB")
                     + " -- re-measure all three gates";
                break;
            default: note = "no X action on this step"; break;
        }
    }

    private void actionY() {
        switch (step) {
            case DIRECTION: {
                // setDirection flips the encoder sign with the power sign, so a
                // motor in RUN_USING_ENCODER stays consistent. Safe to toggle.
                DcMotor m = dirMotor();
                m.setPower(0);
                m.setDirection(m.getDirection() == DcMotor.Direction.FORWARD
                        ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
                dirZero = m.getCurrentPosition();
                note = dirNames[dirIdx()] + " now " + m.getDirection()
                     + "  -- NOT saved, edit RobotHardware";
                break;
            }
            case PWM_CHECK:
                runBreakawayTest();
                break;
            case TURRET_SIGN:
            case ENCODER_SIGN:
                runSignTest();
                break;
            case TURRET_ZERO:
                // Mark the switch. Needs ENCODER_SIGN, which step 9 has
                // already measured by the time you get here.
                boolean atSwitch = robot.magneticLimitSwitch.getState()
                        == TurretController.LIMIT_TRIGGERED_STATE;
                if (!atSwitch) {
                    note = "switch is NOT tripped here -- jog until 'limit switch"
                         + " raw' flips, then press Y";
                    break;
                }
                homeMarkTicks = robot.intakeMot.getCurrentPosition();
                note = "switch marked at " + homeMarkTicks
                     + " ticks. Now jog to TRUE FORWARD and press A.";
                break;
            case SHOOTER:
                openLoop = true;
                robot.pollenShootingMot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.nectarShootingMot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.pollenShootingMot.setPower(1.0);
                robot.nectarShootingMot.setPower(1.0);
                note = "FULL POWER free-run -- read ticks/s, then X to stop";
                break;
            case HOME:
                TurretController.SEARCH_FIRST_DIR = -TurretController.SEARCH_FIRST_DIR;
                note = "search will try " + (TurretController.SEARCH_FIRST_DIR > 0 ? "+" : "-")
                     + " first. Press A to home.";
                break;
            default:
                // On SERVO_LIMITS the turret row homes instead of marking:
                // a mark is redundant there now that A and B record the
                // encoder, and a known home makes the span absolute rather
                // than relative to wherever the encoder happened to power up.
                if (step == Step.SERVO_LIMITS && servoIdx() == 0) {
                    startHoming(false);
                    break;
                }
                if (markable(step)) {
                    double v = markableValue(step);
                    if (Double.isNaN(v)) { note = "nothing to mark here"; break; }
                    servoMark = (servoMark == null) ? v : null;
                    note = servoMark == null
                            ? "mark cleared"
                            : String.format(Locale.US,
                                "marked at %.4f -- now jog to the other stop", servoMark);
                } else {
                    note = "no Y action on this step";
                }
                break;
        }
    }

    // ---- the sign test -----------------------------------------------------
    // 10 deg was unobservable: a finger's width at the rim, held 900 ms, then
    // put straight back -- and draw() cannot run while this method sleeps, so
    // the position on screen never changed either. The test "did nothing" from
    // every angle a human could check. SERVO_SIGN is the one value no sensor
    // can report, so the move has to be announced, big, and held.
    private void runSignTest() {
        // Step toward whichever end has room, like the breakaway test. Only
        // ever stepping + meant that anywhere near the top of the band the
        // test either refused or commanded nothing -- and "commanded nothing"
        // reports as "encoder barely moved", which reads as a dead encoder.
        final double mag = SIGN_TEST_DEG / TurretController.TURRET_TRAVEL_DEG;
        int sdir = (turretPosMax() - turretPos) >= (turretPos - turretPosMin()) ? +1 : -1;
        final double dPos = sdir * mag;
        double sroom = sdir > 0 ? turretPosMax() - turretPos : turretPos - turretPosMin();
        if (sroom < mag) {
            note = String.format(Locale.US,
                    "only %.4f of room either side, need %.4f for a %.0f deg step. "
                    + "Jog back toward %.3f first.",
                    sroom, mag, SIGN_TEST_DEG, TurretController.SERVO_AT_ZERO);
            return;
        }

        posStart = turretPos;
        for (int i = 3; i > 0 && opModeIsActive(); i--) {
            telemetry.addLine("WATCH THE TURRET FROM ABOVE");
            telemetry.addLine("");
            telemetry.addData("moving " + (sdir > 0 ? "+" : "-")
                    + (int) SIGN_TEST_DEG + " deg in", i);
            telemetry.update();
            sleep(600);
        }

        encStart = robot.intakeMot.getCurrentPosition();
        turretPos = clamp01(turretPos + dPos);
        robot.turretRot.setPosition(turretPos);
        sleep(1200);

        int dTicks = robot.intakeMot.getCurrentPosition() - encStart;
        double dDeg = dTicks * 360.0 / TurretController.ENCODER_TICKS_PER_REV;

        telemetry.addLine("HOLDING AT THE FAR END");
        telemetry.addLine("");
        telemetry.addData("ticks moved", dTicks);
        telemetry.addLine("");
        telemetry.addLine(sdir > 0
                ? "Returning in 2s. CW from above = SERVO_SIGN +1."
                : "Returning in 2s. This was a MINUS step: CW = SERVO_SIGN -1.");
        telemetry.update();
        sleep(2000);

        turretPos = posStart;
        robot.turretRot.setPosition(turretPos);

        if (Math.abs(dTicks) < 20) {
            note = "encoder barely moved (" + dTicks + " ticks). Did the TURRET move? "
                 + "If it did, the encoder is not reading: check it is in the "
                 + "encoder port matching intakeMot (E3).";
            return;
        }

        // ENCODER_SIGN relates ticks to COMMANDED degrees, so it must be
        // taken relative to the direction actually stepped. Deriving it from
        // the raw tick sign was only right while the test always went +;
        // stepping - would have inverted the answer with nothing to show it.
        encoderSign = (dTicks * sdir) > 0 ? +1 : -1;
        encoderTicksPerRev = Math.abs(dTicks) * 360.0 / SIGN_TEST_DEG;
        // APPLY it, do not just record it. Step 10 computes HOME_OFFSET_DEG
        // through TurretController.ENCODER_SIGN, so leaving the measurement
        // sitting in a local field means the next step uses the old value and
        // gets the offset backwards -- with nothing on screen to say so.
        TurretController.ENCODER_SIGN = encoderSign;

        note = String.format(Locale.US,
                "%d ticks (%.1f deg). ENCODER_SIGN=%+d, implied ticks/rev=%.0f. "
                + "Now press A to record SERVO_SIGN: that was a %s step, so CW = %s.",
                dTicks, dDeg, encoderSign, encoderTicksPerRev,
                sdir > 0 ? "PLUS" : "MINUS",
                sdir > 0 ? "+1" : "-1");
    }

    // ---- per-loop service --------------------------------------------------

    private void service() {
        serviceHoming();
        serviceDirectionFallback();
        servicePwmRamp();
        if (deployRunning) {
            long since = System.currentTimeMillis() - deployStartMs;
            if (since < MechController.DEPLOY_MS) {
                robot.intakeMot.setPower(MechController.DEPLOY_POWER);
            } else {
                robot.intakeMot.setPower(0);
                if (since >= MechController.DEPLOY_MS + MechController.DEPLOY_SCOOP_DELAY_MS) {
                    robot.flowerIntake.setPosition(MechController.FLOWER_REST);
                    deployRunning = false;
                    note = "deploy complete in " + since + " ms";
                }
            }
        }
        if (feedRunning && !robot.conveyorMot.isBusy()) {
            feedRunning = false;
            note = String.format(Locale.US, "feed done: %d ticks in %d ms",
                    robot.conveyorMot.getCurrentPosition() - feedStartPos,
                    System.currentTimeMillis() - feedStartMs);
        }
        if (dipSpinning || openLoop || step == Step.SHOOTER) {
            double v = openLoop
                    ? Math.max(robot.pollenShootingMot.getVelocity(),
                               robot.nectarShootingMot.getVelocity())
                    : robot.pollenShootingMot.getVelocity();
            if (v > observedMaxTps) observedMaxTps = v;
            if (System.currentTimeMillis() - spreadResetMs > 2500) resetSpread();
            if (v < vMin) vMin = v;
            if (v > vMax) vMax = v;
        }
    }

    // ---- drawing -----------------------------------------------------------

    private void draw() {
        telemetry.addLine(String.format(Locale.US, "%d/%d  [%s]  %s",
                step.ordinal() + 1, Step.values().length, section(step), step));
        telemetry.addLine(hint(step));
        // The note goes FIRST. It is the result of whatever you just pressed,
        // and it was last: as pages grew it fell off the bottom of the Driver
        // Station, so a test could run, finish and report -- and read on the
        // robot as "the button does nothing".
        if (!note.isEmpty()) {
            telemetry.addLine("");
            telemetry.addLine("> " + note);
        }
        telemetry.addLine("");

        switch (step) {
            case DEVICES:        drawDevices();   break;
            case DIRECTION:      drawDirection(); break;
            case SERVO_LIMITS:   drawServoLimits(); break;
            case HOME:           drawHome();        break;
            case PWM_CHECK:      drawPwm();       break;
            case DIGITAL:        drawDigital();   break;
            case ENCODER:        drawEncoder();   break;
            case LIMIT_POLARITY:
            case TURRET_SIGN:
            case ENCODER_SIGN:
            case TURRET_ZERO:    drawTurretStep(); break;
            case HOOD_POLLEN:    drawHoodPollen(); break;
            case HOOD_NECTAR:    drawHoodNectar(); break;
            case SHOOTER:        drawShooter();   break;
            case COLOR:          drawColor();     break;
            case BALLS:          drawBalls();     break;
            case LIMELIGHT:      drawLimelight(); break;
            case PINPOINT:       drawPinpoint();  break;
            case DUMP:           drawDump();      break;
            default:             drawKnobs();     break;
        }

        telemetry.update();
    }

    private String hint(Step s) {
        switch (s) {
            case DEVICES:        return "everything resolved, or init would have failed";
            case DIRECTION:      return "A=fwd  B=rev  X=stop  Y=flip   (MOVES MOTORS)";
            case SERVO_LIMITS:   return "A=LOW  B=HIGH  X=clear  Y=mark (turret: Y=home)";
            case HOME:           return "dpad U/D pick row  A=home  X=abort  Y=flip side";
            case PWM_CHECK:      return "dpad U/D row  A/B sweep ends  X=halt/centre  Y=breakaway";
            case DIGITAL:        return "wave a magnet at the switch";
            case ENCODER:        return "A = zero here";
            case LIMIT_POLARITY: return "jog to home, then A";
            case TURRET_SIGN:    return "Y = run the move   A = record what you saw";
            case ENCODER_SIGN:   return "Y = run the move (reads ticks automatically)";
            case TURRET_ZERO:    return "jog to dead ahead, then A";
            case HOOD_POLLEN:
            case HOOD_NECTAR:    return "jog to 55 deg then A   Y = mark/measure";
            case DEPLOY:         return "A = run the deploy sequence  (MOVES INTAKE)";
            case SCOOP:          return "A = REST   B = SCOOP   Y = mark/measure";
            case BEDS:           return "A = command   B = next pattern";
            case INDEX:
            case FEED:           return "A = run selected row   B = stop  (MOVES BELT)";
            case SHOOTER:        return "A=pollen  B=nectar  Y=FULL POWER  X=stop";
            case DIP:            return "A = spin   B = stop   X = reset spread";
            case LIMELIGHT:      return "dpad U/D pick pipeline   A = FORCE it";
            case BALLS:          return "X = flip BGR/RGB   bumpers coarse";
            case DUMP:           return "paste into source -- NOTHING IS SAVED";
            default:             return "bumpers coarse   triggers fine";
        }
    }

    private void drawDevices() {
        telemetry.addLine("All 21 resolved, or init would have failed.");
        telemetry.addLine("");
        telemetry.addLine("4 motors  6 servos  6 colour  1 digital");
        telemetry.addLine("Webcam 1  limeLight  pinpoint  imu");
        telemetry.addLine("");
        telemetry.addLine("NOT checked here: LFMot RFMot LBMot RBMot.");
        telemetry.addLine("Those live in pedro/Constants.java and are only");
        telemetry.addLine("resolved once create() returns a Follower.");
    }

    private int dirIdx() { return sel % dirMotors.length; }
    private DcMotor dirMotor() { return dirMotors[dirIdx()]; }

    private void stopDirectionMotors() {
        for (DcMotor m : dirMotors) m.setPower(0);
        dirCmdActive = false;
    }

    // Drive the selected motor, CLOSED loop unless we already know better.
    //
    // The two modes are not interchangeable at the same number. In
    // RUN_USING_ENCODER setPower(0.2) requests 20% of max SPEED and the
    // controller supplies whatever duty that needs. In RUN_WITHOUT_ENCODER the
    // same 0.2 is 20% DUTY -- far less torque, and a built conveyor will not
    // break static friction on it. Forcing every motor open loop to work
    // around one dead encoder made conveyorMot hum at a power it had always
    // turned on.
    private void startDirection(int sign) {
        int i = dirIdx();
        DcMotor m = dirMotors[i];
        DcMotor.RunMode want = dirOpen[i]
                ? DcMotor.RunMode.RUN_WITHOUT_ENCODER
                : DcMotor.RunMode.RUN_USING_ENCODER;
        if (m.getMode() != want) m.setMode(want);

        dirZero = m.getCurrentPosition();
        dirCmdPower = sign * dirPower;
        if (dirOpen[i]) {
            dirCmdPower = sign * Math.max(dirPower, DIR_OPEN_LOOP_MIN_POWER);
        }
        m.setPower(dirCmdPower);
        dirCmdMs = System.currentTimeMillis();
        dirCmdActive = true;
        note = String.format(Locale.US, "%s at %.2f  (%s)",
                sign > 0 ? "FORWARD" : "REVERSE", Math.abs(dirCmdPower),
                dirOpen[i] ? "open loop" : "closed loop");
    }

    // A DIRECTION check must not depend on an encoder -- but it must not
    // give up closed-loop torque either. So try closed loop, and fall back
    // only on evidence: commanded, time elapsed, encoder still silent.
    // That is the nectar case (dead encoder, buzzes forever) without
    // penalising every motor whose encoder is fine.
    // Walk the turret toward the target a slice per loop. push() does the
    // commanding, so this only has to move the number -- which is also why
    // halting is just clearing the target: nothing is in flight.
    // Run the real sequence, then hand the servo back with this page's own
    // state synced to where homing left it -- otherwise the next push() would
    // yank the turret back to whatever was on screen before.
    private void serviceHoming() {
        if (!homing) return;
        turret.update();

        TurretController.HomeState st = turret.getHomeState();
        if (st == TurretController.HomeState.HOMED
                || st == TurretController.HomeState.FAILED) {
            homing = false;
            double pos = TurretController.servoPositionFor(turret.getCommandedDeg());
            turretPos = clampR(pos, turretPosMin(), turretPosMax());
            if (servoPos != null) servoPos[0] = pos;
            note = turret.getHomeMessage();
        }
    }

    private void startHoming(boolean forceSearch) {
        homing = true;
        if (forceSearch) turret.beginSearch(); else turret.beginHoming();
        note = forceSearch ? "searching for the magnet..." : "homing...";
    }

    private void servicePwmRamp() {
        if (step != Step.PWM_CHECK || pwmTarget == null) return;
        double d = pwmTarget - turretPos;
        if (Math.abs(d) <= PWM_RAMP_PER_LOOP) {
            turretPos = pwmTarget;
            pwmTarget = null;
            note = String.format(Locale.US, "reached %.4f", turretPos);
            return;
        }
        turretPos = clampR(turretPos + Math.signum(d) * PWM_RAMP_PER_LOOP,
                           turretPosMin(), turretPosMax());
    }

    // How much command error the turret needs before it moves at all.
    //
    // A servo under a geared load does not respond to arbitrarily small
    // commands: below some threshold the torque the error produces cannot
    // break static friction, and it just holds. That threshold is invisible
    // by eye -- the bumper "does nothing" and you cannot tell whether the
    // step was too small or the servo is dead. The turret has an encoder on
    // its output, so creep the command up and watch for the first real tick.
    //
    // This is a mechanical number, not a tuning constant: it should be SMALL.
    // AIM re-solves every loop and ARRIVED_TOLERANCE_DEG is 2 deg, so a
    // breakaway of 9 deg means the turret cannot aim, however good the maths.
    public static double BREAKAWAY_STEP = 0.0005;    // 0.45 deg of turret
    public static int BREAKAWAY_MAX_STEPS = 100;     // up to 45 deg

    private void runBreakawayTest() {
        int startTicks = robot.intakeMot.getCurrentPosition();
        double p0 = turretPos;
        pwmTarget = null;

        // Step toward whichever end has room. Creeping only upward meant that
        // after an A/B sweep -- which parks the turret ON a limit -- every
        // step clamped to the same value, the command never changed, and the
        // test spent twelve seconds proving nothing before reporting "no
        // movement". A test that cannot move is worse than no test: it
        // answers the question wrongly.
        int dir = (turretPosMax() - p0) >= (p0 - turretPosMin()) ? +1 : -1;
        double room = dir > 0 ? turretPosMax() - p0 : p0 - turretPosMin();
        double need = BREAKAWAY_STEP * BREAKAWAY_MAX_STEPS;
        if (room < need) {
            note = String.format(Locale.US,
                    "only %.4f of room either side, need %.4f. X to centre first.",
                    room, need);
            return;
        }

        for (int n = 1; n <= BREAKAWAY_MAX_STEPS && opModeIsActive(); n++) {
            turretPos = clampR(p0 + dir * n * BREAKAWAY_STEP, turretPosMin(), turretPosMax());
            robot.turretRot.setPosition(turretPos);
            sleep(120);

            int moved = Math.abs(robot.intakeMot.getCurrentPosition() - startTicks);
            telemetry.addLine("BREAKAWAY TEST -- creeping the command up");
            telemetry.addLine("");
            telemetry.addData("step", "%d of %d", n, BREAKAWAY_MAX_STEPS);
            telemetry.addData("command delta", "%+.4f  (%.1f deg)",
                    turretPos - p0,
                    Math.abs(turretPos - p0) * TurretController.TURRET_TRAVEL_DEG);
            telemetry.addData("encoder moved", "%d ticks   (need >20)", moved);
            telemetry.update();

            if (moved > 20) {
                double d = turretPos - p0;
                note = String.format(Locale.US,
                        "BROKE AWAY at %+.4f of command = %.1f deg of turret."
                        + " Under 1 deg is healthy; 5+ means it cannot aim.",
                        d, Math.abs(d) * TurretController.TURRET_TRAVEL_DEG);
                return;
            }
        }
        note = String.format(Locale.US,
                "no movement within %.4f (%.0f deg). Not stiction -- the turret"
                + " is jammed, or the servo is not driving this load at all.",
                BREAKAWAY_STEP * BREAKAWAY_MAX_STEPS,
                BREAKAWAY_STEP * BREAKAWAY_MAX_STEPS * TurretController.TURRET_TRAVEL_DEG);
    }

    private void serviceDirectionFallback() {
        if (step != Step.DIRECTION || !dirCmdActive) return;
        int i = dirIdx();
        if (dirOpen[i]) return;
        if (System.currentTimeMillis() - dirCmdMs < DIR_FALLBACK_MS) return;
        DcMotor m = dirMotors[i];
        if (Math.abs(m.getCurrentPosition() - dirZero) >= 10) return;

        dirOpen[i] = true;
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double p = dirCmdPower >= 0
                ? Math.max(dirCmdPower, DIR_OPEN_LOOP_MIN_POWER)
                : Math.min(dirCmdPower, -DIR_OPEN_LOOP_MIN_POWER);
        m.setPower(p);
        dirCmdPower = p;
        // Say what was OBSERVED, not what it means. Nothing moved: the
        // encoder may be dead, or the motor may not be turning at all. An
        // earlier version asserted "that encoder is not reading" and sent a
        // whole session chasing a cable, when the encoder was fine and the
        // hub's motor output was not. Turn the shaft by hand and watch delta
        // -- that separates the two in seconds.
        note = dirNames[i] + ": nothing moved in " + DIR_FALLBACK_MS
             + "ms, now OPEN LOOP at " + String.format(Locale.US, "%.2f", Math.abs(p))
             + ". Dead encoder OR stalled motor -- turn it BY HAND:"
             + " delta moves = encoder fine, so the motor is not driving.";
    }

    private void drawDirection() {
        for (int i = 0; i < dirMotors.length; i++) {
            telemetry.addLine((i == dirIdx() ? "> " : "  ")
                    + String.format(Locale.US, "%-18s %-8s %s", dirNames[i],
                            dirMotors[i].getDirection(),
                            dirOpen[i] ? "open" : "closed"));
        }
        telemetry.addLine("");
        telemetry.addData("power", "%.2f  %s", dirPower,
                dirOpen[dirIdx()] ? "= duty cycle" : "= fraction of max SPEED");
        telemetry.addData("encoder delta", dirMotor().getCurrentPosition() - dirZero);
        if (dirIdx() == 0) {
            telemetry.addLine("  intakeMot's port reads the TURRET encoder, so");
            telemetry.addLine("  it runs open loop and delta stays 0. Expected.");
        }
        telemetry.addLine("");
        telemetry.addLine("Forward means: intake pulls IN, conveyor moves");
        telemetry.addLine("TOWARD the shooter, flywheels throw FORWARD.");
        telemetry.addLine("Y is live only -- edit RobotHardware to keep it.");
    }

    // Degrees of output travel per 1.0 of servo position, per servo. Known
    // for five of six; flowerIntake has no published ratio, so it reports
    // servo units rather than a number that would read like a measurement.
    // Turret index 0 gets a coarse step well above its measured breakaway.
    // The rest keep steps fine enough to creep up on a mechanical stop.
    public static double TURRET_COARSE_STEP = 0.0300;   // 27 deg of turret
    public static double TURRET_FINE_STEP   = 0.0100;   //  9 deg

    private double coarseStep(int i) { return i == 0 ? TURRET_COARSE_STEP : 0.0100; }
    private double fineStep(int i)   { return i == 0 ? TURRET_FINE_STEP   : 0.0020; }

    private double servoDegPerUnit(int i) {
        switch (i) {
            case 0: return TurretController.TURRET_TRAVEL_DEG;   // 900
            case 1: return 1.0 / HoodMap.POLLEN_POS_PER_DEG;     // 227.8
            case 2: return 1.0 / HoodMap.NECTAR_POS_PER_DEG;     // 194.3
            case 4:
            case 5: return MechController.MAX_CONVEYOR_ROTATION; // 300
            default: return 0;                                   // flowerIntake
        }
    }

    private void drawServoLimits() {
        int i = servoIdx();
        for (int k = 0; k < servos.length; k++) {
            telemetry.addLine(String.format(Locale.US, "%s%-16s %.4f  [%s .. %s]",
                    (k == i ? "> " : "  "), servoNames[k], servoPos[k],
                    servoLo[k] == null ? "  -  " : String.format(Locale.US, "%.3f", servoLo[k]),
                    servoHi[k] == null ? "  -  " : String.format(Locale.US, "%.3f", servoHi[k])));
        }
        telemetry.addLine("");
        telemetry.addData("step", "bumpers %.4f   triggers %.4f%s",
                coarseStep(i), fineStep(i),
                i == 0 ? "   (turret: coarser, see breakaway)" : "");
        // The turret is the one servo with an encoder on its output. Show it:
        // "did it move" should not depend on anyone's eye, and a commanded
        // position that changes while the ticks do not is the whole diagnosis.
        if (i == 0) {
            int now = robot.intakeMot.getCurrentPosition();
            telemetry.addData("turret encoder", "%d ticks", now);
            // MEASURED span, from the output. This is the number to trust:
            // the commanded span above lags by the breakaway and wanders
            // between attempts.
            if (turretLoTicks != null && turretHiTicks != null) {
                double deg = Math.abs(turretHiTicks - turretLoTicks)
                        * 360.0 / TurretController.ENCODER_TICKS_PER_REV;
                telemetry.addData("MEASURED span", "%.1f deg   (%d .. %d)",
                        deg, turretLoTicks, turretHiTicks);
                telemetry.addData("vs TURRET_LIMIT_DEG", "%.0f deg of travel allowed",
                        2 * TurretController.TURRET_LIMIT_DEG);
            } else {
                telemetry.addLine("  A and B record the ENCODER too -- that span");
                telemetry.addLine("  is the one that repeats. Approach each stop");
                telemetry.addLine("  from the SAME side and let it settle first.");
            }
        }

        double dpu = servoDegPerUnit(i);
        if (servoLo[i] != null && servoHi[i] != null) {
            double span = Math.abs(servoHi[i] - servoLo[i]);
            if (dpu > 0) {
                telemetry.addData("span", "%.4f  =  %.1f deg", span, span * dpu);
            } else {
                telemetry.addData("span", "%.4f  (no ratio known)", span);
            }
        }
        if (servoMark != null) {
            double d = servoPos[i] - servoMark;
            if (dpu > 0) {
                telemetry.addData("gap from mark", "%+.4f  =  %+.1f deg", d, d * dpu);
            } else {
                telemetry.addData("gap from mark", "%+.4f", d);
            }
        }
        telemetry.addLine("");
        telemetry.addLine("CREEP up on each stop and STOP when it binds.");
        telemetry.addLine("A buzzing servo at rest is held against a stop.");
        telemetry.addLine("A records the LOW end, B the HIGH end.");
        telemetry.addLine("");
        telemetry.addLine("These limits are REFERENCE, not applied -- they");
        telemetry.addLine("tell you the room each later step has to work in.");
    }

    private void drawHome() {
        telemetry.addData("state", turret.getHomeState());
        telemetry.addData("message", turret.getHomeMessage());
        telemetry.addLine("");
        telemetry.addData("limit switch raw", robot.magneticLimitSwitch.getState()
                + (turret.isAtHomeSwitch() ? "   AT HOME" : ""));
        telemetry.addData("commanded deg", "%.1f", turret.getCommandedDeg());
        telemetry.addData("measured deg", "%.1f", turret.getMeasuredDeg());
        telemetry.addData("encoder ticks", turret.getRawEncoderTicks());
        telemetry.addLine("");
        // These three are what homing CONSUMES. If any is wrong the sequence
        // still reports HOMED -- just pointing somewhere else.
        telemetry.addData("SERVO_AT_ZERO", "%.4f", TurretController.SERVO_AT_ZERO);
        telemetry.addData("SERVO_SIGN", "%+d", TurretController.SERVO_SIGN);
        telemetry.addData("LIMIT_TRIGGERED_STATE", TurretController.LIMIT_TRIGGERED_STATE);
        telemetry.addData("HOME_OFFSET_DEG", "%.1f", TurretController.HOME_OFFSET_DEG);
        telemetry.addLine("");
        telemetry.addLine(((sel % 2) == 0 ? "> " : "  ")
                + String.format(Locale.US, "SEARCH_SPAN_DEG   %.0f", TurretController.SEARCH_SPAN_DEG));
        telemetry.addLine(((sel % 2) == 1 ? "> " : "  ")
                + String.format(Locale.US, "start pos         %.4f  (where the sweep begins)",
                        TurretController.SERVO_AT_ZERO));
        telemetry.addData("SEARCH_FIRST_DIR", "%+d   (Y flips it)",
                TurretController.SEARCH_FIRST_DIR);
        telemetry.addLine("");
        telemetry.addLine("A commands 0 and checks the switch; if it is not");
        telemetry.addLine("there it sweeps outward looking for the magnet and");
        telemetry.addLine("ADOPTS what it finds as the new SERVO_AT_ZERO.");
        telemetry.addLine("");
        telemetry.addLine("HOMED and pointing the wrong way = HOME_OFFSET_DEG,");
        telemetry.addLine("measured on step 10. FAILED = switch polarity, or");
        telemetry.addLine("the magnet is outside SEARCH_SPAN_DEG of zero.");
    }

    private void drawPwm() {
        telemetry.addLine("Does turretRot have its full 5 turns of PWM?");
        telemetry.addLine("");
        telemetry.addData("commanded", "%.4f", turretPos);
        telemetry.addLine("");
        telemetry.addLine(((sel % 2) == 0 ? "> " : "  ")
                + String.format(Locale.US, "TURRET_LIMIT_DEG   %.0f   (half the usable travel)",
                        TurretController.TURRET_LIMIT_DEG));
        telemetry.addLine(((sel % 2) == 1 ? "> " : "  ")
                + String.format(Locale.US, "TURRET_TRAVEL_DEG  %.0f   (deg per 1.0 of servo)",
                        TurretController.TURRET_TRAVEL_DEG));
        telemetry.addLine("");
        telemetry.addData("band", "%.3f .. %.3f  -> %.0f deg of sweep",
                turretPosMin(), turretPosMax(),
                2 * TurretController.TURRET_LIMIT_DEG);
        telemetry.addLine("A to B should move the turret by that figure.");
        telemetry.addLine("If it moves HALF, halve TURRET_TRAVEL_DEG.");
        telemetry.addLine("");
        telemetry.addData("ramping to", pwmTarget == null
                ? "-- idle --" : String.format(Locale.US, "%.4f", pwmTarget));
        telemetry.addLine("");
        telemetry.addLine("ANY BUTTON HALTS IT. X halts without moving further.");
        telemetry.addLine("Then dpad to jog it back out by hand.");
        telemetry.addLine("");
        telemetry.addLine("A then B sweeps the full TURRET band, not the full");
        telemetry.addLine("servo band -- 0.0/1.0 would be +/-450 deg into a stop.");
        telemetry.addLine("");
        telemetry.addLine("Turret should sweep 380 deg total. If it falls well");
        telemetry.addLine("short, the PWM range is not widened and every turret");
        telemetry.addLine("angle below will be wrong.");
    }

    private void drawDigital() {
        telemetry.addData("magneticLimitSwitch", robot.magneticLimitSwitch.getState());
        telemetry.addLine("");
        telemetry.addLine("Reading WITH the magnet = LIMIT_TRIGGERED_STATE.");
        telemetry.addLine("Record it on the next step.");
    }

    // Has the encoder EVER moved while this page was open? At power-up
    // RobotHardware does STOP_AND_RESET_ENCODER on intakeMot, so a healthy
    // stationary encoder also reads 0 -- "0" alone proves nothing, and the
    // only honest test is whether it ever changes. Latching min/max means you
    // do not have to be watching at the moment it does.
    private int encMin = Integer.MAX_VALUE, encMax = Integer.MIN_VALUE;

    private void resetEnc() { encMin = Integer.MAX_VALUE; encMax = Integer.MIN_VALUE; }

    private void drawEncoder() {
        int raw = robot.intakeMot.getCurrentPosition();
        int delta = raw - encZero;
        if (raw < encMin) encMin = raw;
        if (raw > encMax) encMax = raw;
        telemetry.addLine("Through Bore reads via the intakeMot port (E3),");
        telemetry.addLine("so its encoder lead must be in ENCODER port E3.");
        telemetry.addLine("");
        telemetry.addData("raw ticks", raw);
        telemetry.addData("since zero", delta);
        telemetry.addData("implied turret deg", "%.1f",
                delta * 360.0 / TurretController.ENCODER_TICKS_PER_REV);
        telemetry.addLine("");
        int seen = (encMax >= encMin) ? (encMax - encMin) : 0;
        telemetry.addData("EVER MOVED", seen == 0
                ? "NO -- not once since this page opened"
                : String.format(Locale.US, "yes, %d ticks  (%.1f deg)",
                        seen, seen * 360.0 / TurretController.ENCODER_TICKS_PER_REV));
        telemetry.addLine("");
        telemetry.addLine("Raw 0 proves NOTHING: init resets this encoder, so a");
        telemetry.addLine("healthy stationary one also reads 0. Movement is the");
        telemetry.addLine("only test, and ONE TICK is enough.");
        telemetry.addLine("");
        telemetry.addLine("Turn the turret any amount -- by hand if it is free,");
        telemetry.addLine("or jog it on step 3 and come back. Still NO after real");
        telemetry.addLine("movement = the through-bore is not in ENCODER port E3.");
        telemetry.addLine("");
        telemetry.addLine("On the TURRET axis 1:1 -- one revolution is 8192.");
    }

    private void drawTurretStep() {
        telemetry.addData("turret servo pos", "%.4f", turretPos);
        telemetry.addData("turret deg (if zero set)", "%.1f",
                (turretPos - (turretZero == null ? TurretController.SERVO_AT_ZERO : turretZero))
                        * TurretController.TURRET_TRAVEL_DEG);
        telemetry.addData("limit switch raw", robot.magneticLimitSwitch.getState()
                + (robot.magneticLimitSwitch.getState() == TurretController.LIMIT_TRIGGERED_STATE
                        ? "   AT SWITCH" : ""));
        telemetry.addData("encoder ticks", robot.intakeMot.getCurrentPosition());

        if (step == Step.TURRET_ZERO) {
            telemetry.addLine("");
            if (homeMarkTicks == null) {
                telemetry.addLine("HOME_OFFSET_DEG, measured in two presses:");
                telemetry.addLine("  1. jog until 'limit switch raw' flips, press Y");
                telemetry.addLine("  2. jog to TRUE FORWARD, press A");
            } else {
                // Live, so the number is visible while you are still jogging.
                double deg = TurretController.ENCODER_SIGN
                        * (homeMarkTicks - robot.intakeMot.getCurrentPosition())
                        * 360.0 / TurretController.ENCODER_TICKS_PER_REV;
                telemetry.addData("switch is", "%+.1f deg from here", deg);
                telemetry.addLine("Jog until the turret points TRUE FORWARD,");
                telemetry.addLine("then press A to record that figure.");
            }
        }

        telemetry.addLine("");
        telemetry.addLine("dpad U/D = 0.45 deg   bumpers = 9 deg");
        telemetry.addLine("The turret does NOT back-drive. Jog it, never force it.");
        telemetry.addLine("");
        results();
    }

    private void drawHoodPollen() {
        double far = pollenHoodPos + HoodMap.POLLEN_SIGN * 25.0 * HoodMap.POLLEN_POS_PER_DEG;
        telemetry.addData("pollen hood pos", "%.4f", pollenHoodPos);
        telemetry.addData("if this is 55 deg, 80 deg is", "%.4f  %s",
                far, (far >= 0 && far <= 1) ? "OK" : "OUT OF RANGE");
        telemetry.addData("sweep needs", "%.4f  (sign %+d)",
                25.0 * HoodMap.POLLEN_POS_PER_DEG, HoodMap.POLLEN_SIGN);
        drawLimitGuard(1, "at 55 deg", pollenHoodPos);
        drawLimitGuard(1, "at 80 deg", far);
        drawMark(Step.HOOD_POLLEN);
        telemetry.addLine("");
        results();
    }

    private void drawHoodNectar() {
        double far = nectarHoodPos + HoodMap.NECTAR_SIGN * 25.0 * HoodMap.NECTAR_POS_PER_DEG;
        telemetry.addData("nectar hood pos", "%.4f", nectarHoodPos);
        telemetry.addData("if this is 55 deg, 80 deg is", "%.4f  %s",
                far, (far >= 0 && far <= 1) ? "OK" : "OUT OF RANGE");
        telemetry.addData("sweep needs", "%.4f  (sign %+d)",
                25.0 * HoodMap.NECTAR_POS_PER_DEG, HoodMap.NECTAR_SIGN);
        drawLimitGuard(2, "at 55 deg", nectarHoodPos);
        drawLimitGuard(2, "at 80 deg", far);
        drawMark(Step.HOOD_NECTAR);
        telemetry.addLine("");
        results();
    }

    // The position a mark refers to on this step. For the hoods it is the
    // live commanded position; for the knob pages it is the selected knob,
    // because that is what the bumpers are moving.
    private double markableValue(Step s) {
        switch (s) {
            case HOOD_POLLEN: return pollenHoodPos;
            case HOOD_NECTAR: return nectarHoodPos;
            default: {
                Knob[] k = knobs(s);
                return k.length == 0 ? Double.NaN : k[sel % k.length].get.get();
            }
        }
    }

    // Degrees of OUTPUT travel per 1.0 of servo position, where a ratio is
    // known. The hoods come off HoodMap's gear ratios; the beds are already
    // tuned in degrees so their knobs need no conversion. The scoop has no
    // published ratio, so it reports servo units only rather than inventing a
    // number that would read like a measurement.
    private double degPerUnit(Step s) {
        switch (s) {
            case HOOD_POLLEN: return 1.0 / HoodMap.POLLEN_POS_PER_DEG;   // 227.8
            case HOOD_NECTAR: return 1.0 / HoodMap.NECTAR_POS_PER_DEG;   // 194.3
            default:          return 0;
        }
    }

    private boolean markable(Step s) {
        return s == Step.HOOD_POLLEN || s == Step.HOOD_NECTAR || s == Step.SCOOP;
    }

    private void drawMark(Step s) {
        if (!markable(s)) return;
        double now = markableValue(s);
        telemetry.addLine("");
        if (servoMark == null) {
            telemetry.addLine("Y marks a reference here. Jog to the other stop");
            telemetry.addLine("and the gap is shown live, in units and degrees.");
            return;
        }
        double d = now - servoMark;
        double dpu = degPerUnit(s);
        if (dpu > 0) {
            telemetry.addData("gap from mark", "%+.4f   =  %+.1f deg", d, d * dpu);
        } else {
            telemetry.addData("gap from mark", "%+.4f   (no ratio known)", d);
        }
        telemetry.addData("mark was at", "%.4f", servoMark);
    }

    // Step 3 surveys the room; steps 5, 6, 9 and 10 choose endpoints inside
    // it. Nothing connected the two, so the survey was decoration: you could
    // record that the scoop stops at 0.08 and then set FLOWER_SCOOP to 0.05
    // with no complaint, and find out by listening to a servo stall.
    //
    // Returns "" when there is nothing to say -- no limits recorded, or the
    // value is comfortably inside them.
    private String limitWarn(int idx, double v) {
        if (idx < 0 || idx >= servos.length) return "";
        Double lo = servoLo[idx], hi = servoHi[idx];
        if (lo == null || hi == null) return "";
        double a = Math.min(lo, hi), b = Math.max(lo, hi);
        if (v < a) return String.format(Locale.US, "  !! %.4f past the LOW stop %.4f", v, a);
        if (v > b) return String.format(Locale.US, "  !! %.4f past the HIGH stop %.4f", v, b);
        double margin = Math.min(v - a, b - v);
        if (margin < 0.010) return String.format(Locale.US, "  (only %.4f from a stop)", margin);
        return "";
    }

    private void drawLimitGuard(int idx, String label, double v) {
        if (servoLo[idx] == null || servoHi[idx] == null) {
            telemetry.addLine("  " + label + ": no limits recorded on step 3");
            return;
        }
        telemetry.addData(label, "%.4f  in [%.4f .. %.4f]%s", v,
                Math.min(servoLo[idx], servoHi[idx]),
                Math.max(servoLo[idx], servoHi[idx]),
                limitWarn(idx, v));
    }

    private void results() {
        telemetry.addLine("results so far:");
        telemetry.addLine("  LIMIT_TRIGGERED_STATE = " + show(limitAtHome));
        telemetry.addLine("  SERVO_SIGN            = " + show(turretSign));
        telemetry.addLine("  ENCODER_SIGN          = " + show(encoderSign));
        telemetry.addLine("  ticks/rev measured    = " + show(encoderTicksPerRev));
        telemetry.addLine("  SERVO_AT_ZERO         = " + show(turretZero));
        telemetry.addLine("  HOME_OFFSET_DEG       = " + show(homeOffsetDeg));
        telemetry.addLine("  POLLEN_POS_AT_55      = " + show(pollenAt55));
        telemetry.addLine("  NECTAR_POS_AT_55      = " + show(nectarAt55));
    }

    private void drawKnobs() {
        Knob[] k = knobs(step);
        for (int i = 0; i < k.length; i++) {
            telemetry.addLine(String.format(Locale.US, "%s%-24s %s",
                    (i == sel % Math.max(1, k.length) ? "> " : "  "), k[i].name, k[i].value()));
        }
        telemetry.addLine("");
        live(step);
        drawMark(step);
    }

    private void live(Step s) {
        switch (s) {
            case BEDS:
                for (int i = 0; i < BED_NAMES.length; i++) {
                    telemetry.addLine((i == (bedPattern % 4) ? "> " : "  ") + BED_NAMES[i]);
                }
                telemetry.addLine("");
                telemetry.addData("pollen eng/dis", "%.3f / %.3f",
                        MechController.bedPollenPos(true), MechController.bedPollenPos(false));
                telemetry.addData("nectar eng/dis", "%.3f / %.3f",
                        MechController.bedNectarPos(true), MechController.bedNectarPos(false));
                telemetry.addData("travel each", "%.3f / %.3f",
                        Math.abs(MechController.bedPollenPos(true) - MechController.bedPollenPos(false)),
                        Math.abs(MechController.bedNectarPos(true) - MechController.bedNectarPos(false)));
                drawLimitGuard(4, "pollen engaged", MechController.bedPollenPos(true));
                drawLimitGuard(4, "pollen disengd", MechController.bedPollenPos(false));
                drawLimitGuard(5, "nectar engaged", MechController.bedNectarPos(true));
                drawLimitGuard(5, "nectar disengd", MechController.bedNectarPos(false));
                telemetry.addLine("Each bed has its OWN two positions now, so the");
                telemetry.addLine("travels need not match -- they are 0.190 and");
                telemetry.addLine("0.070 on this robot. What matters is that each");
                telemetry.addLine("bed REACHES both of its own positions.");
                break;
            case SCOOP: {
                double span = Math.abs(MechController.FLOWER_SCOOP - MechController.FLOWER_REST);
                drawLimitGuard(3, "FLOWER_REST ", MechController.FLOWER_REST);
                drawLimitGuard(3, "FLOWER_SCOOP", MechController.FLOWER_SCOOP);
                telemetry.addData("servo now at", scoopAtPickup ? "SCOOP" : "REST");
                telemetry.addData("span", "%.3f%s", span,
                        span < 0.05 ? "   !! barely moves"
                                    : span > 0.80 ? "   !! near full travel" : "");
                telemetry.addLine("Creep out, STOP when the scoop hits its stop.");
                telemetry.addLine("A buzzing servo at rest is held against a stop.");
                break;
            }
            case FEED:
            case INDEX:
                telemetry.addData("conveyor", robot.conveyorMot.getCurrentPosition());
                telemetry.addData("isBusy", robot.conveyorMot.isBusy());
                telemetry.addLine("A runs the SELECTED row's distance.");
                telemetry.addLine(s == Step.FEED
                        ? "Right value = exactly one ball leaves."
                        : "Right value = one ball clears the intake, no further.");
                telemetry.addLine("isBusy must clear inside ~1s, or every shot");
                telemetry.addLine("times out instead of completing.");
                break;
            case DIP: {
                double tps = robot.pollenShootingMot.getVelocity();
                double target = RobotHardware.rpmToTicksPerSec(dipRpm);
                double trip = target * (1.0 - MechController.DIP_ENTER);
                double ball = target * 0.031;
                double spread = (vMax >= vMin) ? (vMax - vMin) : 0;
                telemetry.addData("pollen", "%.0f ticks/s  of  %.0f", tps, target);
                telemetry.addData("spread (2.5s)", "%.0f   [%.0f .. %.0f]", spread, vMin, vMax);
                telemetry.addData("trip / ball", "%.0f / %.0f ticks/s", trip, ball);
                telemetry.addLine(spread >= trip
                        ? "!! spread >= trip: NOISE WILL COUNT AS BALLS"
                        : (trip >= ball
                            ? "!! trip >= ball: a real ball will be MISSED"
                            : "usable: spread < trip < ball"));
                break;
            }
            case TURRET:
                telemetry.addData("home switch", robot.magneticLimitSwitch.getState());
                telemetry.addData("turret encoder", robot.intakeMot.getCurrentPosition());
                telemetry.addData("commanded / measured", "%.1f / %.1f deg",
                        turret.getCommandedDeg(), turret.getMeasuredDeg());
                telemetry.addData("slip", "%.1f deg", turret.getSlipDeg());
                telemetry.addData("servo trim", "%+.4f%s", turret.getServoTrim(),
                        turret.isTrimSaturated() ? "   SATURATED" : "");
                telemetry.addLine("");
                telemetry.addLine("ENCODER_COUPLED stays 0 until the encoder");
                telemetry.addLine("actually turns with the turret.");
                telemetry.addLine("");
                telemetry.addLine("CLOSED_LOOP trims the command until the ENCODER");
                telemetry.addLine("agrees -- it corrects slip and deadband, which");
                telemetry.addLine("an open loop cannot even see. Needs both signs");
                telemetry.addLine("correct first: a wrong ENCODER_SIGN drives the");
                telemetry.addLine("wrong way and shows SATURATED.");
                telemetry.addLine("");
                telemetry.addLine("Watch slip go to 0 after a move. If it grows,");
                telemetry.addLine("flip ENCODER_SIGN on step 9.");
                break;
            case DEPLOY:
                telemetry.addLine("A runs the real sequence once.");
                telemetry.addLine("deployDone latches on a TIMER, so the arm");
                telemetry.addLine("clearing proves nothing -- listen for it.");
                break;
            case RANGE:
                telemetry.addLine("INTAKE <= DEPLOY < RETRACT, or bands overlap.");
                telemetry.addLine("Built around FLOWER_COLLECT_IN = 14.535 in,");
                telemetry.addLine("measured robot CENTRE to flower centre.");
                if (MechController.FLOWER_INTAKE_IN > MechController.FLOWER_DEPLOY_IN
                        || MechController.FLOWER_DEPLOY_IN >= MechController.FLOWER_RETRACT_IN) {
                    telemetry.addLine("!! ORDER IS WRONG -- scoop behaviour undefined");
                }
                break;
            case FLOWERSHOT:
                telemetry.addLine("Setpoints for SHOOTING_FLOWER_POLLEN/NECTAR.");
                telemetry.addLine("Fixed values, no range table -- you are");
                telemetry.addLine("standing at the flower you shoot into.");
                break;
            case FIRE:
                telemetry.addLine("FIRE_ONLY_WHEN_STEADY holds fire until the");
                telemetry.addLine("robot settles. Leave it 1 for the first");
                telemetry.addLine("session; raise the thresholds only once shots");
                telemetry.addLine("land reliably from a stop.");
                break;
            default: break;
        }
    }

    private void drawShooter() {
        double pTps = robot.pollenShootingMot.getVelocity();
        double nTps = robot.nectarShootingMot.getVelocity();
        String[] lbl = { "target rpm", "P", "I", "D", "F" };
        double[] val = { targetRpm, kP, kI, kD, kF };
        for (int i = 0; i < lbl.length; i++) {
            telemetry.addLine(String.format(Locale.US, "%s%-10s %.2f",
                    (i == sel % 5 ? "> " : "  "), lbl[i], val[i]));
        }
        telemetry.addLine("");
        telemetry.addData("pollen", "%.0f ticks/s = %.0f rpm", pTps,
                RobotHardware.ticksPerSecToRpm(pTps));
        telemetry.addData("nectar", "%.0f ticks/s = %.0f rpm", nTps,
                RobotHardware.ticksPerSecToRpm(nTps));
        telemetry.addData("commanded", "%.0f ticks/s",
                RobotHardware.rpmToTicksPerSec(targetRpm));
        telemetry.addLine("");
        // A number beats eyeballing a jittery readout. Real hunting is a wide
        // spread AND an audible surge; a steady hum with a wide spread is just
        // encoder quantisation at 28 cpr. This sets DIP_ENTER on the next step.
        telemetry.addData("spread (2s)", "%.0f ticks/s   [%.0f .. %.0f]  %s",
                (vMax - vMin), vMin, vMax,
                (vMax - vMin) < 30 ? "steady" : (vMax - vMin) > 100 ? "HUNTING" : "");
        if (openLoop) {
            telemetry.addLine("");
            telemetry.addData("free-run max", "%.0f ticks/s  (~2800 = 28 cpr)", observedMaxTps);
            if (observedMaxTps > 100) {
                telemetry.addData("suggested", "F %.2f   P %.2f   I 0.1xP   D 0",
                        32767.0 / observedMaxTps, 0.1 * 32767.0 / observedMaxTps);
            }
            telemetry.addLine("Note the BATTERY VOLTAGE: free speed scales with");
            telemetry.addLine("it, and an F from a flat pack over-drives a good one.");
        }
    }

    private void drawColor() {
        RevColorSensorV3[] s = { robot.sensorIntakeL, robot.sensorIntakeR,
                                 robot.sensorPollenL, robot.sensorPollenR,
                                 robot.sensorNectarL, robot.sensorNectarR };
        String[] n = { "intakeL", "intakeR", "pollenL", "pollenR", "nectarL", "nectarR" };
        int i = sel % s.length;
        telemetry.addLine("Hold a ball against the selected sensor.");
        telemetry.addLine("");
        telemetry.addLine("> " + n[i]);
        double dist = s[i].getDistance(DistanceUnit.MM);
        int r = s[i].red(), g = s[i].green(), bl = s[i].blue();
        int max = Math.max(r, Math.max(g, bl));
        float[] hsv = new float[3];
        if (max > 0) {
            android.graphics.Color.RGBToHSV(r * 255 / max, g * 255 / max, bl * 255 / max, hsv);
        }
        telemetry.addData("distance", "%.1f mm  (ball if < 20)", dist);
        telemetry.addData("rgb", "%d / %d / %d", r, g, bl);
        telemetry.addData("hsv", "%.1f / %.2f / %.2f", hsv[0], hsv[1], hsv[2]);
        telemetry.addLine("");
        telemetry.addLine("expect H: yellow 80-110  red 15-35  blue 215-240");
        telemetry.addLine("Covering one sensor proves the NAME maps to it.");
    }

    private void drawBalls() {
        BallDetector bd = vision.getBallDetector();
        if (bd == null) { telemetry.addLine("camera not started"); return; }

        BallDetector.BallType[] types = BallDetector.BallType.values();
        BallDetector.BallType t = types[ballTypeIdx % types.length];
        double[] g = BallDetector.getGate(t);
        double[] c = bd.getCentreHsv();
        double[] raw = bd.getCentreRaw();

        // Measuring the ball beats guessing the gate. Hue is 0-179 in OpenCV,
        // so halve any 0-360 figure you look up.
        telemetry.addData("centre HSV", "H %.0f  S %.0f  V %.0f", c[0], c[1], c[2]);
        // Channel-order check. On a YELLOW ball ch0 and ch1 must both be high
        // and ch2 low; if ch0 is low and ch2 high the frame is BGR and the hue
        // above is meaningless, not merely mis-gated.
        telemetry.addData("centre raw", "%.0f/%.0f/%.0f  as %s%s",
                raw[0], raw[1], raw[2],
                BallDetector.FRAME_IS_BGR ? "BGR" : "RGB",
                (raw[0] + raw[1] + raw[2] > 30 && raw[2] > raw[0] + 40)
                        ? "   !! looks SWAPPED, X to flip" : "");
        telemetry.addLine("");
        String[] lbl = { "colour", "H low", "H high", "S min", "V min" };
        double[] val = { ballTypeIdx, g[0], g[1], g[2], g[3] };
        for (int i = 0; i < lbl.length; i++) {
            String v = (i == 0) ? t.toString() : String.format(Locale.US, "%.0f", val[i]);
            telemetry.addLine((i == sel % 5 ? "> " : "  ") + String.format("%-7s %s", lbl[i], v));
        }
        telemetry.addLine("");
        List<BallDetector.Detection> ds = bd.getDetections();
        int p = 0, r = 0, b = 0;
        for (BallDetector.Detection d : ds) {
            if (d.type == BallDetector.BallType.POLLEN) p++;
            else if (d.type == BallDetector.BallType.NECTAR_RED) r++; else b++;
        }
        telemetry.addData("seen", "pollen %d   red %d   blue %d", p, r, b);
        if (!ds.isEmpty()) {
            BallDetector.Detection nd = ds.get(0);
            telemetry.addData("nearest", "%s  %.1f in  %+.1f deg  %.0f px",
                    nd.type, nd.rangeIn, nd.bearingDeg, nd.areaPx);
        } else {
            telemetry.addLine("nearest  none");
        }
        telemetry.addData("fov", "H %.1f  V %.1f deg", bd.hfovDeg(), bd.vfovDeg());
    }

    private void drawLimelight() {
        LLResult res = robot.limeLight.getLatestResult();
        telemetry.addData("index to force", sel % 10);
        telemetry.addLine("");
        telemetry.addData("connected", vision.isLimelightConnected());
        telemetry.addData("last update", "%.2f s ago", vision.limelightAgeSec());
        telemetry.addData("commanded pipeline", vision.getCurrentPipeline());
        if (res == null) {
            telemetry.addLine("NO RESULT -- the Limelight is not responding.");
            telemetry.addLine("  config name exactly 'limeLight', and its USB 3");
            telemetry.addLine("  lead seated. C270 shares the bus on USB 2, so");
            telemetry.addLine("  a hub or power problem takes out both.");
            return;
        }
        telemetry.addData("running pipeline", res.getPipelineIndex());
        // valid / tx / ty describe a PRIMARY TARGET and mean nothing for tag
        // reading: an AprilTag pipeline reports IDs with valid=false, tx=ty=0.
        telemetry.addData("valid / tx / ty", "%s  %.1f / %.1f   (not used)",
                res.isValid(), res.getTx(), res.getTy());
        List<LLResultTypes.FiducialResult> tags = res.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            telemetry.addLine("RESULT but NO FIDUCIALS -- check the pipeline is");
            telemetry.addLine("APRILTAG 36h11 and the ID filter is open.");
        } else {
            StringBuilder ids = new StringBuilder();
            for (LLResultTypes.FiducialResult f : tags) ids.append(f.getFiducialId()).append(" ");
            telemetry.addData("tag ids", ids.toString());
        }
        telemetry.addLine("");
        telemetry.addData("filtered target", "%s   (%d ours in view)",
                vision.getRaisedTarget(), vision.getVisibleTagCount());
        telemetry.addLine("RED 30-33 scoring / 34-37 audience");
        telemetry.addLine("BLUE 42-45 scoring / 38-41 audience");
    }

    private void drawPinpoint() {
        // The Pinpoint latches and only refreshes on update(). On the robot
        // Pedro owns that call, which is why RobotHardware does not make it --
        // but nothing else does here, so without this the readings sit at 0
        // forever and look like dead hardware.
        robot.pinpoint.update();

        // RAW COUNTS ARE THE DIAGNOSTIC. X/Y in inches depend on encoder
        // resolution and pod offsets, neither configured yet, so they can read
        // 0 on perfectly good hardware.
        telemetry.addLine("Spin each pod by hand: raw counts must move.");
        telemetry.addLine("");
        telemetry.addData("device status", robot.pinpoint.getDeviceStatus());
        telemetry.addData("raw pod X / Y", "%d / %d",
                robot.pinpoint.getEncoderX(), robot.pinpoint.getEncoderY());
        telemetry.addData("heading  pp / imu", "%.1f / %.1f deg",
                robot.pinpoint.getHeading(AngleUnit.DEGREES),
                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addLine("");
        robot.addPinpointTelemetry(telemetry);
        telemetry.addLine("");
        telemetry.addLine("Offsets, resolution and directions go in");
        telemetry.addLine("pedro/Constants.java -- not dumped here.");
    }

    private void drawDump() {
        telemetry.addLine("NOTHING IS SAVED. Paste these into source.");
        telemetry.addLine("'back' also writes them to the log.");
        telemetry.addLine("");
        for (String s : dumpLines()) telemetry.addLine(s);
    }

    // ---- helpers -----------------------------------------------------------

    private static final String[] BED_NAMES = {
            "BOTH DISENGAGED", "BOTH ENGAGED", "POLLEN engaged", "NECTAR engaged" };

    private void driveBeds() {
        boolean p, nn;
        switch (bedPattern % 4) {
            case 1:  p = true;  nn = true;  break;
            case 2:  p = true;  nn = false; break;
            case 3:  p = false; nn = true;  break;
            default: p = false; nn = false; break;
        }
        robot.conveyorPollen.setPosition(MechController.bedPollenPos(p));
        robot.conveyorNectar.setPosition(MechController.bedNectarPos(nn));
    }

    private void driveScoop() {
        robot.flowerIntake.setPosition(scoopAtPickup
                ? MechController.FLOWER_SCOOP : MechController.FLOWER_REST);
    }

    private void startFeed() {
        switch (sel % 4) {
            case 1:  runBelt(MechController.TICKS_TO_SHOOT_NECTAR); break;
            case 2:  runBelt(MechController.TICKS_TO_SHOOT_BOTH);   break;
            case 3:  note = "that row is belt POWER -- select a distance"; break;
            default: runBelt(MechController.TICKS_TO_SHOOT_POLLEN); break;
        }
    }

    private void startIndex() {
        switch (sel % 4) {
            case 0:  runBelt(MechController.TICKS_PER_POLLEN_STORAGE); break;
            case 1:  runBelt(MechController.TICKS_PER_NECTAR_STORAGE); break;
            default: note = "that row is a timing -- select a distance"; break;
        }
    }

    private void runBelt(int ticks) {
        if (ticks <= 0) { note = "that distance is 0 -- nothing to run"; return; }
        feedTargetTicks = ticks;
        feedStartPos = robot.conveyorMot.getCurrentPosition();
        robot.conveyorMot.setTargetPosition(feedStartPos + feedTargetTicks);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // The tunable, not a copy -- a page running at a different power than
        // the robot would pass while the robot hunted.
        robot.conveyorMot.setPower(MechController.CONVEYOR_POWER);
        feedStartMs = System.currentTimeMillis();
        feedRunning = true;
        note = "running " + feedTargetTicks + " ticks";
    }

    private void stopFeed() {
        robot.conveyorMot.setPower(0);
        robot.conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feedRunning = false;
    }

    private void stopDip() {
        robot.pollenShootingMot.setVelocity(0);
        robot.pollenShootingMot.setPower(0);
        dipSpinning = false;
    }

    private void stopShooters() {
        robot.pollenShootingMot.setPower(0);
        robot.nectarShootingMot.setPower(0);
        robot.pollenShootingMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.nectarShootingMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pollenShootingMot.setVelocity(0);
        robot.nectarShootingMot.setVelocity(0);
        openLoop = false;
    }

    private void stopEverything() {
        robot.intakeMot.setPower(0);
        deployRunning = false;
        stopDirectionMotors();
        stopFeed();
        stopDip();
        stopShooters();
    }

    private void applyPidf() {
        robot.pollenShootingMot.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        robot.nectarShootingMot.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        note = String.format(Locale.US, "PIDF %.2f / %.2f / %.2f / %.2f", kP, kI, kD, kF);
    }

    private void resetSpread() {
        vMin = 1e9; vMax = -1e9; spreadResetMs = System.currentTimeMillis();
    }

    // ---- dump --------------------------------------------------------------
    // One dump for the whole session. The three old tools each had their own,
    // so stopping the wrong OpMode lost half a session's measurements.

    private String[] dumpLines() {
        ArrayList<String> out = new ArrayList<>();
        out.add("// --- MechController ---");
        for (Step p : new Step[] { Step.DEPLOY, Step.SCOOP, Step.BEDS, Step.INDEX,
                                   Step.FEED, Step.RANGE, Step.FLOWERSHOT,
                                   Step.FIRE, Step.DIP }) {
            for (Knob k : knobs(p)) {
                if (!k.owner.equals("MechController")) continue;
                if (k.name.equals("FIRE_ONLY_WHEN_STEADY")) continue;   // printed below
                out.add(String.format(Locale.US, "%s = %s;", k.name, bare(k)));
            }
        }
        out.add("FIRE_ONLY_WHEN_STEADY = " + MechController.FIRE_ONLY_WHEN_STEADY + ";");
        out.add("// --- TurretController ---");
        for (Knob k : knobs(Step.TURRET)) {
            if (k.name.equals("ENCODER_COUPLED")) {
                out.add("ENCODER_COUPLED = " + TurretController.ENCODER_COUPLED + ";");
            } else if (k.name.equals("CLOSED_LOOP")) {
                out.add("CLOSED_LOOP = " + TurretController.CLOSED_LOOP + ";");
            } else {
                out.add(String.format(Locale.US, "%s = %s;", k.name, bare(k)));
            }
        }
        out.add(String.format(Locale.US, "TURRET_LIMIT_DEG = %.1f;  TURRET_TRAVEL_DEG = %.1f;",
                TurretController.TURRET_LIMIT_DEG, TurretController.TURRET_TRAVEL_DEG));
        out.add("SEARCH_SPAN_DEG = " + (int) TurretController.SEARCH_SPAN_DEG
                + ";  SEARCH_FIRST_DIR = " + TurretController.SEARCH_FIRST_DIR + ";");
        out.add("SERVO_AT_ZERO = " + show(turretZero) + ";");
        out.add("HOME_OFFSET_DEG = " + show(homeOffsetDeg) + ";");
        out.add("SERVO_SIGN = " + show(turretSign) + ";");
        out.add("ENCODER_SIGN = " + show(encoderSign) + ";");
        out.add("LIMIT_TRIGGERED_STATE = " + show(limitAtHome) + ";");
        out.add("ENCODER_TICKS_PER_REV = " + show(encoderTicksPerRev) + ";");
        out.add("// --- HoodMap ---");
        out.add("POLLEN_POS_AT_55 = " + show(pollenAt55) + ";");
        out.add("NECTAR_POS_AT_55 = " + show(nectarAt55) + ";");
        out.add("// --- RobotHardware (shooter PIDF, step 19) ---");
        out.add(String.format(Locale.US,
                "SHOOTER_KP = %.2f; SHOOTER_KI = %.2f; SHOOTER_KD = %.2f; SHOOTER_KF = %.2f;",
                kP, kI, kD, kF));
        out.add("// --- BallDetector HSV gates (step 26) ---");
        for (BallDetector.BallType t : BallDetector.BallType.values()) {
            double[] g = BallDetector.getGate(t);
            out.add(String.format(Locale.US, "// %s  H %.0f-%.0f  S>%.0f  V>%.0f",
                    t, g[0], g[1], g[2], g[3]));
        }
        out.add("FRAME_IS_BGR = " + BallDetector.FRAME_IS_BGR + ";");
        out.add("// --- not dumped: motor directions (edit RobotHardware),");
        out.add("//     Pinpoint offsets (pedro/Constants), ShooterRed/Blue");
        out.add("//     efficiency (ShooterCalibration).");
        return out.toArray(new String[0]);
    }

    private String bare(Knob k) {
        double v = k.get.get();
        boolean isInt = k.fmt.startsWith("%.0f");
        return isInt ? String.valueOf((long) v) : String.format(Locale.US, "%.4f", v);
    }

    private void dumpToLog() {
        for (String s : dumpLines()) android.util.Log.i("RobotSetup", s);
    }

    private static String show(Object o) {
        if (o == null) return "<not set>";
        if (o instanceof Double) return String.format(Locale.US, "%.4f", (Double) o);
        if (o instanceof Integer) return String.format(Locale.US, "%+d", (Integer) o);
        return o.toString();
    }

    private static double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
    private static double clampR(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    private static Step next(Step s) { Step[] v = Step.values(); return v[(s.ordinal()+1)%v.length]; }
    private static Step prev(Step s) { Step[] v = Step.values(); return v[(s.ordinal()+v.length-1)%v.length]; }
}
