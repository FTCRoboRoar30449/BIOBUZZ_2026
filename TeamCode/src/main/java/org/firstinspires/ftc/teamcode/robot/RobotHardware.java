package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

    public final DcMotor intakeMot;
    public final DcMotor conveyorMot;

    public final DcMotorEx nectarShootingMot;
    public final DcMotorEx pollenShootingMot;

    public final Servo turretRot;
    public final Servo flowerIntake;
    public final Servo conveyorPollen;
    public final Servo conveyorNectar;
    public final Servo hoodPollen;
    public final Servo hoodNectar;

    public final GoBildaPinpointDriver pinpoint;
    public final IMU imu;
    public final Telemetry telemetry;

    public final RevColorSensorV3 sensorIntakeR;
    public final RevColorSensorV3 sensorIntakeL;
    public final RevColorSensorV3 sensorNectarL;
    public final RevColorSensorV3 sensorNectarR;
    public final RevColorSensorV3 sensorPollenL;
    public final RevColorSensorV3 sensorPollenR;

    public final DigitalChannel magneticLimitSwitch;
    public final WebcamName camera;
    public final Limelight3A limeLight;

    public static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Measured 14 Nov with the flywheels fitted. Free run gave 2240 ticks/s
    // at 11.55 V, so F = 32767/2240 = 14.63 AT THAT VOLTAGE.
    //
    // F is voltage-dependent, and a match spans a wide range of it:
    //     11.55 V -> 2240 ticks/s -> F 14.63
    //     12.00 V -> 2327         -> F 14.08
    //     12.40 V -> 2405         -> F 13.63
    //     13.00 V -> 2521         -> F 13.00
    //
    // Set for MID-MATCH, about 12.0 V. Taking the figure from a flat pack
    // over-drives a fresh one; taking it from a fresh pack leaves the motor
    // short later, when it matters. P absorbs what is left either way.
    //
    // Was 10.78, from 3040 ticks/s at 13.0 V on a BARE ROTOR. The flywheels
    // cost about 17% of free speed, so that figure was never going to hold.
    //
    // 11.55 V is the LOADED figure -- the pack rested at 12.25 V, so that is
    // 0.70 V of sag with both flywheels at full power. Holding 2650 rpm draws
    // far less than that, so the real operating voltage is around 11.95 and
    // 14.08 lands within half a percent of it. Loaded voltage is the right
    // basis for F: it is what the motor actually sees while it runs.
    //
    // Still worth re-measuring on a pack rested above 12.4 V, which is what
    // the readiness check asks for -- 12.25 is just under.
    public static final double SHOOTER_KF = 14.08;

    // P corrects what F gets wrong, so it is sized from F: 0.1 x F is the
    // usual starting point, which would be 1.41. 1.53 is what the page could
    // actually reach (its fine step was 0.1) and it measured well against
    // 1237 commanded:
    //     pollen  1220..1260   spread 40 ticks/s  (3.2%)
    //     nectar  1240..1260   spread 20 ticks/s  (1.6%)
    // Both hold the target. Chasing 1.41 would be fitting noise.
    //
    // Pollen is twice as noisy as nectar on identical gains, so it is not the
    // tuning: flywheel balance, bearing drag, or that motor's encoder. Not a
    // blocker, but it is the difference that decides the dip question below.
    public static final double SHOOTER_KP = 1.53;
    public static final double SHOOTER_KI = 0.15;
    public static final double SHOOTER_KD = 0.0;

    public static final double SHOOTER_MAX_TICKS_PER_SEC = 3040.0;

    public static final double C270_RIGHT_IN  = 1.732;   //  44.000 mm
    public static final double C270_BEHIND_IN = 5.871;   // 149.128 mm
    public static final double C270_HEIGHT_IN = 12.133;  // 308.175 mm
    public static final double C270_PITCH_DEG = -40.0;   // below horizontal
    public static final double C270_DIAGONAL_FOV_DEG = 55.0;

    // Intake wheel centre, from CAD 6 Nov. Same rear face as the camera and
    // the same 44 mm offset, so the C270 looks straight down the collecting
    // path -- a ball in view is a ball the intake can reach.
    //
    // Measured twice and agreed: origin->intake is Y 258.590, Z -96.440, and
    // camera->intake is dY 109.462, dZ 211.735, which lands on the same point
    // from the camera constants above.
    public static final double INTAKE_RIGHT_IN  = 1.732;    //  44.000 mm
    public static final double INTAKE_BEHIND_IN = 10.181;   // 258.590 mm
    public static final double INTAKE_HEIGHT_IN = 3.797;    //  96.440 mm

    // THE COLLECTING POSE. Robot centre to flower centre with the robot
    // parked and collecting, from the CAD assembly: 369.193 mm horizontal,
    // flower reference at Z 0 like the origin. 4.35 in further out than the
    // intake wheel -- the wheel reaches the balls sitting in the flower, not
    // the flower's centre.
    //
    // MechController's FLOWER_* bands are built around this: scoop out at
    // +5, intake on at +2, so both are live before the robot arrives.
    //
    // Loose balls on the floor are a different problem and use
    // INTAKE_BEHIND_IN instead -- there the wheel itself has to reach the
    // ball, so the offset is 10.181, not 14.535.
    public static final double FLOWER_COLLECT_IN = 14.535;  // 369.193 mm

    public final Position cameraPosition =
            new Position(DistanceUnit.INCH, -C270_RIGHT_IN, -C270_BEHIND_IN, C270_HEIGHT_IN, 0);

    public final YawPitchRollAngles cameraOrientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 180, C270_PITCH_DEG, 0, 0);

    public RobotHardware(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Ports as configured on the robot, 13 Nov. C = Control Hub,
        // E = Expansion Hub. A motor's ENCODER must be in the matching
        // numbered encoder port, or getCurrentPosition() reads nothing while
        // the motor still runs -- that is what the nectar buzz turned out to
        // be. The drivetrain's four motors are NOT resolved here; they are
        // named in pedro/Constants.java and only exist once create() does.
        conveyorMot = hwMap.get(DcMotor.class, "conveyorMot");                   // C3
        intakeMot = hwMap.get(DcMotor.class, "intakeMot");                       // E3

        nectarShootingMot = hwMap.get(DcMotorEx.class, "nectarShootingMot");     // C0
        pollenShootingMot = hwMap.get(DcMotorEx.class, "pollenShootingMot");     // E0

        turretRot = hwMap.get(Servo.class, "turretRot");                         // servo C4
        flowerIntake = hwMap.get(Servo.class, "flowerIntake");                   // servo E2
        conveyorPollen = hwMap.get(Servo.class, "conveyorPollen");               // servo C2
        conveyorNectar = hwMap.get(Servo.class, "conveyorNectar");               // servo E0
        hoodPollen = hwMap.get(Servo.class, "hoodPollen");                       // servo C0
        hoodNectar = hwMap.get(Servo.class, "hoodNectar");                       // servo E4

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");           // I2C C1
        imu = hwMap.get(IMU.class, "imu");                                       // I2C C0

        sensorIntakeR = hwMap.get(RevColorSensorV3.class, "sensorIntakeR");      // I2C E0
        sensorIntakeL = hwMap.get(RevColorSensorV3.class, "sensorIntakeL");      // I2C E1
        sensorNectarL = hwMap.get(RevColorSensorV3.class, "sensorNectarL");      // I2C E3
        sensorNectarR = hwMap.get(RevColorSensorV3.class, "sensorNectarR");      // I2C E2
        sensorPollenL = hwMap.get(RevColorSensorV3.class, "sensorPollenL");      // I2C C3
        sensorPollenR = hwMap.get(RevColorSensorV3.class, "sensorPollenR");      // I2C C2

        magneticLimitSwitch = hwMap.get(DigitalChannel.class, "magneticLimitSwitch"); // digital C0
        magneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        camera = hwMap.get(WebcamName.class, "Webcam 1");                        // USB 2
        limeLight = hwMap.get(Limelight3A.class, "limeLight");                   // USB 3

        configureMotors();
        configureServos();
    }

    private void configureMotors() {
        intakeMot.setDirection(DcMotor.Direction.FORWARD);
        intakeMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        conveyorMot.setDirection(DcMotor.Direction.REVERSE);
        conveyorMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyorMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Both FORWARD, measured on the built robot 13 Nov -- both needed Y on
        // RobotSetup DIRECTION. They were REVERSE, which would have read
        // NEGATIVE velocity: every RPM gate, the exit detector and the dip
        // counter work off getVelocity(), so a wrong direction here is not
        // just a backwards flywheel, it breaks the shot logic outright.
        //
        // Separate calls, not one shared helper: the flywheels are mirror
        // mounted and there is no rule that they match.
        configureShooter(nectarShootingMot, DcMotor.Direction.FORWARD);
        configureShooter(pollenShootingMot, DcMotor.Direction.FORWARD);
    }

    private void configureShooter(DcMotorEx m, DcMotor.Direction dir) {
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Coast, not brake: braking a flywheel between shots wastes spin-up
        // time and heats the motor for no benefit.
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
    }

    private void configureServos() {
        widen(turretRot);
        widen(hoodPollen);
        widen(hoodNectar);
    }

    private static void widen(Servo s) {
        if (s instanceof ServoImplEx) {
            ((ServoImplEx) s).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    public static double rpmToTicksPerSec(double rpm) {
        return rpm * SHOOTER_TICKS_PER_REV / 60.0;
    }

    public static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    public void addPinpointTelemetry(Telemetry telemetry) {
        if (pinpoint == null) {
            telemetry.addData("Pinpoint", "Not configured");
            return;
        }
        telemetry.addData(
                "Pinpoint",
                "X: %.1f in | Y: %.1f in | Heading: %.1f deg",
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.DEGREES)
        );
    }
}
