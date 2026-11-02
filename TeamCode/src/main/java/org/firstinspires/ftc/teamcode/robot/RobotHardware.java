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

/**
 * Maps robot hardware and applies basic motor configuration.
 * Mechanism behavior belongs in MechController, not in this class.
 */
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

    /**
     * goBILDA 5000 Series bare motor, direct drive to the flywheel -- no
     * reduction. 28 countable events per output-shaft revolution.
     * ticks/sec = rpm * 28 / 60.
     */
    public static final double SHOOTER_TICKS_PER_REV = 28.0;

    /**
     * MEASURED on the bench, not inherited. Free-run at full power gave
     * 6214 rpm = 2900 ticks/s, which also confirms the 28 cpr assumption --
     * at 14 cpr that reading would mean 3107 actual rpm, far under this
     * motor's 6000 rpm spec, and at 56 cpr it would mean 12428, impossible.
     *
     * Standard flywheel recipe from the measured maximum:
     *     F = 32767 / max_ticks_per_sec = 32767 / 2900
     *     P = 0.1 x F        I = 0.1 x P        D = 0
     *
     * The old P = 65 was roughly 50x this and caused the velocity loop to
     * oscillate. Re-check once real flywheels are fitted: the extra inertia
     * makes the loop MORE stable, so these may end up slightly conservative.
     */
    public static final double SHOOTER_KP = 1.13;
    public static final double SHOOTER_KI = 0.113;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KF = 11.30;

    /** Free-run ceiling, ticks/s. Useful as a sanity bound on commanded RPM. */
    public static final double SHOOTER_MAX_TICKS_PER_SEC = 2900.0;

    /**
     * Logitech C270, rear right of the robot behind the pollen shooter,
     * fixed, looking backward and 40 deg below horizontal. From CAD:
     * 44.000 mm right, 149.128 mm behind centre, 308.175 mm above the tiles.
     *
     * Robot frame used here and by the shooter classes: +X forward, +Y left,
     * +Z up. The C270 is NOT turret-mounted and does NOT see the HIVE.
     */
    public static final double C270_RIGHT_IN  = 1.732;   //  44.000 mm
    public static final double C270_BEHIND_IN = 5.871;   // 149.128 mm
    public static final double C270_HEIGHT_IN = 12.133;  // 308.175 mm
    public static final double C270_PITCH_DEG = -40.0;   // below horizontal
    /**
     * Logitech's published figure, and the only FOV number that does not
     * depend on capture aspect. Horizontal and vertical fall out of it:
     *      640x480  (4:3)   -> 45.2 x 34.7   <- what the portal uses
     *      1280x720 (16:9)  -> 48.8 x 28.6
     * BallDetector derives one focal length from this and the live frame
     * size, so it stays correct if the resolution changes.
     */
    public static final double C270_DIAGONAL_FOV_DEG = 55.0;

    public final Position cameraPosition =
            new Position(DistanceUnit.INCH, -C270_RIGHT_IN, -C270_BEHIND_IN, C270_HEIGHT_IN, 0);

    /** Rear-facing (yaw 180) and pitched 40 deg down. */
    public final YawPitchRollAngles cameraOrientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 180, C270_PITCH_DEG, 0, 0);

    public RobotHardware(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        conveyorMot = hwMap.get(DcMotor.class, "conveyorMot");
        intakeMot = hwMap.get(DcMotor.class, "intakeMot");

        nectarShootingMot = hwMap.get(DcMotorEx.class, "nectarShootingMot");
        pollenShootingMot = hwMap.get(DcMotorEx.class, "pollenShootingMot");

        turretRot = hwMap.get(Servo.class, "turretRot");
        flowerIntake = hwMap.get(Servo.class, "flowerIntake");
        conveyorPollen = hwMap.get(Servo.class, "conveyorPollen");
        conveyorNectar = hwMap.get(Servo.class, "conveyorNectar");
        hoodPollen = hwMap.get(Servo.class, "hoodPollen");
        hoodNectar = hwMap.get(Servo.class, "hoodNectar");

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        imu = hwMap.get(IMU.class, "imu");

        sensorIntakeR = hwMap.get(RevColorSensorV3.class, "sensorIntakeR");
        sensorIntakeL = hwMap.get(RevColorSensorV3.class, "sensorIntakeL");
        sensorNectarL = hwMap.get(RevColorSensorV3.class, "sensorNectarL");
        sensorNectarR = hwMap.get(RevColorSensorV3.class, "sensorNectarR");
        sensorPollenL = hwMap.get(RevColorSensorV3.class, "sensorPollenL");
        sensorPollenR = hwMap.get(RevColorSensorV3.class, "sensorPollenR");

        magneticLimitSwitch = hwMap.get(DigitalChannel.class, "magneticLimitSwitch");
        magneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        camera = hwMap.get(WebcamName.class, "Webcam 1");
        limeLight = hwMap.get(Limelight3A.class, "limeLight");

        configureMotors();
        configureServos();
    }

    private void configureMotors() {
        /*
         * intakeMot's OWN encoder is disconnected -- that port reads the
         * turret's Through Bore Encoder V2 instead. So it must run OPEN LOOP:
         * in RUN_USING_ENCODER the controller would regulate intake speed
         * against turret rotation, which surges and stalls unpredictably.
         * getCurrentPosition() still reads the turret encoder either way.
         */
        /*
         * MEASURED on the bench: FORWARD turns the intake clockwise, which is
         * the direction that pulls a ball IN. Was REVERSE.
         *
         * WARNING -- this port also carries the turret's Through Bore Encoder.
         * setDirection() flips the sign of getCurrentPosition() as well as the
         * sign of power, so changing this line INVERTS every turret encoder
         * reading. TurretController.ENCODER_SIGN must absorb that, and it is
         * still unmeasured, so this is harmless today and must be measured
         * AFTER the turret is built, not before.
         */
        intakeMot.setDirection(DcMotor.Direction.FORWARD);
        intakeMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        conveyorMot.setDirection(DcMotor.Direction.REVERSE);
        conveyorMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyorMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        configureShooter(nectarShootingMot);
        configureShooter(pollenShootingMot);
    }

    private void configureShooter(DcMotorEx m) {
        m.setDirection(DcMotor.Direction.REVERSE);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Coast, not brake: braking a flywheel between shots wastes spin-up
        // time and heats the motor for no benefit.
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
    }

    /**
     * The turret and both hoods are goBILDA 2000-series 5-turn servos. They
     * only reach their full 1800 deg if the PWM range is widened -- with the
     * SDK default they travel roughly a third of that and every commanded
     * angle silently comes up short.
     */
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

    /** Commanded flywheel RPM -> encoder ticks/second. */
    public static double rpmToTicksPerSec(double rpm) {
        return rpm * SHOOTER_TICKS_PER_REV / 60.0;
    }

    /** Encoder ticks/second -> RPM. */
    public static double ticksPerSecToRpm(double tps) {
        return tps * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    /**
     * Read-only Pinpoint telemetry. The localization system should own
     * pinpoint.update() so it is not updated twice in one control loop.
     */
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


