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

    // Belt-forward direction. ShooterCalibration reads it: under setPower the
    public static final DcMotor.Direction CONVEYOR_DIR = DcMotor.Direction.REVERSE;

    public static final double SHOOTER_TICKS_PER_REV = 28.0;

    public static final double SHOOTER_KF = 14.08;

    public static final double SHOOTER_KP = 1.53;
    public static final double SHOOTER_KI = 0.15;
    public static final double SHOOTER_KD = 0.0;

    public static final double SHOOTER_MAX_TICKS_PER_SEC = 3040.0;

    // Flywheel. Single source: ShooterRed/Blue and ShooterCalibration read it.
    public static final double SHOOTER_WHEEL_DIA_M = 0.072;

    // two balls grip differently, so they do not share a value.
    public static double SHOOTER_EFF_NECTAR = 0.432;
    // RE-MEASURED 22 Nov, streaming. Step 29 calibration fires ONE ball on a
    public static double SHOOTER_EFF_POLLEN = 0.435;

    public static double shooterEfficiency(boolean nectar) {
        return nectar ? SHOOTER_EFF_NECTAR : SHOOTER_EFF_POLLEN;
    }

    public static final double LIMELIGHT_BACK_IN   = 15.066 / 25.4;   // 0.5932
    public static final double LIMELIGHT_HEIGHT_IN = 441.327 / 25.4;  // 17.375
    public static final double LIMELIGHT_PITCH_DEG = 45.0;
    public static final double LIMELIGHT_HFOV_HALF = 27.25;
    public static final double LIMELIGHT_VFOV_HALF = 21.0;

    public static final double C270_RIGHT_IN  = 1.732;   //  44.000 mm
    public static final double C270_BEHIND_IN = 5.871;   // 149.128 mm
    public static final double C270_HEIGHT_IN = 12.133;  // 308.175 mm
    public static final double C270_PITCH_DEG = -40.0;   // below horizontal
    public static final double C270_DIAGONAL_FOV_DEG = 55.0;

    public static final double INTAKE_RIGHT_IN  = 1.732;    //  44.000 mm
    public static final double INTAKE_BEHIND_IN = 10.181;   // 258.590 mm
    public static final double INTAKE_HEIGHT_IN = 3.797;    //  96.440 mm

    public static final double FLOWER_COLLECT_IN = 14.535;  // 369.193 mm

    // calibration distances.
    public static final double BUMPER_FWD_IN = 8.786;       // 223.175 mm

    public final Position cameraPosition =
            new Position(DistanceUnit.INCH, -C270_RIGHT_IN, -C270_BEHIND_IN, C270_HEIGHT_IN, 0);

    public final YawPitchRollAngles cameraOrientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 180, C270_PITCH_DEG, 0, 0);

    public RobotHardware(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

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

        conveyorMot.setDirection(CONVEYOR_DIR);
        conveyorMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyorMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        configureShooter(nectarShootingMot, DcMotor.Direction.FORWARD);
        configureShooter(pollenShootingMot, DcMotor.Direction.FORWARD);
    }

    private void configureShooter(DcMotorEx m, DcMotor.Direction dir) {
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
