package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

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
    public final Servo turretRot, flowerIntake, conveyorPollen, conveyorNectar, hoodPollen, hoodNectar;
    public final GoBildaPinpointDriver pinpoint;
    public final IMU imu;
    public final Telemetry telemetry;
    public final RevColorSensorV3 sensorIntakeR, sensorIntakeL, sensorNectarL, sensorNectarR, sensorPollenL, sensorPollenR;
    public final DigitalChannel magneticLimitSwitch;
    public final WebcamName camera;
    public final Limelight3A limeLight;
    private final HardwareMap hwMap;

    double kP = 65.0;
    double kI = 0.0;
    double kD = 0.0;
    double kF = 13.6;
    /*
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    public final Position cameraPosition = new Position(DistanceUnit.INCH, 0.00, 1.04, 13.82, 0);
    public final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    public RobotHardware(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        conveyorMot = hwMap.get(DcMotor.class, "conveyorMot"); // C3
        intakeMot = hwMap.get(DcMotor.class, "intakeMot"); // E3
        nectarShootingMot = hwMap.get(DcMotorEx.class, "nectarShootingMot"); // E0
        pollenShootingMot = hwMap.get(DcMotorEx.class, "pollenShootingMot"); // C0

        // LFMotor C?
        // LBMotor C?
        // RFMotor E?
        // RBMotor E?`

        turretRot = hwMap.get(Servo.class, "turretRot"); // C4
        flowerIntake = hwMap.get(Servo.class, "flowerIntake"); // E2
        conveyorPollen = hwMap.get(Servo.class, "conveyorPollen"); // C2
        conveyorNectar = hwMap.get(Servo.class, "conveyorNectar"); // E4
        hoodPollen = hwMap.get(Servo.class, "hoodPollen"); // C0
        hoodNectar = hwMap.get(Servo.class, "hoodNectar"); // E0


        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint"); //CI2C1
        imu = hwMap.get(IMU.class, "imu"); //CI2C0

        sensorIntakeR = hwMap.get(RevColorSensorV3.class, "sensorIntakeR"); // EI2C0
        sensorIntakeL = hwMap.get(RevColorSensorV3.class, "sensorIntakeL"); // EI2C1
        sensorNectarL = hwMap.get(RevColorSensorV3.class, "sensorNectarL"); // EI2C3
        sensorNectarR = hwMap.get(RevColorSensorV3.class, "sensorNectarR"); // EI2C2
        sensorPollenL = hwMap.get(RevColorSensorV3.class, "sensorPollenL"); // CI2C3
        sensorPollenR = hwMap.get(RevColorSensorV3.class, "sensorPollenR"); // CI2C2

        magneticLimitSwitch = hwMap.get(DigitalChannel.class, "magneticLimitSwitch"); //C0-1
        camera = hwMap.get(WebcamName.class, "Webcam 1");
        limeLight = hwMap.get(Limelight3A.class, "limeLight");

        setMotorDirections();
    }

    private void setMotorDirections() {
        intakeMot.setDirection(DcMotor.Direction.REVERSE);
        intakeMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        conveyorMot.setDirection(DcMotor.Direction.REVERSE);
        conveyorMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyorMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyorMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        nectarShootingMot.setDirection(DcMotorEx.Direction.REVERSE);
        nectarShootingMot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        nectarShootingMot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        nectarShootingMot.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        pollenShootingMot.setDirection(DcMotorEx.Direction.REVERSE);
        pollenShootingMot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pollenShootingMot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pollenShootingMot.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }
}
