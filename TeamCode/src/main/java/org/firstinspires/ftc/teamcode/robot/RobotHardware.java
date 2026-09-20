package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public final DcMotorEx shootingMot;
    public final Servo indexer, lifter;
    public final GoBildaPinpointDriver pinpoint;
    public final IMU imu;
    public final Telemetry telemetry;
    public final RevColorSensorV3 sensorR, sensorL;
    public final WebcamName camera;
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

        shootingMot = hwMap.get(DcMotorEx.class, "shootingMot"); // E?
        intakeMot = hwMap.get(DcMotor.class, "intakeMot"); // E?
        // LFMotor C?
        // LBMotor C?
        // RFMotor E?
        // RBMotor E?

        indexer = hwMap.get(Servo.class, "indexer"); // E?
        lifter = hwMap.get(Servo.class, "lifter"); // C?

        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint"); //CI2C ?
        imu = hwMap.get(IMU.class, "imu"); //CI2C0

        sensorL = hwMap.get(RevColorSensorV3.class, "sensorL"); // EI2C ?
        sensorR = hwMap.get(RevColorSensorV3.class, "sensorR"); // EI2C ?
        camera = hwMap.get(WebcamName.class, "Webcam 1");

        setMotorDirections();
    }

    private void setMotorDirections() {
        intakeMot.setDirection(DcMotor.Direction.REVERSE);
        intakeMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootingMot.setDirection(DcMotorEx.Direction.FORWARD);
        shootingMot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMot.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }
}
