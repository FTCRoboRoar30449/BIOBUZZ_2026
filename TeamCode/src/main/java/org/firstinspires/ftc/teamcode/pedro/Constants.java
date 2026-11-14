package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// NOT FINISHED. create() returns null, so every autonomous drives nowhere.
// Auto_Base checks for null and says so on the Driver Station rather than
// throwing, but nothing moves until this returns a real Follower.
//
// Still needed: a Localizer built on the Pinpoint (RobotHardware already
// resolves it as "pinpoint"), then the Follower itself. The Pinpoint also
// needs setOffsets / setEncoderResolution / setEncoderDirections, which are
// pod positions on the built robot - see RobotSetup step 26 (PINPOINT).
public class Constants {

    public static Follower create(HardwareMap h) {
        return null;
    }

    // Names corrected 13 Nov to match the robot config, which has them as
    // "...Mot", not "...Motor". Nothing caught this: RobotHardware does not
    // resolve the drivetrain, so RobotSetup step 1 passing proves nothing about
    // these four, and create() returns null so they are never looked up. The
    // first failure would have been a crash on the first real autonomous.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("LFMot");       // E2
        c.frontRightName.set("RFMot");      // C1
        c.backLeftName.set("LBMot");        // E1
        c.backRightName.set("RBMot");       // C2
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });
}
