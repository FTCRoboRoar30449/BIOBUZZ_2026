package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MechController {

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private final VisionController visionController;
    private MechState currentState;

    private MechState previousState = MechState.IDLE;

    // Hardware constants

    // Limit constants

    // Variables

    // Constructor
    public MechController(RobotHardware RoboRoar, VisionController visionController) {
        this.robot = RoboRoar;
        this.telemetry = RoboRoar.telemetry;
        this.visionController = visionController;
        this.currentState = MechState.IDLE;
    }

    // State machine handler
    public void handleMechState(MechState state) {
        switch (state) {
            case START:
                currentState = MechState.START;
                break;

            case IDLE:
                currentState = MechState.IDLE;
                break;
        }
    }

    // Cleanup State

    // State machine methods
    public void update() {
        handleMechState(this.currentState);
    }

    // Telemetry output
    public void allTelemetry() {
        telemetry.addData("State", currentState);

        if (robot.pinpoint != null) {
            robot.pinpoint.update();
            telemetry.addData("Pinpoint",
                    "X: %.1f in | Y: %.1f in | Heading: %.1f°",
                    robot.pinpoint.getPosX(DistanceUnit.INCH),
                    robot.pinpoint.getPosY(DistanceUnit.INCH),
                    robot.pinpoint.getHeading(AngleUnit.DEGREES)
            );
        }

        telemetry.update();
    }
}
