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
    public final int MAX_CONVEYOR_ROTATION = 60;


    // Limit constants
    public final int CONVEYOR_UP_NECTAR = 60; // angle of servo when nectar conveyor bed is up
    public final int CONVEYOR_DOWN_NECTAR = 45; // angle of servo when nectar conveyor bed is down
    public final int CONVEYOR_UP_POLLEN = 45; // angle of servo when pollen conveyor bed is up
    public final int CONVEYOR_DOWN_POLLEN = 90; // angle of servo when pollen conveyor bed is down

    // Variables
    public int countNectar = 0; // count of nectar in the robot
    public int countPollen = 4; // count of pollen in the robot
    public int lastConveyorNectar = 1; // state of nectar bed
    public int lastConveyorPollen = 1; // state of pollen bed


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

            case SHOOTING_HIVE:
                currentState = MechState.SHOOTING_HIVE;
                break;

            case SHOOTING_FLOWER_NECTAR:
                currentState = MechState.SHOOTING_FLOWER_NECTAR;
                break;

            case SHOOTING_FLOWER_POLLEN:
                currentState = MechState.SHOOTING_FLOWER_POLLEN;
                break;

            case INTAKE_FIELD:
                currentState = MechState.INTAKE_FIELD;
                if (countPollen + countNectar < 4) {
                    conveyorNectar(1);
                    conveyorPollen(1);
                }
                break;

            case INTAKE_FLOWER:
                currentState = MechState.INTAKE_FLOWER;
                break;

            case INTAKE_FLOWER_SHOOT_HIVE:
                currentState = MechState.INTAKE_FLOWER_SHOOT_HIVE;
                break;

            case INTAKE_FLOWER_SHOOT_FLOWER:
                currentState = MechState.INTAKE_FLOWER_SHOOT_FLOWER;
                break;
        }
    }

    // Cleanup State

    // State machine methods
    public void update() {
        handleMechState(this.currentState);
    }

    public void conveyorNectar(int up1dn0){
        if (lastConveyorNectar != up1dn0) {
            if (up1dn0 == 1){
                robot.conveyorNectar.setPosition(CONVEYOR_UP_NECTAR / MAX_CONVEYOR_ROTATION);
                lastConveyorNectar = 1;
            } else {
                robot.conveyorNectar.setPosition(CONVEYOR_DOWN_NECTAR / MAX_CONVEYOR_ROTATION);
                lastConveyorNectar = 0;
            }
        }
    }

    public void conveyorPollen(int up1dn0){
        if (lastConveyorPollen != up1dn0) {
            if (up1dn0 == 1){
                robot.conveyorPollen.setPosition(CONVEYOR_UP_POLLEN / MAX_CONVEYOR_ROTATION);
                lastConveyorPollen = 1;
            } else {
                robot.conveyorPollen.setPosition(CONVEYOR_DOWN_POLLEN / MAX_CONVEYOR_ROTATION);
                lastConveyorPollen = 0;
            }
        }
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
