
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MechController {

    private final RobotHardware robot;
    private final Telemetry telemetry;
    private final VisionController visionController;

    private MechState currentState;

    // Conveyor movement type
    private enum ConveyorType {
        NONE,
        POLLEN,
        NECTAR
    }

    // Hardware constants
    public final int MAX_CONVEYOR_ROTATION = 300;

    private final double TICKS_PER_POLLEN = 100.0;
    private final double TICKS_PER_NECTAR = 150.0;

    // Conveyor bed servo positions
    public final int CONVEYOR_UP_NECTAR = 60;
    public final int CONVEYOR_DOWN_NECTAR = 45;
    public final int CONVEYOR_UP_POLLEN = 45;
    public final int CONVEYOR_DOWN_POLLEN = 90;

    // Wrong-ball rejection timer
    private static final long REVERSE_INTAKE_TIME_MS = 5000;

    // Ball counts
    public int countNectar = 0;
    public int countPollen = 4;

    // Initial servo states
    public int lastConveyorNectar = -1;
    public int lastConveyorPollen = -1;

    // 2 = red nectar, 3 = blue nectar
    public int allianceTeamNectar = 2;

    // Intake state
    private long reverseBallStart = 0;
    private boolean reversingWrongBall = false;
    private boolean waitingForWrongBallToClear = false;
    private int previousIntakeColor = 0;

    // Shared conveyor motor state
    private ConveyorType activeConveyor = ConveyorType.NONE;
    private int targetPos = -1;

    // Constructor
    public MechController(
            RobotHardware RoboRoar,
            VisionController visionController
    ) {
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
                updateFieldIntake();
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

    // Main update loop
    public void update() {

        handleMechState(currentState);

        // Continue an active conveyor movement even if the mechanism
        // state changes. Only search for new balls during field intake.
        if (currentState == MechState.INTAKE_FIELD
                || activeConveyor != ConveyorType.NONE) {
            updateConveyor();
        }
    }

    // Field intake and wrong-ball rejection
    private void updateFieldIntake() {

        // Stop taking in balls when capacity is reached
        if (countPollen + countNectar >= 4) {
            robot.intakeMot.setPower(0);

            reversingWrongBall = false;
            waitingForWrongBallToClear = false;
            previousIntakeColor = 0;

            return;
        }

        // Raise both conveyor beds during normal intake
        conveyorNectar(1);
        conveyorPollen(1);

        int ballColor = visionController.ballColor(
                robot.sensorIntakeU,
                robot.sensorIntakeD
        );

        long currentTime = System.currentTimeMillis();

        // CASE 1: Reverse a wrong-color ball for up to 5 seconds
        if (reversingWrongBall) {

            if (currentTime - reverseBallStart
                    >= REVERSE_INTAKE_TIME_MS) {

                reversingWrongBall = false;
                waitingForWrongBallToClear = true;

                // Resume forward intake
                robot.intakeMot.setPower(1);

            } else {
                robot.intakeMot.setPower(-1);
            }
        }

        // CASE 2: Resume forward intake and wait for the rejected
        // ball to clear the intake sensors
        else if (waitingForWrongBallToClear) {

            robot.intakeMot.setPower(1);

            if (ballColor == 0) {
                waitingForWrongBallToClear = false;
                previousIntakeColor = 0;
            }
        }

        // CASE 3: Reject a newly detected wrong-color ball
        else if (ballColor != 0
                && ballColor != 1
                && ballColor != allianceTeamNectar) {

            reversingWrongBall = true;
            reverseBallStart = currentTime;

            robot.intakeMot.setPower(-1);
        }

        // CASE 4: Normal forward intake
        else {

            robot.intakeMot.setPower(1);

            // Count each newly detected ball once
            if (ballColor != 0 && previousIntakeColor == 0) {

                if (ballColor == 1) {
                    countPollen++;

                } else if (ballColor == allianceTeamNectar) {
                    countNectar++;
                }
            }
        }

        previousIntakeColor = ballColor;
    }

    // Set nectar conveyor bed
    // 1 = up, 0 = down
    public void conveyorNectar(int up1dn0) {

        if (lastConveyorNectar == up1dn0) {
            return;
        }

        if (up1dn0 == 1) {
            robot.conveyorNectar.setPosition(
                    (double) CONVEYOR_UP_NECTAR
                            / MAX_CONVEYOR_ROTATION
            );

            lastConveyorNectar = 1;

        } else {
            robot.conveyorNectar.setPosition(
                    (double) CONVEYOR_DOWN_NECTAR
                            / MAX_CONVEYOR_ROTATION
            );

            lastConveyorNectar = 0;
        }
    }

    // Set pollen conveyor bed
    // 1 = up, 0 = down
    public void conveyorPollen(int up1dn0) {

        if (lastConveyorPollen == up1dn0) {
            return;
        }

        if (up1dn0 == 1) {
            robot.conveyorPollen.setPosition(
                    (double) CONVEYOR_UP_POLLEN
                            / MAX_CONVEYOR_ROTATION
            );

            lastConveyorPollen = 1;

        } else {
            robot.conveyorPollen.setPosition(
                    (double) CONVEYOR_DOWN_POLLEN
                            / MAX_CONVEYOR_ROTATION
            );

            lastConveyorPollen = 0;
        }
    }

    // Shared conveyor controller
    public void updateConveyor() {

        // Finish the active movement before starting another
        if (activeConveyor != ConveyorType.NONE) {

            if (!robot.conveyorMot.isBusy()) {

                robot.conveyorMot.setPower(0);

                robot.conveyorMot.setMode(
                        DcMotor.RunMode.RUN_USING_ENCODER
                );

                targetPos = -1;
                activeConveyor = ConveyorType.NONE;
            }

            return;
        }

        // Do not start a new conveyor movement outside field intake
        if (currentState != MechState.INTAKE_FIELD) {
            return;
        }

        // Read the two stopper sensor pairs
        int pollenColor = visionController.ballColor(
                robot.sensorPollenL,
                robot.sensorPollenR
        );

        int nectarColor = visionController.ballColor(
                robot.sensorNectarL,
                robot.sensorNectarR
        );

        // Pollen has priority if both stoppers detect balls
        if (pollenColor == 1) {

            startConveyor(ConveyorType.POLLEN);

        } else if (nectarColor == allianceTeamNectar) {

            startConveyor(ConveyorType.NECTAR);
        }
    }

    // Start one conveyor movement
    private void startConveyor(ConveyorType type) {

        // Prevent simultaneous commands to the shared motor
        if (activeConveyor != ConveyorType.NONE) {
            return;
        }

        if (type == ConveyorType.POLLEN) {

            conveyorNectar(0);
            conveyorPollen(1);

        } else if (type == ConveyorType.NECTAR) {

            conveyorNectar(1);
            conveyorPollen(0);

        } else {
            return;
        }

        double ticks;

        if (type == ConveyorType.POLLEN) {
            ticks = TICKS_PER_POLLEN;
        } else {
            ticks = TICKS_PER_NECTAR;
        }

        targetPos = robot.conveyorMot.getCurrentPosition()
                + (int) ticks;

        activeConveyor = type;

        robot.conveyorMot.setTargetPosition(targetPos);

        robot.conveyorMot.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        robot.conveyorMot.setPower(1);
    }

    // Telemetry output
    public void allTelemetry() {

        telemetry.addData("State", currentState);
        telemetry.addData(
                "Pollen Count",
                countPollen
        );
        telemetry.addData(
                "Nectar Count",
                countNectar
        );
        telemetry.addData(
                "Rejecting Wrong Ball",
                reversingWrongBall
        );
        telemetry.addData(
                "Active Conveyor",
                activeConveyor
        );

        if (robot.pinpoint != null) {

            robot.pinpoint.update();

            telemetry.addData(
                    "Pinpoint",
                    "X: %.1f in | Y: %.1f in | Heading: %.1f°",
                    robot.pinpoint.getPosX(DistanceUnit.INCH),
                    robot.pinpoint.getPosY(DistanceUnit.INCH),
                    robot.pinpoint.getHeading(AngleUnit.DEGREES)
            );
        }

        telemetry.update();
    }
}
