
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

@TeleOp(name = "Limelight")
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;

    private static final int APRILTAG_PIPELINE = 1;
    private static final int BALL_COUNT_PIPELINE = 4;

    // Start with ball-counting pipeline enabled
    private boolean ballPipelineEnabled = true;
    private boolean lastA = false;

    @Override
    public void runOpMode() {

        // Initialize Limelight
        limelight = hardwareMap.get(
                Limelight3A.class, "limelight"
        );

        // Start with Pipeline 4
        limelight.pipelineSwitch(BALL_COUNT_PIPELINE);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            // Press A to switch pipelines
            if (gamepad1.a && !lastA) {
                ballPipelineEnabled = !ballPipelineEnabled;

                if (ballPipelineEnabled) {
                    limelight.pipelineSwitch(BALL_COUNT_PIPELINE);
                } else {
                    limelight.pipelineSwitch(APRILTAG_PIPELINE);
                }
            }

            lastA = gamepad1.a;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                if (ballPipelineEnabled) {

                    // Pipeline 4: Pollen and nectar counts
                    double[] output = result.getPythonOutput();

                    if (output != null && output.length >= 2) {
                        telemetry.addData(
                                "Pollen (2.8in)", (int) output[0]
                        );

                        telemetry.addData(
                                "Nectar (3.6in)", (int) output[1]
                        );
                    }

                } else {

                    // Pipeline 1: AprilTag detection
                    boolean tagDetected = false;

                    for (LLResultTypes.FiducialResult tag :
                            result.getFiducialResults()) {

                        telemetry.addData(
                                "Detected AprilTag ID",
                                tag.getFiducialId()
                        );

                        tagDetected = true;
                    }

                    if (!tagDetected) {
                        telemetry.addData(
                                "AprilTag", "None detected"
                        );
                    }
                }
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
