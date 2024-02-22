package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Concept: TensorFlow Object Detection", group = "Concept")
public class ObjDetectAuton extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/RedModel.tflite";
    private static final String[] LABELS = {
            "Center",
            "Left",
            "Right",
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initTfod();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryTfod();
                telemetry.update();
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }
                sleep(20);
            }
        }
        visionPortal.close();
    }

    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(tfod);
        visionPortal = builder.build();
    }

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            String label = recognition.getLabel();
            float confidence = recognition.getConfidence();
            telemetry.addData("Location Detected", "%s (%.0f %% Conf.)", label, confidence * 100);

            switch (label) {
                case "Location1":
                    executeCodeForLocation1();
                    break;
                case "Location2":
                    executeCodeForLocation2();
                    break;
                case "Location3":
                    executeCodeForLocation3();
                    break;
                default:
                    telemetry.addData("Error", "Unknown location");
                    break;
            }
        }
    }

    private void executeCodeForLocation1() {
        telemetry.addData("Action", "Executing code for Location 1");
    }

    private void executeCodeForLocation2() {
        telemetry.addData("Action", "Executing code for Location 2");
    }

    private void executeCodeForLocation3() {
        telemetry.addData("Action", "Executing code for Location 3");
    }
}
