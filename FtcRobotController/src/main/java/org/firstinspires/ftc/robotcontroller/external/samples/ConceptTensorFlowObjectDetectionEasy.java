//package org.firstinspires.ftc.robotcontroller.external.samples;
//
//import android.graphics.Bitmap;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.List;
//
//@TeleOp(name = "Concept: TensorFlow Object Detection Easy", group = "Concept")
//@Disabled
//public class ConceptTensorFlowObjectDetectionEasy extends LinearOpMode {
//
//    private static final boolean USE_WEBCAM = true;
//    private TfodProcessor tfod;
//    private VisionPortal visionPortal;
//
//    @Override
//    public void runOpMode() {
//        initTfod();
//
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch Play to start OpMode");
//        telemetry.update();
//        waitForStart();
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                // Capture and split the frame
//                Bitmap fullImage = captureFrameFromCamera();
//                Bitmap[] sections = splitFrame(fullImage);
//
//                int sectionNumber = 1;
//                for (Bitmap section : sections) {
//                    List<Recognition> recognitions = tfod.processFrame(section); // This line might need to be adjusted
//                    displayRecognitionResults(recognitions, sectionNumber);
//                    sectionNumber++;
//                }
//
//                if (gamepad1.dpad_down) {
//                    visionPortal.stopStreaming();
//                } else if (gamepad1.dpad_up) {
//                    visionPortal.resumeStreaming();
//                }
//
//                sleep(20);
//            }
//        }
//        visionPortal.close();
//    }
//
//    private void initTfod() {
//        tfod = TfodProcessor.easyCreateWithDefaults();
//        if (USE_WEBCAM) {
//            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
//        } else {
//            visionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, tfod);
//        }
//    }
//
//    private void telemetryTfod() {
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }
//        telemetry.update();
//    }
//
//    // Method to capture a frame from the camera (to be implemented)
//    private Bitmap captureFrameFromCamera() {
//        // Implement frame capture from the camera
//    }
//
//    // Method to split the frame into sections (to be implemented)
//    private Bitmap[] splitFrame(Bitmap fullImage) {
//        int height = fullImage.getHeight();
//        int width = fullImage.getWidth();
//        int splitHeight = height / 2;
//        Bitmap upperSection = Bitmap.createBitmap(fullImage, 0, 0, width, splitHeight);
//        Bitmap lowerSection = Bitmap.createBitmap(fullImage, 0, splitHeight, width, height - splitHeight);
//        return new Bitmap[]{upperSection, lowerSection};
//    }
//
//    // Method to display recognition results (optional modification for clarity)
//    private void displayRecognitionResults(List<Recognition> recognitions, int sectionNumber) {
//        telemetry.addData("Section", "Processing Section: " + sectionNumber);
//        for (Recognition recognition : recognitions) {
//            telemetry.addData("Label", recognition.getLabel());
//            telemetry.addData("Confidence", recognition.getConfidence());
//            // Add more telemetry data as needed
//        }
//        telemetry.update();
//    }
//}
