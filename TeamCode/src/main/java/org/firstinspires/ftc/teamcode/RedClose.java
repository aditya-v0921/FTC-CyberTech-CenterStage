package org.firstinspires.ftc.teamcode;

// Import necessary libraries and packages
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.example.meepmeeptesting.MeepMeepTesting;
import java.util.List;


/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Red Close", group = "Concept")

public class RedClose extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "RedDetectModel.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "RedObj"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        DcMotorEx motorLeftLinear = hardwareMap.get(DcMotorEx.class, "LLinearMotor");
        DcMotorEx motorRightLinear = hardwareMap.get(DcMotorEx.class, "RLinearMotor");
        DcMotorEx motorIntake = hardwareMap.get(DcMotorEx.class, "Intake");
        DcMotor motorWinch = hardwareMap.get(DcMotor.class, "Winch");
        Servo outtakeServo = hardwareMap.get(Servo.class, "Outtake");
        Servo dropIntake = hardwareMap.get(Servo.class, "dropIntake");
        Servo fourBar = hardwareMap.get(Servo.class, "fourBar");
        Servo drone = hardwareMap.get(Servo.class, "drone");
        drone.setDirection(Servo.Direction.REVERSE);

        fourBar.setDirection(Servo.Direction.REVERSE);
        fourBar.setPosition(0.78); // Four bar Down
        dropIntake.setPosition(1); // Intake Up

        motorRightLinear.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftLinear.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorWinch.setDirection(DcMotorSimple.Direction.FORWARD);

        motorRightLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeftLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int initialRightPos = motorRightLinear.getCurrentPosition();
        int initalLeftPos = motorLeftLinear.getCurrentPosition();

        int curRightPos = motorRightLinear.getCurrentPosition();
        int curLeftPos = motorLeftLinear.getCurrentPosition();

        // Initialize TFOD and Vision Portal
        initTfod();

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get the position from TensorFlow detection
                String detectedPosition = telemetryTfod();
                telemetry.update();
                sleep(20); // Small delay to save resources

                if (detectedPosition.equals("center")) {
                    // Define and follow trajectory if the detected object is in the center
                    Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(12, -68, Math.toRadians(90)))
                            //.forward(24)
                            //.back(10)
                            // Adjust subsequent commands as needed based on the new starting position and heading
                            //.splineToSplineHeading(new Pose2d(38, -34, Math.toRadians(0)), Math.toRadians(90))
                            .build();

                    drive.followTrajectory(trajectory);
                    linearslides(motorLeftLinear, motorRightLinear, 2000);
                    visionPortal.close();

                    // Optionally, break out of the loop if you only want to execute this once
                    break;
                }
                else if(detectedPosition.equals("left")) {
                    Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                            //.splineTo(new Vector2d(29, 5), Math.toRadians(90)) // Adjust your trajectory as needed
                            .build();

                    drive.followTrajectory(trajectory);
                    visionPortal.close();
                }
                else{
                    Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                            //.splineTo(new Vector2d(29, -5), Math.toRadians(270)) // Adjust your trajectory as needed
                            .build();

                    drive.followTrajectory(trajectory);
                    visionPortal.close();
                }


                telemetry.update();
                sleep(20); // Small delay to save resources
            }
        }

        // Close Vision Portal to save resources
        visionPortal.close();
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 123"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private String telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Assuming a 640x480 resolution; adjust this according to your actual camera resolution
        final double IMAGE_WIDTH = 640.0;

        String realPos = "right"; // Default position is "right"

        for (Recognition recognition : currentRecognitions) {
            double centerX = (recognition.getLeft() + recognition.getRight()) / 2.0;

            // Determine the object's position based on its centerX
            realPos = centerX < (IMAGE_WIDTH / 2) ? "left" : "center";

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "X: %.0f - %s", centerX, realPos);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }

        return realPos;
    }

    public void linearslides(DcMotorEx motor1, DcMotorEx motor2, int encoderTicks) {
        // Convert the encoderTicks to int for comparison (if necessary)
        int targetEncoderTicks = encoderTicks;

        // Set the motors to run using the encoder feedback
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Continue moving the motors until they are within +/- 100 ticks of the target position
        while (Math.abs(motor1.getCurrentPosition() - targetEncoderTicks) > 100 ||
                Math.abs(motor2.getCurrentPosition() - targetEncoderTicks) > 100) {

            // Set the motors' power to move up
            motor1.setPower(-1);
            motor2.setPower(-1);
        }

        // Stop the motors after reaching the target range
        motor1.setPower(0);
        motor2.setPower(0);
    }


}   // end method telemetryTfod()

  // end class