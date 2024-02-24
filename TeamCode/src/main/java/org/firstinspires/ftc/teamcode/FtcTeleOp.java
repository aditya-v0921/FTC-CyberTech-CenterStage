package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Func;
import java.util.Base64;
import com.acmerobotics.dashboard.FtcDashboard;



@TeleOp
public class FtcTeleOp extends LinearOpMode {

    private Servo continuousServo;
    private int leftLinearPosition = 0;
    private int rightLinearPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        //Drive Motors
        DcMotorEx motorFrontLeft = hardwareMap.get(DcMotorEx.class, "LFMotor");
        DcMotorEx motorBackLeft = hardwareMap.get(DcMotorEx.class, "LBMotor");
        DcMotorEx motorFrontRight = hardwareMap.get(DcMotorEx.class, "RFMotor");
        DcMotorEx motorBackRight = hardwareMap.get(DcMotorEx.class, "RBMotor");
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

        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRightLinear.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftLinear.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorWinch.setDirection(DcMotorSimple.Direction.FORWARD);

        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorRightLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeftLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWinch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int initialRightPos = motorRightLinear.getCurrentPosition();
        int initalLeftPos = motorLeftLinear.getCurrentPosition();

        int curRightPos = motorRightLinear.getCurrentPosition();
        int curLeftPos = motorLeftLinear.getCurrentPosition();


        //motorLeftLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorRightLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //motorLeftLinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorRightLinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int inOutchecker = 1;
        boolean previousXState = false;

        //speed adjust thing
        double speedAdjust = 1;

        waitForStart();

        while (opModeIsActive()) {
            // gamepad controls
            double y = -gamepad2.left_stick_y;
            double x = gamepad2.left_stick_x * -1; // Invert the x-axis input
            double rx = gamepad2.right_stick_x;

            // Calculate motor power with corrected x input for strafing
            double frontRightPower = y - x - rx;
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double backRightPower = y + x - rx;

            // Set motor power with corrected values for strafing direction
            motorBackLeft.setPower(backLeftPower * speedAdjust);
            motorBackRight.setPower(backRightPower * speedAdjust);
            motorFrontLeft.setPower(frontLeftPower * speedAdjust);
            motorFrontRight.setPower(frontRightPower * speedAdjust);

//            telemetry.addData("right", curRightPos);
//            telemetry.addData("left", curLeftPos);
//            telemetry.update();

            if (gamepad1.left_trigger > 0) {
                motorLeftLinear.setPower(gamepad1.left_trigger);
                motorRightLinear.setPower(gamepad1.left_trigger);

                curRightPos = motorRightLinear.getCurrentPosition();
                curLeftPos = motorLeftLinear.getCurrentPosition();


            } else if (gamepad1.left_trigger == 0) {
                motorLeftLinear.setPower(0);
                motorRightLinear.setPower(0);
                curRightPos = motorRightLinear.getCurrentPosition();
                curLeftPos = motorLeftLinear.getCurrentPosition();

            }

            if (gamepad1.right_trigger > 0) {
                motorLeftLinear.setPower(-gamepad1.right_trigger);
                motorRightLinear.setPower(-gamepad1.right_trigger);
                curRightPos = motorRightLinear.getCurrentPosition();
                curLeftPos = motorLeftLinear.getCurrentPosition();


            } else if (gamepad1.right_trigger == 0) {
                motorLeftLinear.setPower(0);
                motorRightLinear.setPower(0);
                curRightPos = motorRightLinear.getCurrentPosition();
                curLeftPos = motorLeftLinear.getCurrentPosition();

            }

            if (gamepad2.y) {
                outtakeServo.setPosition(1);
            } else {
                outtakeServo.setPosition(0.5);
            }

            if (gamepad1.b) {
                motorIntake.setPower(5);
                drone.setPosition(0.2);

            } else {
                motorIntake.setPower(0);
                drone.setPosition(0.5);
            }

            if (gamepad1.right_bumper) {
                dropIntake.setPosition(0.65);
                // moves the intake down
            } else if (gamepad1.left_bumper) {
                // moves the intake up
                dropIntake.setPosition(1);
            }
//
//            if(gamepad1.x && !previousXState) {  // Check if X is pressed and was not pressed in the last cycle
//                fourBar.setPosition(inOutchecker == 1 ? 0.8 : -0.2);
//                inOutchecker = 1 - inOutchecker; // Toggle the state
//            }
//                previousXState = gamepad1.x;
//                //0.6, 1.6
            if (gamepad1.x) {
                fourBar.setPosition(0.3);
            }

            if (gamepad2.a) {
                motorWinch.setPower(0.5);
            } else if (gamepad2.b) {
                motorWinch.setPower(-0.5);
            } else {
                motorWinch.setPower(0);
            }

            if (gamepad1.a) {
                drone.setPosition(1);
            } else {
                drone.setPosition(0.5);
            }

            if (gamepad1.y) {
                // Define a tolerance range for the motor position
                int tolerance = 100; // You can adjust this value based on your setup

                fourBar.setPosition(0.8);
                sleep(500);

                // Moving the linear motors back to their initial positions
                while (true) { // Loop indefinitely
                    // Determine direction for each motor
                    double rightPower = (motorRightLinear.getCurrentPosition() < initialRightPos) ? -0.5 : 0.5;
                    double leftPower = (motorLeftLinear.getCurrentPosition() < initalLeftPos) ? -0.5 : 0.5;

                    // Check if motors are within the tolerance range of initial positions
                    boolean rightInPosition = Math.abs(motorRightLinear.getCurrentPosition() - initialRightPos) <= tolerance;
                    boolean leftInPosition = Math.abs(motorLeftLinear.getCurrentPosition() - initalLeftPos) <= tolerance;

                    // Stop motors if they are in position
                    if (rightInPosition) {
                        rightPower = 0;
                    }
                    if (leftInPosition) {
                        leftPower = 0;
                    }

                    // Set power to motors
                    motorRightLinear.setPower(rightPower);
                    motorLeftLinear.setPower(leftPower);

                    // Break the loop if both motors are in position
                    if (rightInPosition && leftInPosition) {
                        break;
                    }
                }
                //Stop motors after breaking out of the loop
                motorRightLinear.setPower(0);
                motorLeftLinear.setPower(0);
            }
        }

    }
}
//}