package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutonTimedTest extends LinearOpMode{
    DcMotorEx motorFrontLeft = hardwareMap.get(DcMotorEx.class, "LFMotor");
    DcMotorEx motorBackLeft = hardwareMap.get(DcMotorEx.class, "LBMotor");
    DcMotorEx motorFrontRight = hardwareMap.get(DcMotorEx.class, "RFMotor");
    DcMotorEx motorBackRight = hardwareMap.get(DcMotorEx.class, "RBMotor");
    DcMotorEx motorLeftLinear = hardwareMap.get(DcMotorEx.class, "LLinearMotor");
    DcMotorEx motorRightLinear = hardwareMap.get(DcMotorEx.class, "RLinearMotor");
    DcMotorEx motorIntake = hardwareMap.get(DcMotorEx.class, "Intake");
    DcMotor motorWinch = hardwareMap.get(DcMotor.class, "Winch");

    Servo spedBox = hardwareMap.get(Servo.class, "spedBox");
    Servo dropIntake = hardwareMap.get(Servo.class, "dropIntake");
    Servo fourBar = hardwareMap.get(Servo.class, "fourBar");
//        Servo drone = hardwareMap.get(Servo.class, "drone");
//        Servo rightClimb = hardwareMap.get(Servo.class, "rightClimb");
//        Servo leftClimb = hardwareMap.get(Servo.class, "leftClimb");

    private void forward(double power, long time) throws InterruptedException {
        // Set power for all motors
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);

        sleep(time);

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    private void backward(double power, long time) throws InterruptedException {
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(-power);

        sleep(time);

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public void runOpMode() throws InterruptedException{
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                forward(0.5, 1000);
            }
        }
    }

}