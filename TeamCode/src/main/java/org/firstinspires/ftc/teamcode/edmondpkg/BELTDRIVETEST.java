package org.firstinspires.ftc.teamcode.edmondpkg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
/**
 * Config file
 * Port 00: frontLeft
 * Port 01: frontRight
 * Port 02: backLeft
 * Port 03: backRight
 */
//@Disabled
@TeleOp(group = "Zalkin", name = "beltDriveTest")
public class BELTDRIVETEST extends LinearOpMode {
    // Motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // speed Constant
    private double mecanumDriveSpeedMultiplier = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(); // method inits the hardware motors, servos and sensors
        // single command to run once


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();


        telemetry.addData("Status", "Running");
        telemetry.update();


        while (opModeIsActive()) {
            drive();
        }
    }


    public void initHardware() {
        // Initialize Motors
        gamepad1 = new Gamepad();
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void drive() {
        // Get gamepad input
        double leftStickY = gamepad1.left_stick_y; // not Inverted for forward/backward
        double leftStickX = -gamepad1.left_stick_x; // inverted Strafe left/right
        double rightStickX = -gamepad1.right_stick_x; // inverted to turn properly


        // motor powers
        double frontLeftPower = leftStickY + leftStickX + rightStickX;
        double frontRightPower = leftStickY - leftStickX - rightStickX;
        double backLeftPower = leftStickY - leftStickX + rightStickX;
        double backRightPower = leftStickY + leftStickX - rightStickX;


        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }


        // Apply speed multiplier
        frontLeftPower *= mecanumDriveSpeedMultiplier;
        frontRightPower *= mecanumDriveSpeedMultiplier;
        backLeftPower *= mecanumDriveSpeedMultiplier;
        backRightPower *= mecanumDriveSpeedMultiplier;




        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);


        // Telemetry
        telemetry.addData("Front Left", frontLeftPower);
        telemetry.addData("Front Right", frontRightPower);
        telemetry.addData("Back Left", backLeftPower);
        telemetry.addData("Back Right", backRightPower);
        telemetry.addData("Left Stick Y", leftStickY);
        telemetry.addData("Left Stick X", leftStickX);
        telemetry.addData("Right Stick X", rightStickX);
        telemetry.update();
    }
}

