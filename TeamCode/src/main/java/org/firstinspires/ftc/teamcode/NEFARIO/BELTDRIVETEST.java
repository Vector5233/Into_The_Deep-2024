package org.firstinspires.ftc.teamcode.NEFARIO;

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
@TeleOp(group = "Practive Bot", name = "beltDriveTest")
public class BELTDRIVETEST extends LinearOpMode {
    // Motors
    private DcMotorEx LEFT_FRONT,  RIGHT_FRONT, LEFT_BACK, RIGHT_BACK;

    // Gamepad
    private Gamepad gamepad1;

    // Drive Constants
    private double mecanumDriveSpeedMultiplier = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(); // method inits the hardware motors, servos and sensors
        // single command to run
        // /once

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
        LEFT_FRONT = hardwareMap.get(DcMotorEx.class, "frontLeft");
        RIGHT_FRONT = hardwareMap.get(DcMotorEx.class, "frontRight");
        LEFT_BACK = hardwareMap.get(DcMotorEx.class, "backLeft");
        RIGHT_BACK = hardwareMap.get(DcMotorEx.class, "backRight");


        LEFT_FRONT.setDirection(DcMotor.Direction.FORWARD);
        RIGHT_FRONT.setDirection(DcMotor.Direction.REVERSE);
        LEFT_BACK.setDirection(DcMotor.Direction.FORWARD);
        RIGHT_BACK.setDirection(DcMotor.Direction.REVERSE);

        LEFT_FRONT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RIGHT_FRONT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LEFT_BACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RIGHT_BACK.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LEFT_FRONT.setPower(0);
        RIGHT_FRONT.setPower(0);
        LEFT_BACK.setPower(0);
        RIGHT_BACK.setPower(0);

        LEFT_FRONT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RIGHT_FRONT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LEFT_BACK.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RIGHT_BACK.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive() {
        // Get gamepad input
        double leftStickY = -gamepad1.left_stick_y; // Inverted for forward/backward
        double leftStickX = gamepad1.left_stick_x; // Strafe left/right
        double rightStickX = gamepad1.right_stick_x; // Turn

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


        LEFT_FRONT.setPower(frontLeftPower);
        RIGHT_FRONT.setPower(frontRightPower);
        LEFT_BACK.setPower(backLeftPower);
        RIGHT_BACK.setPower(backRightPower);

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
